"""
WebSocket桥接节点 - 连接WebSocket服务器和ROS 2舵机驱动

数据流:
1. WebSocket客户端 -> WebSocket服务器 -> bridge_node -> /servo/command话题 -> 舵机驱动节点
2. 舵机驱动节点 -> /servo/state话题 -> bridge_node -> WebSocket服务器 -> WebSocket客户端
"""

import asyncio
import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from .ws_server import WebSocketBridgeServer


class WebSocketROS2Bridge(Node):
    """WebSocket到ROS 2的桥接节点

    功能:
    1. 启动WebSocket服务器监听Web客户端
    2. 将WebSocket消息转换为ROS 2话题消息
    3. 订阅ROS 2话题并转发到WebSocket客户端
    """

    def __init__(self, ws_host: str = "0.0.0.0", ws_port: int = 9102,
                 device_id: str = "robot", debug: bool = False):
        """
        初始化桥接节点

        Args:
            ws_host: WebSocket服务器监听地址
            ws_port: WebSocket服务器监听端口
            device_id: 设备ID
            debug: 是否启用调试模式
        """
        super().__init__('websocket_ros2_bridge')

        # 参数
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.device_id = device_id
        self.debug = debug

        # ROS 2话题
        # 发布舵机命令到驱动节点
        self.servo_command_pub = self.create_publisher(
            String,
            '/servo/command',
            10
        )

        # 订阅舵机状态反馈
        self.servo_state_sub = self.create_subscription(
            String,
            '/servo/state',
            self.servo_state_callback,
            10
        )

        # WebSocket服务器
        self.ws_server: Optional[WebSocketBridgeServer] = None
        self.ws_loop: Optional[asyncio.AbstractEventLoop] = None
        self.ws_thread: Optional[threading.Thread] = None

        self.get_logger().info(
            f'WebSocket桥接节点已初始化: ws://{ws_host}:{ws_port}'
        )

    def start_websocket_server(self):
        """在独立线程中启动WebSocket服务器"""
        def run_ws_server():
            """WebSocket服务器运行函数（在独立线程中）"""
            # 创建新的事件循环
            self.ws_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.ws_loop)

            # 创建WebSocket服务器
            self.ws_server = WebSocketBridgeServer(
                host=self.ws_host,
                port=self.ws_port,
                device_id=self.device_id,
                debug=self.debug
            )

            # 注册回调
            self.ws_server.set_servo_command_callback(self.handle_servo_command)
            self.ws_server.set_heartbeat_callback(self.handle_heartbeat)
            self.ws_server.set_status_query_callback(self.handle_status_query)

            # 运行服务器
            try:
                self.ws_loop.run_until_complete(self.ws_server.start())
                self.get_logger().info('WebSocket服务器已启动')

                # 保持运行
                self.ws_loop.run_forever()
            except Exception as e:
                self.get_logger().error(f'WebSocket服务器异常: {e}')
            finally:
                if self.ws_loop:
                    self.ws_loop.close()

        # 启动WebSocket线程
        self.ws_thread = threading.Thread(target=run_ws_server, daemon=True)
        self.ws_thread.start()

        self.get_logger().info('WebSocket服务器线程已启动')

    async def handle_servo_command(self, servo_cmd: dict):
        """
        处理来自WebSocket的舵机控制命令

        Args:
            servo_cmd: 舵机命令字典
                {
                    "servo_type": "bus" | "pca",
                    "servo_id": int,
                    "position": int,
                    "speed": int (可选，仅总线舵机)
                }
        """
        try:
            # 转换为JSON字符串
            msg = String()
            msg.data = json.dumps(servo_cmd, ensure_ascii=False)

            # 发布到ROS 2话题
            self.servo_command_pub.publish(msg)

            if self.debug:
                self.get_logger().info(
                    f'发布舵机命令: {servo_cmd["servo_type"]} '
                    f'ID={servo_cmd["servo_id"]} POS={servo_cmd["position"]}'
                )

        except Exception as e:
            self.get_logger().error(f'处理舵机命令失败: {e}')
            raise

    async def handle_heartbeat(self):
        """处理心跳消息"""
        if self.debug:
            self.get_logger().debug('收到WebSocket心跳')

    async def handle_status_query(self) -> dict:
        """
        处理状态查询请求

        Returns:
            dict: 当前系统状态
        """
        # 可以扩展为查询实际的系统状态
        now = self.get_clock().now()
        return {
            "type": "status_response",
            "device_id": self.device_id,
            "status": "online",
            "ros_nodes": self.get_node_names(),
            "timestamp": now.nanoseconds / 1e9  # 转换为秒（浮点数）
        }

    def servo_state_callback(self, msg: String):
        """
        处理舵机状态反馈（来自驱动节点）

        Args:
            msg: 舵机状态消息
                {
                    "servo_type": "bus" | "pca",
                    "servo_id": int,
                    "position": int,
                    "timestamp": {...}
                }
        """
        try:
            state = json.loads(msg.data)

            if self.debug:
                self.get_logger().info(
                    f'收到舵机状态: {state["servo_type"]} '
                    f'ID={state["servo_id"]} POS={state["position"]}'
                )

            # 更新WebSocket服务器的状态
            if self.ws_server:
                servo_type = state.get("servo_type")
                servo_id = state.get("servo_id")
                position = state.get("position")

                if servo_type and servo_id is not None and position is not None:
                    self.ws_server.update_servo_state(servo_id, servo_type, position)

                # 广播状态到所有WebSocket客户端
                if self.ws_loop and not self.ws_loop.is_closed() and self.ws_loop.is_running():
                    try:
                        asyncio.run_coroutine_threadsafe(
                            self.ws_server.broadcast_status(state),
                            self.ws_loop
                        )
                    except RuntimeError as e:
                        if self.debug:
                            self.get_logger().warning(f'无法广播状态（事件循环不可用）: {e}')
                elif self.debug:
                    self.get_logger().debug('跳过广播：WebSocket事件循环未运行')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析舵机状态失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理舵机状态回调异常: {e}')

    def shutdown(self):
        """关闭节点和WebSocket服务器"""
        self.get_logger().info('开始关闭WebSocket桥接节点...')

        # 停止WebSocket服务器
        if self.ws_server and self.ws_loop:
            try:
                # 在WebSocket事件循环中运行stop
                future = asyncio.run_coroutine_threadsafe(
                    self.ws_server.stop(),
                    self.ws_loop
                )
                future.result(timeout=5.0)
            except Exception as e:
                self.get_logger().error(f'停止WebSocket服务器失败: {e}')

        # 停止事件循环
        if self.ws_loop:
            self.ws_loop.call_soon_threadsafe(self.ws_loop.stop)

        # 等待WebSocket线程结束
        if self.ws_thread and self.ws_thread.is_alive():
            self.ws_thread.join(timeout=2.0)

        self.get_logger().info('WebSocket桥接节点已关闭')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 创建桥接节点
    bridge = WebSocketROS2Bridge(
        ws_host="0.0.0.0",
        ws_port=9102,
        device_id="robot",
        debug=True  # 启用调试日志
    )

    # 启动WebSocket服务器（在独立线程）
    bridge.start_websocket_server()

    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    try:
        # 运行ROS 2节点
        bridge.get_logger().info('WebSocket桥接节点开始运行...')
        executor.spin()
    except KeyboardInterrupt:
        bridge.get_logger().info('收到退出信号')
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
