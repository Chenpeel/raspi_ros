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
from servo_msgs.msg import ServoCommand, ServoState
from sensor_msgs.msg import ImuData

from .ws_server import WebSocketBridgeServer


class WebSocketROS2Bridge(Node):
    """WebSocket到ROS 2的桥接节点

    功能:
    1. 启动WebSocket服务器监听Web客户端
    2. 将WebSocket消息转换为ROS 2话题消息
    3. 订阅ROS 2话题并转发到WebSocket客户端
    """

    def __init__(self, ws_host: str = "0.0.0.0", ws_port: int = 9105,
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

        # 声明ROS参数
        self.declare_parameter('ws_host', ws_host)
        self.declare_parameter('ws_port', ws_port)
        self.declare_parameter('device_id', device_id)
        self.declare_parameter('debug', debug)
        self.declare_parameter('imu_debug', debug)

        # 从ROS参数读取配置
        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = self.get_parameter('ws_port').value
        self.device_id = self.get_parameter('device_id').value
        self.debug = self.get_parameter('debug').value
        self.imu_debug = self.get_parameter('imu_debug').value

        # ROS 2话题
        # 发布舵机命令到驱动节点
        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            '/servo/command',
            10
        )

        # 订阅舵机状态反馈
        self.servo_state_sub = self.create_subscription(
            ServoState,
            '/servo/state',
            self.servo_state_callback,
            10
        )

        # 订阅 IMU 传感器数据
        self.imu_data_sub = self.create_subscription(
            ImuData,
            '/sensor/imu',
            self.imu_data_callback,
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

    @staticmethod
    def _coerce_float(value, default=None):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _coerce_uint16(value, default=None):
        try:
            val = int(round(float(value)))
        except (TypeError, ValueError):
            return default
        return max(0, min(65535, val))

    @staticmethod
    def _map_centered_angle_to_pulse(angle: float) -> int:
        """Map angle in [-90, 90] to pulse width 500-2500us."""
        angle = max(-90.0, min(90.0, angle))
        return int(round(500 + (angle + 90.0) * (2000.0 / 180.0)))

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
            servo_type = servo_cmd.get("servo_type", "bus")
            raw_position = servo_cmd.get("position")
            if raw_position is None:
                raise ValueError("缺少 position 字段")

            position_val = self._coerce_float(raw_position, None)
            if position_val is None:
                raise ValueError(f"非法 position: {raw_position}")

            if servo_type == "bus":
                # 支持中心角 [-90, 90]，映射到 500-2500us
                if position_val < -90.0:
                    position = self._map_centered_angle_to_pulse(-90.0)
                elif position_val > 90.0:
                    # 超出中心角范围时保持兼容（0-180角度或直接脉宽）
                    position = self._coerce_uint16(position_val, 0)
                else:
                    position = self._map_centered_angle_to_pulse(position_val)
            else:
                # PCA 舵机只接受非负值
                position = self._coerce_uint16(position_val, 0)

            speed = self._coerce_uint16(servo_cmd.get("speed", 100), 100)

            # 转换为ServoCommand消息
            msg = ServoCommand()
            msg.servo_type = servo_type
            msg.servo_id = servo_cmd["servo_id"]
            msg.position = position
            msg.speed = speed  # 默认速度100ms
            msg.stamp = self.get_clock().now().to_msg()

            # 发布到ROS 2话题
            self.servo_command_pub.publish(msg)

            if self.debug:
                self.get_logger().info(
                    f'发布舵机命令: {msg.servo_type} '
                    f'ID={msg.servo_id} POS={msg.position} SPEED={msg.speed}'
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

    def servo_state_callback(self, msg: ServoState):
        """
        处理舵机状态反馈（来自驱动节点）

        Args:
            msg: ServoState消息
        """
        try:
            # 转换为JSON字典
            state = {
                "servo_type": msg.servo_type,
                "servo_id": msg.servo_id,
                "position": msg.position,
                "load": msg.load,
                "temperature": msg.temperature,
                "error_code": msg.error_code,
                "timestamp": msg.stamp.sec + msg.stamp.nanosec / 1e9
            }

            if self.debug:
                self.get_logger().info(
                    f'收到舵机状态: {msg.servo_type} '
                    f'ID={msg.servo_id} POS={msg.position} ERR={msg.error_code}'
                )

            # 更新WebSocket服务器的状态
            if self.ws_server:
                self.ws_server.update_servo_state(
                    msg.servo_id,
                    msg.servo_type,
                    msg.position
                )

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

        except Exception as e:
            self.get_logger().error(f'处理舵机状态回调异常: {e}')

    def imu_data_callback(self, msg: ImuData):
        """
        处理 IMU 传感器数据（来自 IMU 驱动节点）

        Args:
            msg: ImuData消息
        """
        try:
            # 转换为 JSON 字典
            imu_data = {
                "type": "sensor_data",
                "sensor_type": msg.sensor_type,
                "sensor_id": msg.sensor_id,
                "data": {
                    "accel": {
                        "x": msg.accel_x,
                        "y": msg.accel_y,
                        "z": msg.accel_z,
                        "unit": "g"
                    },
                    "gyro": {
                        "x": msg.gyro_x,
                        "y": msg.gyro_y,
                        "z": msg.gyro_z,
                        "unit": "rad/s"
                    },
                    "mag": {
                        "x": msg.mag_x,
                        "y": msg.mag_y,
                        "z": msg.mag_z,
                        "unit": "uT"
                    },
                    "quaternion": {
                        "w": msg.quat_w,
                        "x": msg.quat_x,
                        "y": msg.quat_y,
                        "z": msg.quat_z
                    },
                    "euler": {
                        "roll": msg.roll,
                        "pitch": msg.pitch,
                        "yaw": msg.yaw,
                        "unit": "deg"
                    },
                    "baro": {
                        "height": msg.baro_height,
                        "temperature": msg.baro_temp,
                        "pressure": msg.baro_pressure
                    }
                },
                "error_code": msg.error_code,
                "timestamp": msg.stamp.sec + msg.stamp.nanosec / 1e9
            }

            if self.imu_debug:
                self.get_logger().info(
                    f'收到 IMU 数据: euler=({msg.roll:.1f}, {msg.pitch:.1f}, {msg.yaw:.1f}) '
                    f'accel=({msg.accel_x:.2f}, {msg.accel_y:.2f}, {msg.accel_z:.2f})'
                )

            # 广播到所有 WebSocket 客户端
            if self.ws_server and self.ws_loop and not self.ws_loop.is_closed():
                if self.ws_loop.is_running():
                    try:
                        asyncio.run_coroutine_threadsafe(
                            self.ws_server.broadcast_status(imu_data),
                            self.ws_loop
                        )
                    except RuntimeError as e:
                        if self.imu_debug:
                            self.get_logger().warning(f'无法广播 IMU 数据（事件循环不可用）: {e}')

        except Exception as e:
            self.get_logger().error(f'处理 IMU 数据回调异常: {e}')

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

    # 创建桥接节点(使用默认值,实际值从launch文件参数传入)
    bridge = WebSocketROS2Bridge()

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
