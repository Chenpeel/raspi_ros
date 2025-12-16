#!/usr/bin/env python3
"""
舵机控制 WebSocket + ROS 2 集成节点
负责启动 WebSocket 服务器并处理来自树莓派的消息
"""

import asyncio
import json
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

# WebSocket 相关
from websocket_bridge import WebSocketBridgeServer, WebSocketHandler

# ROS 2 相关
from servo_core import ServoCoreNode, StatusManager


class ServoControlNode(Node):
    """
    舵机控制节点 - 集成 WebSocket 服务器和 ROS 2 功能

    架构:
    1. WebSocket 服务器监听 9102 端口 (来自树莓派/Web 的连接)
    2. ROS 2 节点处理舵机控制和状态发布
    3. 通过事件循环将异步 WebSocket 和同步 ROS 2 集成
    """

    def __init__(self):
        super().__init__('servo_control')

        # 获取参数
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 9102)
        self.declare_parameter('device_id', 'default')
        self.declare_parameter('debug', False)

        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = self.get_parameter('ws_port').value
        self.device_id = self.get_parameter('device_id').value
        self.debug = self.get_parameter('debug').value

        self.get_logger().info(
            f"[ServoControl] 初始化: host={self.ws_host}, port={self.ws_port}"
        )

        # 初始化 WebSocket 服务器
        self.ws_server = WebSocketBridgeServer(
            host=self.ws_host,
            port=self.ws_port,
            device_id=self.device_id,
            debug=self.debug
        )

        # 初始化状态管理器
        self.status_manager = StatusManager()

        # 创建核心节点引用
        self.core_node = None

        # 异步循环
        self.loop = None
        self.ws_task = None

        # 设置 WebSocket 回调
        self._setup_websocket_callbacks()

        # 创建发布/订阅
        self._setup_ros_topics()

        # 启动异步任务
        self.create_timer(0.1, self._ros_spin_callback)

    def _setup_websocket_callbacks(self):
        """设置 WebSocket 服务器回调"""

        # 舵机命令回调
        async def on_servo_command(servo_cmd: dict):
            """处理来自 Web/树莓派的舵机命令"""
            try:
                if self.debug:
                    self.get_logger().info(f"[ServoControl] 收到舵机命令: {servo_cmd}")

                # 发布到 ROS 2 话题
                msg_str = json.dumps(servo_cmd, ensure_ascii=False)
                await self._publish_servo_command(msg_str)

            except Exception as e:
                self.get_logger().error(f"[ServoControl] 处理舵机命令异常: {e}")

        # 心跳回调
        async def on_heartbeat():
            """处理心跳"""
            if self.debug:
                self.get_logger().debug("[ServoControl] 收到心跳")

        # 状态查询回调
        async def on_status_query():
            """处理状态查询"""
            status = {
                "character_name": "robot",
                "device_id": self.device_id,
                "status": "online",
                "connected_clients": self.ws_server.get_client_count(),
                "servo_states": {},
                "system_state": self.status_manager.system_state
            }
            return status

        self.ws_server.set_servo_command_callback(on_servo_command)
        self.ws_server.set_heartbeat_callback(on_heartbeat)
        self.ws_server.set_status_query_callback(on_status_query)

    def _setup_ros_topics(self):
        """设置 ROS 2 话题"""
        from std_msgs.msg import String

        # 舵机命令发布者
        self.servo_command_pub = self.create_publisher(
            String,
            '/servo/command',
            10
        )

        # 舵机状态订阅者
        self.servo_state_sub = self.create_subscription(
            String,
            '/servo/state',
            self._servo_state_callback,
            10
        )

        # 系统状态发布者
        self.system_state_pub = self.create_publisher(
            String,
            '/system/state',
            10
        )

    async def _publish_servo_command(self, cmd_str: str):
        """发布舵机命令到 ROS 2"""
        from std_msgs.msg import String

        try:
            msg = String()
            msg.data = cmd_str
            self.servo_command_pub.publish(msg)

            if self.debug:
                self.get_logger().debug(f"[ServoControl] 发布舵机命令")

        except Exception as e:
            self.get_logger().error(f"[ServoControl] 发布舵机命令异常: {e}")

    def _servo_state_callback(self, msg):
        """处理舵机状态反馈"""
        try:
            state_dict = json.loads(msg.data)

            if self.debug:
                self.get_logger().debug(f"[ServoControl] 舵机状态更新: {state_dict}")

            # 更新状态管理器
            if "servo_type" in state_dict and "servo_id" in state_dict:
                self.status_manager.update_servo_state(
                    servo_id=state_dict.get("servo_id", 0),
                    servo_type=state_dict.get("servo_type", "bus"),
                    position=state_dict.get("position", 0)
                )

                # 更新 WebSocket 处理器状态
                self.ws_server.handler.update_servo_state(
                    servo_id=state_dict.get("servo_id", 0),
                    servo_type=state_dict.get("servo_type", "bus"),
                    position=state_dict.get("position", 0)
                )

        except Exception as e:
            self.get_logger().error(f"[ServoControl] 处理舵机状态异常: {e}")

    async def _start_websocket_server(self):
        """启动 WebSocket 服务器（异步）"""
        try:
            await self.ws_server.start()
            self.get_logger().info("[ServoControl] WebSocket 服务器已启动")

            # 保持服务器运行
            while rclpy.ok():
                await asyncio.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f"[ServoControl] WebSocket 启动异常: {e}")
        finally:
            await self.ws_server.stop()

    def _ros_spin_callback(self):
        """ROS 2 回调，在主循环中调用异步任务"""
        if self.loop is None:
            # 首次创建事件循环
            try:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)

                # 启动 WebSocket 服务器
                self.ws_task = self.loop.create_task(self._start_websocket_server())

            except Exception as e:
                self.get_logger().error(f"[ServoControl] 初始化异步循环异常: {e}")

        # 处理一些异步事件
        try:
            if self.loop:
                # 运行一个短时间的事件循环步骤，避免阻塞 ROS 2
                self.loop.run_until_complete(asyncio.sleep(0.01))
        except Exception as e:
            if not isinstance(e, RuntimeError):  # 忽略 "no running event loop" 错误
                self.get_logger().debug(f"[ServoControl] 异步循环步骤异常: {e}")

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("[ServoControl] 正在清理资源...")

        if self.loop and self.ws_task:
            try:
                self.loop.run_until_complete(self.ws_server.stop())
            except Exception as e:
                self.get_logger().error(f"[ServoControl] 停止 WebSocket 异常: {e}")

        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = ServoControlNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[Main] 收到中断信号")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
