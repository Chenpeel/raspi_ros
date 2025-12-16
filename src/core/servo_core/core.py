"""
舵机核心模块 - ROS 2 节点集成
"""

import json
import asyncio
import time
from typing import Optional, Dict, Any, Callable
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist


class ServoCoreNode(Node):
    """
    舵机核心 ROS 2 节点

    职责:
    1. 与 WebSocket 服务器通信
    2. 管理舵机控制话题
    3. 管理系统状态话题
    """

    def __init__(self):
        super().__init__('servo_core')

        # 参数
        self.declare_parameter('device_id', 'default')
        self.declare_parameter('debug', False)

        self.device_id = self.get_parameter('device_id').value
        self.debug = self.get_parameter('debug').value

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
            self.servo_state_callback,
            10
        )

        # 系统状态发布者
        self.system_state_pub = self.create_publisher(
            String,
            '/system/state',
            10
        )

        # 运动控制发布者（可选）
        self.motion_pub = self.create_publisher(
            Twist,
            '/motion/cmd_vel',
            10
        )

        # WebSocket 处理器引用
        self.ws_handler = None

        # 状态缓存
        self.last_servo_state = {}
        self.last_system_state = {}

        self.get_logger().info(
            f"[ServoCore] 初始化完成 (device_id: {self.device_id})"
        )

    def set_websocket_handler(self, handler):
        """设置 WebSocket 处理器"""
        self.ws_handler = handler

    async def handle_servo_command(self, servo_cmd: Dict[str, Any]):
        """
        处理来自 WebSocket 的舵机命令

        Args:
            servo_cmd: 舵机控制命令
                {
                    "servo_type": "bus" | "pca",
                    "servo_id": int,
                    "position": int,
                    "speed": int (可选),
                    "port": int (可选)
                }
        """
        try:
            if self.debug:
                self.get_logger().info(f"[ServoCore] 收到舵机命令: {servo_cmd}")

            # 转换为 ROS 2 消息并发布
            msg = String()
            msg.data = json.dumps(servo_cmd, ensure_ascii=False)
            self.servo_command_pub.publish(msg)

            # 异步处理，避免阻塞
            await asyncio.sleep(0.01)

        except Exception as e:
            self.get_logger().error(f"[ServoCore] 处理舵机命令异常: {e}")

    def servo_state_callback(self, msg: String):
        """
        处理舵机状态反馈

        Args:
            msg: 舵机状态消息
        """
        try:
            state_dict = json.loads(msg.data)
            self.last_servo_state = state_dict

            if self.debug:
                self.get_logger().info(f"[ServoCore] 舵机状态更新: {state_dict}")

            # 如果有 WebSocket 处理器，更新其状态
            if self.ws_handler:
                if "servo_type" in state_dict and "servo_id" in state_dict:
                    self.ws_handler.update_servo_state(
                        servo_id=state_dict.get("servo_id", 0),
                        servo_type=state_dict.get("servo_type", "bus"),
                        position=state_dict.get("position", 0)
                    )

        except Exception as e:
            self.get_logger().error(f"[ServoCore] 处理舵机状态异常: {e}")

    def publish_system_state(self, **kwargs):
        """
        发布系统状态

        Args:
            cpu_temp: CPU 温度
            free_memory: 空闲内存 (MB)
            uptime: 运行时间 (秒)
        """
        try:
            state = {
                "timestamp": time.time(),
                **kwargs
            }

            msg = String()
            msg.data = json.dumps(state, ensure_ascii=False)
            self.system_state_pub.publish(msg)

            self.last_system_state = state

            if self.debug:
                self.get_logger().debug(f"[ServoCore] 发布系统状态")

        except Exception as e:
            self.get_logger().error(f"[ServoCore] 发布系统状态异常: {e}")

    def publish_motion_command(self, linear_x: float = 0.0,
                             angular_z: float = 0.0):
        """
        发布运动命令

        Args:
            linear_x: 线性速度 (m/s)
            angular_z: 角速度 (rad/s)
        """
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.motion_pub.publish(twist)

            if self.debug:
                self.get_logger().debug(
                    f"[ServoCore] 发布运动命令: linear_x={linear_x}, angular_z={angular_z}"
                )

        except Exception as e:
            self.get_logger().error(f"[ServoCore] 发布运动命令异常: {e}")


class StatusManager:
    """系统状态管理器"""

    def __init__(self, ros_node: Optional['ServoCoreNode'] = None):
        """
        初始化状态管理器

        Args:
            ros_node: ServoCoreNode 实例
        """
        self.node = ros_node
        self.servo_states = {}
        self.system_state = {
            "cpu_temp": 0.0,
            "free_memory": 0.0,
            "uptime": 0.0
        }
        self.start_time = time.time()

    def update_servo_state(self, servo_id: int, servo_type: str,
                          position: int, **kwargs):
        """
        更新舵机状态

        Args:
            servo_id: 舵机 ID
            servo_type: 舵机类型 ("bus" 或 "pca")
            position: 当前位置
            **kwargs: 其他状态信息
        """
        key = f"{servo_type}_{servo_id}"
        self.servo_states[key] = {
            "servo_id": servo_id,
            "servo_type": servo_type,
            "position": position,
            "timestamp": time.time(),
            **kwargs
        }

    def update_system_state(self, **kwargs):
        """
        更新系统状态

        Args:
            cpu_temp: CPU 温度
            free_memory: 空闲内存
            custom_field: 自定义字段
        """
        self.system_state.update(kwargs)
        self.system_state["uptime"] = time.time() - self.start_time

    def get_all_states(self) -> Dict[str, Any]:
        """获取所有状态"""
        return {
            "servo_states": self.servo_states,
            "system_state": self.system_state
        }

    def get_servo_state(self, servo_id: int, servo_type: str = "bus") -> Optional[Dict]:
        """获取特定舵机的状态"""
        key = f"{servo_type}_{servo_id}"
        return self.servo_states.get(key)


def create_servo_command_msg(servo_type: str, servo_id: int,
                            position: int, speed: int = 100) -> str:
    """
    创建舵机命令消息

    Args:
        servo_type: "bus" 或 "pca"
        servo_id: 舵机 ID
        position: 位置
        speed: 速度

    Returns:
        str: JSON 格式的命令
    """
    cmd = {
        "servo_type": servo_type,
        "servo_id": servo_id,
        "position": position,
        "speed": speed,
        "timestamp": time.time()
    }
    return json.dumps(cmd, ensure_ascii=False)
