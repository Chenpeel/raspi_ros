"""Isaac-ROS桥接节点。

用于在仿真侧和 ROS 舵机总线侧之间做消息转发:
- /sim/servo_command -> /servo/command
- /servo/state -> /sim/servo_state
"""

import rclpy
from rclpy.node import Node

from servo_msgs.msg import ServoCommand, ServoState

from .isaac_bridge_utils import clamp_servo_position
from .isaac_bridge_utils import normalize_servo_type
from .isaac_bridge_utils import normalize_speed


class IsaacROSBridge(Node):
    """Isaac-ROS消息桥接节点。"""

    def __init__(self):
        super().__init__('isaac_ros_bridge')

        self.declare_parameter('isaac_command_topic', '/sim/servo_command')
        self.declare_parameter('isaac_state_topic', '/sim/servo_state')
        self.declare_parameter('servo_command_topic', '/servo/command')
        self.declare_parameter('servo_state_topic', '/servo/state')
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('enforce_position_limits', True)
        self.declare_parameter('debug', False)

        self.isaac_command_topic = self.get_parameter('isaac_command_topic').value
        self.isaac_state_topic = self.get_parameter('isaac_state_topic').value
        self.servo_command_topic = self.get_parameter('servo_command_topic').value
        self.servo_state_topic = self.get_parameter('servo_state_topic').value
        self.default_speed = int(self.get_parameter('default_speed').value)
        self.enforce_position_limits = bool(
            self.get_parameter('enforce_position_limits').value
        )
        self.debug = bool(self.get_parameter('debug').value)

        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            self.servo_command_topic,
            10
        )
        self.isaac_state_pub = self.create_publisher(
            ServoState,
            self.isaac_state_topic,
            10
        )

        self.isaac_command_sub = self.create_subscription(
            ServoCommand,
            self.isaac_command_topic,
            self.isaac_command_callback,
            10
        )
        self.servo_state_sub = self.create_subscription(
            ServoState,
            self.servo_state_topic,
            self.servo_state_callback,
            10
        )

        self.command_forward_count = 0
        self.state_forward_count = 0

        self.get_logger().info(
            'Isaac桥接已启动: '
            f'{self.isaac_command_topic} -> {self.servo_command_topic}, '
            f'{self.servo_state_topic} -> {self.isaac_state_topic}'
        )

    def isaac_command_callback(self, msg: ServoCommand):
        """处理Isaac下发的舵机命令。"""
        servo_type = normalize_servo_type(msg.servo_type)
        if servo_type is None:
            self.get_logger().warn(
                f'忽略未知舵机类型命令: servo_type={msg.servo_type}'
            )
            return

        position = clamp_servo_position(
            servo_type=servo_type,
            position=msg.position,
            enforce_limits=self.enforce_position_limits
        )
        if position is None:
            self.get_logger().warn(
                f'忽略非法位置命令: id={msg.servo_id}, position={msg.position}'
            )
            return

        forward_msg = ServoCommand()
        forward_msg.servo_type = servo_type
        forward_msg.servo_id = int(msg.servo_id)
        forward_msg.position = int(position)
        forward_msg.speed = normalize_speed(msg.speed, self.default_speed)
        forward_msg.stamp = msg.stamp
        if self._is_zero_stamp(msg.stamp.sec, msg.stamp.nanosec):
            forward_msg.stamp = self.get_clock().now().to_msg()

        self.servo_command_pub.publish(forward_msg)
        self.command_forward_count += 1

        if self.debug:
            self.get_logger().info(
                '转发Isaac命令: '
                f'type={forward_msg.servo_type}, '
                f'id={forward_msg.servo_id}, '
                f'pos={forward_msg.position}, '
                f'speed={forward_msg.speed}'
            )

    def servo_state_callback(self, msg: ServoState):
        """将底层舵机状态转发到Isaac侧。"""
        servo_type = normalize_servo_type(msg.servo_type)
        if servo_type is None:
            return

        position = clamp_servo_position(
            servo_type=servo_type,
            position=msg.position,
            enforce_limits=self.enforce_position_limits
        )
        if position is None:
            return

        state_msg = ServoState()
        state_msg.servo_type = servo_type
        state_msg.servo_id = int(msg.servo_id)
        state_msg.position = int(position)
        state_msg.load = int(msg.load)
        state_msg.temperature = int(msg.temperature)
        state_msg.error_code = int(msg.error_code)
        state_msg.stamp = msg.stamp
        if self._is_zero_stamp(msg.stamp.sec, msg.stamp.nanosec):
            state_msg.stamp = self.get_clock().now().to_msg()

        self.isaac_state_pub.publish(state_msg)
        self.state_forward_count += 1

        if self.debug:
            self.get_logger().info(
                '转发舵机状态: '
                f'type={state_msg.servo_type}, '
                f'id={state_msg.servo_id}, '
                f'pos={state_msg.position}, '
                f'err={state_msg.error_code}'
            )

    @staticmethod
    def _is_zero_stamp(sec: int, nanosec: int) -> bool:
        """判断时间戳是否为零值。"""
        return int(sec) == 0 and int(nanosec) == 0


def main(args=None):
    """入口函数。"""
    rclpy.init(args=args)
    node = IsaacROSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
