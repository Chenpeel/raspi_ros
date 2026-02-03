"""
3-DOF并联控制器ROS 2节点

订阅脚踝姿态命令,转换为舵机控制命令并发布
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from servo_msgs.msg import ServoCommand
from std_msgs.msg import Float32MultiArray
import numpy as np
from typing import Optional

from .kinematics_solver import Parallel3DOFKinematicsSolver


class Parallel3DOFControllerNode(Node):
    """
    3-DOF并联控制器节点

    功能:
    - 订阅脚踝RPY姿态命令
    - 将RPY转换为舵机角度
    - 发布舵机控制命令

    话题:
    - 订阅: ~/ankle_rpy (Vector3) - 脚踝RPY命令 (度)
    - 发布: ~/servo/command (ServoCommand) - 舵机控制命令
    - 发布: ~/ankle_theta (Float32MultiArray) - Theta角反馈 (度)

    参数:
    - l0: 平台半径 (米, 默认0.02)
    - l1: 动平台距离 (米, 默认0.02)
    - l2: 静平台距离 (米, 默认0.02)
    - ankle_side: 控制哪侧脚踝 ('right'/'left', 默认'right')
    - servo_ids: 自定义舵机ID列表 (3个元素)
    - servo_offsets: 自定义舵机offset列表 (3个元素, 可选)
    - servo_directions: 自定义舵机direction列表 (3个元素, 可选)
    - default_speed: 默认舵机速度 (毫秒, 默认100)
    - debug: 是否打印调试信息 (默认False)
    """

    def __init__(self):
        super().__init__('ankle_controller_node')

        # 声明参数
        self.declare_parameter('l0', 0.02)
        self.declare_parameter('l1', 0.02)
        self.declare_parameter('l2', 0.02)
        self.declare_parameter('ankle_side', 'right')
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('debug', False)
        self.declare_parameter('servo_ids', [])
        self.declare_parameter('servo_offsets', [])
        self.declare_parameter('servo_directions', [])

        # 获取参数
        l0 = self.get_parameter('l0').value
        l1 = self.get_parameter('l1').value
        l2 = self.get_parameter('l2').value
        self.ankle_side = self.get_parameter('ankle_side').value
        self.default_speed = self.get_parameter('default_speed').value
        self.debug = self.get_parameter('debug').value
        servo_ids = self.get_parameter('servo_ids').value
        servo_offsets = self.get_parameter('servo_offsets').value
        servo_directions = self.get_parameter('servo_directions').value

        servo_config = None
        if servo_ids:
            servo_ids = list(servo_ids)
            if len(servo_ids) != 3:
                raise ValueError("servo_ids must contain exactly 3 elements")

            servo_offsets = list(servo_offsets) if servo_offsets else [0.0, 0.0, 0.0]
            servo_directions = list(servo_directions) if servo_directions else [1, 1, 1]
            if len(servo_offsets) != 3:
                raise ValueError("servo_offsets must contain exactly 3 elements")
            if len(servo_directions) != 3:
                raise ValueError("servo_directions must contain exactly 3 elements")

            base_mapping = Parallel3DOFKinematicsSolver._default_servo_config()['position_mapping']
            servo_config = {
                'custom': {
                    'servo_1': {
                        'id': int(servo_ids[0]),
                        'offset': float(servo_offsets[0]),
                        'direction': int(servo_directions[0])
                    },
                    'servo_2': {
                        'id': int(servo_ids[1]),
                        'offset': float(servo_offsets[1]),
                        'direction': int(servo_directions[1])
                    },
                    'servo_3': {
                        'id': int(servo_ids[2]),
                        'offset': float(servo_offsets[2]),
                        'direction': int(servo_directions[2])
                    }
                },
                'position_mapping': dict(base_mapping)
            }

            # 使用自定义舵机组
            self.ankle_side = 'custom'

        # 初始化运动学求解器
        try:
            self.solver = Parallel3DOFKinematicsSolver(
                l0=l0,
                l1=l1,
                l2=l2,
                servo_config=servo_config
            )
            self.get_logger().info(f"运动学求解器初始化成功")
        except Exception as e:
            self.get_logger().error(f"运动学求解器初始化失败: {e}")
            raise

        # 订阅脚踝RPY命令
        self.rpy_sub = self.create_subscription(
            Vector3,
            '~/ankle_rpy',
            self.ankle_rpy_callback,
            10
        )

        # 发布舵机命令
        self.servo_cmd_pub = self.create_publisher(
            ServoCommand,
            '~/servo/command',
            10
        )

        # 发布theta角反馈 (可选,用于调试)
        self.theta_pub = self.create_publisher(
            Float32MultiArray,
            '~/ankle_theta',
            10
        )

        self.get_logger().info(
            f"3-DOF并联控制器节点已启动 (侧: {self.ankle_side}, "
            f"l0={l0}m, l1={l1}m, l2={l2}m)"
        )

        # 打印工作空间限制
        limits = self.solver.get_workspace_limits()
        self.get_logger().info("工作空间限制:")
        for axis, lim in limits.items():
            self.get_logger().info(
                f"  {axis}: {lim['deg'][0]:.1f}° ~ {lim['deg'][1]:.1f}°"
            )

    def ankle_rpy_callback(self, msg: Vector3):
        """
        处理脚踝RPY命令

        参数:
            msg: Vector3消息,包含 x=roll, y=pitch, z=yaw (度)
        """
        try:
            # 将度转换为弧度
            roll_rad = np.radians(msg.x)
            pitch_rad = np.radians(msg.y)
            yaw_rad = np.radians(msg.z)

            if self.debug:
                self.get_logger().info(
                    f"收到RPY命令 (度): R={msg.x:.2f}, P={msg.y:.2f}, Y={msg.z:.2f}"
                )

            # 转换为舵机命令
            commands = self.solver.rpy_to_servo_commands(
                roll_rad,
                pitch_rad,
                yaw_rad,
                ankle_side=self.ankle_side,
                speed=self.default_speed
            )

            # 发布舵机命令
            for cmd in commands:
                servo_msg = ServoCommand()
                servo_msg.servo_type = "bus"  # 总线舵机
                servo_msg.servo_id = cmd['id']
                servo_msg.position = cmd['position']
                servo_msg.speed = cmd['speed']
                self.servo_cmd_pub.publish(servo_msg)

                if self.debug:
                    self.get_logger().info(
                        f"  舵机ID={cmd['id']}: position={cmd['position']}us, "
                        f"theta={cmd['theta_deg']:.2f}°"
                    )

            # 发布theta角反馈
            theta_msg = Float32MultiArray()
            theta_msg.data = [
                float(commands[0]['theta_deg']),
                float(commands[1]['theta_deg']),
                float(commands[2]['theta_deg'])
            ]
            self.theta_pub.publish(theta_msg)

        except Exception as e:
            self.get_logger().error(f"处理RPY命令失败: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = Parallel3DOFControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点异常: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
