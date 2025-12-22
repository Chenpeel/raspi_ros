"""
舵机命令路由节点

功能:
- 订阅全局话题 /servo/command（来自WebSocket桥接）
- 根据配置将命令路由到不同驱动节点的私有话题
- 聚合各驱动节点的状态反馈并发布到全局话题 /servo/state

使用场景:
- WebSocket -> /servo/command -> 路由节点 -> /bus_servo_driver/command -> 总线舵机驱动
- WebSocket -> /servo/command -> 路由节点 -> /pca_servo_driver/command -> PCA舵机驱动
- 驱动节点 -> /xxx_driver/state -> 路由节点 -> /servo/state -> WebSocket
"""

import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState


class ServoCommandRouter(Node):
    """舵机命令路由节点

    将全局舵机命令路由到各个驱动节点的私有命名空间
    """

    def __init__(self):
        super().__init__('servo_command_router')

        # 声明参数
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        # 订阅全局舵机命令
        self.global_command_sub = self.create_subscription(
            ServoCommand,
            '/servo/command',
            self.command_callback,
            10
        )

        # 发布到各驱动节点的私有话题
        self.bus_command_pub = self.create_publisher(
            ServoCommand,
            '/bus_servo_driver/command',
            10
        )

        self.pca_command_pub = self.create_publisher(
            ServoCommand,
            '/pca_servo_driver/command',
            10
        )

        # 订阅各驱动节点的状态反馈
        self.bus_state_sub = self.create_subscription(
            ServoState,
            '/bus_servo_driver/state',
            self.state_callback,
            10
        )

        self.pca_state_sub = self.create_subscription(
            ServoState,
            '/pca_servo_driver/state',
            self.state_callback,
            10
        )

        # 发布全局状态反馈
        self.global_state_pub = self.create_publisher(
            ServoState,
            '/servo/state',
            10
        )

        self.get_logger().info('舵机命令路由节点已启动')

    def command_callback(self, msg: ServoCommand):
        """处理全局舵机命令并路由到对应驱动节点"""
        try:
            if self.debug:
                self.get_logger().info(
                    f'收到命令: type={msg.servo_type}, id={msg.servo_id}, '
                    f'pos={msg.position}, speed={msg.speed}'
                )

            # 根据舵机类型路由到对应驱动节点
            if msg.servo_type == 'bus':
                self.bus_command_pub.publish(msg)
                if self.debug:
                    self.get_logger().info(f'路由到总线舵机驱动: ID={msg.servo_id}')
            elif msg.servo_type == 'pca':
                self.pca_command_pub.publish(msg)
                if self.debug:
                    self.get_logger().info(f'路由到PCA舵机驱动: ID={msg.servo_id}')
            else:
                self.get_logger().warn(
                    f'未知的舵机类型: {msg.servo_type}, 忽略命令'
                )

        except Exception as e:
            self.get_logger().error(f'命令路由失败: {e}')

    def state_callback(self, msg: ServoState):
        """聚合各驱动节点的状态反馈并发布到全局话题"""
        try:
            # 直接转发状态消息到全局话题
            self.global_state_pub.publish(msg)

            if self.debug:
                self.get_logger().info(
                    f'转发状态: type={msg.servo_type}, id={msg.servo_id}, '
                    f'pos={msg.position}, error={msg.error_code}'
                )

        except Exception as e:
            self.get_logger().error(f'状态转发失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandRouter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
