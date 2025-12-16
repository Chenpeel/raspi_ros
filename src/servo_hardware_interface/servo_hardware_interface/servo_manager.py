"""
ROS2舵机管理器节点

类似ros2_control的ControllerManager，统一管理所有舵机硬件
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from servo_msgs.msg import ServoCommand, ServoState, ServoStatus

from servo_hardware_interface import ServoResourceManager
from servo_drivers import BusServoPlugin, PCAServoPlugin


class ServoManager(Node):
    """舵机管理器节点

    功能：
    - 加载和管理硬件插件（BusServo、PCA9685等）
    - 订阅/servo/command话题接收控制命令
    - 发布/servo/state话题反馈舵机状态
    - 发布/servo/status话题提供系统整体状态
    - 周期性更新循环（100Hz）：调用所有硬件的read()/write()

    设计理念：
    - 参考ros2_control的ControllerManager
    - 插件化架构，易于扩展新硬件
    - 统一命令路由和状态聚合
    """

    def __init__(self):
        """初始化舵机管理器节点"""
        super().__init__('servo_manager')

        # 声明参数
        self.declare_parameter('update_rate', 100.0)  # 更新频率(Hz)
        self.declare_parameter('device_id', 'robot')  # 设备ID

        # Bus舵机参数
        self.declare_parameter('bus_servo.enabled', True)
        self.declare_parameter('bus_servo.port', '/dev/ttyAMA0')
        self.declare_parameter('bus_servo.baudrate', 115200)
        self.declare_parameter('bus_servo.timeout', 0.1)
        self.declare_parameter('bus_servo.default_speed', 100)
        self.declare_parameter('bus_servo.debug', False)

        # PCA舵机参数
        self.declare_parameter('pca_servo.enabled', True)
        self.declare_parameter('pca_servo.i2c_address', 0x40)
        self.declare_parameter('pca_servo.bus_number', 1)
        self.declare_parameter('pca_servo.frequency', 50)
        self.declare_parameter('pca_servo.min_pwm', 110)
        self.declare_parameter('pca_servo.max_pwm', 520)
        self.declare_parameter('pca_servo.debug', False)

        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.device_id = self.get_parameter('device_id').value

        # 创建资源管理器
        self.resource_manager = ServoResourceManager()

        # 命令缓冲
        self.pending_commands = {}

        # 加载硬件插件
        self._load_hardware_plugins()

        # ROS话题
        self.command_sub = self.create_subscription(
            ServoCommand,
            '/servo/command',
            self.command_callback,
            10
        )

        self.state_pub = self.create_publisher(
            ServoState,
            '/servo/state',
            10
        )

        self.status_pub = self.create_publisher(
            ServoStatus,
            '/servo/status',
            10
        )

        # 周期性更新循环
        update_period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(
            update_period,
            self.update_loop
        )

        self.get_logger().info(
            f'舵机管理器已启动: 更新频率={self.update_rate}Hz, 设备ID={self.device_id}'
        )

    def _load_hardware_plugins(self):
        """加载硬件插件"""

        # 加载Bus舵机插件
        if self.get_parameter('bus_servo.enabled').value:
            bus_plugin = BusServoPlugin("bus_servo")
            self.resource_manager.register_hardware(bus_plugin)

            bus_config = {
                'port': self.get_parameter('bus_servo.port').value,
                'baudrate': self.get_parameter('bus_servo.baudrate').value,
                'timeout': self.get_parameter('bus_servo.timeout').value,
                'default_speed': self.get_parameter('bus_servo.default_speed').value,
                'debug': self.get_parameter('bus_servo.debug').value,
            }

            if not bus_plugin.configure(bus_config):
                self.get_logger().error(f'Bus舵机配置失败: {bus_plugin.get_error()}')
            elif not bus_plugin.activate():
                self.get_logger().error(f'Bus舵机激活失败: {bus_plugin.get_error()}')
            else:
                self.get_logger().info('Bus舵机插件加载成功')

                # 映射servo_id到硬件（假设Bus舵机ID范围1-200）
                for servo_id in range(1, 201):
                    self.resource_manager.map_servo(servo_id, "bus_servo")

        # 加载PCA舵机插件
        if self.get_parameter('pca_servo.enabled').value:
            pca_plugin = PCAServoPlugin("pca_servo")
            self.resource_manager.register_hardware(pca_plugin)

            pca_config = {
                'i2c_address': self.get_parameter('pca_servo.i2c_address').value,
                'bus_number': self.get_parameter('pca_servo.bus_number').value,
                'frequency': self.get_parameter('pca_servo.frequency').value,
                'min_pwm': self.get_parameter('pca_servo.min_pwm').value,
                'max_pwm': self.get_parameter('pca_servo.max_pwm').value,
                'debug': self.get_parameter('pca_servo.debug').value,
            }

            if not pca_plugin.configure(pca_config):
                self.get_logger().error(f'PCA舵机配置失败: {pca_plugin.get_error()}')
            elif not pca_plugin.activate():
                self.get_logger().error(f'PCA舵机激活失败: {pca_plugin.get_error()}')
            else:
                self.get_logger().info('PCA舵机插件加载成功')

                # 映射servo_id到硬件（PCA通道0-15，使用不同ID范围避免冲突）
                # 这里使用1000+channel作为servo_id
                for channel in range(16):
                    self.resource_manager.map_servo(1000 + channel, "pca_servo")

    def command_callback(self, msg: ServoCommand):
        """处理舵机控制命令

        Args:
            msg: 舵机命令消息
        """
        # 缓存命令，在update_loop中执行
        self.pending_commands[msg.servo_id] = msg

    def update_loop(self):
        """周期性更新循环（100Hz）

        执行顺序：
        1. 写入命令到硬件
        2. 读取硬件状态
        3. 发布状态消息
        4. 发布系统状态
        """
        now = self.get_clock().now()
        period = 1.0 / self.update_rate

        # 1. 写入命令
        if self.pending_commands:
            self.resource_manager.write_all(now, period, self.pending_commands)
            self.pending_commands.clear()

        # 2. 读取状态
        states = self.resource_manager.read_all(now, period)

        # 3. 发布状态消息
        for state in states.values():
            state.stamp = now.to_msg()
            self.state_pub.publish(state)

        # 4. 每1秒发布一次系统状态
        if not hasattr(self, '_last_status_time'):
            self._last_status_time = now

        if (now - self._last_status_time).nanoseconds / 1e9 >= 1.0:
            self._publish_system_status(states)
            self._last_status_time = now

    def _publish_system_status(self, states: dict):
        """发布系统整体状态

        Args:
            states: 所有舵机的状态字典
        """
        status_msg = ServoStatus()
        status_msg.device_id = self.device_id
        status_msg.stamp = self.get_clock().now().to_msg()

        # 分类舵机状态
        bus_servos = []
        pca_servos = []

        for state in states.values():
            if state.servo_type == "bus":
                bus_servos.append(state)
            elif state.servo_type == "pca":
                pca_servos.append(state)

        status_msg.bus_servos = bus_servos
        status_msg.pca_servos = pca_servos

        # 系统状态判断
        hw_states = self.resource_manager.get_hardware_states()
        if all(state.name == "ACTIVE" for state in hw_states.values()):
            status_msg.system_state = "running"
        elif any(hw.get_error() for hw in self.resource_manager.hardware_interfaces.values()):
            status_msg.system_state = "error"
        else:
            status_msg.system_state = "idle"

        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.get_logger().info('正在关闭舵机管理器...')

        # 停用并清理所有硬件
        self.resource_manager.deactivate_all()
        self.resource_manager.cleanup_all()

        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = ServoManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
