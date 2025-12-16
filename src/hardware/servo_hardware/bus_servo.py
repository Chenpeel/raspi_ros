"""
总线舵机驱动节点 - ROS 2版本

基于原始的BusDriver实现,适配ROS 2架构
支持协议格式: #ID(3位)P位置(4位)T速度(4位)!
"""

import serial
import time
import threading
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState


class BusServoDriver(Node):
    """总线舵机驱动节点

    功能:
    - 订阅/servo/command话题接收舵机控制命令
    - 通过串口发送控制指令到总线舵机
    - 发布舵机状态反馈到/servo/state话题

    协议:
    - 帧格式: #ID(3位)P位置(4位)T速度(4位)!
    - 示例: #001P1500T0100!
    - 位置: 角度0-180° 或 脉宽500-2500us
    - 速度: 运动时间(毫秒)
    """

    def __init__(self):
        super().__init__('bus_servo_driver')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('debug', False)
        self.declare_parameter('log_id', True)  # 是否打印舵机ID日志
        self.declare_parameter('servo_ids', [])  # 本驱动板负责的舵机ID列表

        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.default_speed = self.get_parameter('default_speed').value
        self.debug = self.get_parameter('debug').value
        self.log_id = self.get_parameter('log_id').value
        servo_ids_param = self.get_parameter('servo_ids').value

        # 转换为set以提高查找效率
        self.servo_ids = set(servo_ids_param) if servo_ids_param else set()

        # 如果未配置servo_ids，则处理所有ID（向后兼容）
        self.filter_enabled = len(self.servo_ids) > 0

        # 串口对象
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.recv_buffer = bytearray()
        self.running = True

        # 记录已控制的舵机ID
        self.connected_servos: Set[int] = set()

        # 初始化串口
        if not self._init_serial():
            self.get_logger().error(f'串口初始化失败: {self.port}')

        # 启动接收线程
        self.recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        # 订阅舵机命令
        self.command_sub = self.create_subscription(
            ServoCommand,
            '~/command',  # 使用私有命名空间
            self.command_callback,
            10
        )

        # 发布舵机状态
        self.state_pub = self.create_publisher(
            ServoState,
            '~/state',  # 使用私有命名空间
            10
        )

        if self.filter_enabled:
            ids_str = ', '.join(map(str, sorted(self.servo_ids)))
            self.get_logger().info(
                f'总线舵机驱动已启动: {self.port}@{self.baudrate} '
                f'(负责舵机ID: {ids_str})'
            )
        else:
            self.get_logger().info(
                f'总线舵机驱动已启动: {self.port}@{self.baudrate} '
                f'(处理所有舵机ID)'
            )

    def _init_serial(self) -> bool:
        """初始化/打开串口"""
        try:
            if self.ser is None:
                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout
                )
            if not self.ser.is_open:
                self.ser.open()
            return True
        except Exception as e:
            self.get_logger().error(f'初始化串口失败: {e}')
            return False

    def _recv_loop(self):
        """后台接收线程: 非阻塞读取串口并缓存数据"""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        with self.lock:
                            self.recv_buffer.extend(data)

                        # 尝试解析返回数据
                        try:
                            text = data.decode(
                                'utf-8', errors='ignore').strip()
                            if text and (self.debug or self.log_id):
                                # 尝试提取舵机ID
                                sid = None
                                try:
                                    if text.startswith('#') and 'P' in text:
                                        sid = int(text[1:4])
                                except Exception:
                                    sid = None

                                if sid is not None:
                                    self.get_logger().info(
                                        f'[Recv] ID={sid} Raw={text}')
                                else:
                                    self.get_logger().info(
                                        f'[Recv] Raw={text}')
                        except Exception:
                            pass
                else:
                    time.sleep(0.05)
            except Exception as e:
                if self.debug:
                    self.get_logger().error(f'接收线程异常: {e}')
                time.sleep(0.2)

    def _format_frame(self, servo_id: int, pulse: int, speed: int) -> bytes:
        """格式化控制帧: #ID(3位)P位置(4位)T速度(4位)!

        Args:
            servo_id: 舵机ID (0-999)
            pulse: 脉宽值 (0-9999)
            speed: 速度/时间 (0-9999)

        Returns:
            bytes: 格式化的控制帧
        """
        # 限制范围
        servo_id = max(0, min(999, int(servo_id)))
        pulse = max(0, min(9999, int(pulse)))
        speed = max(0, min(9999, int(speed)))

        frame_str = f"#{servo_id:03d}P{pulse:04d}T{speed:04d}!"
        frame = frame_str.encode('utf-8')

        if self.debug:
            self.get_logger().debug(f'格式化帧: {frame_str}')
        if self.log_id:
            self.get_logger().info(
                f'[Send] ID={servo_id} PULSE={pulse} T={speed}ms Frame={frame_str}')

        return frame

    def send_position(self, servo_id: int, position: int, speed: Optional[int] = None) -> bool:
        """发送舵机位置指令

        Args:
            servo_id: 舵机ID
            position: 位置值
                - 0-180: 视为角度(转换为脉宽500-2500us)
                - 500-2500: 视为脉宽(直接发送)
            speed: 运动速度/时间, 默认使用default_speed

        Returns:
            bool: 发送是否成功
        """
        if speed is None:
            speed = self.default_speed

        try:
            sid = int(servo_id)
        except Exception:
            self.get_logger().error(f'非法舵机ID: {servo_id}')
            return False

        try:
            pos = int(position)
        except Exception:
            self.get_logger().error(f'非法位置参数: {position}')
            return False

        # 判断position是角度还是脉宽
        if 0 <= pos <= 180:
            # 角度 -> 脉宽 (500-2500us)
            pulse = int(500 + (pos / 180.0) * (2500 - 500))
        elif 500 <= pos <= 2500:
            pulse = pos
        else:
            # 超出范围,尝试直接发送并警告
            pulse = pos
            self.get_logger().warn(
                f'位置值 {pos} 非常规(既不是角度也不是脉宽),将尝试直接发送'
            )

        frame = self._format_frame(sid, pulse, speed)

        if not self._init_serial():
            self.get_logger().error('无法打开串口,发送失败')
            return False

        with self.lock:
            try:
                self.ser.write(frame)
                self.get_logger().debug(
                    f'发送成功: Port={self.port} Bytes={len(frame)} Frame={frame.decode("utf-8", errors="ignore")}'
                )
            except Exception as e:
                self.get_logger().error(f'向总线舵机写入失败: {e}')
                return False

        # 记录已控制的舵机
        self.connected_servos.add(sid)

        # 给设备一点处理时间
        time.sleep(0.005)
        return True

    def command_callback(self, msg: ServoCommand):
        """处理舵机控制命令（使用ServoCommand消息）"""
        try:
            # 仅处理总线舵机命令
            if msg.servo_type != 'bus':
                if self.debug:
                    self.get_logger().warn(
                        f'忽略非总线舵机命令: servo_type={msg.servo_type}')
                return

            # 提取命令参数
            servo_id = msg.servo_id
            position = msg.position
            speed = msg.speed if msg.speed > 0 else self.default_speed

            # ID列表过滤：仅处理本节点负责的舵机ID
            if self.filter_enabled and servo_id not in self.servo_ids:
                if self.debug:
                    self.get_logger().debug(
                        f'忽略ID={servo_id}的命令（不在本驱动板的舵机列表中）'
                    )
                return

            # 发送舵机命令
            success = self.send_position(servo_id, position, speed)

            if success:
                # 发布状态反馈
                state_msg = ServoState()
                state_msg.servo_type = "bus"
                state_msg.servo_id = servo_id
                state_msg.position = position
                state_msg.load = -1  # 总线舵机不支持负载反馈
                state_msg.temperature = -1  # 总线舵机不支持温度反馈
                state_msg.error_code = 0  # 0表示正常
                state_msg.stamp = self.get_clock().now().to_msg()

                self.state_pub.publish(state_msg)
            else:
                # 发布错误状态
                error_msg = ServoState()
                error_msg.servo_type = "bus"
                error_msg.servo_id = servo_id
                error_msg.position = position
                error_msg.load = -1
                error_msg.temperature = -1
                error_msg.error_code = 1  # 1表示命令执行失败
                error_msg.stamp = self.get_clock().now().to_msg()

                self.state_pub.publish(error_msg)

        except Exception as e:
            self.get_logger().error(f'命令处理失败: {e}')

    def reset_servos(self):
        """复位所有已控制的舵机到中间位置(90度/1500us)"""
        if not self.connected_servos:
            return

        self.get_logger().info('开始复位总线舵机到中间位置...')
        for sid in list(self.connected_servos):
            try:
                self.send_position(sid, 1500, speed=self.default_speed)
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f'复位舵机 {sid} 失败: {e}')
        self.get_logger().info('总线舵机复位完成')

    def close(self):
        """停止接收线程并关闭串口"""
        self.running = False

        # 复位舵机
        self.reset_servos()

        # 等待线程退出
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=0.5)

        # 关闭串口
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info('串口已关闭')
        except Exception as e:
            self.get_logger().error(f'关闭串口异常: {e}')

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BusServoDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
