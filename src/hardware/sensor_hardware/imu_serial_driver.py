"""
IMU 传感器串口驱动节点 - ROS 2版本

基于 YbImu 传感器，使用串口进行通信（替代 I2C）
支持加速度计、陀螺仪、磁力计、四元数、欧拉角、气压计数据
"""

import struct
import math
import time
import serial
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import ImuData


class ImuSerialDriver(Node):
    """IMU 传感器串口驱动节点

    功能:
    - 读取 YbImu 传感器数据（串口通信）
    - 发布传感器数据到 /sensor/imu 话题
    - 支持数据类型：加速度、陀螺仪、磁力计、四元数、欧拉角、气压

    数据单位:
    - 加速度: g (重力加速度)
    - 陀螺仪: rad/s (弧度/秒)
    - 磁力计: µT (微特斯拉)
    - 欧拉角: deg (角度)
    - 四元数: 归一化 [w, x, y, z]
    - 气压: m (高度), °C (温度), Pa (压力)
    """

    # YbImu 协议帧头
    HEAD1 = 0x7E
    HEAD2 = 0x23

    # 功能码定义
    FUNC_VERSION = 0x01
    FUNC_REPORT_IMU_RAW = 0x04
    FUNC_REPORT_IMU_QUAT = 0x16
    FUNC_REPORT_IMU_EULER = 0x26
    FUNC_REPORT_BARO = 0x32
    FUNC_REPORT_RATE = 0x60
    FUNC_ALGO_TYPE = 0x61
    FUNC_CALIB_IMU = 0x70
    FUNC_REQUEST_DATA = 0x80

    def __init__(self):
        super().__init__('imu_serial_driver')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyAMA4')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('algo_type', 9)  # 6轴或9轴融合算法
        self.declare_parameter('calibrate_on_start', False)
        self.declare_parameter('sensor_id', 0)

        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.algo_type = self.get_parameter('algo_type').value
        self.calibrate_on_start = self.get_parameter('calibrate_on_start').value
        self.sensor_id = self.get_parameter('sensor_id').value

        # 串口对象
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

        # 传感器数据缓存
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0
        self._gx = 0.0
        self._gy = 0.0
        self._gz = 0.0
        self._mx = 0.0
        self._my = 0.0
        self._mz = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._q0 = 1.0
        self._q1 = 0.0
        self._q2 = 0.0
        self._q3 = 0.0
        self._height = 0.0
        self._temperature = 0.0
        self._pressure = 0.0

        # 接收状态
        self._rx_state = 0
        self._rx_func = 0
        self._rx_data = []
        self._data_len = 0
        self._data_func = 0

        # 初始化串口
        if not self._init_serial():
            self.get_logger().error(f'串口初始化失败: {self.port}')
            return

        # 启动后台接收线程
        self._start_receive_thread()

        # 初始化传感器配置
        if not self._init_imu():
            self.get_logger().error('IMU 传感器初始化失败')
            return

        # 创建发布器
        self.data_pub = self.create_publisher(
            ImuData,
            '~/data',  # 使用私有命名空间
            10
        )

        # 创建定时器（根据发布频率）
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'IMU 串口驱动已启动: {self.port} @ {self.baudrate} bps, 发布频率 {self.publish_rate}Hz'
        )

    def _init_serial(self) -> bool:
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            time.sleep(0.1)  # 等待串口稳定
            self.ser.flushInput()  # 清空输入缓冲区
            self.get_logger().info(f'串口已打开: {self.port} @ {self.baudrate}bps')
            return True
        except Exception as e:
            self.get_logger().error(f'打开串口失败: {e}')
            return False

    def _init_imu(self) -> bool:
        """初始化 IMU 传感器配置"""
        try:
            time.sleep(0.5)  # 等待传感器准备就绪

            # 设置融合算法类型
            if self.algo_type in [6, 9]:
                self._set_algo_type(self.algo_type)
                self.get_logger().info(f'融合算法设置为: {self.algo_type}轴')

            # 启动时校准（如果需要）
            if self.calibrate_on_start:
                self.get_logger().info('开始 IMU 校准，请保持设备静止...')
                self._calibration_imu()
                self.get_logger().info('IMU 校准完成')

            return True

        except Exception as e:
            self.get_logger().error(f'初始化 IMU 失败: {e}')
            return False

    def _send_command(self, function: int, param: int = 0) -> bool:
        """发送命令到 IMU

        Args:
            function: 功能码
            param: 参数

        Returns:
            是否成功
        """
        try:
            cmd = [self.HEAD1, self.HEAD2, 0x00, self.FUNC_REQUEST_DATA,
                   function & 0xFF, param & 0xFF]
            cmd[2] = len(cmd) + 1
            checksum = sum(cmd) & 0xFF
            cmd.append(checksum)

            with self.lock:
                self.ser.write(bytes(cmd))

            if self.debug:
                self.get_logger().debug(f'[发送命令] 功能={hex(function)} 参数={param}')
            return True

        except Exception as e:
            self.get_logger().error(f'发送命令失败: {e}')
            return False

    def _set_algo_type(self, algo: int):
        """设置融合算法类型

        Args:
            algo: 6 (六轴) 或 9 (九轴)
        """
        if algo not in [6, 9]:
            return

        try:
            cmd = [self.HEAD1, self.HEAD2, 0x00, self.FUNC_ALGO_TYPE, algo, 0x5F]
            cmd[2] = len(cmd) + 1
            checksum = sum(cmd) & 0xFF
            cmd.append(checksum)

            with self.lock:
                self.ser.write(bytes(cmd))

            time.sleep(1.0)  # 等待设置生效
            self.get_logger().info(f'算法类型设置完成: {algo}轴')

        except Exception as e:
            self.get_logger().error(f'设置算法类型失败: {e}')

    def _calibration_imu(self):
        """校准 IMU（陀螺仪和加速度计）"""
        try:
            cmd = [self.HEAD1, self.HEAD2, 0x00, self.FUNC_CALIB_IMU, 0x01, 0x5F]
            cmd[2] = len(cmd) + 1
            checksum = sum(cmd) & 0xFF
            cmd.append(checksum)

            with self.lock:
                self.ser.write(bytes(cmd))

            time.sleep(7.0)  # 等待校准完成（约7秒）
            self.get_logger().info('IMU 校准完成')

        except Exception as e:
            self.get_logger().error(f'IMU 校准失败: {e}')

    def _receive_data(self, byte: int):
        """接收并解析串口数据

        Args:
            byte: 接收到的字节
        """
        # 状态机解析协议帧
        if self._rx_state == 0:  # 等待帧头1
            if byte == self.HEAD1:
                self._rx_state = 1
                self._rx_data = [byte]

        elif self._rx_state == 1:  # 等待帧头2
            if byte == self.HEAD2:
                self._rx_state = 2
                self._rx_data.append(byte)
            else:
                self._rx_state = 0

        elif self._rx_state == 2:  # 接收数据长度
            self._data_len = byte
            self._rx_data.append(byte)
            self._rx_state = 3

        elif self._rx_state == 3:  # 接收功能码
            self._data_func = byte
            self._rx_data.append(byte)
            self._rx_state = 4

        elif self._rx_state == 4:  # 接收数据和校验和
            self._rx_data.append(byte)
            if len(self._rx_data) >= self._data_len:
                self._rx_state = 0
                self._parse_frame()

    def _parse_frame(self):
        """解析完整数据帧"""
        try:
            # 校验和验证
            checksum = sum(self._rx_data[:-1]) & 0xFF
            if checksum != self._rx_data[-1]:
                if self.debug:
                    self.get_logger().warn('校验和错误')
                return

            func = self._data_func
            data = self._rx_data[4:-1]  # 提取有效数据部分

            # 根据功能码解析数据
            if func == self.FUNC_REPORT_IMU_RAW and len(data) >= 18:
                # 原始 IMU 数据（加速度、陀螺仪、磁力计）
                self._parse_raw_imu_data(data)

            elif func == self.FUNC_REPORT_IMU_QUAT and len(data) >= 16:
                # 四元数数据
                self._parse_quaternion_data(data)

            elif func == self.FUNC_REPORT_IMU_EULER and len(data) >= 12:
                # 欧拉角数据
                self._parse_euler_data(data)

            elif func == self.FUNC_REPORT_BARO and len(data) >= 16:
                # 气压计数据
                self._parse_baro_data(data)

        except Exception as e:
            if self.debug:
                self.get_logger().error(f'解析数据帧失败: {e}')

    def _parse_raw_imu_data(self, data: list):
        """解析原始 IMU 数据（加速度、陀螺仪、磁力计）"""
        try:
            # 加速度计 (int16, g)
            accel_ratio = 16.0 / 32767.0
            self._ax = struct.unpack('h', bytes(data[0:2]))[0] * accel_ratio
            self._ay = struct.unpack('h', bytes(data[2:4]))[0] * accel_ratio
            self._az = struct.unpack('h', bytes(data[4:6]))[0] * accel_ratio

            # 陀螺仪 (int16, rad/s)
            gyro_ratio = (2000.0 / 32767.0) * (math.pi / 180.0)
            self._gx = struct.unpack('h', bytes(data[6:8]))[0] * gyro_ratio
            self._gy = struct.unpack('h', bytes(data[8:10]))[0] * gyro_ratio
            self._gz = struct.unpack('h', bytes(data[10:12]))[0] * gyro_ratio

            # 磁力计 (int16, µT)
            mag_ratio = 800.0 / 32767.0
            self._mx = struct.unpack('h', bytes(data[12:14]))[0] * mag_ratio
            self._my = struct.unpack('h', bytes(data[14:16]))[0] * mag_ratio
            self._mz = struct.unpack('h', bytes(data[16:18]))[0] * mag_ratio

        except Exception as e:
            if self.debug:
                self.get_logger().error(f'解析原始 IMU 数据失败: {e}')

    def _parse_quaternion_data(self, data: list):
        """解析四元数数据"""
        try:
            self._q0 = struct.unpack('f', bytes(data[0:4]))[0]
            self._q1 = struct.unpack('f', bytes(data[4:8]))[0]
            self._q2 = struct.unpack('f', bytes(data[8:12]))[0]
            self._q3 = struct.unpack('f', bytes(data[12:16]))[0]
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'解析四元数数据失败: {e}')

    def _parse_euler_data(self, data: list):
        """解析欧拉角数据"""
        try:
            # 数据为弧度，转换为角度
            roll_rad = struct.unpack('f', bytes(data[0:4]))[0]
            pitch_rad = struct.unpack('f', bytes(data[4:8]))[0]
            yaw_rad = struct.unpack('f', bytes(data[8:12]))[0]

            self._roll = roll_rad * (180.0 / math.pi)
            self._pitch = pitch_rad * (180.0 / math.pi)
            self._yaw = yaw_rad * (180.0 / math.pi)
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'解析欧拉角数据失败: {e}')

    def _parse_baro_data(self, data: list):
        """解析气压计数据"""
        try:
            self._height = struct.unpack('f', bytes(data[0:4]))[0]
            self._temperature = struct.unpack('f', bytes(data[4:8]))[0]
            self._pressure = struct.unpack('f', bytes(data[8:12]))[0]
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'解析气压计数据失败: {e}')

    def _data_handle(self):
        """后台线程：持续读取串口数据"""
        self.get_logger().info('串口接收线程已启动')
        self.ser.flushInput()  # 清空缓冲区

        while rclpy.ok():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data_bytes = self.ser.read(self.ser.in_waiting)
                    for byte in data_bytes:
                        self._receive_data(byte)
                else:
                    time.sleep(0.001)  # 避免 CPU 占用过高
            except Exception as e:
                self.get_logger().error(f'串口读取异常: {e}')
                time.sleep(0.1)

    def _start_receive_thread(self):
        """启动后台接收线程"""
        try:
            receive_thread = threading.Thread(
                target=self._data_handle,
                daemon=True,
                name='imu_serial_receive'
            )
            receive_thread.start()
            time.sleep(0.1)  # 等待线程启动
            self.get_logger().info('后台接收线程已创建')
        except Exception as e:
            self.get_logger().error(f'创建接收线程失败: {e}')

    def timer_callback(self):
        """定时器回调：发布传感器数据"""
        try:
            # 构造消息
            msg = ImuData()
            msg.sensor_type = 'imu'
            msg.sensor_id = self.sensor_id

            # 加速度计
            msg.accel_x = self._ax
            msg.accel_y = self._ay
            msg.accel_z = self._az

            # 陀螺仪
            msg.gyro_x = self._gx
            msg.gyro_y = self._gy
            msg.gyro_z = self._gz

            # 磁力计
            msg.mag_x = self._mx
            msg.mag_y = self._my
            msg.mag_z = self._mz

            # 四元数
            msg.quat_w = self._q0
            msg.quat_x = self._q1
            msg.quat_y = self._q2
            msg.quat_z = self._q3

            # 欧拉角
            msg.roll = self._roll
            msg.pitch = self._pitch
            msg.yaw = self._yaw

            # 气压计
            msg.baro_height = self._height
            msg.baro_temp = self._temperature
            msg.baro_pressure = self._pressure

            # 错误状态
            msg.error_code = 0  # 0 = 正常

            # 时间戳
            msg.stamp = self.get_clock().now().to_msg()

            # 发布消息
            self.data_pub.publish(msg)

            if self.debug:
                self.get_logger().debug(
                    f'[IMU] accel=({self._ax:.2f},{self._ay:.2f},{self._az:.2f}) '
                    f'gyro=({self._gx:.2f},{self._gy:.2f},{self._gz:.2f}) '
                    f'euler=({self._roll:.1f},{self._pitch:.1f},{self._yaw:.1f})'
                )

        except Exception as e:
            self.get_logger().error(f'发布 IMU 数据失败: {e}')

    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')
        self.get_logger().info('IMU 串口驱动已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
