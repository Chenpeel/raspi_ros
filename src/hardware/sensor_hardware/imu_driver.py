"""
IMU 传感器驱动节点 - ROS 2版本

基于 YbImu 传感器，使用 smbus2 进行 I2C 通信
支持加速度计、陀螺仪、磁力计、四元数、欧拉角、气压计数据
"""

import struct
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import ImuData
from smbus2 import SMBus


class ImuDriver(Node):
    """IMU 传感器驱动节点

    功能:
    - 读取 YbImu 传感器数据（I2C 地址 0x23）
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

    # YbImu 寄存器地址（功能码）
    FUNC_VERSION = 0x01
    FUNC_RAW_ACCEL = 0x04
    FUNC_RAW_GYRO = 0x0A
    FUNC_RAW_MAG = 0x10
    FUNC_QUAT = 0x16
    FUNC_EULER = 0x26
    FUNC_BARO = 0x32
    FUNC_ALGO_TYPE = 0x61
    FUNC_CALIB_IMU = 0x70
    FUNC_CALIB_MAG = 0x71

    def __init__(self):
        super().__init__('imu_driver')

        # 声明参数
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x23)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('algo_type', 9)  # 6轴或9轴融合算法
        self.declare_parameter('calibrate_on_start', False)
        self.declare_parameter('sensor_id', 0)

        # 获取参数
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        self.algo_type = self.get_parameter('algo_type').value
        self.calibrate_on_start = self.get_parameter('calibrate_on_start').value
        self.sensor_id = self.get_parameter('sensor_id').value

        # I2C 总线对象（每次读写时创建上下文管理器）
        self.bus: Optional[SMBus] = None

        # 初始化传感器
        if not self._init_imu():
            self.get_logger().error(f'IMU 传感器初始化失败: I2C {self.i2c_bus}:{hex(self.i2c_address)}')
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
            f'IMU 传感器驱动已启动: I2C {self.i2c_bus}:{hex(self.i2c_address)} @ {self.publish_rate}Hz'
        )

    def _init_imu(self) -> bool:
        """初始化 IMU 传感器"""
        try:
            # 测试 I2C 通信
            version = self._get_version()
            if version:
                self.get_logger().info(f'IMU 固件版本: {version}')
            else:
                self.get_logger().warn('无法读取固件版本（I2C 模式下可能正常）')

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

    def _read_i2c(self, reg: int, num_bytes: int) -> Optional[list]:
        """读取 I2C 数据

        Args:
            reg: 寄存器地址
            num_bytes: 读取字节数

        Returns:
            数据列表，失败返回 None
        """
        if num_bytes > 32:
            num_bytes = 32
        try:
            with SMBus(self.i2c_bus) as bus:
                data = bus.read_i2c_block_data(self.i2c_address, reg, num_bytes)
            return data
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'I2C 读取失败 (reg={hex(reg)}): {e}')
            return None

    def _write_i2c(self, reg: int, value: int) -> bool:
        """写入 I2C 数据

        Args:
            reg: 寄存器地址
            value: 写入值

        Returns:
            是否成功
        """
        try:
            with SMBus(self.i2c_bus) as bus:
                bus.write_byte_data(self.i2c_address, reg, value)
            return True
        except Exception as e:
            if self.debug:
                self.get_logger().error(f'I2C 写入失败 (reg={hex(reg)}): {e}')
            return False

    def _get_version(self) -> Optional[str]:
        """获取固件版本号"""
        values = self._read_i2c(self.FUNC_VERSION, 3)
        if values:
            return f"{values[0]}.{values[1]}.{values[2]}"
        return None

    def _set_algo_type(self, algo: int):
        """设置融合算法类型

        Args:
            algo: 6 (六轴) 或 9 (九轴)
        """
        if algo not in [6, 9]:
            return
        self._write_i2c(self.FUNC_ALGO_TYPE, algo)
        time.sleep(1.0)  # 等待设置生效

    def _calibration_imu(self):
        """校准 IMU（陀螺仪和加速度计）"""
        self._write_i2c(self.FUNC_CALIB_IMU, 0x01)
        time.sleep(0.001)
        # 等待校准完成（最多7秒）
        for _ in range(70):
            time.sleep(0.1)
            values = self._read_i2c(self.FUNC_CALIB_IMU, 1)
            if values and values[0]:
                return

    def get_accelerometer_data(self) -> tuple:
        """获取加速度计三轴数据

        Returns:
            (ax, ay, az) 单位: g
        """
        values = self._read_i2c(self.FUNC_RAW_ACCEL, 6)
        if not values:
            return (0.0, 0.0, 0.0)

        accel_ratio = 16.0 / 32767.0
        ax = struct.unpack('h', bytearray(values[0:2]))[0] * accel_ratio
        ay = struct.unpack('h', bytearray(values[2:4]))[0] * accel_ratio
        az = struct.unpack('h', bytearray(values[4:6]))[0] * accel_ratio
        return (ax, ay, az)

    def get_gyroscope_data(self) -> tuple:
        """获取陀螺仪三轴数据

        Returns:
            (gx, gy, gz) 单位: rad/s
        """
        values = self._read_i2c(self.FUNC_RAW_GYRO, 6)
        if not values:
            return (0.0, 0.0, 0.0)

        gyro_ratio = (2000.0 / 32767.0) * (math.pi / 180.0)
        gx = struct.unpack('h', bytearray(values[0:2]))[0] * gyro_ratio
        gy = struct.unpack('h', bytearray(values[2:4]))[0] * gyro_ratio
        gz = struct.unpack('h', bytearray(values[4:6]))[0] * gyro_ratio
        return (gx, gy, gz)

    def get_magnetometer_data(self) -> tuple:
        """获取磁力计三轴数据

        Returns:
            (mx, my, mz) 单位: µT
        """
        values = self._read_i2c(self.FUNC_RAW_MAG, 6)
        if not values:
            return (0.0, 0.0, 0.0)

        mag_ratio = 800.0 / 32767.0
        mx = struct.unpack('h', bytearray(values[0:2]))[0] * mag_ratio
        my = struct.unpack('h', bytearray(values[2:4]))[0] * mag_ratio
        mz = struct.unpack('h', bytearray(values[4:6]))[0] * mag_ratio
        return (mx, my, mz)

    def get_quaternion_data(self) -> tuple:
        """获取四元数

        Returns:
            (qw, qx, qy, qz) 归一化四元数
        """
        values = self._read_i2c(self.FUNC_QUAT, 16)
        if not values:
            return (1.0, 0.0, 0.0, 0.0)

        qw = struct.unpack('f', bytearray(values[0:4]))[0]
        qx = struct.unpack('f', bytearray(values[4:8]))[0]
        qy = struct.unpack('f', bytearray(values[8:12]))[0]
        qz = struct.unpack('f', bytearray(values[12:16]))[0]
        return (qw, qx, qy, qz)

    def get_euler_data(self) -> tuple:
        """获取欧拉角

        Returns:
            (roll, pitch, yaw) 单位: deg
        """
        values = self._read_i2c(self.FUNC_EULER, 12)
        if not values:
            return (0.0, 0.0, 0.0)

        roll = struct.unpack('f', bytearray(values[0:4]))[0]
        pitch = struct.unpack('f', bytearray(values[4:8]))[0]
        yaw = struct.unpack('f', bytearray(values[8:12]))[0]

        # 转换为角度
        roll = roll * (180.0 / math.pi)
        pitch = pitch * (180.0 / math.pi)
        yaw = yaw * (180.0 / math.pi)

        return (roll, pitch, yaw)

    def get_baro_data(self) -> tuple:
        """获取气压计数据

        Returns:
            (height, temp, pressure) 单位: m, °C, Pa
        """
        values = self._read_i2c(self.FUNC_BARO, 16)
        if not values:
            return (0.0, 0.0, 0.0)

        height = struct.unpack('f', bytearray(values[0:4]))[0]
        temp = struct.unpack('f', bytearray(values[4:8]))[0]
        pressure = struct.unpack('f', bytearray(values[8:12]))[0]

        return (height, temp, pressure)

    def timer_callback(self):
        """定时器回调：读取传感器数据并发布"""
        try:
            # 读取所有传感器数据
            ax, ay, az = self.get_accelerometer_data()
            gx, gy, gz = self.get_gyroscope_data()
            mx, my, mz = self.get_magnetometer_data()
            qw, qx, qy, qz = self.get_quaternion_data()
            roll, pitch, yaw = self.get_euler_data()
            height, temp, pressure = self.get_baro_data()

            # 构造消息
            msg = ImuData()
            msg.sensor_type = 'imu'
            msg.sensor_id = self.sensor_id

            # 加速度计
            msg.accel_x = ax
            msg.accel_y = ay
            msg.accel_z = az

            # 陀螺仪
            msg.gyro_x = gx
            msg.gyro_y = gy
            msg.gyro_z = gz

            # 磁力计
            msg.mag_x = mx
            msg.mag_y = my
            msg.mag_z = mz

            # 四元数
            msg.quat_w = qw
            msg.quat_x = qx
            msg.quat_y = qy
            msg.quat_z = qz

            # 欧拉角
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw

            # 气压计
            msg.baro_height = height
            msg.baro_temp = temp
            msg.baro_pressure = pressure

            # 错误状态
            msg.error_code = 0  # 0 = 正常

            # 时间戳
            msg.stamp = self.get_clock().now().to_msg()

            # 发布消息
            self.data_pub.publish(msg)

            if self.debug:
                self.get_logger().debug(
                    f'[IMU] accel=({ax:.2f},{ay:.2f},{az:.2f}) '
                    f'gyro=({gx:.2f},{gy:.2f},{gz:.2f}) '
                    f'euler=({roll:.1f},{pitch:.1f},{yaw:.1f})'
                )

        except Exception as e:
            self.get_logger().error(f'读取 IMU 数据失败: {e}')

            # 发布错误消息
            error_msg = ImuData()
            error_msg.sensor_type = 'imu'
            error_msg.sensor_id = self.sensor_id
            error_msg.error_code = 1  # 1 = I2C错误
            error_msg.stamp = self.get_clock().now().to_msg()
            self.data_pub.publish(error_msg)

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.get_logger().info('IMU 传感器驱动已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
