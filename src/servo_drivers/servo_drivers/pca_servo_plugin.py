"""
PCA9685舵机硬件接口插件

将原pca_servo驱动封装为符合ServoHardwareInterface的插件
"""

import time
from typing import Dict, Set, Optional

from servo_hardware_interface import ServoHardwareInterface
from servo_msgs.msg import ServoCommand, ServoState

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False
    SMBus = None


class PCAServoPlugin(ServoHardwareInterface):
    """PCA9685舵机硬件接口插件

    协议：
    - I2C地址: 默认0x40
    - PWM频率: 50Hz(舵机标准)
    - PWM范围: 0-4095(12位)
    - 实际脉宽: 通常110-520对应0-180°
    """

    def __init__(self, name: str = "pca_servo"):
        """初始化PCA9685插件

        Args:
            name: 硬件接口名称
        """
        super().__init__(name)

        # I2C相关
        self.i2c_address: int = 0x40
        self.bus_number: int = 1
        self.bus: Optional[SMBus] = None

        # PWM参数
        self.frequency: int = 50
        self.min_pwm: int = 110
        self.max_pwm: int = 520
        self.min_us: int = 500
        self.max_us: int = 2500

        # 舵机状态缓存
        self.servo_states: Dict[int, ServoState] = {}
        self.controlled_channels: Set[int] = set()

        # 参数
        self.debug: bool = False

        # 检查依赖
        if not HAS_SMBUS:
            self.set_error("缺少smbus2库，请安装: pip3 install smbus2")

    # ==================== 生命周期接口实现 ====================

    def on_configure(self, config: Dict) -> bool:
        """配置阶段：加载参数并初始化I2C对象

        Args:
            config: 配置字典，支持的参数：
                - i2c_address: I2C地址
                - bus_number: I2C总线号
                - frequency: PWM频率
                - min_pwm: 最小PWM值
                - max_pwm: 最大PWM值
                - min_us: 最小脉宽(微秒)
                - max_us: 最大脉宽(微秒)
                - debug: 是否开启调试

        Returns:
            bool: 配置是否成功
        """
        if not HAS_SMBUS:
            self.set_error("smbus2库不可用")
            return False

        try:
            # 加载配置参数
            self.i2c_address = config.get('i2c_address', self.i2c_address)
            self.bus_number = config.get('bus_number', self.bus_number)
            self.frequency = config.get('frequency', self.frequency)
            self.min_pwm = config.get('min_pwm', self.min_pwm)
            self.max_pwm = config.get('max_pwm', self.max_pwm)
            self.min_us = config.get('min_us', self.min_us)
            self.max_us = config.get('max_us', self.max_us)
            self.debug = config.get('debug', self.debug)

            if self.debug:
                print(f"[PCAServoPlugin] 配置完成: 地址=0x{self.i2c_address:02X}, 频率={self.frequency}Hz")

            return True

        except Exception as e:
            self.set_error(f"配置失败: {e}")
            return False

    def on_activate(self) -> bool:
        """激活阶段：打开I2C总线并初始化PCA9685

        Returns:
            bool: 激活是否成功
        """
        try:
            # 打开I2C总线
            self.bus = SMBus(self.bus_number)

            # 验证设备连接
            try:
                self.bus.read_byte_data(self.i2c_address, 0x00)
                if self.debug:
                    print(f"[PCAServoPlugin] PCA9685设备检测成功(地址: 0x{self.i2c_address:02X})")
            except OSError as e:
                self.set_error(f"无法连接PCA9685(地址: 0x{self.i2c_address:02X}): {e}")
                return False

            # 软件复位
            self._software_reset()

            # 设置PWM频率
            self._set_pwm_freq(self.frequency)

            # 配置推挽输出(适配舵机驱动)
            self.bus.write_byte_data(self.i2c_address, 0x01, 0x04)

            # 启用寄存器地址自动递增
            try:
                mode1 = self.bus.read_byte_data(self.i2c_address, 0x00)
                self.bus.write_byte_data(self.i2c_address, 0x00, mode1 | 0x20)
            except Exception as e:
                if self.debug:
                    print(f"[PCAServoPlugin] 启用AI失败(不影响功能): {e}")

            if self.debug:
                print(f"[PCAServoPlugin] 激活成功: PCA9685已初始化")

            return True

        except Exception as e:
            self.set_error(f"激活失败: {e}")
            return False

    def on_deactivate(self) -> bool:
        """停用阶段：关闭I2C总线

        Returns:
            bool: 停用是否成功
        """
        try:
            if self.bus:
                self.bus.close()
                self.bus = None

            if self.debug:
                print(f"[PCAServoPlugin] 停用成功: I2C总线已关闭")

            return True

        except Exception as e:
            self.set_error(f"停用失败: {e}")
            return False

    def on_cleanup(self) -> bool:
        """清理阶段：释放所有资源

        Returns:
            bool: 清理是否成功
        """
        try:
            # 确保I2C总线关闭
            if self.bus:
                self.bus.close()
                self.bus = None

            # 清理数据
            self.servo_states.clear()
            self.controlled_channels.clear()

            if self.debug:
                print(f"[PCAServoPlugin] 清理完成")

            return True

        except Exception as e:
            self.set_error(f"清理失败: {e}")
            return False

    # ==================== 数据读写接口实现 ====================

    def read(self, time, period) -> Dict[int, ServoState]:
        """读取舵机状态（返回缓存状态）

        PCA9685不支持状态反馈，返回缓存的命令状态

        Args:
            time: 当前时间戳
            period: 控制周期

        Returns:
            Dict[int, ServoState]: 舵机状态字典
        """
        return self.servo_states.copy()

    def write(self, time, period, commands: Dict[int, ServoCommand]) -> bool:
        """写入命令到PCA9685

        Args:
            time: 当前时间戳
            period: 控制周期
            commands: 命令字典 {servo_id: ServoCommand}

        Returns:
            bool: 写入是否成功
        """
        if not self.bus:
            self.set_error("I2C总线未打开")
            return False

        success = True

        for servo_id, cmd in commands.items():
            # PCA9685的servo_id就是通道号(0-15)
            channel = servo_id

            if not (0 <= channel <= 15):
                self.set_error(f"PCA通道必须在0-15之间(当前: {channel})")
                success = False
                continue

            try:
                position = cmd.position

                # 判断是角度还是PWM tick值
                if 0 <= position <= 180:
                    # 角度 -> PWM tick
                    pulse = int(self.min_pwm + (position / 180.0) * (self.max_pwm - self.min_pwm))
                    self._set_pwm(channel, off=pulse)
                elif position > 1000:
                    # 可能是微秒值，转换为tick
                    ticks = self._us_to_ticks(position)
                    self._set_pwm(channel, off=ticks)
                else:
                    # 直接作为tick值
                    self._set_pwm(channel, off=position)

                # 更新缓存状态
                state = ServoState()
                state.servo_type = "pca"
                state.servo_id = channel
                state.position = position
                state.load = -1
                state.temperature = -1
                state.error_code = 0

                self.servo_states[channel] = state
                self.controlled_channels.add(channel)

            except Exception as e:
                self.set_error(f"写入通道 {channel} 失败: {e}")
                success = False

        return success

    # ==================== 内部辅助方法 ====================

    def _software_reset(self):
        """软件复位"""
        self.bus.write_byte_data(self.i2c_address, 0x00, 0x00)
        time.sleep(0.1)
        if self.debug:
            print("[PCAServoPlugin] 软件复位完成")

    def _set_pwm_freq(self, freq: int):
        """设置PWM频率

        Args:
            freq: PWM频率(Hz)
        """
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(round(prescaleval))

        old_mode = self.bus.read_byte_data(self.i2c_address, 0x00)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.i2c_address, 0x00, new_mode)
        self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
        self.bus.write_byte_data(self.i2c_address, 0x00, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.i2c_address, 0x00, old_mode | 0x80)

        if self.debug:
            print(f"[PCAServoPlugin] PWM频率设置为 {freq}Hz(预分频值: {prescale})")

    def _us_to_ticks(self, us: int) -> int:
        """将微秒脉宽转换为PWM tick值

        Args:
            us: 脉宽(微秒)

        Returns:
            int: PWM tick值(0-4095)
        """
        try:
            ticks = int(round(us * self.frequency * 4096 / 1_000_000))
        except Exception:
            ticks = self.min_pwm
        return max(self.min_pwm, min(self.max_pwm, ticks))

    def _set_pwm(self, channel: int, off: int, on: int = 0):
        """设置PWM值(ON固定为0,OFF为有效脉冲值)

        Args:
            channel: 通道号(0-15)
            off: PWM OFF值(tick)
            on: PWM ON值(通常为0)
        """
        # 限制范围
        off = max(self.min_pwm, min(self.max_pwm, off))

        base_reg = 0x06 + 4 * channel

        try:
            # 优先使用块写入，减少I2C事务
            self.bus.write_i2c_block_data(
                self.i2c_address,
                base_reg,
                [on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F],
            )
        except Exception:
            # 降级为逐字节写入
            self.bus.write_byte_data(self.i2c_address, base_reg, on & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 1, on >> 8)
            self.bus.write_byte_data(self.i2c_address, base_reg + 2, off & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 3, off >> 8)

        if self.debug:
            print(f"[PCAServoPlugin] 通道{channel}: PWM={off}")
