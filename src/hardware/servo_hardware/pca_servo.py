"""
PCA9685舵机驱动节点 - ROS 2版本

基于原始的PCA9685实现,适配ROS 2架构
使用I2C接口控制PCA9685 16通道PWM驱动板
"""

import json
import os
import time
from pathlib import Path
from typing import Set, Optional

import rclpy
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for non-ROS envs
    get_package_share_directory = None

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False
    SMBus = None


class PCA9685ServoDriver(Node):
    """PCA9685舵机驱动节点

    功能:
    - 订阅/servo/command话题接收舵机控制命令
    - 通过I2C控制PCA9685产生PWM信号驱动舵机
    - 发布舵机状态反馈到/servo/state话题

    协议:
    - I2C地址: 默认0x40
    - PWM频率: 50Hz(舵机标准)
    - PWM范围: 0-4095(12位)
    - 实际脉宽: 通常110-520对应0-180°
    """

    def __init__(self):
        super().__init__('pca_servo_driver')

        # 声明参数
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('bus_number', 1)
        self.declare_parameter('frequency', 50)
        self.declare_parameter('min_pwm', 110)
        self.declare_parameter('max_pwm', 520)
        self.declare_parameter('min_us', 500)
        self.declare_parameter('max_us', 2500)
        self.declare_parameter('debug', False)
        self.declare_parameter('offset_map', '')  # 舵机偏移量配置文件
        self.declare_parameter('limit_map', '')  # 舵机限位配置文件

        # 获取参数
        self.i2c_address = self.get_parameter('i2c_address').value
        self.bus_number = self.get_parameter('bus_number').value
        self.frequency = self.get_parameter('frequency').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_us = self.get_parameter('min_us').value
        self.max_us = self.get_parameter('max_us').value
        self.debug = self.get_parameter('debug').value
        offset_map_param = self.get_parameter('offset_map').value
        limit_map_param = self.get_parameter('limit_map').value

        # 检查依赖
        if not HAS_SMBUS:
            self.get_logger().error(
                '缺少smbus2库,请安装: pip3 install smbus2'
            )
            self.bus = None
            return

        # I2C总线对象
        self.bus: Optional[SMBus] = None
        self.controlled_channels: Set[int] = set()

        # 加载舵机偏移量
        self.offset_map = self._load_offset_map(offset_map_param)
        self.limit_map = self._load_limit_map(limit_map_param)

        # 初始化PCA9685
        if not self._init_device():
            self.get_logger().error(f'PCA9685初始化失败')
            return

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

        self.get_logger().info(
            f'PCA9685舵机驱动已启动: 地址=0x{self.i2c_address:02X}, 频率={self.frequency}Hz'
        )

    def _resolve_offset_map_path(self, override_path: str) -> str:
        if override_path and os.path.exists(override_path):
            return override_path

        if get_package_share_directory:
            try:
                share_dir = get_package_share_directory('servo_hardware')
                candidate = os.path.join(
                    share_dir, 'config', 'servo_offset_map.json'
                )
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass

        candidate = Path(__file__).resolve().parent / 'config' / 'servo_offset_map.json'
        if candidate.exists():
            return str(candidate)

        return ''

    def _resolve_limit_map_path(self, override_path: str) -> str:
        if override_path and os.path.exists(override_path):
            return override_path

        if get_package_share_directory:
            try:
                share_dir = get_package_share_directory('servo_hardware')
                candidate = os.path.join(
                    share_dir, 'config', 'servo_limit_map.json'
                )
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass

        candidate = Path(__file__).resolve().parent / 'config' / 'servo_limit_map.json'
        if candidate.exists():
            return str(candidate)

        return ''

    def _load_offset_map(self, override_path: str) -> dict:
        path = self._resolve_offset_map_path(override_path)
        if not path:
            self.get_logger().info('未找到舵机偏移量配置文件，使用默认偏移(0)')
            return {}

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'加载舵机偏移量失败: {e}')
            return {}

        offset_map: dict[int, float] = {}
        if isinstance(data, dict):
            # 兼容旧格式: {ids: [...], offsets: [...]}
            if 'ids' in data and 'offsets' in data:
                ids = data.get('ids', [])
                offsets = data.get('offsets', [])
                for sid, offset in zip(ids, offsets):
                    try:
                        offset_map[int(sid)] = float(offset)
                    except (TypeError, ValueError):
                        continue
            else:
                for key, value in data.items():
                    try:
                        offset_map[int(key)] = float(value)
                    except (TypeError, ValueError):
                        continue

        if offset_map:
            self.get_logger().info(
                f'已加载舵机偏移量配置: {path} (数量={len(offset_map)})'
            )
        else:
            self.get_logger().info(f'舵机偏移量为空: {path}')

        return offset_map

    def _load_limit_map(self, override_path: str) -> dict:
        path = self._resolve_limit_map_path(override_path)
        if not path:
            return {}

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().warn(f'加载舵机限位配置失败: {exc}')
            return {}

        if isinstance(data, dict) and 'servo_limits' in data:
            data = data.get('servo_limits') or {}

        limit_map: dict[int, dict] = {}
        if isinstance(data, dict) and 'ids' in data:
            ids = data.get('ids') or []
            mins = data.get('mins') or []
            maxs = data.get('maxs') or []
            for idx, sid in enumerate(ids):
                try:
                    sid_int = int(sid)
                except (TypeError, ValueError):
                    continue
                min_val = mins[idx] if idx < len(mins) else None
                max_val = maxs[idx] if idx < len(maxs) else None
                limit_map[sid_int] = {
                    'min': min_val,
                    'max': max_val
                }
        elif isinstance(data, dict):
            for key, value in data.items():
                try:
                    sid_int = int(key)
                except (TypeError, ValueError):
                    continue
                if isinstance(value, dict):
                    limit_map[sid_int] = {
                        'min': value.get('min'),
                        'max': value.get('max')
                    }
        if limit_map:
            self.get_logger().info(
                f'已加载舵机限位配置: {path} (数量={len(limit_map)})'
            )
        return limit_map

    def _apply_limits(self, channel: int, position: int) -> int:
        if not self.limit_map:
            return position
        entry = self.limit_map.get(channel)
        if not isinstance(entry, dict):
            return position
        min_val = entry.get('min')
        max_val = entry.get('max')
        try:
            if min_val is not None:
                position = max(int(round(float(min_val))), position)
            if max_val is not None:
                position = min(int(round(float(max_val))), position)
        except (TypeError, ValueError):
            return position
        return position

    def _init_device(self) -> bool:
        """初始化PCA9685设备"""
        try:
            self.bus = SMBus(self.bus_number)

            # 验证设备连接
            try:
                self.bus.read_byte_data(self.i2c_address, 0x00)
                self.get_logger().info(f'PCA9685设备检测成功(地址: 0x{self.i2c_address:02X})')
            except OSError as e:
                self.get_logger().error(f'无法连接PCA9685(地址: 0x{self.i2c_address:02X}): {e}')
                return False

            # 软件复位
            self._software_reset()

            # 设置PWM频率
            self._set_pwm_freq(self.frequency)

            # 配置推挽输出(适配舵机驱动)
            self.bus.write_byte_data(self.i2c_address, 0x01, 0x04)
            if self.debug:
                self.get_logger().info('PCA9685已配置推挽输出模式')

            # 启用寄存器地址自动递增(Auto-Increment)
            try:
                mode1 = self.bus.read_byte_data(self.i2c_address, 0x00)
                self.bus.write_byte_data(self.i2c_address, 0x00, mode1 | 0x20)
                if self.debug:
                    self.get_logger().info('PCA9685已启用寄存器自动递增(AI)')
            except Exception as e:
                if self.debug:
                    self.get_logger().warn(f'启用AI失败(不影响功能): {e}')

            return True

        except Exception as e:
            self.get_logger().error(f'PCA9685初始化异常: {e}')
            return False

    def _software_reset(self):
        """软件复位"""
        self.bus.write_byte_data(self.i2c_address, 0x00, 0x00)
        time.sleep(0.1)
        if self.debug:
            self.get_logger().info('PCA9685软件复位完成')

    def _set_pwm_freq(self, freq: int):
        """设置PWM频率

        Args:
            freq: PWM频率(Hz), 舵机通常使用50Hz
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
            self.get_logger().info(f'PWM频率设置为 {freq}Hz(预分频值: {prescale})')

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

    def set_pwm(self, channel: int, off: int, on: int = 0):
        """设置PWM值(ON固定为0,OFF为有效脉冲值)

        Args:
            channel: 通道号(0-15)
            off: PWM OFF值(tick)
            on: PWM ON值(通常为0)
        """
        if not (0 <= channel <= 15):
            self.get_logger().error(f'通道必须在0-15之间(当前: {channel})')
            return

        if self.bus is None:
            self.get_logger().warn('PCA9685未初始化')
            return

        # 限制范围
        off = max(self.min_pwm, min(self.max_pwm, off))

        base_reg = 0x06 + 4 * channel

        try:
            # 优先使用块写入,减少I2C事务
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

        self.controlled_channels.add(channel)

        if self.debug:
            self.get_logger().debug(f'通道{channel}: PWM={off}')

    def set_angle(self, channel: int, angle: int):
        """设置舵机角度(0-180°)

        Args:
            channel: 通道号(0-15)
            angle: 目标角度(0-180°)
        """
        if not (0 <= angle <= 180):
            self.get_logger().error(f'角度必须在0-180之间(当前: {angle})')
            return

        pulse = int(self.min_pwm + (angle / 180.0) * (self.max_pwm - self.min_pwm))
        self.set_pwm(channel, off=pulse)

        if self.debug:
            self.get_logger().debug(f'舵机通道{channel}: 角度{angle}° → PWM值{pulse}')

    def command_callback(self, msg: ServoCommand):
        """处理舵机控制命令（使用ServoCommand消息）"""
        try:
            # 仅处理PCA舵机命令
            if msg.servo_type != 'pca':
                if self.debug:
                    self.get_logger().warn(
                        f'忽略非PCA舵机命令: servo_type={msg.servo_type}')
                return

            # 提取命令参数
            channel = msg.servo_id  # 对于PCA,servo_id即通道号
            position = msg.position
            position = self._apply_limits(channel, position)

            # 验证通道范围
            if not (0 <= channel <= 15):
                self.get_logger().error(f'PCA通道必须在0-15之间(当前: {channel})')
                # 发布错误状态
                error_msg = ServoState()
                error_msg.servo_type = "pca"
                error_msg.servo_id = channel
                error_msg.position = position
                error_msg.load = -1
                error_msg.temperature = -1
                error_msg.error_code = 1  # 参数错误
                error_msg.stamp = self.get_clock().now().to_msg()
                self.state_pub.publish(error_msg)
                return

            # 执行舵机控制
            success = True
            offset = self.offset_map.get(channel, 0.0)
            try:
                # 判断是角度还是PWM tick值
                if 0 <= position <= 180:
                    # 角度 -> PWM tick
                    ticks = int(self.min_pwm + (position / 180.0) *
                                (self.max_pwm - self.min_pwm))
                elif position > 1000:
                    # 可能是微秒值,转换为tick
                    ticks = self._us_to_ticks(position)
                else:
                    # 直接作为tick值
                    ticks = int(position)

                if offset:
                    ticks = int(round(ticks + offset))

                self.set_pwm(channel, off=ticks)
            except Exception as e:
                self.get_logger().error(f'PCA舵机控制失败: {e}')
                success = False

            # 发布状态反馈
            state_msg = ServoState()
            state_msg.servo_type = "pca"
            state_msg.servo_id = channel
            state_msg.position = position
            state_msg.load = -1  # PCA舵机不支持负载反馈
            state_msg.temperature = -1  # PCA舵机不支持温度反馈
            state_msg.error_code = 0 if success else 1  # 0表示正常
            state_msg.stamp = self.get_clock().now().to_msg()

            self.state_pub.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f'命令处理失败: {e}')

    def reset_servos(self):
        """复位所有已控制的舵机到中间位置(90°)"""
        if not self.controlled_channels:
            return

        self.get_logger().info('开始复位PCA舵机到中间位置...')
        mid_angle = 90
        mid_pulse = int(self.min_pwm + (mid_angle / 180.0) * (self.max_pwm - self.min_pwm))

        for ch in self.controlled_channels:
            try:
                self.set_pwm(ch, off=mid_pulse)
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'复位通道{ch}失败: {e}')

        self.get_logger().info('PCA舵机复位完成')

    def destroy_node(self):
        """节点销毁时清理资源"""
        # 复位舵机
        self.reset_servos()

        # 关闭I2C总线
        if self.bus:
            try:
                self.bus.close()
                self.get_logger().info('I2C总线已关闭')
            except Exception as e:
                self.get_logger().error(f'关闭I2C总线异常: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685ServoDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
