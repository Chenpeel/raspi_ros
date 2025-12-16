"""
总线舵机硬件接口插件

将原bus_servo驱动封装为符合ServoHardwareInterface的插件
"""

import serial
import time
import threading
from typing import Dict, Set, Optional

from servo_hardware_interface import ServoHardwareInterface
from servo_msgs.msg import ServoCommand, ServoState


class BusServoPlugin(ServoHardwareInterface):
    """总线舵机硬件接口插件

    协议：
    - 帧格式: #ID(3位)P位置(4位)T速度(4位)!
    - 示例: #001P1500T0100!
    - 位置: 角度0-180° 或 脉宽500-2500us
    - 速度: 运动时间(毫秒)
    """

    def __init__(self, name: str = "bus_servo"):
        """初始化总线舵机插件

        Args:
            name: 硬件接口名称
        """
        super().__init__(name)

        # 串口相关
        self.port: str = "/dev/ttyAMA0"
        self.baudrate: int = 115200
        self.timeout: float = 0.1
        self.ser: Optional[serial.Serial] = None

        # 运行状态
        self.lock = threading.Lock()
        self.recv_buffer = bytearray()
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None

        # 舵机状态缓存
        self.servo_states: Dict[int, ServoState] = {}
        self.controlled_servos: Set[int] = set()

        # 参数
        self.default_speed: int = 100
        self.debug: bool = False

    # ==================== 生命周期接口实现 ====================

    def on_configure(self, config: Dict) -> bool:
        """配置阶段：加载参数并初始化串口对象

        Args:
            config: 配置字典，支持的参数：
                - port: 串口路径
                - baudrate: 波特率
                - timeout: 超时时间
                - default_speed: 默认速度
                - debug: 是否开启调试

        Returns:
            bool: 配置是否成功
        """
        try:
            # 加载配置参数
            self.port = config.get('port', self.port)
            self.baudrate = config.get('baudrate', self.baudrate)
            self.timeout = config.get('timeout', self.timeout)
            self.default_speed = config.get('default_speed', self.default_speed)
            self.debug = config.get('debug', self.debug)

            # 初始化串口对象（不打开）
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = self.timeout

            if self.debug:
                print(f"[BusServoPlugin] 配置完成: {self.port}@{self.baudrate}")

            return True

        except Exception as e:
            self.set_error(f"配置失败: {e}")
            return False

    def on_activate(self) -> bool:
        """激活阶段：打开串口并启动接收线程

        Returns:
            bool: 激活是否成功
        """
        try:
            # 打开串口
            if self.ser and not self.ser.is_open:
                self.ser.open()

            # 启动接收线程
            self.running = True
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()

            if self.debug:
                print(f"[BusServoPlugin] 激活成功: 串口已打开，接收线程已启动")

            return True

        except Exception as e:
            self.set_error(f"激活失败: {e}")
            return False

    def on_deactivate(self) -> bool:
        """停用阶段：停止接收线程并关闭串口

        Returns:
            bool: 停用是否成功
        """
        try:
            # 停止接收线程
            self.running = False
            if self.recv_thread and self.recv_thread.is_alive():
                self.recv_thread.join(timeout=0.5)

            # 关闭串口
            if self.ser and self.ser.is_open:
                self.ser.close()

            if self.debug:
                print(f"[BusServoPlugin] 停用成功: 串口已关闭")

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
            # 确保串口关闭
            if self.ser:
                if self.ser.is_open:
                    self.ser.close()
                self.ser = None

            # 清理数据
            self.servo_states.clear()
            self.controlled_servos.clear()
            self.recv_buffer.clear()

            if self.debug:
                print(f"[BusServoPlugin] 清理完成")

            return True

        except Exception as e:
            self.set_error(f"清理失败: {e}")
            return False

    # ==================== 数据读写接口实现 ====================

    def read(self, time, period) -> Dict[int, ServoState]:
        """读取舵机状态（返回缓存状态）

        总线舵机通常不支持状态反馈，返回缓存的命令状态

        Args:
            time: 当前时间戳
            period: 控制周期

        Returns:
            Dict[int, ServoState]: 舵机状态字典
        """
        return self.servo_states.copy()

    def write(self, time, period, commands: Dict[int, ServoCommand]) -> bool:
        """写入命令到总线舵机

        Args:
            time: 当前时间戳
            period: 控制周期
            commands: 命令字典 {servo_id: ServoCommand}

        Returns:
            bool: 写入是否成功
        """
        if not self.ser or not self.ser.is_open:
            self.set_error("串口未打开")
            return False

        success = True

        for servo_id, cmd in commands.items():
            try:
                # 格式化控制帧
                pulse = self._position_to_pulse(cmd.position)
                speed = cmd.speed if cmd.speed > 0 else self.default_speed
                frame = self._format_frame(servo_id, pulse, speed)

                # 发送到串口
                with self.lock:
                    self.ser.write(frame)

                # 更新缓存状态
                state = ServoState()
                state.servo_type = "bus"
                state.servo_id = servo_id
                state.position = cmd.position
                state.load = -1
                state.temperature = -1
                state.error_code = 0

                self.servo_states[servo_id] = state
                self.controlled_servos.add(servo_id)

                # 给设备一点处理时间
                time.sleep(0.005)

            except Exception as e:
                self.set_error(f"写入舵机 {servo_id} 失败: {e}")
                success = False

        return success

    # ==================== 内部辅助方法 ====================

    def _recv_loop(self):
        """后台接收线程：非阻塞读取串口并缓存数据"""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        with self.lock:
                            self.recv_buffer.extend(data)

                        if self.debug:
                            try:
                                text = data.decode('utf-8', errors='ignore').strip()
                                if text:
                                    print(f"[BusServoPlugin] Recv: {text}")
                            except Exception:
                                pass
                else:
                    time.sleep(0.05)

            except Exception as e:
                if self.debug:
                    print(f"[BusServoPlugin] 接收线程异常: {e}")
                time.sleep(0.2)

    def _position_to_pulse(self, position: int) -> int:
        """将位置值转换为脉宽

        Args:
            position: 位置值（角度0-180 或 脉宽500-2500）

        Returns:
            int: 脉宽值
        """
        if 0 <= position <= 180:
            # 角度 -> 脉宽 (500-2500us)
            return int(500 + (position / 180.0) * (2500 - 500))
        elif 500 <= position <= 2500:
            # 已经是脉宽
            return position
        else:
            # 超出范围，尝试直接发送
            return position

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
        return frame_str.encode('utf-8')
