"""
ROS2舵机硬件抽象层 - 资源管理器

参考ros2_control的ResourceManager，统一管理所有硬件接口
"""

from typing import Dict, List, Optional
from .hardware_interface import ServoHardwareInterface, HardwareInterfaceState
from servo_msgs.msg import ServoCommand, ServoState


class ServoResourceManager:
    """舵机资源管理器

    功能：
    - 注册和管理多个硬件接口（BusServo、PCA9685等）
    - 统一配置和激活所有硬件
    - 周期性读写：调用所有硬件的read()/write()
    - 命令路由：根据servo_id将命令分发到对应硬件
    - 状态聚合：收集所有硬件的状态反馈

    设计理念：
    - 类似ros2_control的ResourceManager
    - 支持动态添加/移除硬件接口
    - 提供统一的错误处理和日志记录
    """

    def __init__(self):
        """初始化资源管理器"""
        self.hardware_interfaces: Dict[str, ServoHardwareInterface] = {}
        self.servo_map: Dict[int, str] = {}  # servo_id → hardware_name 映射
        self._error_msg: Optional[str] = None

    # ==================== 硬件接口注册管理 ====================

    def register_hardware(self, hw_interface: ServoHardwareInterface) -> bool:
        """注册硬件接口

        Args:
            hw_interface: 硬件接口实例

        Returns:
            bool: 注册是否成功
        """
        name = hw_interface.get_name()

        if name in self.hardware_interfaces:
            self._error_msg = f"硬件接口 '{name}' 已存在"
            return False

        self.hardware_interfaces[name] = hw_interface
        self._error_msg = None
        return True

    def unregister_hardware(self, name: str) -> bool:
        """注销硬件接口

        Args:
            name: 硬件接口名称

        Returns:
            bool: 注销是否成功
        """
        if name not in self.hardware_interfaces:
            self._error_msg = f"硬件接口 '{name}' 不存在"
            return False

        # 从servo_map中移除该硬件的所有映射
        self.servo_map = {
            sid: hw_name
            for sid, hw_name in self.servo_map.items()
            if hw_name != name
        }

        del self.hardware_interfaces[name]
        self._error_msg = None
        return True

    def get_hardware(self, name: str) -> Optional[ServoHardwareInterface]:
        """获取硬件接口

        Args:
            name: 硬件接口名称

        Returns:
            Optional[ServoHardwareInterface]: 硬件接口实例，不存在时返回None
        """
        return self.hardware_interfaces.get(name)

    def list_hardware(self) -> List[str]:
        """列出所有注册的硬件接口名称

        Returns:
            List[str]: 硬件接口名称列表
        """
        return list(self.hardware_interfaces.keys())

    # ==================== Servo ID 映射管理 ====================

    def map_servo(self, servo_id: int, hardware_name: str) -> bool:
        """映射servo_id到硬件接口

        Args:
            servo_id: 舵机ID
            hardware_name: 硬件接口名称

        Returns:
            bool: 映射是否成功
        """
        if hardware_name not in self.hardware_interfaces:
            self._error_msg = f"硬件接口 '{hardware_name}' 不存在"
            return False

        if servo_id in self.servo_map:
            old_hw = self.servo_map[servo_id]
            if old_hw != hardware_name:
                self._error_msg = f"舵机 {servo_id} 已映射到 '{old_hw}'，无法重新映射到 '{hardware_name}'"
                return False

        self.servo_map[servo_id] = hardware_name
        self._error_msg = None
        return True

    def unmap_servo(self, servo_id: int) -> bool:
        """取消servo_id的映射

        Args:
            servo_id: 舵机ID

        Returns:
            bool: 取消映射是否成功
        """
        if servo_id not in self.servo_map:
            self._error_msg = f"舵机 {servo_id} 未映射"
            return False

        del self.servo_map[servo_id]
        self._error_msg = None
        return True

    def get_hardware_for_servo(self, servo_id: int) -> Optional[str]:
        """获取servo_id对应的硬件接口名称

        Args:
            servo_id: 舵机ID

        Returns:
            Optional[str]: 硬件接口名称，未映射时返回None
        """
        return self.servo_map.get(servo_id)

    # ==================== 生命周期管理 ====================

    def configure_all(self, configs: Dict[str, Dict]) -> bool:
        """配置所有硬件接口

        Args:
            configs: 配置字典 {hardware_name: config_dict}

        Returns:
            bool: 所有硬件配置是否成功
        """
        success = True

        for name, hw in self.hardware_interfaces.items():
            config = configs.get(name, {})

            if not hw.configure(config):
                self._error_msg = f"配置硬件 '{name}' 失败: {hw.get_error()}"
                success = False
                break

        return success

    def activate_all(self) -> bool:
        """激活所有硬件接口

        Returns:
            bool: 所有硬件激活是否成功
        """
        success = True

        for name, hw in self.hardware_interfaces.items():
            if not hw.activate():
                self._error_msg = f"激活硬件 '{name}' 失败: {hw.get_error()}"
                success = False
                break

        return success

    def deactivate_all(self) -> bool:
        """停用所有硬件接口

        Returns:
            bool: 所有硬件停用是否成功
        """
        success = True

        for name, hw in self.hardware_interfaces.items():
            if not hw.deactivate():
                self._error_msg = f"停用硬件 '{name}' 失败: {hw.get_error()}"
                success = False
                # 继续尝试停用其他硬件

        return success

    def cleanup_all(self) -> bool:
        """清理所有硬件接口

        Returns:
            bool: 所有硬件清理是否成功
        """
        success = True

        for name, hw in self.hardware_interfaces.items():
            if not hw.cleanup():
                self._error_msg = f"清理硬件 '{name}' 失败: {hw.get_error()}"
                success = False
                # 继续尝试清理其他硬件

        return success

    # ==================== 周期性读写 ====================

    def read_all(self, time, period) -> Dict[int, ServoState]:
        """从所有激活的硬件读取状态

        Args:
            time: 当前时间戳
            period: 控制周期（秒）

        Returns:
            Dict[int, ServoState]: 所有舵机的状态 {servo_id: ServoState}
        """
        all_states = {}

        for name, hw in self.hardware_interfaces.items():
            if hw.is_active():
                try:
                    states = hw.read(time, period)
                    all_states.update(states)
                except Exception as e:
                    # 记录错误但不中断其他硬件的读取
                    hw.set_error(f"读取失败: {e}")

        return all_states

    def write_all(self, time, period, commands: Dict[int, ServoCommand]) -> bool:
        """写入命令到对应硬件

        根据servo_id将命令路由到对应的硬件接口

        Args:
            time: 当前时间戳
            period: 控制周期（秒）
            commands: 命令字典 {servo_id: ServoCommand}

        Returns:
            bool: 所有命令写入是否成功
        """
        # 按硬件分组命令
        hw_commands: Dict[str, Dict[int, ServoCommand]] = {}

        for servo_id, cmd in commands.items():
            hw_name = self.servo_map.get(servo_id)

            if hw_name is None:
                # servo_id未映射，跳过
                continue

            if hw_name not in hw_commands:
                hw_commands[hw_name] = {}

            hw_commands[hw_name][servo_id] = cmd

        # 写入各硬件
        success = True

        for hw_name, cmds in hw_commands.items():
            hw = self.hardware_interfaces.get(hw_name)

            if hw is None or not hw.is_active():
                continue

            try:
                if not hw.write(time, period, cmds):
                    success = False
            except Exception as e:
                hw.set_error(f"写入失败: {e}")
                success = False

        return success

    # ==================== 状态查询 ====================

    def get_error(self) -> Optional[str]:
        """获取管理器错误信息

        Returns:
            Optional[str]: 错误信息，无错误时返回None
        """
        return self._error_msg

    def get_hardware_states(self) -> Dict[str, HardwareInterfaceState]:
        """获取所有硬件接口的状态

        Returns:
            Dict[str, HardwareInterfaceState]: {hardware_name: state}
        """
        return {
            name: hw.get_state()
            for name, hw in self.hardware_interfaces.items()
        }

    def get_hardware_errors(self) -> Dict[str, Optional[str]]:
        """获取所有硬件接口的错误信息

        Returns:
            Dict[str, Optional[str]]: {hardware_name: error_msg}
        """
        return {
            name: hw.get_error()
            for name, hw in self.hardware_interfaces.items()
        }

    def is_all_active(self) -> bool:
        """检查所有硬件是否都处于激活状态

        Returns:
            bool: 是否全部激活
        """
        return all(hw.is_active() for hw in self.hardware_interfaces.values())
