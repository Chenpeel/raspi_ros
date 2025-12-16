"""
ROS2舵机硬件抽象层 - 硬件接口基类

参考ros2_control架构设计，提供标准化的硬件接口
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, Optional
from servo_msgs.msg import ServoCommand, ServoState


class HardwareInterfaceState(Enum):
    """硬件接口状态（参考ros2_control生命周期）"""
    UNCONFIGURED = 0  # 未配置
    INACTIVE = 1      # 已配置但未激活
    ACTIVE = 2        # 激活运行中
    FINALIZED = 3     # 已清理


class ServoHardwareInterface(ABC):
    """舵机硬件接口基类

    遵循ros2_control的设计哲学：
    - 生命周期管理：configure → activate → deactivate → cleanup
    - 周期性读写：read() 和 write() 在固定频率下被调用
    - 状态查询：提供硬件状态和错误信息

    子类需要实现：
    - on_configure(): 配置阶段，加载参数、初始化资源
    - on_activate(): 激活阶段，启动硬件通信
    - on_deactivate(): 停用阶段，停止硬件通信但保留资源
    - on_cleanup(): 清理阶段，释放所有资源
    - read(): 读取硬件状态（周期性调用）
    - write(): 写入命令到硬件（周期性调用）
    """

    def __init__(self, name: str):
        """初始化硬件接口

        Args:
            name: 硬件接口名称（如 "bus_servo", "pca_servo"）
        """
        self.name = name
        self.state = HardwareInterfaceState.UNCONFIGURED
        self._error_msg: Optional[str] = None

    # ==================== 生命周期管理接口 ====================

    @abstractmethod
    def on_configure(self, config: Dict) -> bool:
        """配置阶段：加载参数、初始化资源

        此阶段应该：
        - 解析配置参数
        - 初始化硬件连接对象（但不打开连接）
        - 验证配置有效性

        Args:
            config: 配置字典，包含硬件相关参数

        Returns:
            bool: 配置是否成功
        """
        pass

    @abstractmethod
    def on_activate(self) -> bool:
        """激活阶段：启动硬件通信

        此阶段应该：
        - 打开硬件连接（串口、I2C等）
        - 初始化硬件设备
        - 验证硬件可用性

        Returns:
            bool: 激活是否成功
        """
        pass

    @abstractmethod
    def on_deactivate(self) -> bool:
        """停用阶段：停止硬件通信

        此阶段应该：
        - 停止硬件通信
        - 可选：复位硬件到安全状态
        - 保留资源以便重新激活

        Returns:
            bool: 停用是否成功
        """
        pass

    @abstractmethod
    def on_cleanup(self) -> bool:
        """清理阶段：释放所有资源

        此阶段应该：
        - 关闭硬件连接
        - 释放所有资源
        - 清理缓存数据

        Returns:
            bool: 清理是否成功
        """
        pass

    # ==================== 数据读写接口 ====================

    @abstractmethod
    def read(self, time, period) -> Dict[int, ServoState]:
        """读取硬件状态（周期性调用）

        此方法在控制循环中被周期性调用，应该：
        - 从硬件读取最新状态
        - 返回所有受控舵机的状态
        - 处理读取错误并更新错误信息

        Args:
            time: 当前时间戳
            period: 控制周期（秒）

        Returns:
            Dict[int, ServoState]: 舵机状态字典 {servo_id: ServoState}
        """
        pass

    @abstractmethod
    def write(self, time, period, commands: Dict[int, ServoCommand]) -> bool:
        """写入命令到硬件（周期性调用）

        此方法在控制循环中被周期性调用，应该：
        - 将命令写入硬件
        - 处理写入错误并更新错误信息
        - 返回写入是否成功

        Args:
            time: 当前时间戳
            period: 控制周期（秒）
            commands: 命令字典 {servo_id: ServoCommand}

        Returns:
            bool: 写入是否成功
        """
        pass

    # ==================== 状态查询接口 ====================

    def get_state(self) -> HardwareInterfaceState:
        """获取硬件接口状态

        Returns:
            HardwareInterfaceState: 当前状态
        """
        return self.state

    def get_name(self) -> str:
        """获取硬件接口名称

        Returns:
            str: 硬件接口名称
        """
        return self.name

    def get_error(self) -> Optional[str]:
        """获取错误信息

        Returns:
            Optional[str]: 错误信息，无错误时返回None
        """
        return self._error_msg

    def set_error(self, msg: str):
        """设置错误信息

        Args:
            msg: 错误描述
        """
        self._error_msg = msg

    def clear_error(self):
        """清除错误信息"""
        self._error_msg = None

    def is_active(self) -> bool:
        """检查硬件是否处于激活状态

        Returns:
            bool: 是否激活
        """
        return self.state == HardwareInterfaceState.ACTIVE

    # ==================== 生命周期状态转换 ====================

    def configure(self, config: Dict) -> bool:
        """执行配置并转换状态

        Args:
            config: 配置字典

        Returns:
            bool: 配置是否成功
        """
        if self.state != HardwareInterfaceState.UNCONFIGURED:
            self.set_error(f"配置失败：当前状态为 {self.state.name}，必须在 UNCONFIGURED 状态")
            return False

        if self.on_configure(config):
            self.state = HardwareInterfaceState.INACTIVE
            self.clear_error()
            return True
        else:
            self.set_error("配置失败：on_configure() 返回 False")
            return False

    def activate(self) -> bool:
        """执行激活并转换状态

        Returns:
            bool: 激活是否成功
        """
        if self.state != HardwareInterfaceState.INACTIVE:
            self.set_error(f"激活失败：当前状态为 {self.state.name}，必须在 INACTIVE 状态")
            return False

        if self.on_activate():
            self.state = HardwareInterfaceState.ACTIVE
            self.clear_error()
            return True
        else:
            self.set_error("激活失败：on_activate() 返回 False")
            return False

    def deactivate(self) -> bool:
        """执行停用并转换状态

        Returns:
            bool: 停用是否成功
        """
        if self.state != HardwareInterfaceState.ACTIVE:
            self.set_error(f"停用失败：当前状态为 {self.state.name}，必须在 ACTIVE 状态")
            return False

        if self.on_deactivate():
            self.state = HardwareInterfaceState.INACTIVE
            self.clear_error()
            return True
        else:
            self.set_error("停用失败：on_deactivate() 返回 False")
            return False

    def cleanup(self) -> bool:
        """执行清理并转换状态

        Returns:
            bool: 清理是否成功
        """
        if self.state == HardwareInterfaceState.UNCONFIGURED:
            return True  # 已经是未配置状态

        if self.on_cleanup():
            self.state = HardwareInterfaceState.FINALIZED
            self.clear_error()
            return True
        else:
            self.set_error("清理失败：on_cleanup() 返回 False")
            return False
