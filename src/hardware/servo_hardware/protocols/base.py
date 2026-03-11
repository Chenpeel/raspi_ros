"""总线舵机协议抽象基类。"""

from abc import ABC, abstractmethod
from typing import Optional


class BusServoProtocol(ABC):
    """总线舵机协议抽象。

    子类只关心协议编解码，串口IO由驱动层负责。
    """

    name: str = "unknown"

    @abstractmethod
    def encode_move_command(self, servo_id: int, position: int, speed: int) -> bytes:
        """编码位置控制命令。"""

    @abstractmethod
    def encode_read_position_command(self, servo_id: int) -> bytes:
        """编码读取当前位置命令。"""

    @abstractmethod
    def decode_position_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        """从串口数据中解析当前位置。

        Args:
            data: 原始串口缓冲区
            expected_servo_id: 期望的舵机ID；为None时不校验ID

        Returns:
            int | None: 解析出的协议位置值，解析失败返回None
        """
