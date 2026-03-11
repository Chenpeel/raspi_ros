"""众灵总线舵机协议。"""

import re
from typing import Optional

from .base import BusServoProtocol


class ZLBusServoProtocol(BusServoProtocol):
    """众灵文本协议实现。"""

    name = "zl"
    _POS_RE = re.compile(r"#(?P<sid>\d{3})P(?P<pos>-?\d{1,5})!")

    def encode_move_command(self, servo_id: int, position: int, speed: int) -> bytes:
        servo_id = max(0, min(999, int(servo_id)))
        position = max(0, min(9999, int(position)))
        speed = max(0, min(9999, int(speed)))
        return f"#{servo_id:03d}P{position:04d}T{speed:04d}!".encode("utf-8")

    def encode_read_position_command(self, servo_id: int) -> bytes:
        servo_id = max(0, min(999, int(servo_id)))
        return f"#{servo_id:03d}PRAD!".encode("utf-8")

    def decode_position_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        if not data:
            return None

        text = data.decode("utf-8", errors="ignore")
        for match in self._POS_RE.finditer(text):
            sid = int(match.group("sid"))
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            return int(match.group("pos"))
        return None
