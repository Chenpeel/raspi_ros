"""众灵总线舵机协议。"""

import re
from typing import Optional, Tuple

from .base import BusServoProtocol


class ZLBusServoProtocol(BusServoProtocol):
    """众灵文本协议实现。"""

    name = "zl"
    _POS_RE = re.compile(r"#(?P<sid>\d{3})P(?P<pos>-?\d{1,5})!")
    _ID_RE = re.compile(r"#(?P<sid>\d{3})P!")
    _VER_RE = re.compile(r"#(?P<sid>\d{3})PV(?P<ver>[^!]+)!")
    _MODE_RE = re.compile(r"#(?P<sid>\d{3})PMOD(?P<mode>[1-8])!")
    _TV_RE = re.compile(
        r"#(?P<sid>\d{3})T(?P<temp>-?\d+(?:\.\d+)?)V(?P<volt>-?\d+(?:\.\d+)?)!"
    )

    @staticmethod
    def _sid(servo_id: int) -> int:
        return max(0, min(999, int(servo_id)))

    @staticmethod
    def _format_command(servo_id: int, body: str) -> bytes:
        sid = ZLBusServoProtocol._sid(servo_id)
        return f"#{sid:03d}{body}!".encode("utf-8")

    # ======= 兼容 BusServoProtocol 抽象接口 =======
    def encode_move_command(self, servo_id: int, position: int, speed: int) -> bytes:
        position = max(0, min(9999, int(position)))
        speed = max(0, min(9999, int(speed)))
        return self._format_command(servo_id, f"P{position:04d}T{speed:04d}")

    def encode_read_position_command(self, servo_id: int) -> bytes:
        return self.encode_position_read(servo_id)

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

    # ======= 旧驱动完整指令（编码） =======
    def encode_version_read(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PVER")

    def encode_id_read(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PID")

    def encode_id_write(self, servo_id: int, new_id: int) -> bytes:
        nid = self._sid(new_id)
        return self._format_command(servo_id, f"PID{nid:03d}")

    def encode_torque_release(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PULK")

    def encode_torque_restore(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PULR")

    def encode_mode_read(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PMOD")

    def encode_mode_write(self, servo_id: int, mode: int) -> bytes:
        mode = max(1, min(8, int(mode)))
        return self._format_command(servo_id, f"PMOD{mode}")

    def encode_position_read(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PRAD")

    def encode_motion_pause(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PDPT")

    def encode_motion_continue(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PDCT")

    def encode_motion_stop(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PDST")

    def encode_baudrate_write(self, servo_id: int, baudrate_code: int) -> bytes:
        code = max(1, min(8, int(baudrate_code)))
        return self._format_command(servo_id, f"PBD{code}")

    def encode_middle_calibrate(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PSCK")

    def encode_startup_position_set(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PCSD")

    def encode_startup_position_disable(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PCSM")

    def encode_startup_position_restore(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PCSR")

    def encode_min_position_set(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PSMI")

    def encode_max_position_set(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PSMX")

    def encode_factory_reset(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PCLE")

    def encode_temp_voltage_read(self, servo_id: int) -> bytes:
        return self._format_command(servo_id, "PRTV")

    # ======= 常用解码 =======
    @staticmethod
    def decode_ok_response(data: bytes) -> bool:
        if not data:
            return False
        text = data.decode("utf-8", errors="ignore")
        return "#OK!" in text

    def decode_id_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        if not data:
            return None
        text = data.decode("utf-8", errors="ignore")
        for match in self._ID_RE.finditer(text):
            sid = int(match.group("sid"))
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            return sid
        return None

    def decode_version_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[str]:
        if not data:
            return None
        text = data.decode("utf-8", errors="ignore")
        for match in self._VER_RE.finditer(text):
            sid = int(match.group("sid"))
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            return str(match.group("ver"))
        return None

    def decode_mode_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        if not data:
            return None
        text = data.decode("utf-8", errors="ignore")
        for match in self._MODE_RE.finditer(text):
            sid = int(match.group("sid"))
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            return int(match.group("mode"))
        return None

    def decode_temp_voltage_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[Tuple[float, float]]:
        if not data:
            return None
        text = data.decode("utf-8", errors="ignore")
        for match in self._TV_RE.finditer(text):
            sid = int(match.group("sid"))
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            return float(match.group("temp")), float(match.group("volt"))
        return None
