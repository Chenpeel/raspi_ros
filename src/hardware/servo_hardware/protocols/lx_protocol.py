"""幻尔总线舵机协议。"""

from typing import Iterable, Optional

from .base import BusServoProtocol


class LXBusServoProtocol(BusServoProtocol):
    """幻尔协议实现（0x55 0x55帧头）。"""

    name = "lx"

    CMD_MOVE_TIME_WRITE = 0x01
    CMD_POS_READ = 0x1C

    @staticmethod
    def compute_checksum(data: Iterable[int]) -> int:
        """按协议计算校验和。"""
        return (~(sum(int(x) & 0xFF for x in data)) & 0xFF)

    def _build_packet(self, servo_id: int, cmd: int, params: Iterable[int]) -> bytes:
        sid = max(0, min(253, int(servo_id)))
        payload = [int(v) & 0xFF for v in params]
        length = 3 + len(payload)  # Length 字段包含 Length/Cmd/Params
        body = [sid, length, int(cmd) & 0xFF, *payload]
        checksum = self.compute_checksum(body)
        return bytes([0x55, 0x55, *body, checksum])

    def encode_move_command(self, servo_id: int, position: int, speed: int) -> bytes:
        # 幻尔位置控制: 0~1000 对应 0~240°
        position = max(0, min(1000, int(position)))
        speed = max(0, min(30000, int(speed)))
        params = [
            position & 0xFF,
            (position >> 8) & 0xFF,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
        ]
        return self._build_packet(servo_id, self.CMD_MOVE_TIME_WRITE, params)

    def encode_read_position_command(self, servo_id: int) -> bytes:
        return self._build_packet(servo_id, self.CMD_POS_READ, [])

    def decode_position_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        for sid, cmd, params in self._iter_valid_packets(data):
            if cmd != self.CMD_POS_READ or len(params) < 2:
                continue
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue

            raw = (params[1] << 8) | params[0]
            # 文档注明该值按 signed short 解析
            if raw >= 0x8000:
                raw -= 0x10000
            return raw
        return None

    def _iter_valid_packets(self, data: bytes):
        idx = 0
        n = len(data)
        while idx + 6 <= n:
            if data[idx] != 0x55 or data[idx + 1] != 0x55:
                idx += 1
                continue

            if idx + 4 > n:
                break
            length = int(data[idx + 3]) & 0xFF
            total = length + 3  # 帧总长度（含帧头到校验）
            if total < 6:
                idx += 1
                continue
            if idx + total > n:
                break

            frame = data[idx:idx + total]
            sid = frame[2]
            cmd = frame[4]
            params = frame[5:-1]
            checksum = frame[-1]
            expected = self.compute_checksum(frame[2:-1])

            if checksum == expected:
                yield sid, cmd, params

            idx += total
