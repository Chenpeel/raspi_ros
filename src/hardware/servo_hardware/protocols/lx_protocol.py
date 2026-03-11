"""幻尔总线舵机协议。

参考文档: docs/lx_bus_servo_protocol.pdf
"""

from typing import Iterable, Optional, Tuple

from .base import BusServoProtocol


class LXBusServoProtocol(BusServoProtocol):
    """幻尔协议实现（0x55 0x55 帧头，低字节在前）。"""

    name = "lx"

    # 写指令
    CMD_MOVE_TIME_WRITE = 0x01
    CMD_MOVE_TIME_WAIT_WRITE = 0x07
    CMD_MOVE_START = 0x0B
    CMD_MOVE_STOP = 0x0C
    CMD_ID_WRITE = 0x0D
    CMD_ANGLE_OFFSET_ADJUST = 0x11
    CMD_ANGLE_OFFSET_WRITE = 0x12
    CMD_ANGLE_LIMIT_WRITE = 0x14
    CMD_VIN_LIMIT_WRITE = 0x16
    CMD_TEMP_MAX_LIMIT_WRITE = 0x18
    CMD_OR_MOTOR_MODE_WRITE = 0x1D
    CMD_LOAD_OR_UNLOAD_WRITE = 0x1F
    # 兼容部分历史固件: 扭力开关使用 0x1E（与标准 OR_MOTOR_MODE_READ 冲突）
    # 仅用于写指令兜底，不用于读回包解析。
    CMD_TORQUE_SWITCH_COMPAT_WRITE = 0x1E
    CMD_LED_CTRL_WRITE = 0x21
    CMD_LED_ERROR_WRITE = 0x23

    # 读指令
    CMD_MOVE_TIME_READ = 0x02
    CMD_MOVE_TIME_WAIT_READ = 0x08
    CMD_ID_READ = 0x0E
    CMD_ANGLE_OFFSET_READ = 0x13
    CMD_ANGLE_LIMIT_READ = 0x15
    CMD_VIN_LIMIT_READ = 0x17
    CMD_TEMP_MAX_LIMIT_READ = 0x19
    CMD_TEMP_READ = 0x1A
    CMD_VIN_READ = 0x1B
    CMD_POS_READ = 0x1C
    CMD_OR_MOTOR_MODE_READ = 0x1E
    CMD_LOAD_OR_UNLOAD_READ = 0x20
    CMD_LED_CTRL_READ = 0x22
    CMD_LED_ERROR_READ = 0x24
    CMD_DIS_READ = 0x30

    @staticmethod
    def compute_checksum(data: Iterable[int]) -> int:
        """按协议计算校验和。"""
        return (~(sum(int(x) & 0xFF for x in data)) & 0xFF)

    @staticmethod
    def _u8(value: int) -> int:
        return int(value) & 0xFF

    @staticmethod
    def _split_u16(value: int) -> Tuple[int, int]:
        raw = int(value) & 0xFFFF
        return raw & 0xFF, (raw >> 8) & 0xFF

    @staticmethod
    def _join_u16(low: int, high: int) -> int:
        return ((int(high) & 0xFF) << 8) | (int(low) & 0xFF)

    @staticmethod
    def _u16_to_i16(value: int) -> int:
        raw = int(value) & 0xFFFF
        return raw - 0x10000 if raw >= 0x8000 else raw

    @staticmethod
    def _i16_to_u16(value: int) -> int:
        val = int(value)
        if val < -32768:
            val = -32768
        if val > 32767:
            val = 32767
        return val & 0xFFFF

    def _build_packet(self, servo_id: int, cmd: int, params: Iterable[int]) -> bytes:
        sid = max(0, min(253, int(servo_id)))
        payload = [self._u8(v) for v in params]
        # Length 字段包含自身 + Cmd + Params
        length = 3 + len(payload)
        body = [sid, length, int(cmd) & 0xFF, *payload]
        checksum = self.compute_checksum(body)
        return bytes([0x55, 0x55, *body, checksum])

    def _build_read_packet(self, servo_id: int, cmd: int) -> bytes:
        return self._build_packet(servo_id, cmd, [])

    def _first_params(
        self,
        data: bytes,
        cmd: int,
        expected_servo_id: Optional[int] = None,
        min_len: int = 0,
    ) -> Optional[Tuple[int, bytes]]:
        for sid, packet_cmd, params in self._iter_valid_packets(data):
            if packet_cmd != int(cmd):
                continue
            if expected_servo_id is not None and sid != int(expected_servo_id):
                continue
            if len(params) < int(min_len):
                continue
            return sid, params
        return None

    # ======= 兼容 BusServoProtocol 抽象接口 =======
    def encode_move_command(self, servo_id: int, position: int, speed: int) -> bytes:
        return self.encode_move_time_write(servo_id=servo_id, position=position, time_ms=speed)

    def encode_read_position_command(self, servo_id: int) -> bytes:
        return self.encode_pos_read(servo_id)

    def decode_position_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        return self.decode_pos_response(data=data, expected_servo_id=expected_servo_id)

    # ======= 写指令 =======
    def encode_move_time_write(self, servo_id: int, position: int, time_ms: int) -> bytes:
        position = max(0, min(1000, int(position)))
        time_ms = max(0, min(30000, int(time_ms)))
        pos_l, pos_h = self._split_u16(position)
        time_l, time_h = self._split_u16(time_ms)
        return self._build_packet(
            servo_id,
            self.CMD_MOVE_TIME_WRITE,
            [pos_l, pos_h, time_l, time_h],
        )

    def encode_move_time_wait_write(self, servo_id: int, position: int, time_ms: int) -> bytes:
        position = max(0, min(1000, int(position)))
        time_ms = max(0, min(30000, int(time_ms)))
        pos_l, pos_h = self._split_u16(position)
        time_l, time_h = self._split_u16(time_ms)
        return self._build_packet(
            servo_id,
            self.CMD_MOVE_TIME_WAIT_WRITE,
            [pos_l, pos_h, time_l, time_h],
        )

    def encode_move_start(self, servo_id: int) -> bytes:
        return self._build_packet(servo_id, self.CMD_MOVE_START, [])

    def encode_move_stop(self, servo_id: int) -> bytes:
        return self._build_packet(servo_id, self.CMD_MOVE_STOP, [])

    def encode_id_write(self, servo_id: int, new_id: int) -> bytes:
        new_id = max(0, min(253, int(new_id)))
        return self._build_packet(servo_id, self.CMD_ID_WRITE, [new_id])

    def encode_angle_offset_adjust(self, servo_id: int, offset: int) -> bytes:
        # signed char (-125~125) 传输时按 u8 补码
        offset = max(-125, min(125, int(offset)))
        return self._build_packet(servo_id, self.CMD_ANGLE_OFFSET_ADJUST, [offset & 0xFF])

    def encode_angle_offset_write(self, servo_id: int) -> bytes:
        return self._build_packet(servo_id, self.CMD_ANGLE_OFFSET_WRITE, [])

    def encode_angle_limit_write(self, servo_id: int, min_pos: int, max_pos: int) -> bytes:
        min_pos = max(0, min(1000, int(min_pos)))
        max_pos = max(0, min(1000, int(max_pos)))
        if min_pos > max_pos:
            min_pos, max_pos = max_pos, min_pos
        min_l, min_h = self._split_u16(min_pos)
        max_l, max_h = self._split_u16(max_pos)
        return self._build_packet(
            servo_id,
            self.CMD_ANGLE_LIMIT_WRITE,
            [min_l, min_h, max_l, max_h],
        )

    def encode_vin_limit_write(self, servo_id: int, min_mv: int, max_mv: int) -> bytes:
        min_mv = max(4500, min(14000, int(min_mv)))
        max_mv = max(4500, min(14000, int(max_mv)))
        if min_mv > max_mv:
            min_mv, max_mv = max_mv, min_mv
        min_l, min_h = self._split_u16(min_mv)
        max_l, max_h = self._split_u16(max_mv)
        return self._build_packet(
            servo_id,
            self.CMD_VIN_LIMIT_WRITE,
            [min_l, min_h, max_l, max_h],
        )

    def encode_temp_max_limit_write(self, servo_id: int, temp_c: int) -> bytes:
        temp_c = max(50, min(100, int(temp_c)))
        return self._build_packet(servo_id, self.CMD_TEMP_MAX_LIMIT_WRITE, [temp_c])

    def encode_or_motor_mode_write(
        self,
        servo_id: int,
        mode: int,
        drive_mode: int,
        speed: int,
    ) -> bytes:
        mode = 1 if int(mode) else 0
        drive_mode = 1 if int(drive_mode) else 0
        # 协议描述为 signed short
        sp_l, sp_h = self._split_u16(self._i16_to_u16(int(speed)))
        return self._build_packet(
            servo_id,
            self.CMD_OR_MOTOR_MODE_WRITE,
            [mode, drive_mode, sp_l, sp_h],
        )

    def encode_load_or_unload_write(self, servo_id: int, load: int) -> bytes:
        load = 1 if int(load) else 0
        return self._build_packet(servo_id, self.CMD_LOAD_OR_UNLOAD_WRITE, [load])

    def encode_torque_restore(self, servo_id: int) -> bytes:
        return self.encode_load_or_unload_write(servo_id=servo_id, load=1)

    def encode_torque_release(self, servo_id: int) -> bytes:
        return self.encode_load_or_unload_write(servo_id=servo_id, load=0)

    def encode_torque_switch_compat_write(self, servo_id: int, load: int) -> bytes:
        load = 1 if int(load) else 0
        return self._build_packet(servo_id, self.CMD_TORQUE_SWITCH_COMPAT_WRITE, [load])

    def encode_torque_restore_compat(self, servo_id: int) -> bytes:
        return self.encode_torque_switch_compat_write(servo_id=servo_id, load=1)

    def encode_torque_release_compat(self, servo_id: int) -> bytes:
        return self.encode_torque_switch_compat_write(servo_id=servo_id, load=0)

    def encode_led_ctrl_write(self, servo_id: int, led_off: int) -> bytes:
        led_off = 1 if int(led_off) else 0
        return self._build_packet(servo_id, self.CMD_LED_CTRL_WRITE, [led_off])

    def encode_led_error_write(self, servo_id: int, mask: int) -> bytes:
        mask = max(0, min(7, int(mask)))
        return self._build_packet(servo_id, self.CMD_LED_ERROR_WRITE, [mask])

    # ======= 读指令 =======
    def encode_move_time_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_MOVE_TIME_READ)

    def encode_move_time_wait_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_MOVE_TIME_WAIT_READ)

    def encode_id_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_ID_READ)

    def encode_angle_offset_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_ANGLE_OFFSET_READ)

    def encode_angle_limit_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_ANGLE_LIMIT_READ)

    def encode_vin_limit_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_VIN_LIMIT_READ)

    def encode_temp_max_limit_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_TEMP_MAX_LIMIT_READ)

    def encode_temp_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_TEMP_READ)

    def encode_vin_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_VIN_READ)

    def encode_pos_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_POS_READ)

    def encode_or_motor_mode_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_OR_MOTOR_MODE_READ)

    def encode_load_or_unload_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_LOAD_OR_UNLOAD_READ)

    def encode_led_ctrl_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_LED_CTRL_READ)

    def encode_led_error_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_LED_ERROR_READ)

    def encode_dis_read(self, servo_id: int) -> bytes:
        return self._build_read_packet(servo_id, self.CMD_DIS_READ)

    # ======= 读回包解码 =======
    def decode_move_time_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
        wait_mode: bool = False,
    ) -> Optional[Tuple[int, int]]:
        cmd = self.CMD_MOVE_TIME_WAIT_READ if wait_mode else self.CMD_MOVE_TIME_READ
        parsed = self._first_params(data, cmd, expected_servo_id, min_len=4)
        if parsed is None:
            return None
        _, params = parsed
        position = self._join_u16(params[0], params[1])
        time_ms = self._join_u16(params[2], params[3])
        return position, time_ms

    def decode_id_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_ID_READ, expected_servo_id, min_len=1)
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_angle_offset_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_ANGLE_OFFSET_READ, expected_servo_id, min_len=1)
        if parsed is None:
            return None
        _, params = parsed
        raw = int(params[0])
        return raw - 256 if raw >= 128 else raw

    def decode_angle_limit_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[Tuple[int, int]]:
        parsed = self._first_params(data, self.CMD_ANGLE_LIMIT_READ, expected_servo_id, min_len=4)
        if parsed is None:
            return None
        _, params = parsed
        return self._join_u16(params[0], params[1]), self._join_u16(params[2], params[3])

    def decode_vin_limit_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[Tuple[int, int]]:
        parsed = self._first_params(data, self.CMD_VIN_LIMIT_READ, expected_servo_id, min_len=4)
        if parsed is None:
            return None
        _, params = parsed
        return self._join_u16(params[0], params[1]), self._join_u16(params[2], params[3])

    def decode_temp_max_limit_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(
            data,
            self.CMD_TEMP_MAX_LIMIT_READ,
            expected_servo_id,
            min_len=1,
        )
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_temp_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_TEMP_READ, expected_servo_id, min_len=1)
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_vin_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_VIN_READ, expected_servo_id, min_len=2)
        if parsed is None:
            return None
        _, params = parsed
        return self._join_u16(params[0], params[1])

    def decode_pos_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_POS_READ, expected_servo_id, min_len=2)
        if parsed is None:
            return None
        _, params = parsed
        raw = self._join_u16(params[0], params[1])
        # 文档注明按 signed short 解析
        return self._u16_to_i16(raw)

    def decode_or_motor_mode_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[Tuple[int, int, int]]:
        parsed = self._first_params(data, self.CMD_OR_MOTOR_MODE_READ, expected_servo_id, min_len=4)
        if parsed is None:
            return None
        _, params = parsed
        mode = int(params[0])
        drive_mode = int(params[1])
        speed_raw = self._join_u16(params[2], params[3])
        speed = self._u16_to_i16(speed_raw)
        return mode, drive_mode, speed

    def decode_load_or_unload_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(
            data,
            self.CMD_LOAD_OR_UNLOAD_READ,
            expected_servo_id,
            min_len=1,
        )
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_led_ctrl_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_LED_CTRL_READ, expected_servo_id, min_len=1)
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_led_error_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_LED_ERROR_READ, expected_servo_id, min_len=1)
        if parsed is None:
            return None
        _, params = parsed
        return int(params[0])

    def decode_dis_response(
        self,
        data: bytes,
        expected_servo_id: Optional[int] = None,
    ) -> Optional[int]:
        parsed = self._first_params(data, self.CMD_DIS_READ, expected_servo_id, min_len=4)
        if parsed is None:
            return None
        _, params = parsed
        # 文档示例为 4 字节无符号距离计数（低字节在前）
        return (
            (int(params[0]) & 0xFF)
            | ((int(params[1]) & 0xFF) << 8)
            | ((int(params[2]) & 0xFF) << 16)
            | ((int(params[3]) & 0xFF) << 24)
        )

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
