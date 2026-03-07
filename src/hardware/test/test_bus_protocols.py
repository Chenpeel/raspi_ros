"""总线协议编解码单元测试。"""

from servo_hardware.protocols.lx_protocol import LXBusServoProtocol
from servo_hardware.protocols.zl_protocol import ZLBusServoProtocol


def test_zl_encode_move_command():
    proto = ZLBusServoProtocol()
    frame = proto.encode_move_command(servo_id=7, position=1500, speed=100)
    assert frame == b"#007P1500T0100!"


def test_zl_decode_read_response():
    proto = ZLBusServoProtocol()
    raw = b"noise#007P1623!tail"
    assert proto.decode_position_response(raw, expected_servo_id=7) == 1623
    assert proto.decode_position_response(raw, expected_servo_id=8) is None


def test_zl_encode_full_commands_sample():
    proto = ZLBusServoProtocol()
    assert proto.encode_version_read(servo_id=1) == b"#001PVER!"
    assert proto.encode_id_write(servo_id=1, new_id=10) == b"#001PID010!"
    assert proto.encode_mode_write(servo_id=1, mode=3) == b"#001PMOD3!"


def test_zl_decode_temp_voltage_response():
    proto = ZLBusServoProtocol()
    raw = b"abc#001T28.1V7.4!xyz"
    assert proto.decode_temp_voltage_response(raw, expected_servo_id=1) == (28.1, 7.4)


def test_lx_encode_read_command():
    proto = LXBusServoProtocol()
    frame = proto.encode_read_position_command(servo_id=1)
    assert frame[:5] == bytes([0x55, 0x55, 0x01, 0x03, 0x1C])
    assert frame[-1] == LXBusServoProtocol.compute_checksum(frame[2:-1])


def test_lx_decode_position_response():
    proto = LXBusServoProtocol()
    sid = 0x01
    length = 0x05
    cmd = 0x1C
    # 返回位置 500 (0x01F4), 低字节在前
    params = [0xF4, 0x01]
    checksum = LXBusServoProtocol.compute_checksum([sid, length, cmd, *params])
    raw = bytes([0x55, 0x55, sid, length, cmd, *params, checksum])
    assert proto.decode_position_response(raw, expected_servo_id=1) == 500


def test_lx_encode_move_command_matches_doc_example():
    proto = LXBusServoProtocol()
    # 文档示例：55 55 01 07 01 F4 01 E8 03 16
    frame = proto.encode_move_command(servo_id=1, position=500, speed=1000)
    assert frame == bytes([0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03, 0x16])


def test_lx_decode_signed_position_response():
    proto = LXBusServoProtocol()
    sid = 0x01
    length = 0x05
    cmd = LXBusServoProtocol.CMD_POS_READ
    # 返回位置 -10，对应 0xFFF6，低字节在前
    params = [0xF6, 0xFF]
    checksum = LXBusServoProtocol.compute_checksum([sid, length, cmd, *params])
    raw = bytes([0x55, 0x55, sid, length, cmd, *params, checksum])
    assert proto.decode_pos_response(raw, expected_servo_id=1) == -10


def test_lx_encode_angle_limit_write():
    proto = LXBusServoProtocol()
    frame = proto.encode_angle_limit_write(servo_id=1, min_pos=200, max_pos=800)
    # 55 55 01 07 14 C8 00 20 03 crc
    assert frame[:9] == bytes([0x55, 0x55, 0x01, 0x07, 0x14, 0xC8, 0x00, 0x20, 0x03])
    assert frame[-1] == LXBusServoProtocol.compute_checksum(frame[2:-1])


def test_lx_encode_or_motor_mode_write_with_negative_speed():
    proto = LXBusServoProtocol()
    frame = proto.encode_or_motor_mode_write(
        servo_id=1,
        mode=1,
        drive_mode=1,
        speed=-1000,
    )
    # -1000 的16位补码是 0xFC18，低字节在前
    assert frame[:9] == bytes([0x55, 0x55, 0x01, 0x07, 0x1D, 0x01, 0x01, 0x18, 0xFC])
    assert frame[-1] == LXBusServoProtocol.compute_checksum(frame[2:-1])


def test_lx_decode_dis_read_response():
    proto = LXBusServoProtocol()
    # 文档示例：55 55 01 07 30 31 24 01 00 71
    raw = bytes([0x55, 0x55, 0x01, 0x07, 0x30, 0x31, 0x24, 0x01, 0x00, 0x71])
    assert proto.decode_dis_response(raw, expected_servo_id=1) == 74801


def test_lx_encode_torque_switch_standard():
    proto = LXBusServoProtocol()
    frame_restore = proto.encode_torque_restore(servo_id=1)
    frame_release = proto.encode_torque_release(servo_id=1)
    # 标准扭力开关: 0x1F
    assert frame_restore[:6] == bytes([0x55, 0x55, 0x01, 0x04, 0x1F, 0x01])
    assert frame_release[:6] == bytes([0x55, 0x55, 0x01, 0x04, 0x1F, 0x00])
    assert frame_restore[-1] == LXBusServoProtocol.compute_checksum(frame_restore[2:-1])
    assert frame_release[-1] == LXBusServoProtocol.compute_checksum(frame_release[2:-1])


def test_lx_encode_torque_switch_compat():
    proto = LXBusServoProtocol()
    frame_restore = proto.encode_torque_restore_compat(servo_id=1)
    frame_release = proto.encode_torque_release_compat(servo_id=1)
    # 兼容扭力开关: 0x1E
    assert frame_restore[:6] == bytes([0x55, 0x55, 0x01, 0x04, 0x1E, 0x01])
    assert frame_release[:6] == bytes([0x55, 0x55, 0x01, 0x04, 0x1E, 0x00])
    assert frame_restore[-1] == LXBusServoProtocol.compute_checksum(frame_restore[2:-1])
    assert frame_release[-1] == LXBusServoProtocol.compute_checksum(frame_release[2:-1])
