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
