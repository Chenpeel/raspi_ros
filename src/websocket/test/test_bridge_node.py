"""bridge_node WebSocket 总线舵机角度映射测试。"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("builtin_interfaces.msg")
pytest.importorskip("servo_msgs.msg")

from builtin_interfaces.msg import Time

from websocket_bridge.bridge_node import WebSocketROS2Bridge


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeLogger:
    def __init__(self):
        self.infos = []

    def info(self, message):
        self.infos.append(message)


class _FakeClock:
    def now(self):
        return self

    @staticmethod
    def to_msg():
        return Time(sec=1, nanosec=2)


def _build_bridge_stub():
    bridge = WebSocketROS2Bridge.__new__(WebSocketROS2Bridge)
    bridge.debug = False
    bridge.servo_command_pub = _FakePublisher()
    bridge._logger = _FakeLogger()
    bridge.get_logger = lambda: bridge._logger
    bridge.get_clock = lambda: _FakeClock()
    bridge._debug_log = lambda *args, **kwargs: None
    return bridge


@pytest.mark.parametrize(
    ("centered_angle", "servo_angle"),
    [
        (-90.0, 180),
        (-45.0, 135),
        (0.0, 90),
        (45.0, 45),
        (90.0, 0),
    ],
)
def test_map_centered_angle_to_servo_angle(centered_angle, servo_angle):
    assert (
        WebSocketROS2Bridge._map_centered_angle_to_servo_angle(centered_angle)
        == servo_angle
    )


@pytest.mark.parametrize(
    ("pulse", "centered_angle"),
    [
        (500, 90.0),
        (1500, 0.0),
        (2500, -90.0),
    ],
)
def test_map_pulse_to_centered_angle(pulse, centered_angle):
    assert (
        WebSocketROS2Bridge._map_pulse_to_centered_angle(pulse)
        == centered_angle
    )


@pytest.mark.asyncio
async def test_handle_bus_servo_command_shifts_centered_angle_and_logs():
    bridge = _build_bridge_stub()

    await bridge.handle_servo_command(
        {
            "servo_type": "bus",
            "servo_id": 36,
            "position": -30,
            "speed": 300,
        }
    )

    assert len(bridge.servo_command_pub.messages) == 1
    msg = bridge.servo_command_pub.messages[0]
    assert msg.servo_type == "bus"
    assert msg.servo_id == 36
    assert msg.position == 120
    assert msg.speed == 300
    assert bridge._logger.infos == [
        "收到WS总线舵机角度: ID=36 RAW_WS_ANGLE=-30.00deg -> SERVO_ANGLE=120"
    ]


@pytest.mark.asyncio
async def test_handle_bus_servo_command_rejects_out_of_range_angle():
    bridge = _build_bridge_stub()

    with pytest.raises(ValueError, match=r"bus position 超出范围\[-90, 90\]"):
        await bridge.handle_servo_command(
            {
                "servo_type": "bus",
                "servo_id": 36,
                "position": 91,
                "speed": 300,
            }
        )

    assert bridge.servo_command_pub.messages == []
