"""bus_port_driver 偏移与限位计算单测。

说明：
- 这里只覆盖纯函数行为，不依赖真实串口与 ROS 节点运行。
- 如果运行环境缺少 rclpy/pyserial，会自动 skip。
"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("serial")

from servo_hardware.bus_port_driver import BusPortDriver


def test_apply_pulse_offset_and_limit_no_config_passthrough():
    assert BusPortDriver.apply_pulse_offset_and_limit(1500, 0.0, None) == 1500


def test_apply_pulse_offset_and_limit_clamps_base_range():
    assert BusPortDriver.apply_pulse_offset_and_limit(-10, 0.0, None) == 500
    assert BusPortDriver.apply_pulse_offset_and_limit(99999, 0.0, None) == 2500


def test_apply_pulse_offset_and_limit_positive_negative_offset():
    # 18deg -> 200 pulse (2000/180 * 18)
    assert BusPortDriver.apply_pulse_offset_and_limit(1500, 18.0, None) == 1700
    assert BusPortDriver.apply_pulse_offset_and_limit(1500, -18.0, None) == 1300


def test_apply_pulse_offset_and_limit_limit_swap_and_apply_before_offset():
    limit_entry = {"min": 2000, "max": 1000}  # 故意反转
    # base=900 -> 先按限位限幅到1000，再加 offset=0
    assert BusPortDriver.apply_pulse_offset_and_limit(900, 0.0, limit_entry) == 1000


def test_apply_pulse_offset_and_limit_limit_then_offset_order_matters():
    # 如果先 offset 再 limit：1600+200=1800 -> clamp 到 max=1500
    # 需求是先 limit 再 offset：min/max 限幅到1500，再 +200 -> 1700
    limit_entry = {"min": 500, "max": 1500}
    assert BusPortDriver.apply_pulse_offset_and_limit(1600, 18.0, limit_entry) == 1700


def test_apply_pulse_offset_and_limit_final_clamp_after_offset():
    assert BusPortDriver.apply_pulse_offset_and_limit(2400, 18.0, None) == 2500
    assert BusPortDriver.apply_pulse_offset_and_limit(500, -18.0, None) == 500

