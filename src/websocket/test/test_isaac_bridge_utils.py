"""isaac_bridge_utils 单元测试。"""

import importlib.util
from pathlib import Path

try:
    from websocket_bridge.isaac_bridge_utils import clamp_servo_position
    from websocket_bridge.isaac_bridge_utils import normalize_servo_type
    from websocket_bridge.isaac_bridge_utils import normalize_speed
except ModuleNotFoundError:
    module_path = Path(__file__).resolve().parents[1] / 'websocket_bridge' / 'isaac_bridge_utils.py'
    spec = importlib.util.spec_from_file_location('isaac_bridge_utils', module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    clamp_servo_position = module.clamp_servo_position
    normalize_servo_type = module.normalize_servo_type
    normalize_speed = module.normalize_speed


def test_normalize_servo_type():
    """舵机类型归一化。"""
    assert normalize_servo_type('bus') == 'bus'
    assert normalize_servo_type('  PCA ') == 'pca'
    assert normalize_servo_type('invalid') is None
    assert normalize_servo_type(None) is None


def test_clamp_bus_position():
    """总线舵机位置限幅。"""
    assert clamp_servo_position('bus', 400) == 500
    assert clamp_servo_position('bus', 1500) == 1500
    assert clamp_servo_position('bus', 3000) == 2500


def test_clamp_pca_position():
    """PCA舵机位置限幅。"""
    assert clamp_servo_position('pca', -10) == 0
    assert clamp_servo_position('pca', 1024) == 1024
    assert clamp_servo_position('pca', 5000) == 4095


def test_position_without_limits():
    """关闭限幅后保留原值。"""
    assert clamp_servo_position('bus', 3000, enforce_limits=False) == 3000
    assert clamp_servo_position('pca', -10, enforce_limits=False) == -10


def test_invalid_position():
    """非法位置应返回None。"""
    assert clamp_servo_position('bus', 'x') is None
    assert clamp_servo_position('unknown', 1000) is None


def test_normalize_speed():
    """速度清洗。"""
    assert normalize_speed(100, 120) == 100
    assert normalize_speed(0, 120) == 120
    assert normalize_speed(-1, 120) == 120
    assert normalize_speed('abc', 120) == 120
    assert normalize_speed(999999, 120) == 65535
