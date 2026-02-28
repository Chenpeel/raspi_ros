"""Isaac-ROS桥接工具函数。"""

from typing import Optional

BUS_MIN_PULSE = 500
BUS_MAX_PULSE = 2500
PCA_MIN_VALUE = 0
PCA_MAX_VALUE = 4095
DEFAULT_SPEED_MS = 100


def normalize_servo_type(servo_type: str) -> Optional[str]:
    """归一化舵机类型，仅允许 bus/pca。"""
    if not isinstance(servo_type, str):
        return None

    normalized = servo_type.strip().lower()
    if normalized in {"bus", "pca"}:
        return normalized
    return None


def clamp_servo_position(
    servo_type: str,
    position: int,
    enforce_limits: bool = True
) -> Optional[int]:
    """根据舵机类型清洗位置值。"""
    normalized_type = normalize_servo_type(servo_type)
    if normalized_type is None:
        return None

    try:
        value = int(position)
    except (TypeError, ValueError):
        return None

    if not enforce_limits:
        return value

    if normalized_type == "bus":
        return max(BUS_MIN_PULSE, min(BUS_MAX_PULSE, value))
    return max(PCA_MIN_VALUE, min(PCA_MAX_VALUE, value))


def normalize_speed(speed: int, default_speed: int = DEFAULT_SPEED_MS) -> int:
    """标准化速度，限定到 uint16 范围。"""
    try:
        value = int(speed)
    except (TypeError, ValueError):
        value = int(default_speed)

    if value <= 0:
        value = int(default_speed)

    return max(0, min(65535, value))
