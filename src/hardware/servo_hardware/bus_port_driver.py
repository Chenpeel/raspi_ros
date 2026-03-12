"""多协议总线舵机串口驱动节点。

职责：
1. 独占一个串口端口；
2. 接收上层路由后的协议命令（众灵/幻尔）并发送；
3. 提供读角度服务接口，返回统一脉宽语义（500~2500us）；
4. 提供通用命令服务接口，暴露协议层全部已实现指令。
"""

import json
import os
import threading
import time
from inspect import Parameter, signature
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set

import rclpy
import serial
from rclpy.node import Node
from servo_msgs.msg import ServoCommand, ServoState
from servo_msgs.srv import ExecuteBusCommand, ReadServoPosition

from .protocols import BusServoProtocol, LXBusServoProtocol, ZLBusServoProtocol

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for non-ROS envs
    get_package_share_directory = None


class BusPortDriver(Node):
    """多协议总线舵机串口驱动。"""

    def __init__(self):
        super().__init__("bus_port_driver")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("read_timeout", 0.15)
        self.declare_parameter("default_speed", 100)
        self.declare_parameter("debug", False)
        self.declare_parameter("log_id", True)
        self.declare_parameter("zl_servo_ids", [0])
        self.declare_parameter("lx_servo_ids", [0])
        self.declare_parameter("offset_map", "")  # 舵机偏移配置文件(单位: deg)
        self.declare_parameter("limit_map", "")  # 舵机限位配置文件(单位: pulse/us)
        self.declare_parameter("config_reload_interval_sec", 1.0)  # 运行时自动重载配置文件

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.read_timeout = float(self.get_parameter("read_timeout").value)
        self.default_speed = int(self.get_parameter("default_speed").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.log_id = bool(self.get_parameter("log_id").value)
        offset_map_param = str(self.get_parameter("offset_map").value or "").strip()
        limit_map_param = str(self.get_parameter("limit_map").value or "").strip()
        self.config_reload_interval_sec = float(
            self.get_parameter("config_reload_interval_sec").value
        )

        zl_ids = self.get_parameter("zl_servo_ids").value
        lx_ids = self.get_parameter("lx_servo_ids").value
        self.zl_servo_ids: Set[int] = {int(x) for x in zl_ids if int(x) > 0}
        self.lx_servo_ids: Set[int] = {int(x) for x in lx_ids if int(x) > 0}

        # 偏移量与限位配置
        self.offset_map_path = ""
        self.limit_map_path = ""
        self._offset_map_mtime = 0.0
        self._limit_map_mtime = 0.0
        self.offset_map_deg = self._load_offset_map(override_path=offset_map_param)
        self.limit_map = self._load_limit_map(override_path=limit_map_param)
        self._offset_map_mtime = self._safe_get_mtime(self.offset_map_path)
        self._limit_map_mtime = self._safe_get_mtime(self.limit_map_path)

        # 用于支持“校准工具写入json后无需重启节点即可生效”
        interval = float(self.config_reload_interval_sec)
        if interval > 0:
            self._reload_timer = self.create_timer(
                max(0.2, interval),
                self._maybe_reload_configs,
            )

        self.protocols: Dict[str, BusServoProtocol] = {
            "zl": ZLBusServoProtocol(),
            "lx": LXBusServoProtocol(),
        }
        self.command_specs = self._build_command_specs()
        self.command_aliases = self._build_command_aliases()

        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        if not self._init_serial():
            self.get_logger().error(f"串口初始化失败: {self.port}")

        self.state_pub = self.create_publisher(ServoState, "~/state", 10)
        self.zl_sub = self.create_subscription(
            ServoCommand,
            "~/command_zl",
            self._on_zl_command,
            10,
        )
        self.lx_sub = self.create_subscription(
            ServoCommand,
            "~/command_lx",
            self._on_lx_command,
            10,
        )
        self.read_position_srv = self.create_service(
            ReadServoPosition,
            "~/read_position",
            self._handle_read_position,
        )
        self.execute_command_srv = self.create_service(
            ExecuteBusCommand,
            "~/execute_command",
            self._handle_execute_command,
        )

        self.get_logger().info(
            "bus_port_driver 已启动: "
            f"{self.port}@{self.baudrate} "
            f"(zl_ids={sorted(self.zl_servo_ids) or 'ALL'}, "
            f"lx_ids={sorted(self.lx_servo_ids) or 'ALL'})"
        )

    @staticmethod
    def _deg_to_pulse_delta(offset_deg: float) -> float:
        """将角度偏移(deg)转换为统一脉宽增量(500~2500域的 delta)。"""
        try:
            return float(offset_deg) * 2000.0 / 180.0
        except Exception:
            return 0.0

    @staticmethod
    def _pulse_delta_to_deg(offset_pulse: float) -> float:
        """将统一脉宽增量转换为角度偏移(deg)。"""
        try:
            return float(offset_pulse) * 180.0 / 2000.0
        except Exception:
            return 0.0

    @staticmethod
    def _clamp_int(value: int, min_val: int, max_val: int) -> int:
        return max(int(min_val), min(int(max_val), int(value)))

    @staticmethod
    def _safe_get_mtime(path: str) -> float:
        path = str(path or "").strip()
        if not path:
            return 0.0
        try:
            return float(os.path.getmtime(path))
        except Exception:
            return 0.0

    def _maybe_reload_configs(self):
        """当 offset/limit 配置文件发生变化时自动重载。"""
        try:
            if self.offset_map_path:
                mtime = self._safe_get_mtime(self.offset_map_path)
                if mtime and mtime != self._offset_map_mtime:
                    self.offset_map_deg = self._load_offset_map(
                        override_path=self.offset_map_path
                    )
                    self._offset_map_mtime = mtime
                    self.get_logger().info(f"已重载 offset_map: {self.offset_map_path}")

            if self.limit_map_path:
                mtime = self._safe_get_mtime(self.limit_map_path)
                if mtime and mtime != self._limit_map_mtime:
                    self.limit_map = self._load_limit_map(override_path=self.limit_map_path)
                    self._limit_map_mtime = mtime
                    self.get_logger().info(f"已重载 limit_map: {self.limit_map_path}")
        except Exception as exc:
            self.get_logger().warn(f"配置重载失败: {exc}")

    @staticmethod
    def apply_pulse_offset_and_limit(
        base_pulse: int,
        offset_deg: float,
        limit_entry: Optional[Dict[str, Any]] = None,
    ) -> int:
        """对统一脉宽应用限位与偏移补偿，返回最终下发脉宽。

        执行顺序（与需求保持一致）：
        1) base_pulse 归一化到 [500, 2500]
        2) 按 limit_entry 对“期望脉宽”限幅（仍在统一脉宽域）
        3) 加上 offset_deg 对应的脉宽增量
        4) 最终 clamp 到 [500, 2500]
        """
        try:
            pulse = int(base_pulse)
        except Exception:
            pulse = 1500
        pulse = BusPortDriver._clamp_int(pulse, 500, 2500)

        # 先对期望值做限位（不包含 offset）
        if isinstance(limit_entry, dict) and limit_entry:
            min_val = None
            max_val = None
            raw_min = limit_entry.get("min")
            raw_max = limit_entry.get("max")
            try:
                if raw_min is not None:
                    min_val = int(round(float(raw_min)))
            except Exception:
                min_val = None
            try:
                if raw_max is not None:
                    max_val = int(round(float(raw_max)))
            except Exception:
                max_val = None

            if min_val is not None:
                min_val = BusPortDriver._clamp_int(min_val, 500, 2500)
            if max_val is not None:
                max_val = BusPortDriver._clamp_int(max_val, 500, 2500)
            if min_val is not None and max_val is not None and min_val > max_val:
                min_val, max_val = max_val, min_val

            if min_val is not None and pulse < min_val:
                pulse = min_val
            if max_val is not None and pulse > max_val:
                pulse = max_val

        # 再应用 offset（单位deg -> 统一脉宽增量）
        pulse_delta = int(round(BusPortDriver._deg_to_pulse_delta(offset_deg)))
        pulse = int(pulse) + int(pulse_delta)
        return BusPortDriver._clamp_int(pulse, 500, 2500)

    def _resolve_limit_map_path(self, override_path: str) -> str:
        if override_path and os.path.exists(override_path):
            return override_path

        if get_package_share_directory:
            try:
                share_dir = get_package_share_directory("servo_hardware")
                candidate = os.path.join(share_dir, "config", "servo_limit_map.json")
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass

        candidate = Path(__file__).resolve().parent / "config" / "servo_limit_map.json"
        if candidate.exists():
            return str(candidate)
        return ""

    def _load_limit_map(self, override_path: str) -> Dict[int, Dict[str, int]]:
        path = self._resolve_limit_map_path(override_path)
        self.limit_map_path = path
        if not path:
            return {}

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"加载舵机限位配置失败: {exc}")
            return {}

        if isinstance(data, dict) and "servo_limits" in data:
            data = data.get("servo_limits") or {}

        limit_map: Dict[int, Dict[str, int]] = {}
        if isinstance(data, dict) and "ids" in data:
            ids = data.get("ids") or []
            mins = data.get("mins") or []
            maxs = data.get("maxs") or []
            for idx, sid in enumerate(ids):
                try:
                    sid_int = int(sid)
                except (TypeError, ValueError):
                    continue
                min_val = mins[idx] if idx < len(mins) else None
                max_val = maxs[idx] if idx < len(maxs) else None
                limit_map[sid_int] = {"min": min_val, "max": max_val}
        elif isinstance(data, dict):
            for key, value in data.items():
                try:
                    sid_int = int(key)
                except (TypeError, ValueError):
                    continue
                if isinstance(value, dict):
                    limit_map[sid_int] = {"min": value.get("min"), "max": value.get("max")}

        if limit_map:
            self.get_logger().info(f"已加载舵机限位配置: {path} (数量={len(limit_map)})")
        return limit_map

    def _resolve_offset_map_path(self, override_path: str) -> str:
        if override_path and os.path.exists(override_path):
            return override_path

        # 偏移量文件默认不做隐式回退，避免误用其他驱动的 offset 配置造成危险动作。
        return ""

    def _load_offset_map(self, override_path: str) -> Dict[int, float]:
        path = self._resolve_offset_map_path(override_path)
        self.offset_map_path = path
        if not path:
            if override_path:
                self.get_logger().warn(f"未找到舵机偏移配置文件: {override_path}")
            return {}

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"加载舵机偏移量失败: {exc}")
            return {}

        unit = "deg"
        offsets_obj = data
        if isinstance(data, dict) and "offsets" in data:
            offsets_obj = data.get("offsets") or {}
            unit = str(data.get("unit") or unit).strip().lower()

        unit = unit.replace("degree", "deg").replace("degrees", "deg")
        if unit in ("us", "pulse", "pwm", "ticks"):
            unit = "pulse"
        elif unit not in ("deg", "pulse"):
            self.get_logger().warn(f"未知 offset unit={unit}，将按deg解析")
            unit = "deg"

        offset_map_deg: Dict[int, float] = {}
        if isinstance(offsets_obj, dict):
            for key, value in offsets_obj.items():
                try:
                    sid = int(key)
                    raw_val = float(value)
                except (TypeError, ValueError):
                    continue
                if unit == "pulse":
                    offset_map_deg[sid] = self._pulse_delta_to_deg(raw_val)
                else:
                    offset_map_deg[sid] = raw_val

        if offset_map_deg:
            self.get_logger().info(
                f"已加载舵机偏移量配置: {path} (数量={len(offset_map_deg)}, unit={unit})"
            )
        else:
            self.get_logger().info(f"舵机偏移量为空: {path}")
        return offset_map_deg

    @staticmethod
    def _build_command_specs() -> Dict[str, Dict[str, Dict[str, Any]]]:
        return {
            "lx": {
                "move_command": {"encode": "encode_move_command"},
                "read_position_command": {
                    "encode": "encode_read_position_command",
                    "decode": "decode_position_response",
                },
                "move_time_write": {"encode": "encode_move_time_write"},
                "move_time_read": {
                    "encode": "encode_move_time_read",
                    "decode": "decode_move_time_response",
                },
                "move_time_wait_write": {"encode": "encode_move_time_wait_write"},
                "move_time_wait_read": {
                    "encode": "encode_move_time_wait_read",
                    "decode": "decode_move_time_response",
                    "decode_kwargs": {"wait_mode": True},
                },
                "move_start": {"encode": "encode_move_start"},
                "move_stop": {"encode": "encode_move_stop"},
                "id_write": {"encode": "encode_id_write"},
                "id_read": {"encode": "encode_id_read", "decode": "decode_id_response"},
                "angle_offset_adjust": {"encode": "encode_angle_offset_adjust"},
                "angle_offset_write": {"encode": "encode_angle_offset_write"},
                "angle_offset_read": {
                    "encode": "encode_angle_offset_read",
                    "decode": "decode_angle_offset_response",
                },
                "angle_limit_write": {"encode": "encode_angle_limit_write"},
                "angle_limit_read": {
                    "encode": "encode_angle_limit_read",
                    "decode": "decode_angle_limit_response",
                },
                "vin_limit_write": {"encode": "encode_vin_limit_write"},
                "vin_limit_read": {
                    "encode": "encode_vin_limit_read",
                    "decode": "decode_vin_limit_response",
                },
                "temp_max_limit_write": {"encode": "encode_temp_max_limit_write"},
                "temp_max_limit_read": {
                    "encode": "encode_temp_max_limit_read",
                    "decode": "decode_temp_max_limit_response",
                },
                "temp_read": {"encode": "encode_temp_read", "decode": "decode_temp_response"},
                "vin_read": {"encode": "encode_vin_read", "decode": "decode_vin_response"},
                "pos_read": {"encode": "encode_pos_read", "decode": "decode_pos_response"},
                "or_motor_mode_write": {"encode": "encode_or_motor_mode_write"},
                "or_motor_mode_read": {
                    "encode": "encode_or_motor_mode_read",
                    "decode": "decode_or_motor_mode_response",
                },
                "load_or_unload_write": {"encode": "encode_load_or_unload_write"},
                "torque_restore": {"encode": "encode_torque_restore"},
                "torque_release": {"encode": "encode_torque_release"},
                "torque_switch_compat_write": {"encode": "encode_torque_switch_compat_write"},
                "torque_restore_compat": {"encode": "encode_torque_restore_compat"},
                "torque_release_compat": {"encode": "encode_torque_release_compat"},
                "load_or_unload_read": {
                    "encode": "encode_load_or_unload_read",
                    "decode": "decode_load_or_unload_response",
                },
                "led_ctrl_write": {"encode": "encode_led_ctrl_write"},
                "led_ctrl_read": {
                    "encode": "encode_led_ctrl_read",
                    "decode": "decode_led_ctrl_response",
                },
                "led_error_write": {"encode": "encode_led_error_write"},
                "led_error_read": {
                    "encode": "encode_led_error_read",
                    "decode": "decode_led_error_response",
                },
                "dis_read": {"encode": "encode_dis_read", "decode": "decode_dis_response"},
            },
            "zl": {
                "move_command": {"encode": "encode_move_command"},
                "read_position_command": {
                    "encode": "encode_read_position_command",
                    "decode": "decode_position_response",
                },
                "version_read": {
                    "encode": "encode_version_read",
                    "decode": "decode_version_response",
                },
                "id_read": {"encode": "encode_id_read", "decode": "decode_id_response"},
                "id_write": {"encode": "encode_id_write"},
                "torque_release": {"encode": "encode_torque_release"},
                "torque_restore": {"encode": "encode_torque_restore"},
                "mode_read": {"encode": "encode_mode_read", "decode": "decode_mode_response"},
                "mode_write": {"encode": "encode_mode_write"},
                "position_read": {
                    "encode": "encode_position_read",
                    "decode": "decode_position_response",
                },
                "motion_pause": {"encode": "encode_motion_pause"},
                "motion_continue": {"encode": "encode_motion_continue"},
                "motion_stop": {"encode": "encode_motion_stop"},
                "baudrate_write": {"encode": "encode_baudrate_write"},
                "middle_calibrate": {"encode": "encode_middle_calibrate"},
                "startup_position_set": {"encode": "encode_startup_position_set"},
                "startup_position_disable": {"encode": "encode_startup_position_disable"},
                "startup_position_restore": {"encode": "encode_startup_position_restore"},
                "min_position_set": {"encode": "encode_min_position_set"},
                "max_position_set": {"encode": "encode_max_position_set"},
                "factory_reset": {"encode": "encode_factory_reset"},
                "temp_voltage_read": {
                    "encode": "encode_temp_voltage_read",
                    "decode": "decode_temp_voltage_response",
                },
            },
        }

    @staticmethod
    def _build_command_aliases() -> Dict[str, Dict[str, str]]:
        return {
            "lx": {
                "move": "move_command",
                "read_position": "read_position_command",
                "position_read": "pos_read",
                "load_write": "load_or_unload_write",
                "load_read": "load_or_unload_read",
                "restore_torque": "torque_restore",
                "release_torque": "torque_release",
                "restore_torque_compat": "torque_restore_compat",
                "release_torque_compat": "torque_release_compat",
                "torque_switch_compat": "torque_switch_compat_write",
            },
            "zl": {
                "move": "move_command",
                "send_position": "move_command",
                "read_position": "read_position_command",
                "get_version": "version_read",
                "read_id": "id_read",
                "set_id": "id_write",
                "release_torque": "torque_release",
                "restore_torque": "torque_restore",
                "read_mode": "mode_read",
                "set_mode": "mode_write",
                "pause_motion": "motion_pause",
                "continue_motion": "motion_continue",
                "stop_motion": "motion_stop",
                "set_baudrate": "baudrate_write",
                "calibrate_middle": "middle_calibrate",
                "set_startup_position": "startup_position_set",
                "disable_startup_position": "startup_position_disable",
                "restore_startup_position": "startup_position_restore",
                "set_min_position": "min_position_set",
                "set_max_position": "max_position_set",
                "read_temp_voltage": "temp_voltage_read",
            },
        }

    def _init_serial(self) -> bool:
        try:
            if self.ser is None:
                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout,
                )
            elif not self.ser.is_open:
                self.ser.open()
            return True
        except Exception as exc:
            self.get_logger().error(f"打开串口失败: {exc}")
            return False

    def _is_bus_command(self, msg: ServoCommand) -> bool:
        servo_type = str(msg.servo_type or "").strip().lower()
        return servo_type in ("", "bus", "bus_servo", "bus_zl", "bus_lx", "zl", "lx")

    def _is_allowed_id(self, protocol: str, servo_id: int) -> bool:
        if protocol == "zl":
            return (not self.zl_servo_ids) or (servo_id in self.zl_servo_ids)
        if protocol == "lx":
            return (not self.lx_servo_ids) or (servo_id in self.lx_servo_ids)
        return False

    def _position_to_pulse(self, position: int) -> int:
        pos = int(position)
        if 0 <= pos <= 180:
            return int(round(500 + (pos / 180.0) * 2000.0))
        if 500 <= pos <= 2500:
            return pos
        if 0 <= pos <= 1000:
            # 兼容将幻尔单位值直接传入的场景
            return int(round(500 + (pos / 1000.0) * 2000.0))

        if self.debug:
            self.get_logger().warn(f"非常规位置值: {pos}，将限幅到[500,2500]")
        return max(500, min(2500, pos))

    @staticmethod
    def _pulse_to_lx_unit(pulse: int) -> int:
        """将统一脉宽(500~2500)映射到 LX 受控区间(125~875)。"""
        pulse = max(500, min(2500, int(pulse)))
        return int(round(125 + (pulse - 500) * 750.0 / 2000.0))

    @staticmethod
    def _lx_unit_to_pulse(unit: int) -> int:
        """将 LX 受控区间(125~875)映射回统一脉宽(500~2500)。"""
        unit = max(125, min(875, int(unit)))
        return int(round(500 + (unit - 125) * 2000.0 / 750.0))

    @staticmethod
    def _pulse_to_zl_position(pulse: int) -> int:
        """将统一脉宽(500~2500)映射到 ZL 受控区间(833~2167)。"""
        pulse = max(500, min(2500, int(pulse)))
        return int(round(833 + (pulse - 500) * 1334.0 / 2000.0))

    @staticmethod
    def _zl_position_to_pulse(position: int) -> int:
        """将 ZL 受控区间(833~2167)映射回统一脉宽(500~2500)。"""
        position = max(833, min(2167, int(position)))
        return int(round(500 + (position - 833) * 2000.0 / 1334.0))

    def _normalize_move_target(self, protocol: str, position: int) -> tuple[int, int]:
        pos = int(position)
        if protocol == "lx":
            # 统一入口:
            # - WebSocket 路径会将中心角[-90, 90]平移为[0, 180]
            # - 其他生产者仍可直接传统一脉宽500~2500或协议原生位置值
            if 500 <= pos <= 2500:
                pulse = pos
                return self._pulse_to_lx_unit(pulse), pulse
            if 0 <= pos <= 180:
                pulse = int(round(500 + (pos / 180.0) * 2000.0))
                return self._pulse_to_lx_unit(pulse), pulse
            if 125 <= pos <= 875:
                unit = pos
                return unit, self._lx_unit_to_pulse(unit)

            clamped_unit = max(125, min(875, pos))
            if self.debug:
                self.get_logger().warn(
                    f"LX非常规位置值: {pos}，将按LX单位限幅到[125,875]"
                )
            return clamped_unit, self._lx_unit_to_pulse(clamped_unit)

        if protocol == "zl":
            # 统一入口:
            # - WebSocket 路径会将中心角[-90, 90]平移为[0, 180]
            # - 其他生产者仍可直接传统一脉宽500~2500或协议原生位置值
            if 500 <= pos <= 2500:
                pulse = pos
                return self._pulse_to_zl_position(pulse), pulse
            if 0 <= pos <= 180:
                pulse = int(round(500 + (pos / 180.0) * 2000.0))
                return self._pulse_to_zl_position(pulse), pulse
            if 833 <= pos <= 2167:
                zl_pos = pos
                return zl_pos, self._zl_position_to_pulse(zl_pos)

            clamped_zl = max(833, min(2167, pos))
            if self.debug:
                self.get_logger().warn(
                    f"ZL非常规位置值: {pos}，将按ZL位置限幅到[833,2167]"
                )
            return clamped_zl, self._zl_position_to_pulse(clamped_zl)

        pulse = self._position_to_pulse(pos)
        return pulse, pulse

    def _publish_state(self, servo_id: int, pulse: int, error_code: int):
        msg = ServoState()
        msg.servo_type = "bus"
        msg.servo_id = int(servo_id)
        msg.position = int(max(0, min(65535, pulse)))
        msg.load = -1
        msg.temperature = -1
        msg.error_code = int(max(0, min(255, error_code)))
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def _write_frame(self, frame: bytes) -> bool:
        if not self._init_serial():
            return False
        with self.lock:
            try:
                self.ser.write(frame)
                if self.debug:
                    self.get_logger().debug(f"发送字节: {frame.hex(' ')}")
                return True
            except Exception as exc:
                self.get_logger().error(f"串口写入失败: {exc}")
                return False

    def _execute_move(self, protocol: str, servo_id: int, position: int, speed: int):
        if not self._is_allowed_id(protocol, servo_id):
            if self.debug:
                self.get_logger().debug(
                    f"忽略ID={servo_id}的{protocol}命令（不在本节点负责范围）"
                )
            return

        _proto_pos, base_pulse = self._normalize_move_target(protocol, position)
        offset_deg = float(self.offset_map_deg.get(int(servo_id), 0.0) or 0.0)
        limit_entry = self.limit_map.get(int(servo_id))
        target_pulse = self.apply_pulse_offset_and_limit(
            base_pulse=base_pulse,
            offset_deg=0.0,
            limit_entry=limit_entry,
        )
        adjusted_pulse = self.apply_pulse_offset_and_limit(
            base_pulse=base_pulse,
            offset_deg=offset_deg,
            limit_entry=limit_entry,
        )
        if protocol == "lx":
            proto_pos = self._pulse_to_lx_unit(adjusted_pulse)
        else:
            proto_pos = self._pulse_to_zl_position(adjusted_pulse)
        speed = max(0, min(30000, int(speed)))

        frame = self.protocols[protocol].encode_move_command(
            servo_id=servo_id,
            position=proto_pos,
            speed=speed,
        )
        success = self._write_frame(frame)
        # 注意：状态保持上层期望的 pulse（限幅后），不包含 offset。
        self._publish_state(
            servo_id=servo_id,
            pulse=target_pulse,
            error_code=0 if success else 1,
        )

        if self.log_id:
            self.get_logger().info(
                f"[Send/{protocol}] ID={servo_id} POS={proto_pos} SPEED={speed}"
            )

    def _on_zl_command(self, msg: ServoCommand):
        try:
            if not self._is_bus_command(msg):
                return
            speed = msg.speed if msg.speed > 0 else self.default_speed
            self._execute_move(
                protocol="zl",
                servo_id=int(msg.servo_id),
                position=int(msg.position),
                speed=int(speed),
            )
        except Exception as exc:
            self.get_logger().error(f"处理众灵命令失败: {exc}")

    def _on_lx_command(self, msg: ServoCommand):
        try:
            if not self._is_bus_command(msg):
                return
            speed = msg.speed if msg.speed > 0 else self.default_speed
            self._execute_move(
                protocol="lx",
                servo_id=int(msg.servo_id),
                position=int(msg.position),
                speed=int(speed),
            )
        except Exception as exc:
            self.get_logger().error(f"处理幻尔命令失败: {exc}")

    @staticmethod
    def _normalize_command_name(command: str) -> str:
        text = str(command or "").strip().lower()
        text = text.replace("-", "_").replace(" ", "_")
        if text.startswith("encode_"):
            text = text[len("encode_"):]
        if text.startswith("servo_"):
            text = text[len("servo_"):]
        return text

    def _resolve_command(self, protocol: str, command: str) -> Optional[Dict[str, Any]]:
        proto_specs = self.command_specs.get(protocol, {})
        cmd_key = self._normalize_command_name(command)
        alias = self.command_aliases.get(protocol, {}).get(cmd_key)
        if alias:
            cmd_key = alias
        spec = proto_specs.get(cmd_key)
        if spec is None:
            return None
        return {"name": cmd_key, **spec}

    @staticmethod
    def _call_with_checked_params(func, args: Sequence[int]):
        sig = signature(func)
        params = [
            p for p in sig.parameters.values()
            if p.kind in (Parameter.POSITIONAL_ONLY, Parameter.POSITIONAL_OR_KEYWORD)
        ]
        required = [p for p in params if p.default is Parameter.empty]
        min_count = len(required)
        max_count = len(params)
        if not (min_count <= len(args) <= max_count):
            raise ValueError(
                f"参数数量不匹配: 需要[{min_count}, {max_count}]，实际{len(args)}"
            )
        return func(*args)

    @staticmethod
    def _call_decode(decode_func, raw: bytes, servo_id: int, decode_kwargs: Dict[str, Any]):
        kwargs = dict(decode_kwargs or {})
        sig = signature(decode_func)
        if "expected_servo_id" in sig.parameters:
            kwargs["expected_servo_id"] = int(servo_id)
        return decode_func(raw, **kwargs)

    def _exchange_with_servo(
        self,
        frame: bytes,
        servo_id: int,
        decode_func=None,
        decode_kwargs: Optional[Dict[str, Any]] = None,
    ):
        if not self._init_serial():
            return False, None, b"", "serial not ready"

        buffer = bytearray()
        deadline = time.monotonic() + max(0.02, self.read_timeout)

        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                if self.debug:
                    self.get_logger().debug(f"[Exec] -> {frame.hex(' ')}")

                if decode_func is None:
                    return True, None, bytes(buffer), "ok"

                while time.monotonic() < deadline:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        buffer.extend(self.ser.read(waiting))
                        decoded = self._call_decode(
                            decode_func,
                            bytes(buffer),
                            servo_id=servo_id,
                            decode_kwargs=decode_kwargs or {},
                        )
                        if decoded is not None:
                            return True, decoded, bytes(buffer), "ok"
                    time.sleep(0.005)
            except Exception as exc:
                return False, None, bytes(buffer), str(exc)

        return True, None, bytes(buffer), "timeout"

    @staticmethod
    def _decoded_to_json(decoded: Any) -> str:
        try:
            return json.dumps(decoded, ensure_ascii=False)
        except TypeError:
            return json.dumps(str(decoded), ensure_ascii=False)

    @staticmethod
    def _fill_numeric_result(response: ExecuteBusCommand.Response, decoded: Any):
        response.value = 0
        response.values = []
        if isinstance(decoded, bool):
            response.value = int(decoded)
            response.values = [int(decoded)]
            return
        if isinstance(decoded, int):
            response.value = int(decoded)
            response.values = [int(decoded)]
            return
        if isinstance(decoded, (list, tuple)):
            out: List[int] = []
            for item in decoded:
                if isinstance(item, bool):
                    out.append(int(item))
                elif isinstance(item, int):
                    out.append(int(item))
                else:
                    return
            response.values = out
            if out:
                response.value = out[0]

    @staticmethod
    def _set_execute_error(response: ExecuteBusCommand.Response, protocol: str, code: int, message: str):
        response.success = False
        response.protocol = str(protocol or "")
        response.error_code = int(max(1, min(255, int(code))))
        response.message = str(message or "error")
        response.value = 0
        response.values = []
        response.raw_hex = ""
        response.result_json = "null"
        return response

    def _maybe_publish_read_state(self, protocol: str, command_name: str, servo_id: int, decoded: Any):
        pulse = None
        if command_name in ("read_position_command", "position_read") and isinstance(decoded, int):
            if protocol == "lx":
                pulse = self._lx_unit_to_pulse(int(decoded))
            elif protocol == "zl":
                pulse = self._zl_position_to_pulse(int(decoded))
        elif command_name == "pos_read" and isinstance(decoded, int):
            if 0 <= int(decoded) <= 1000:
                pulse = self._lx_unit_to_pulse(int(decoded))

        if pulse is not None:
            self._publish_state(servo_id=servo_id, pulse=pulse, error_code=0)

    def _read_position_once(self, protocol: str, servo_id: int) -> Optional[int]:
        if protocol not in self.protocols:
            return None
        if not self._init_serial():
            return None

        proto = self.protocols[protocol]
        cmd = proto.encode_read_position_command(servo_id)
        deadline = time.monotonic() + max(0.02, self.read_timeout)
        buffer = bytearray()

        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                if self.debug:
                    self.get_logger().debug(f"[Read/{protocol}] -> {cmd.hex(' ')}")

                while time.monotonic() < deadline:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        buffer.extend(self.ser.read(waiting))
                        pos = proto.decode_position_response(
                            bytes(buffer),
                            expected_servo_id=servo_id,
                        )
                        if pos is not None:
                            return int(pos)
                    time.sleep(0.005)
            except Exception as exc:
                self.get_logger().error(f"读取位置失败({protocol}): {exc}")
                return None
        return None

    def _pick_protocol_order(self, requested: str, servo_id: int):
        requested = (requested or "").strip().lower()
        if requested in ("zl", "lx"):
            return [requested]

        # auto: 优先根据ID归属推断
        order = []
        if self._is_allowed_id("lx", servo_id):
            order.append("lx")
        if self._is_allowed_id("zl", servo_id):
            order.append("zl")

        if not order:
            order = ["lx", "zl"]
        return order

    def _handle_read_position(self, request: ReadServoPosition.Request, response: ReadServoPosition.Response):
        servo_id = int(request.servo_id)
        protocols = self._pick_protocol_order(request.protocol, servo_id)

        for proto_name in protocols:
            pos = self._read_position_once(proto_name, servo_id)
            if pos is None:
                continue

            pulse = self._zl_position_to_pulse(pos) if proto_name == "zl" else self._lx_unit_to_pulse(pos)
            self._publish_state(servo_id=servo_id, pulse=pulse, error_code=0)

            response.success = True
            response.position = int(max(0, min(65535, pulse)))
            response.protocol = proto_name
            response.error_code = 0
            response.message = "ok"
            response.stamp = self.get_clock().now().to_msg()

            if self.log_id:
                self.get_logger().info(
                    f"[Read/{proto_name}] ID={servo_id} RAW={pos} PULSE={pulse}"
                )
            return response

        response.success = False
        response.position = 0
        response.protocol = ""
        response.error_code = 1
        response.message = f"no response from protocols={protocols}"
        response.stamp = self.get_clock().now().to_msg()
        return response

    def _handle_execute_command(
        self,
        request: ExecuteBusCommand.Request,
        response: ExecuteBusCommand.Response,
    ):
        servo_id = int(request.servo_id)
        protocol = str(request.protocol or "").strip().lower()
        command = str(request.command or "").strip()
        params = [int(x) for x in list(request.params or [])]

        if protocol not in ("zl", "lx"):
            return self._set_execute_error(response, protocol, 2, "protocol must be zl or lx")
        if not self._is_allowed_id(protocol, servo_id):
            return self._set_execute_error(response, protocol, 3, f"id {servo_id} not allowed")

        spec = self._resolve_command(protocol, command)
        if spec is None:
            available = sorted(self.command_specs.get(protocol, {}).keys())
            return self._set_execute_error(
                response,
                protocol,
                4,
                f"unknown command: {command}, available={available}",
            )

        proto = self.protocols[protocol]
        encode_name = str(spec["encode"])
        encode_func = getattr(proto, encode_name, None)
        if encode_func is None:
            return self._set_execute_error(response, protocol, 5, f"missing encode method: {encode_name}")

        try:
            args = [servo_id, *params]
            frame = self._call_with_checked_params(encode_func, args)
            if not isinstance(frame, (bytes, bytearray)):
                return self._set_execute_error(response, protocol, 6, "encode result is not bytes")
        except Exception as exc:
            return self._set_execute_error(response, protocol, 7, f"encode failed: {exc}")

        decode_name = str(spec.get("decode", "") or "").strip()
        decode_kwargs = dict(spec.get("decode_kwargs", {}) or {})
        decode_func = getattr(proto, decode_name, None) if decode_name else None
        if decode_name and decode_func is None:
            return self._set_execute_error(response, protocol, 8, f"missing decode method: {decode_name}")

        ok, decoded, raw, detail = self._exchange_with_servo(
            bytes(frame),
            servo_id=servo_id,
            decode_func=decode_func,
            decode_kwargs=decode_kwargs,
        )
        if not ok:
            return self._set_execute_error(response, protocol, 9, f"serial exchange failed: {detail}")
        if decode_func is not None and decoded is None:
            return self._set_execute_error(response, protocol, 10, f"decode timeout: {detail}")

        response.success = True
        response.protocol = protocol
        response.error_code = 0
        response.message = "ok"
        response.raw_hex = bytes(raw).hex(" ")
        response.result_json = self._decoded_to_json(decoded)
        self._fill_numeric_result(response, decoded)
        response.stamp = self.get_clock().now().to_msg()

        self._maybe_publish_read_state(protocol, str(spec.get("name", "")), servo_id, decoded)

        if self.log_id:
            self.get_logger().info(
                f"[Exec/{protocol}] ID={servo_id} CMD={spec.get('name')} PARAMS={params}"
            )
        return response

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("串口已关闭")
            except Exception as exc:
                self.get_logger().error(f"关闭串口失败: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BusPortDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
