"""舵机安装偏移量探针校准工具（输出单位: deg）。

用途:
- 通过全局服务 `/servo/read_position` 读取舵机当前位置（统一脉宽语义: 500~2500）；
- 结合 `servo_limit_map.json` 的中点或默认 1500us 作为参考值；
- 计算并写入 offset_deg:

    offset_deg = (current_pulse - reference_pulse) * 180 / 2000

说明:
- 该 offset_deg 设计为“发送目标 -> 加偏移 -> 下发到舵机”的补偿量；
- 为了兼容不同协议舵机，文件中仅保存角度偏移（deg），驱动侧再换算到脉宽域。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node

try:
    # ROS 2 推荐用法：过滤 --ros-args 等参数，交给 argparse 解析自身参数
    from rclpy.utilities import remove_ros_args
except Exception:  # pragma: no cover
    remove_ros_args = lambda argv: argv  # type: ignore

from servo_msgs.srv import ReadServoPosition

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for non-ROS envs
    get_package_share_directory = None


def _load_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _pick_first_existing(candidates: Sequence[Path]) -> Optional[Path]:
    for candidate in candidates:
        try:
            if candidate and candidate.exists():
                return candidate
        except Exception:
            continue
    return None


def _atomic_write_json(path: Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fd, tmp_path = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
            f.write("\n")
        os.replace(tmp_path, str(path))
    finally:
        try:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
        except Exception:
            pass


def _default_output_path() -> Path:
    return Path.cwd() / "src" / "websocket" / "config" / "servo_offset_deg_map.json"


def _default_bus_map_path() -> Optional[Path]:
    candidates: List[Path] = [
        Path("/root/ros_ws/src/websocket/config/bus_servo_map.json"),
        Path.cwd() / "src" / "websocket" / "config" / "bus_servo_map.json",
    ]
    if get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory("websocket_bridge"))
            candidates.append(share_dir / "config" / "bus_servo_map.json")
        except Exception:
            pass
    return _pick_first_existing(candidates)


def _default_limit_map_path() -> Optional[Path]:
    candidates: List[Path] = [
        Path("/root/ros_ws/src/hardware/servo_hardware/config/servo_limit_map.json"),
        Path.cwd()
        / "src"
        / "hardware"
        / "servo_hardware"
        / "config"
        / "servo_limit_map.json",
    ]
    if get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory("servo_hardware"))
            candidates.append(share_dir / "config" / "servo_limit_map.json")
        except Exception:
            pass
    return _pick_first_existing(candidates)


def _load_servo_ids_from_bus_map(path: Path) -> List[int]:
    try:
        data = _load_json(path)
    except Exception:
        return []

    if not isinstance(data, dict):
        return []

    out = set()
    for _port, ids in data.items():
        if not isinstance(ids, list):
            continue
        for item in ids:
            try:
                sid = int(item)
            except (TypeError, ValueError):
                continue
            if sid > 0:
                out.add(sid)
    return sorted(out)


def _load_limit_map(path: Optional[Path]) -> Dict[int, Dict[str, Optional[float]]]:
    if not path:
        return {}
    try:
        data = _load_json(path)
    except Exception:
        return {}

    if isinstance(data, dict) and "servo_limits" in data:
        data = data.get("servo_limits") or {}

    limit_map: Dict[int, Dict[str, Optional[float]]] = {}

    # 兼容旧格式: {ids: [...], mins: [...], maxs: [...]}
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
        return limit_map

    # 新格式: {"1": {"min": 500, "max": 2500}, ...}
    if isinstance(data, dict):
        for key, value in data.items():
            try:
                sid_int = int(key)
            except (TypeError, ValueError):
                continue
            if isinstance(value, dict):
                limit_map[sid_int] = {"min": value.get("min"), "max": value.get("max")}
        return limit_map

    return {}


def _reference_pulse(servo_id: int, limit_map: Dict[int, Dict[str, Optional[float]]]) -> int:
    entry = limit_map.get(int(servo_id))
    if not isinstance(entry, dict):
        return 1500
    min_val = entry.get("min")
    max_val = entry.get("max")
    if min_val is None or max_val is None:
        return 1500
    try:
        return int(round((float(min_val) + float(max_val)) / 2.0))
    except (TypeError, ValueError):
        return 1500


def _load_existing_offsets_deg(path: Path) -> Dict[int, float]:
    if not Path(path).exists():
        return {}
    try:
        data = _load_json(path)
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}

    # 兼容两种格式:
    # 1) {"unit": "deg", "offsets": {"1": 0.0, ...}}
    # 2) {"1": 0.0, "2": 1.23, ...}
    offsets_obj = data.get("offsets") if isinstance(data.get("offsets"), dict) else data

    out: Dict[int, float] = {}
    if isinstance(offsets_obj, dict):
        for key, value in offsets_obj.items():
            try:
                sid = int(key)
                out[sid] = float(value)
            except (TypeError, ValueError):
                continue
    return out


def _normalize_protocol_hint(text: str) -> str:
    text = str(text or "").strip().lower()
    if text in ("zl", "lx"):
        return text
    return ""


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    default_bus_map = _default_bus_map_path()
    default_limit_map = _default_limit_map_path()

    parser = argparse.ArgumentParser(
        prog="servo_offset_calibrator",
        description="读取舵机当前脉宽，生成 offset_deg_map.json",
    )
    parser.add_argument(
        "--output",
        default=str(_default_output_path()),
        help="输出 offset_deg JSON 文件路径",
    )
    parser.add_argument(
        "--bus-map",
        default=str(default_bus_map) if default_bus_map else "",
        help="bus_servo_map.json 路径（默认自动探测）",
    )
    parser.add_argument(
        "--limit-map",
        default=str(default_limit_map) if default_limit_map else "",
        help="servo_limit_map.json 路径（用于计算参考脉宽中点）",
    )
    parser.add_argument(
        "--ids",
        nargs="*",
        type=int,
        default=[],
        help="要校准的舵机ID列表；不填则从 bus-map 解析所有ID",
    )
    parser.add_argument(
        "--protocol",
        default="auto",
        help="协议提示(zl/lx/auto)，一般保持 auto 即可",
    )
    parser.add_argument(
        "--wait-service-sec",
        type=float,
        default=6.0,
        help="等待 /servo/read_position 服务就绪的超时(秒)",
    )
    parser.add_argument(
        "--call-timeout-sec",
        type=float,
        default=0.6,
        help="单个舵机读位置调用超时(秒)",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=3,
        help="offset_deg 输出保留小数位数",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印结果，不写文件",
    )
    return parser.parse_args(list(argv or []))


class ServoOffsetCalibrator(Node):
    """通过 /servo/read_position 读取脉宽并计算 offset_deg。"""

    def __init__(self, service_name: str = "/servo/read_position"):
        super().__init__("servo_offset_calibrator")
        self._client = self.create_client(ReadServoPosition, service_name)

    def wait_ready(self, timeout_sec: float) -> bool:
        return self._client.wait_for_service(timeout_sec=max(0.1, float(timeout_sec)))

    def read_pulse(
        self,
        servo_id: int,
        protocol_hint: str,
        timeout_sec: float,
    ) -> Optional[int]:
        req = ReadServoPosition.Request()
        req.servo_id = int(servo_id)
        req.protocol = str(protocol_hint or "")
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=max(0.05, float(timeout_sec)),
        )
        if not future.done():
            return None
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"读取失败: id={servo_id}, exc={exc}")
            return None
        if not bool(resp.success):
            self.get_logger().warn(f"读取失败: id={servo_id}, msg={resp.message}")
            return None
        try:
            return int(resp.position)
        except Exception:
            return None


def main(args=None):
    """console_scripts 入口。"""
    argv = args if args is not None else sys.argv
    cli_argv = remove_ros_args(argv)[1:]
    ns = _parse_args(cli_argv)

    output_path = Path(ns.output).expanduser()
    bus_map_path = Path(ns.bus_map).expanduser() if ns.bus_map else None
    limit_map_path = Path(ns.limit_map).expanduser() if ns.limit_map else None

    servo_ids = [int(x) for x in list(ns.ids or []) if int(x) > 0]
    if not servo_ids:
        if not bus_map_path:
            bus_map_path = _default_bus_map_path()
        if not bus_map_path:
            raise SystemExit("未指定 --ids，且无法自动找到 bus_servo_map.json")
        servo_ids = _load_servo_ids_from_bus_map(bus_map_path)
    if not servo_ids:
        raise SystemExit("舵机ID列表为空")

    protocol_hint = _normalize_protocol_hint(ns.protocol)
    limit_map = _load_limit_map(limit_map_path or _default_limit_map_path())
    existing = _load_existing_offsets_deg(output_path)

    rclpy.init(args=argv)
    node = ServoOffsetCalibrator()

    ok: Dict[int, float] = {}
    failed: List[int] = []

    try:
        if not node.wait_ready(timeout_sec=float(ns.wait_service_sec)):
            raise SystemExit("等待 /servo/read_position 服务超时")

        for sid in servo_ids:
            pulse = node.read_pulse(
                servo_id=sid,
                protocol_hint=protocol_hint,
                timeout_sec=float(ns.call_timeout_sec),
            )
            ref = _reference_pulse(sid, limit_map)
            if pulse is None:
                failed.append(int(sid))
                continue

            offset_deg = (float(pulse) - float(ref)) * 180.0 / 2000.0
            offset_deg = round(offset_deg, int(ns.precision))
            ok[int(sid)] = float(offset_deg)
            node.get_logger().info(
                f"id={sid} pulse={pulse} ref={ref} -> offset_deg={offset_deg}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    merged: Dict[int, float] = dict(existing)
    for sid, value in ok.items():
        merged[int(sid)] = float(value)
    for sid in servo_ids:
        merged.setdefault(int(sid), 0.0)

    offsets_out = {str(sid): float(merged[sid]) for sid in sorted(merged.keys())}
    out = {"unit": "deg", "offsets": offsets_out}

    if failed:
        print(f"[WARN] 读取失败的舵机ID: {sorted(set(failed))}")

    if ns.dry_run:
        print("[DRY-RUN] 不写文件，计算结果如下:")
        print(json.dumps(out, ensure_ascii=False, indent=4))
        return

    _atomic_write_json(output_path, out)
    print(f"[OK] 已写入 offset_deg 配置: {output_path}")


if __name__ == "__main__":  # pragma: no cover
    main()
