"""总线协议路由缓存与规则管理。"""

import json
import os
from datetime import datetime, timezone
from typing import Dict, Iterable, List, Optional, Tuple


VALID_PROTOCOLS = {"zl", "lx"}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _normalize_protocol(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    proto = str(value).strip().lower()
    return proto if proto in VALID_PROTOCOLS else None


def _parse_range_token(token: str) -> Optional[Tuple[int, int]]:
    token = str(token).strip()
    if not token:
        return None
    if "-" in token:
        left, right = token.split("-", 1)
        start = int(left.strip())
        end = int(right.strip())
    else:
        start = int(token)
        end = int(token)
    if start > end:
        start, end = end, start
    return start, end


def parse_id_ranges(value) -> List[Tuple[int, int]]:
    """解析 ID 范围配置。

    支持:
    - "21-34,40,41-45"
    - ["21-34", "40", "41-45"]
    """
    tokens: List[str] = []
    if value is None:
        return []
    if isinstance(value, str):
        tokens.extend([item.strip() for item in value.split(",") if item.strip()])
    elif isinstance(value, (list, tuple)):
        for item in value:
            if item is None:
                continue
            tokens.extend([part.strip() for part in str(item).split(",") if part.strip()])
    else:
        tokens.append(str(value).strip())

    ranges: List[Tuple[int, int]] = []
    for token in tokens:
        try:
            parsed = _parse_range_token(token)
        except (TypeError, ValueError):
            continue
        if parsed is not None:
            ranges.append(parsed)
    return ranges


def _match_ranges(servo_id: int, ranges: Iterable[Tuple[int, int]]) -> bool:
    sid = int(servo_id)
    for start, end in ranges:
        if start <= sid <= end:
            return True
    return False


class ProtocolRegistry:
    """协议解析优先级:
    1. manual_map
    2. cache_file
    3. default ranges
    """

    def __init__(
        self,
        cache_file: str,
        manual_map: Optional[Dict[int, str]] = None,
        lx_ranges=None,
        zl_ranges=None,
    ):
        self.cache_file = str(cache_file or "").strip()
        self.manual_map: Dict[int, str] = {}
        self.cache_map: Dict[int, Dict[str, str]] = {}
        self.lx_ranges = parse_id_ranges(lx_ranges)
        self.zl_ranges = parse_id_ranges(zl_ranges)

        if manual_map:
            for sid, proto in manual_map.items():
                normalized = _normalize_protocol(proto)
                if normalized is not None:
                    self.manual_map[int(sid)] = normalized

        self.load_cache()

    def load_cache(self):
        self.cache_map = {}
        if not self.cache_file:
            return
        if not os.path.exists(self.cache_file):
            return

        try:
            with open(self.cache_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return

        mappings = data.get("mappings", data) if isinstance(data, dict) else {}
        if not isinstance(mappings, dict):
            return

        for key, val in mappings.items():
            try:
                sid = int(key)
            except (TypeError, ValueError):
                continue

            proto = None
            source = "cache"
            if isinstance(val, dict):
                proto = _normalize_protocol(val.get("protocol"))
                source = str(val.get("source", "cache"))
            else:
                proto = _normalize_protocol(str(val))
            if proto is None:
                continue
            self.cache_map[sid] = {
                "protocol": proto,
                "source": source,
                "updated_at": _now_iso(),
            }

    def save_cache(self):
        if not self.cache_file:
            return
        os.makedirs(os.path.dirname(self.cache_file), exist_ok=True)
        payload = {
            "version": 1,
            "updated_at": _now_iso(),
            "mappings": self.cache_map,
        }
        tmp_file = f"{self.cache_file}.tmp"
        with open(tmp_file, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_file, self.cache_file)

    def set_protocol(self, servo_id: int, protocol: str, source: str = "probe"):
        proto = _normalize_protocol(protocol)
        if proto is None:
            return
        sid = int(servo_id)
        self.cache_map[sid] = {
            "protocol": proto,
            "source": str(source or "probe"),
            "updated_at": _now_iso(),
        }

    def get_protocol(self, servo_id: int) -> Tuple[Optional[str], str]:
        sid = int(servo_id)

        manual = self.manual_map.get(sid)
        if manual:
            return manual, "manual"

        cached = self.cache_map.get(sid, {}).get("protocol")
        if _normalize_protocol(cached):
            return cached, "cache"

        if _match_ranges(sid, self.lx_ranges):
            return "lx", "range"
        if _match_ranges(sid, self.zl_ranges):
            return "zl", "range"

        return None, "unknown"

    def unresolved_ids(self, servo_ids: Iterable[int]) -> List[int]:
        unresolved: List[int] = []
        for sid in servo_ids:
            protocol, _ = self.get_protocol(int(sid))
            if protocol is None:
                unresolved.append(int(sid))
        return unresolved


def load_manual_protocol_map(path: str) -> Dict[int, str]:
    """加载手动协议映射文件。

    支持格式:
    1. {"21": "lx", "35": "zl"}
    2. {"mappings": {"21": {"protocol":"lx"}, "35": "zl"}}
    """
    result: Dict[int, str] = {}
    path = str(path or "").strip()
    if not path or not os.path.exists(path):
        return result

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return result

    mappings = data.get("mappings", data) if isinstance(data, dict) else {}
    if not isinstance(mappings, dict):
        return result

    for key, val in mappings.items():
        try:
            sid = int(key)
        except (TypeError, ValueError):
            continue

        proto = None
        if isinstance(val, dict):
            proto = _normalize_protocol(val.get("protocol"))
        else:
            proto = _normalize_protocol(str(val))

        if proto is not None:
            result[sid] = proto
    return result
