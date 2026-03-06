"""bus_protocol_registry 单元测试。"""

import json
from pathlib import Path

from servo_hardware.bus_protocol_registry import (
    ProtocolRegistry,
    load_manual_protocol_map,
    parse_id_ranges,
)


def test_parse_id_ranges():
    assert parse_id_ranges("21-34, 40, 43-41") == [(21, 34), (40, 40), (41, 43)]
    assert parse_id_ranges(["1-2", "3", "5-4"]) == [(1, 2), (3, 3), (4, 5)]


def test_registry_priority_and_save(tmp_path: Path):
    cache_file = tmp_path / "cache.json"

    # 预写缓存
    cache_file.write_text(
        json.dumps({"mappings": {"21": {"protocol": "lx"}, "35": "zl"}}),
        encoding="utf-8",
    )

    registry = ProtocolRegistry(
        cache_file=str(cache_file),
        manual_map={35: "lx"},
        lx_ranges="21-34",
        zl_ranges="35-43",
    )

    # manual > cache > range
    assert registry.get_protocol(35) == ("lx", "manual")
    assert registry.get_protocol(21) == ("lx", "cache")
    assert registry.get_protocol(40) == ("zl", "range")
    assert registry.get_protocol(60) == (None, "unknown")

    registry.set_protocol(60, "zl", source="probe")
    registry.save_cache()

    saved = json.loads(cache_file.read_text(encoding="utf-8"))
    assert saved["mappings"]["60"]["protocol"] == "zl"
    assert saved["mappings"]["60"]["source"] == "probe"


def test_load_manual_protocol_map(tmp_path: Path):
    path = tmp_path / "manual.json"
    path.write_text(
        json.dumps(
            {
                "mappings": {
                    "21": "lx",
                    "35": {"protocol": "zl"},
                    "bad": "invalid",
                }
            }
        ),
        encoding="utf-8",
    )
    data = load_manual_protocol_map(str(path))
    assert data == {21: "lx", 35: "zl"}
