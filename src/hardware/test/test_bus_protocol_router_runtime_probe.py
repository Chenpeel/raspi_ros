"""bus_protocol_router 运行时探测队列逻辑测试。"""

import threading
from collections import deque

import pytest

pytest.importorskip("rclpy")
from servo_hardware.bus_protocol_router import BusProtocolRouter


class _DummyLogger:
    def __init__(self):
        self.info_logs = []
        self.debug_logs = []

    def info(self, msg):
        self.info_logs.append(str(msg))

    def debug(self, msg):
        self.debug_logs.append(str(msg))

    def warn(self, _msg):
        pass

    def error(self, _msg):
        pass


def _new_router_stub():
    router = object.__new__(BusProtocolRouter)
    router.probe_on_unknown_command = True
    router.probe_retry_interval_sec = 3.0
    router.last_probe_attempt = {}
    router.probe_lock = threading.Lock()
    router.runtime_probe_queue = deque()
    router.runtime_probe_pending = set()
    router.debug = False
    return router


def test_schedule_runtime_probe_debounce(monkeypatch):
    router = _new_router_stub()
    logger = _DummyLogger()
    router.get_logger = lambda: logger

    ts = {"value": 10.0}
    monkeypatch.setattr("servo_hardware.bus_protocol_router.time.monotonic", lambda: ts["value"])

    assert router._schedule_runtime_probe(21) is True
    assert list(router.runtime_probe_queue) == [21]
    assert 21 in router.runtime_probe_pending

    # 同一ID在重试窗口内不会重复入队
    assert router._schedule_runtime_probe(21) is False
    assert list(router.runtime_probe_queue) == [21]

    # 时间推进后，仍会先检查 pending 防重复
    ts["value"] = 20.0
    assert router._schedule_runtime_probe(21) is False


def test_process_runtime_probe_queue_updates_registry(monkeypatch):
    router = _new_router_stub()
    logger = _DummyLogger()
    router.get_logger = lambda: logger
    router.runtime_probe_queue.append(35)
    router.runtime_probe_pending.add(35)

    captured = []

    def _probe_servo_id(servo_id, use_spin=False):
        assert use_spin is False
        return "zl" if servo_id == 35 else None

    def _update_registry(servo_id, protocol, source):
        captured.append((servo_id, protocol, source))

    router._probe_servo_id = _probe_servo_id
    router._update_registry = _update_registry

    router._process_runtime_probe_queue()

    assert captured == [(35, "zl", "probe")]
    assert 35 not in router.runtime_probe_pending
    assert list(router.runtime_probe_queue) == []
    assert any("运行时探测成功" in msg for msg in logger.info_logs)


def test_process_runtime_probe_queue_failure_clears_pending():
    router = _new_router_stub()
    logger = _DummyLogger()
    router.get_logger = lambda: logger
    router.runtime_probe_queue.append(99)
    router.runtime_probe_pending.add(99)

    router._probe_servo_id = lambda _servo_id, use_spin=False: None
    router._update_registry = lambda **_kwargs: None

    router._process_runtime_probe_queue()

    assert 99 not in router.runtime_probe_pending
    assert list(router.runtime_probe_queue) == []
