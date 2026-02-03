import threading
import time


class DebugAggregator:
    def __init__(self, emit, max_len=120):
        self._emit = emit
        self._max_len = max_len
        self._lock = threading.Lock()
        self._counts = {}
        self._last = {}

    def record(self, category, message):
        if not category:
            category = "misc"
        if message is None:
            message = ""
        with self._lock:
            self._counts[category] = self._counts.get(category, 0) + 1
            self._last[category] = message

    def flush(self):
        with self._lock:
            if not self._counts:
                return
            counts = dict(self._counts)
            last = dict(self._last)
            self._counts.clear()
            self._last.clear()

        timestamp = time.strftime("%H:%M:%S")
        lines = [f"[DebugSummary] {timestamp}"]
        for category in sorted(counts.keys()):
            msg = last.get(category, "")
            if msg and len(msg) > self._max_len:
                msg = msg[: self._max_len - 3] + "..."
            if msg:
                lines.append(f"- {category}: {counts[category]} (last: {msg})")
            else:
                lines.append(f"- {category}: {counts[category]}")

        self._emit("\n".join(lines))
