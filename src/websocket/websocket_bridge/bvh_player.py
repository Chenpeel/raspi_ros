import json
import os
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for non-ROS envs
    get_package_share_directory = None


class BvhActionPlayer:
    def __init__(
        self,
        publish_callback: Callable[[str, int, int, int], None],
        config_path: str = '',
        logger=None
    ):
        self._publish = publish_callback
        self._config_path = config_path or ''
        self._logger = logger
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def _log(self, level: str, message: str) -> None:
        if self._logger is None:
            return
        log_fn = getattr(self._logger, level, None)
        if log_fn:
            log_fn(message)

    def _resolve_config_path(self, override_path: str) -> str:
        if override_path and os.path.exists(override_path):
            return override_path

        if get_package_share_directory:
            try:
                share_dir = get_package_share_directory('websocket_bridge')
                candidate = os.path.join(
                    share_dir, 'config', 'bvh_action_map.json'
                )
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass

        candidate = Path(__file__).resolve().parent.parent / 'config' / 'bvh_action_map.json'
        if candidate.exists():
            return str(candidate)

        return ''

    def _load_config(self) -> Tuple[Dict, str]:
        path = self._resolve_config_path(self._config_path)
        if not path:
            self._log('warn', 'BVH action config not found')
            return {}, ''
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f), path
        except Exception as exc:
            self._log('warn', f'Failed to load BVH action config: {exc}')
            return {}, path

    @staticmethod
    def _resolve_action_name(action, config: Dict) -> Optional[str]:
        if action is None:
            return None
        if isinstance(action, (int, float)):
            idx = int(action)
            bvh_list = config.get('bvh_list') or []
            if 0 <= idx < len(bvh_list):
                name = bvh_list[idx]
                return name if isinstance(name, str) else None
            return None
        if isinstance(action, str):
            action = action.strip()
            if not action:
                return None
            if action.isdigit():
                idx = int(action)
                bvh_list = config.get('bvh_list') or []
                if 0 <= idx < len(bvh_list):
                    name = bvh_list[idx]
                    return name if isinstance(name, str) else None
            return action
        return None

    @staticmethod
    def _normalize_frames(action_data: Dict, default_speed: int) -> List[List[Dict]]:
        if not isinstance(action_data, dict):
            return []

        if 'frames' in action_data:
            frames = action_data.get('frames') or []
        elif 'ids' in action_data and 'positions' in action_data:
            ids = action_data.get('ids') or []
            positions = action_data.get('positions') or []
            speeds = action_data.get('speeds') or []
            frames = []
            for i, servo_id in enumerate(ids):
                if i >= len(positions):
                    break
                speed_val = speeds[i] if i < len(speeds) else default_speed
                frames.append([{
                    'id': servo_id,
                    'position': positions[i],
                    'speed': speed_val
                }])
        else:
            return []

        if not frames:
            return []

        if isinstance(frames, list) and frames and isinstance(frames[0], dict):
            frames = [frames]

        normalized: List[List[Dict]] = []
        for frame in frames:
            if isinstance(frame, dict):
                frame = [frame]
            if not isinstance(frame, list):
                continue
            frame_cmds = []
            for cmd in frame:
                if not isinstance(cmd, dict):
                    continue
                if 'id' not in cmd or 'position' not in cmd:
                    continue
                frame_cmds.append(cmd)
            if frame_cmds:
                normalized.append(frame_cmds)

        return normalized

    def play(self, action, loop: bool = False, speed_ms: Optional[int] = None) -> bool:
        if action in (None, '', 'null'):
            self.stop()
            return False

        with self._lock:
            self.stop()
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=self._run, args=(action, loop, speed_ms), daemon=True
            )
            self._thread.start()
            return True

    def stop(self) -> None:
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join(timeout=1.0)
        self._thread = None

    def _run(self, action, loop: bool, speed_ms: Optional[int]) -> None:
        config, path = self._load_config()
        if not config:
            self._log('warn', 'BVH action config is empty')
            return

        action_name = self._resolve_action_name(action, config)
        if not action_name:
            self._log('warn', f'BVH action not resolved: {action}')
            return

        action_data = (config.get('bvh_data') or {}).get(action_name)
        if not action_data:
            self._log('warn', f'BVH action not found: {action_name}')
            return

        default_speed = int(action_data.get(
            'default_speed_ms',
            config.get('default_speed_ms', 33)
        ))
        speed_override = None
        if speed_ms is not None:
            try:
                speed_override = int(speed_ms)
            except (TypeError, ValueError):
                speed_override = None
        default_servo_type = action_data.get(
            'default_servo_type',
            config.get('default_servo_type', 'bus')
        )

        frames = self._normalize_frames(action_data, default_speed)
        if not frames:
            self._log('warn', f'BVH action has no frames: {action_name}')
            return

        self._log('info', f'BVH play start: {action_name} ({path})')

        while True:
            for frame in frames:
                for cmd in frame:
                    if self._stop_event.is_set():
                        self._log('info', f'BVH play stopped: {action_name}')
                        return
                    try:
                        servo_type = cmd.get('servo_type', default_servo_type)
                        servo_id = int(cmd.get('id'))
                        position = int(cmd.get('position'))
                        speed = int(cmd.get('speed', default_speed))
                        if speed_override is not None:
                            speed = speed_override
                        self._publish(servo_type, servo_id, position, speed)
                    except Exception as exc:
                        self._log('warn', f'BVH publish failed: {exc}')
                        continue

                    if speed > 0:
                        time.sleep(speed / 1000.0)

            if not loop:
                break

        self._log('info', f'BVH play finished: {action_name}')
