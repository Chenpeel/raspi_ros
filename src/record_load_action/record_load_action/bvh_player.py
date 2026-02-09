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
        self._bvh_cache: Dict[str, Dict] = {}

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
            for package_name in ('record_load_action', 'websocket_bridge'):
                try:
                    share_dir = get_package_share_directory(package_name)
                    candidate = os.path.join(
                        share_dir, 'config', 'bvh_action_map.json'
                    )
                    if os.path.exists(candidate):
                        return candidate
                except Exception:
                    continue

        candidate = Path(__file__).resolve().parent.parent / 'config' / 'bvh_action_map.json'
        if candidate.exists():
            return str(candidate)

        workspace_root = Path(__file__).resolve().parents[2]
        candidate = workspace_root / 'record_load_action' / 'config' / 'bvh_action_map.json'
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
    def _coerce_float(value, default=None) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _coerce_int(value, default=None) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

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

    def _resolve_bvh_path(self, action_data: Dict, config: Dict,
                          config_path: str) -> str:
        if not isinstance(action_data, dict):
            return ''
        bvh_file = (
            action_data.get('bvh_file') or
            action_data.get('bvh_path') or
            action_data.get('file') or
            action_data.get('path')
        )
        if not bvh_file:
            return ''
        if os.path.isabs(bvh_file):
            return bvh_file

        base_dir = os.path.dirname(config_path) if config_path else ''
        bvh_dir = action_data.get('bvh_dir') or config.get('bvh_dir') or ''
        if bvh_dir and not os.path.isabs(bvh_dir):
            bvh_dir = os.path.join(base_dir, bvh_dir)
        if bvh_dir:
            return os.path.join(bvh_dir, bvh_file)
        if base_dir:
            return os.path.join(base_dir, bvh_file)
        return bvh_file

    def _parse_bvh_file(self, path: str) -> Optional[Dict]:
        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except Exception as exc:
            self._log('warn', f'Failed to read BVH file: {exc}')
            return None

        channel_order: List[Tuple[str, str]] = []
        joint_channels: Dict[str, List[str]] = {}
        block_stack: List[str] = []
        joint_stack: List[str] = []
        current_joint: Optional[str] = None
        in_motion = False
        total_channels = 0
        frame_count = None
        frame_time = None
        frames: List[List[float]] = []
        pending_values: List[float] = []

        for raw in lines:
            line = raw.strip()
            if not line:
                continue

            if not in_motion:
                upper = line.upper()
                if upper == 'MOTION':
                    in_motion = True
                    continue
                if line.startswith('ROOT ') or line.startswith('JOINT '):
                    name = line.split()[1]
                    current_joint = name
                    joint_stack.append(name)
                    block_stack.append('JOINT')
                    continue
                if line.startswith('End Site'):
                    block_stack.append('ENDSITE')
                    continue
                if line.startswith('CHANNELS'):
                    parts = line.split()
                    if len(parts) < 2 or current_joint is None:
                        continue
                    try:
                        count = int(parts[1])
                    except ValueError:
                        continue
                    names = parts[2:2 + count]
                    joint_channels[current_joint] = names
                    for chan in names:
                        channel_order.append((current_joint, chan))
                    continue
                if line.startswith('}'):
                    if not block_stack:
                        continue
                    closing = block_stack.pop()
                    if closing == 'JOINT' and joint_stack:
                        joint_stack.pop()
                        current_joint = joint_stack[-1] if joint_stack else None
                    continue
                continue

            if line.startswith('Frames:'):
                frame_count = self._coerce_int(line.split(':', 1)[1].strip(), frame_count)
                continue
            if line.startswith('Frame Time:'):
                frame_time = self._coerce_float(line.split(':', 1)[1].strip(), frame_time)
                continue

            if total_channels == 0:
                total_channels = len(channel_order)

            for token in line.split():
                try:
                    pending_values.append(float(token))
                except ValueError:
                    continue

            while total_channels and len(pending_values) >= total_channels:
                frame = pending_values[:total_channels]
                pending_values = pending_values[total_channels:]
                frames.append(frame)
                if frame_count is not None and len(frames) >= frame_count:
                    break

            if frame_count is not None and len(frames) >= frame_count:
                break

        if not frames or total_channels == 0:
            return None

        return {
            'channels': channel_order,
            'frames': frames,
            'frame_time': frame_time,
            'frame_count': frame_count,
            'joint_channels': joint_channels
        }

    def _build_bvh_frames(self, parsed: Dict, action_data: Dict,
                          config: Dict, default_speed: int,
                          default_servo_type: str) -> Tuple[List[List[Dict]], Optional[float]]:
        if not parsed:
            return [], None

        frames_raw = parsed.get('frames') or []
        channel_order = parsed.get('channels') or []
        frame_time = parsed.get('frame_time')
        if not frames_raw or not channel_order:
            return [], None

        channel_index: Dict[Tuple[str, str], int] = {}
        for idx, (joint_name, channel_name) in enumerate(channel_order):
            channel_index[(joint_name, channel_name)] = idx

        default_channel = (
            action_data.get('default_channel') or
            config.get('default_channel') or
            'Zrotation'
        )
        default_scale = self._coerce_float(
            action_data.get('default_scale', config.get('default_scale', 1.0)),
            1.0
        )
        default_bias = self._coerce_float(
            action_data.get('default_bias', config.get('default_bias', 0.0)),
            0.0
        )
        default_min = self._coerce_float(
            action_data.get('default_min', config.get('default_min')),
            None
        )
        default_max = self._coerce_float(
            action_data.get('default_max', config.get('default_max')),
            None
        )
        servo_limits = action_data.get('servo_limits') or config.get(
            'servo_limits'
        ) or {}
        axis_channel_map = {
            'roll': 'Xrotation',
            'pitch': 'Yrotation',
            'yaw': 'Zrotation'
        }
        axis_override = action_data.get('axis_channel_map') or config.get(
            'axis_channel_map'
        )
        if isinstance(axis_override, dict):
            for axis_name, channel_name in axis_override.items():
                if not isinstance(axis_name, str) or not isinstance(channel_name, str):
                    continue
                axis_channel_map[axis_name.strip().lower()] = channel_name.strip()

        def _lookup_limits(servo_id: int) -> Tuple[Optional[float], Optional[float]]:
            if not isinstance(servo_limits, dict):
                return None, None
            entry = servo_limits.get(str(servo_id))
            if entry is None:
                entry = servo_limits.get(servo_id)
            if isinstance(entry, dict):
                return (
                    self._coerce_float(entry.get('min'), None),
                    self._coerce_float(entry.get('max'), None)
                )
            return None, None

        joint_map = action_data.get('joint_map') or config.get('joint_map') or {}
        alias_map = action_data.get('joint_alias') or config.get('joint_alias') or {}
        reverse_alias: Dict[str, str] = {}
        if isinstance(alias_map, dict):
            for bvh_joint, alias in alias_map.items():
                if isinstance(alias, str) and alias not in reverse_alias:
                    reverse_alias[alias] = bvh_joint

        target_descs: List[Dict] = []
        if isinstance(joint_map, dict):
            for canonical_name, mapping in joint_map.items():
                if not isinstance(canonical_name, str):
                    continue
                bvh_joint = reverse_alias.get(canonical_name, canonical_name)
                if (bvh_joint, default_channel) not in channel_index:
                    # Try other channels in case per-target override is used.
                    if not any(
                        (bvh_joint, ch) in channel_index
                        for ch in ('Xrotation', 'Yrotation', 'Zrotation')
                    ):
                        continue
                axis_targets = []
                if isinstance(mapping, dict):
                    for key, value in mapping.items():
                        if not isinstance(key, str) or not key.isdigit():
                            continue
                        if not isinstance(value, str) or '.' not in value:
                            continue
                        joint_part, axis_part = value.rsplit('.', 1)
                        axis_key = axis_part.strip().lower()
                        channel = axis_channel_map.get(axis_key)
                        if not channel:
                            continue
                        joint_name = joint_part.strip() or canonical_name
                        axis_targets.append({
                            'joint': joint_name,
                            'id': int(key),
                            'channel': channel
                        })

                if axis_targets:
                    for target in axis_targets:
                        target_id = target.get('id')
                        channel = target.get('channel', default_channel)
                        joint_name = target.get('joint', canonical_name)
                        bvh_joint = reverse_alias.get(joint_name, joint_name)
                        if (bvh_joint, channel) not in channel_index:
                            continue
                        channel_idx = channel_index.get((bvh_joint, channel))
                        if channel_idx is None:
                            continue
                        servo_id = self._coerce_int(target_id, None)
                        if servo_id is None:
                            continue
                        limit_min, limit_max = _lookup_limits(servo_id)
                        min_val = default_min
                        max_val = default_max
                        if limit_min is not None:
                            min_val = limit_min
                        if limit_max is not None:
                            max_val = limit_max
                        target_descs.append({
                            'id': servo_id,
                            'channel_idx': channel_idx,
                            'scale': default_scale,
                            'bias': default_bias,
                            'min': min_val,
                            'max': max_val,
                            'servo_type': default_servo_type,
                            'speed': default_speed,
                            'sign': 1.0
                        })
                    continue

                if isinstance(mapping, dict):
                    items = list(mapping.items())
                else:
                    items = [(canonical_name, mapping)]

                for _, target in items:
                    if isinstance(target, dict):
                        target_id = target.get('id')
                        channel = target.get('channel', default_channel)
                        scale = self._coerce_float(
                            target.get('scale', default_scale),
                            default_scale
                        )
                        bias = self._coerce_float(target.get('bias', default_bias), default_bias)
                        min_val = self._coerce_float(target.get('min', default_min), default_min)
                        max_val = self._coerce_float(target.get('max', default_max), default_max)
                        servo_type = target.get('servo_type', default_servo_type)
                        speed_val = target.get('speed_ms', default_speed)
                        sign = target.get('sign', None)
                        invert = target.get('invert', False)
                        target_has_limits = 'min' in target or 'max' in target
                    else:
                        target_id = target
                        channel = default_channel
                        scale = default_scale
                        bias = default_bias
                        min_val = default_min
                        max_val = default_max
                        servo_type = default_servo_type
                        speed_val = default_speed
                        sign = None
                        invert = False
                        target_has_limits = False

                    if isinstance(target_id, str) and target_id.lower() == 'null':
                        continue
                    servo_id = self._coerce_int(target_id, None)
                    if servo_id is None:
                        continue

                    limit_min, limit_max = _lookup_limits(servo_id)
                    if not target_has_limits:
                        if limit_min is not None:
                            min_val = limit_min
                        if limit_max is not None:
                            max_val = limit_max

                    channel_idx = channel_index.get((bvh_joint, channel))
                    if channel_idx is None:
                        continue

                    if sign is None:
                        sign_val = -1.0 if str(invert).lower() in ('1', 'true', 'yes') else 1.0
                    else:
                        sign_val = self._coerce_float(sign, 1.0)

                    target_descs.append({
                        'id': servo_id,
                        'channel_idx': channel_idx,
                        'scale': scale,
                        'bias': bias,
                        'min': min_val,
                        'max': max_val,
                        'servo_type': servo_type,
                        'speed': speed_val,
                        'sign': sign_val
                    })

        if not target_descs:
            self._log('warn', 'BVH joint_map produced no servo targets')
            return [], None

        frames: List[List[Dict]] = []
        for frame in frames_raw:
            frame_cmds: List[Dict] = []
            for target in target_descs:
                angle = frame[target['channel_idx']]
                position = target['bias'] + angle * target['scale'] * target['sign']
                if target['min'] is not None:
                    position = max(target['min'], position)
                if target['max'] is not None:
                    position = min(target['max'], position)
                frame_cmds.append({
                    'id': target['id'],
                    'position': int(round(position)),
                    'speed': target['speed'],
                    'servo_type': target['servo_type']
                })
            frames.append(frame_cmds)

        frame_time_ms = None
        if frame_time is not None:
            frame_time_ms = frame_time * 1000.0

        return frames, frame_time_ms

    def _load_bvh_frames(self, action_name: str, action_data: Dict,
                         config: Dict, config_path: str,
                         default_speed: int,
                         default_servo_type: str) -> Tuple[List[List[Dict]], Optional[float]]:
        bvh_path = self._resolve_bvh_path(action_data, config, config_path)
        if not bvh_path or not os.path.exists(bvh_path):
            self._log('warn', f'BVH file not found for action: {action_name}')
            return [], None

        cache_key = f'{action_name}:{bvh_path}'
        cached = self._bvh_cache.get(cache_key)
        if cached:
            return cached['frames'], cached.get('frame_time_ms')

        parsed = self._parse_bvh_file(bvh_path)
        if not parsed:
            self._log('warn', f'Failed to parse BVH file: {bvh_path}')
            return [], None

        frames, frame_time_ms = self._build_bvh_frames(
            parsed, action_data, config, default_speed, default_servo_type
        )
        if not frames:
            return [], frame_time_ms

        self._bvh_cache[cache_key] = {
            'frames': frames,
            'frame_time_ms': frame_time_ms
        }
        return frames, frame_time_ms

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
        frame_delay_ms = self._coerce_float(
            action_data.get('frame_delay_ms', action_data.get('frame_time_ms')),
            None
        )
        servo_limits = action_data.get('servo_limits') or config.get(
            'servo_limits'
        ) or {}

        def _limit_position(servo_id: int, position: int) -> int:
            if not isinstance(servo_limits, dict):
                return position
            entry = servo_limits.get(str(servo_id))
            if entry is None:
                entry = servo_limits.get(servo_id)
            if not isinstance(entry, dict):
                return position
            min_val = self._coerce_float(entry.get('min'), None)
            max_val = self._coerce_float(entry.get('max'), None)
            if min_val is not None:
                position = max(int(round(min_val)), position)
            if max_val is not None:
                position = min(int(round(max_val)), position)
            return position

        frames = self._normalize_frames(action_data, default_speed)
        if not frames:
            frames, parsed_delay = self._load_bvh_frames(
                action_name, action_data, config, path, default_speed, default_servo_type
            )
            if parsed_delay is not None and frame_delay_ms is None:
                frame_delay_ms = parsed_delay

        if not frames:
            self._log('warn', f'BVH action has no frames: {action_name}')
            return

        self._log('info', f'BVH play start: {action_name} ({path})')

        while True:
            for frame in frames:
                if self._stop_event.is_set():
                    self._log('info', f'BVH play stopped: {action_name}')
                    return

                if frame_delay_ms is not None:
                    start_ts = time.time()
                    for cmd in frame:
                        try:
                            servo_type = cmd.get('servo_type', default_servo_type)
                            servo_id = int(cmd.get('id'))
                            position = int(cmd.get('position'))
                            position = _limit_position(servo_id, position)
                            speed = int(cmd.get('speed', default_speed))
                            if speed_override is not None:
                                speed = speed_override
                            self._publish(servo_type, servo_id, position, speed)
                        except Exception as exc:
                            self._log('warn', f'BVH publish failed: {exc}')
                            continue

                    delay = frame_delay_ms
                    if speed_override is not None:
                        delay = speed_override
                    if delay > 0:
                        elapsed = (time.time() - start_ts) * 1000.0
                        remaining = max(0.0, delay - elapsed)
                        if remaining > 0:
                            time.sleep(remaining / 1000.0)
                else:
                    for cmd in frame:
                        if self._stop_event.is_set():
                            self._log('info', f'BVH play stopped: {action_name}')
                            return
                        try:
                            servo_type = cmd.get('servo_type', default_servo_type)
                            servo_id = int(cmd.get('id'))
                            position = int(cmd.get('position'))
                            position = _limit_position(servo_id, position)
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
