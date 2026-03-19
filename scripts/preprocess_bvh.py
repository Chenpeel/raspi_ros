#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple


def _default_config_path() -> Path:
    repo_root = Path(__file__).resolve().parents[1]
    return repo_root / 'src' / 'record_load_action' / 'config' / 'bvh_action_map.json'


def _load_json(path: Path) -> Dict:
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)


def _save_json(path: Path, data: Dict) -> None:
    with path.open('w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=4)


def _normalize_channel(channel: str) -> str:
    if not channel:
        return ''
    ch = channel.strip()
    upper = ch.upper()
    if upper in ('X', 'Y', 'Z'):
        return f'{upper}rotation'
    if upper in ('XROTATION', 'YROTATION', 'ZROTATION'):
        return f'{upper[0]}rotation'
    return ch


def _parse_bvh(path: Path) -> Tuple[List[Tuple[str, str]], List[List[float]], float]:
    lines = path.read_text(encoding='utf-8', errors='ignore').splitlines()
    channels: List[Tuple[str, str]] = []
    frames: List[List[float]] = []
    current_joint = None
    in_motion = False
    frame_time = 0.0
    total_frames = None

    idx = 0
    while idx < len(lines):
        line = lines[idx].strip()
        if not in_motion:
            if line.startswith('ROOT') or line.startswith('JOINT'):
                parts = line.split()
                if len(parts) >= 2:
                    current_joint = parts[1]
            elif line.startswith('CHANNELS'):
                parts = line.split()
                try:
                    count = int(parts[1])
                except (IndexError, ValueError):
                    count = 0
                for ch in parts[2:2 + count]:
                    channels.append((current_joint, ch))
            elif line.startswith('MOTION'):
                in_motion = True
            idx += 1
            continue

        if line.startswith('Frames:'):
            try:
                total_frames = int(line.split()[1])
            except (IndexError, ValueError):
                total_frames = None
        elif line.startswith('Frame Time:'):
            parts = line.split()
            try:
                frame_time = float(parts[-1])
            except (IndexError, ValueError):
                frame_time = 0.0
            idx += 1
            break
        idx += 1

    channel_count = len(channels)
    if channel_count == 0:
        return channels, frames, frame_time

    buffer: List[float] = []
    for line in lines[idx:]:
        stripped = line.strip()
        if not stripped:
            continue
        try:
            buffer.extend(float(x) for x in stripped.split())
        except ValueError:
            continue

        while len(buffer) >= channel_count:
            frames.append(buffer[:channel_count])
            buffer = buffer[channel_count:]
            if total_frames and len(frames) >= total_frames:
                return channels, frames, frame_time

    return channels, frames, frame_time


def _parse_servo_id(value) -> int:
    if value is None:
        return None
    if isinstance(value, str):
        if value.strip().lower() in ('null', 'none', ''):
            return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _expand_joint_map(joint_map, defaults: Dict) -> List[Dict]:
    entries: List[Dict] = []

    if isinstance(joint_map, list):
        for item in joint_map:
            if not isinstance(item, dict):
                continue
            bvh_joint = item.get('bvh_joint') or item.get('joint') or item.get('name')
            servo_id = _parse_servo_id(item.get('servo_id') or item.get('id'))
            if not bvh_joint or servo_id is None:
                continue
            entry = {
                'bvh_joint': bvh_joint,
                'servo_id': servo_id,
                'channel': _normalize_channel(item.get('channel') or item.get('axis') or defaults['channel']),
                'scale': float(item.get('scale', defaults['scale'])),
                'bias': float(item.get('bias', defaults['bias'])),
                'min': float(item.get('min', defaults['min'])),
                'max': float(item.get('max', defaults['max'])),
                'servo_type': item.get('servo_type', defaults['servo_type'])
            }
            entries.append(entry)
        return entries

    if isinstance(joint_map, dict):
        for bvh_joint, mapping in joint_map.items():
            if mapping is None:
                continue
            if isinstance(mapping, dict):
                for _, servo_id in mapping.items():
                    sid = _parse_servo_id(servo_id)
                    if sid is None:
                        continue
                    entries.append({
                        'bvh_joint': bvh_joint,
                        'servo_id': sid,
                        'channel': defaults['channel'],
                        'scale': defaults['scale'],
                        'bias': defaults['bias'],
                        'min': defaults['min'],
                        'max': defaults['max'],
                        'servo_type': defaults['servo_type']
                    })
            elif isinstance(mapping, list):
                for servo_id in mapping:
                    sid = _parse_servo_id(servo_id)
                    if sid is None:
                        continue
                    entries.append({
                        'bvh_joint': bvh_joint,
                        'servo_id': sid,
                        'channel': defaults['channel'],
                        'scale': defaults['scale'],
                        'bias': defaults['bias'],
                        'min': defaults['min'],
                        'max': defaults['max'],
                        'servo_type': defaults['servo_type']
                    })
            else:
                sid = _parse_servo_id(mapping)
                if sid is None:
                    continue
                entries.append({
                    'bvh_joint': bvh_joint,
                    'servo_id': sid,
                    'channel': defaults['channel'],
                    'scale': defaults['scale'],
                    'bias': defaults['bias'],
                    'min': defaults['min'],
                    'max': defaults['max'],
                    'servo_type': defaults['servo_type']
                })

    return entries


def _build_channel_index(channels: List[Tuple[str, str]]) -> Dict[Tuple[str, str], int]:
    index = {}
    for i, (joint, ch) in enumerate(channels):
        index[(joint, _normalize_channel(ch))] = i
    return index


def _coerce_speed_ms(frame_time: float, config: Dict, override: int) -> int:
    if override is not None:
        return max(1, int(override))
    if config.get('default_speed_ms') is not None:
        try:
            return max(1, int(config['default_speed_ms']))
        except (TypeError, ValueError):
            pass
    if frame_time > 0:
        return max(1, int(round(frame_time * 1000)))
    return 33


def main() -> None:
    parser = argparse.ArgumentParser(description='Preprocess BVH into action map frames')
    parser.add_argument('--config', type=Path, default=_default_config_path(),
                        help='Path to bvh_action_map.json')
    parser.add_argument('--output', type=Path, default=None,
                        help='Output path (defaults to --config)')
    parser.add_argument('--input-dir', type=Path, default=None,
                        help='Directory containing *.bvh files')
    parser.add_argument('--all', action='store_true',
                        help='Process all *.bvh files in the input directory')
    parser.add_argument('--speed-ms', type=int, default=None,
                        help='Override speed ms for each command')

    args = parser.parse_args()
    config_path = args.config
    if not config_path.exists():
        raise SystemExit(f'Config file not found: {config_path}')

    config = _load_json(config_path)
    input_dir = args.input_dir or Path(config.get('bvh_dir') or '')
    if not input_dir:
        raise SystemExit('Missing --input-dir and no bvh_dir in config')
    input_dir = input_dir.expanduser().resolve()

    if not input_dir.exists():
        raise SystemExit(f'BVH directory not found: {input_dir}')

    if args.all:
        action_names = sorted(p.stem for p in input_dir.glob('*.bvh'))
    else:
        bvh_list = config.get('bvh_list') or []
        action_names = [name for name in bvh_list if isinstance(name, str) and name]

    if not action_names:
        raise SystemExit('No BVH actions to process')

    defaults = {
        'channel': _normalize_channel(config.get('default_channel', 'Zrotation')),
        'scale': float(config.get('default_scale', 10.0)),
        'bias': float(config.get('default_bias', 1500.0)),
        'min': float(config.get('default_min', 500.0)),
        'max': float(config.get('default_max', 2500.0)),
        'servo_type': config.get('default_servo_type', 'bus')
    }

    joint_map = config.get('joint_map') or {}
    mapping_entries = _expand_joint_map(joint_map, defaults)
    if not mapping_entries:
        raise SystemExit('joint_map is empty or unsupported')

    bvh_data = config.get('bvh_data')
    if not isinstance(bvh_data, dict):
        bvh_data = {}

    for action in action_names:
        bvh_path = input_dir / f'{action}.bvh'
        if not bvh_path.exists():
            print(f'Skip: BVH file not found {bvh_path}')
            continue

        channels, frames, frame_time = _parse_bvh(bvh_path)
        if not frames:
            print(f'Skip: BVH has no frames {bvh_path}')
            continue

        channel_index = _build_channel_index(channels)
        speed_ms = _coerce_speed_ms(frame_time, config, args.speed_ms)
        fps = int(round(1.0 / frame_time)) if frame_time > 0 else 0

        entries_with_index = []
        for entry in mapping_entries:
            idx = channel_index.get((entry['bvh_joint'], entry['channel']))
            if idx is None:
                continue
            enriched = dict(entry)
            enriched['index'] = idx
            entries_with_index.append(enriched)

        if not entries_with_index:
            print(f'Skip: BVH has no matching joint channels {bvh_path}')
            continue

        action_frames: List[List[Dict]] = []
        for frame_values in frames:
            frame_cmds: List[Dict] = []
            for entry in entries_with_index:
                angle = frame_values[entry['index']]
                pos = entry['bias'] + entry['scale'] * angle
                pos = max(entry['min'], min(entry['max'], pos))
                cmd = {
                    'id': entry['servo_id'],
                    'position': int(round(pos)),
                    'speed': speed_ms,
                    'servo_type': entry['servo_type']
                }
                frame_cmds.append(cmd)
            action_frames.append(frame_cmds)

        bvh_data[action] = {
            'fps': fps,
            'frames': action_frames,
            'source': str(bvh_path)
        }

        print(f'Generated action: {action} (frames={len(action_frames)}, fps={fps})')

    config['bvh_data'] = bvh_data
    if args.input_dir:
        config['bvh_dir'] = str(input_dir)

    output_path = args.output or config_path
    _save_json(output_path, config)
    print(f'Wrote output: {output_path}')


if __name__ == '__main__':
    main()
