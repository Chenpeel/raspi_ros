import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .bvh_player import BvhActionPlayer


def _load_json(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def _write_json(path: str, data: Dict) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')


def _default_output_path(config_path: str) -> str:
    config_file = Path(config_path)
    return str(config_file.with_name(f'{config_file.stem}.frames.json'))


def _coerce_int(value, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def convert_bvh_config(config_path: str,
                       output_path: Optional[str] = None,
                       in_place: bool = False,
                       force: bool = False,
                       drop_bvh_file: bool = False,
                       only_actions: Optional[List[str]] = None
                       ) -> Tuple[str, List[str], List[str]]:
    player = BvhActionPlayer(lambda *args: None, config_path=config_path)
    resolved_path = player._resolve_config_path(config_path)
    if not resolved_path:
        raise FileNotFoundError('BVH action config not found')

    config = _load_json(resolved_path)
    bvh_data = config.get('bvh_data') or {}
    if not isinstance(bvh_data, dict):
        raise ValueError('bvh_data must be a dict')

    action_names = list(bvh_data.keys())
    if only_actions:
        action_names = [name for name in action_names if name in only_actions]

    converted: List[str] = []
    skipped: List[str] = []

    for action_name in action_names:
        action_data = bvh_data.get(action_name)
        if not isinstance(action_data, dict):
            skipped.append(action_name)
            continue

        if 'frames' in action_data and not force:
            skipped.append(action_name)
            continue

        default_speed = _coerce_int(
            action_data.get('default_speed_ms', config.get('default_speed_ms', 33)),
            33
        )
        default_servo_type = action_data.get(
            'default_servo_type',
            config.get('default_servo_type', 'bus')
        )

        frames, frame_delay_ms = player._load_bvh_frames(
            action_name,
            action_data,
            config,
            resolved_path,
            default_speed,
            default_servo_type
        )

        if not frames:
            skipped.append(action_name)
            continue

        action_data['frames'] = frames
        if frame_delay_ms is not None:
            action_data['frame_delay_ms'] = frame_delay_ms

        if drop_bvh_file:
            for key in ('bvh_file', 'bvh_path', 'file', 'path'):
                if key in action_data:
                    del action_data[key]

        converted.append(action_name)

    if in_place:
        output_path = resolved_path
    elif not output_path:
        output_path = _default_output_path(resolved_path)

    _write_json(output_path, config)
    return output_path, converted, skipped


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Convert BVH files into static frame data.'
    )
    parser.add_argument(
        '--config',
        default='',
        help='Path to bvh_action_map.json (optional, auto-resolve if empty).'
    )
    parser.add_argument(
        '--output',
        default='',
        help='Output JSON path (default: <config>.frames.json).'
    )
    parser.add_argument(
        '--in-place',
        action='store_true',
        help='Overwrite the original config file.'
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Re-generate frames even if they already exist.'
    )
    parser.add_argument(
        '--drop-bvh-file',
        action='store_true',
        help='Remove bvh_file/path fields after conversion.'
    )
    parser.add_argument(
        '--only',
        default='',
        help='Comma-separated action names to convert.'
    )

    args = parser.parse_args()
    if args.in_place and args.output:
        raise SystemExit('Do not set --output when using --in-place')

    only_actions = [s.strip() for s in args.only.split(',') if s.strip()]

    output_path, converted, skipped = convert_bvh_config(
        args.config,
        output_path=args.output or None,
        in_place=args.in_place,
        force=args.force,
        drop_bvh_file=args.drop_bvh_file,
        only_actions=only_actions or None
    )

    print(f'Written: {output_path}')
    if converted:
        print('Converted actions:')
        for name in converted:
            print(f'  - {name}')
    if skipped:
        print('Skipped actions:')
        for name in skipped:
            print(f'  - {name}')


if __name__ == '__main__':
    main()
