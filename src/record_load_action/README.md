# Action Resource Package

This package stores action resources and playback helpers (BVH now, more modes later).

## Layout

- config/bvh_action_map.json: BVH action config
- config/bvh/*.bvh: BVH files
- config/bvh/*.json: per-action frame data (for example `walking.json`)
- config/bvh/name.json: BVH skeleton bones list

## BVH Config (bvh_action_map.json)

Key fields:
- bvh_list: action list (index or name triggers)
- bvh_dir: BVH directory (relative to this config)
- bvh_action_files: action file map, supports `action_name -> action.json` and `name.bvh -> name.json`
- bvh_data: inline action map (legacy fallback, still supported)
- joint_alias: BVH bone name to robot joint key alias
- joint_map: robot joint key to servo ID mapping
- servo_limits: per-servo min/max (override defaults)

Directory default:

```
config/bvh/
```

Override the config path if needed:

```
ros2 launch websocket_bridge full_system.launch.py \
  bvh_action_file:=<配置文件路径>
```

Example:

```json
{
  "bvh_dir": "bvh",
  "bvh_list": [null, "walking"],
  "bvh_action_files": {
    "walking.bvh": "walking.json"
  },
  "joint_alias": { "pelvis.L": "hip.L" },
  "joint_map": { "hip.L": { "left_hip_joint": "3" } },
  "servo_limits": { "3": { "min": 700, "max": 2300 } }
}
```

Action file (`config/bvh/walking.json`) example:

```json
{
  "bvh_file": "walking.bvh",
  "frame_delay_ms": 33.0,
  "frames": [
    [{ "id": 1, "position": 1500, "speed": 33, "servo_type": "bus" }]
  ]
}
```

## Static Conversion (optional)

Runtime playback parses BVH directly in `record_load_action/bvh_player.py`.
Use `bvh_static_convert` when you want offline conversion to pre-baked frames.

Use the built-in converter:

```
ros2 run record_load_action bvh_static_convert --in-place
```

Or write to a new file (default):

```
ros2 run record_load_action bvh_static_convert
```

Notes:
- `--in-place` updates source files directly.
- For decoupled config, action files in `config/bvh/*.json` are updated in-place.
- Without `--in-place`, output is a merged JSON file with converted `bvh_data`.

Optional flags:

- `--output ./bvh_action_map.frames.json`
- `--force` re-generate existing frames
- `--drop-bvh-file` remove bvh_file/path fields
- `--only walking,strafing` convert specific actions
