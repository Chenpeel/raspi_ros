# Action Resource Package

This package stores action resources and playback helpers (BVH now, more modes later).

## Layout

- config/bvh_action_map.json: BVH action config
- config/bvh/*.bvh: BVH files
- config/bvh/name.json: BVH skeleton bones list

## BVH Config (bvh_action_map.json)

Key fields:
- bvh_list: action list (index or name triggers)
- bvh_dir: BVH directory (relative to this config)
- bvh_data: action map, recommend using bvh_file
- joint_alias: BVH bone name to robot joint key alias
- joint_map: robot joint key to servo ID mapping
- servo_limits: per-servo min/max (override defaults)

Directory default:

```
src/record_load_action/config/bvh/
```

Override the config path if needed:

```
ros2 launch websocket_bridge full_system.launch.py \
  bvh_action_file:=/path/to/bvh_action_map.json
```

Example:

```json
{
  "bvh_dir": "bvh",
  "bvh_list": [null, "walking"],
  "bvh_data": {
    "walking": { "bvh_file": "walking.bvh" }
  },
  "joint_alias": { "pelvis.L": "hip.L" },
  "joint_map": { "hip.L": { "left_hip_joint": "3" } },
  "servo_limits": { "3": { "min": 700, "max": 2300 } }
}
```

## Static Conversion (optional)

Runtime playback parses BVH directly in `record_load_action/bvh_player.py`.
If you want offline conversion to pre-baked frame data, this package is the
right place to add a converter script later.

Use the built-in converter:

```
ros2 run record_load_action bvh_static_convert --in-place
```

Or write to a new file (default):

```
ros2 run record_load_action bvh_static_convert
```

Optional flags:

- `--output /path/to/bvh_action_map.frames.json`
- `--force` re-generate existing frames
- `--drop-bvh-file` remove bvh_file/path fields
- `--only walking,strafing` convert specific actions
