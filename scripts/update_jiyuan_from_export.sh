#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/update_jiyuan_from_export.sh [EXPORT_DIR]

Default EXPORT_DIR: ~/Desktop/jiyuan

This script:
  1) Fixes jiyuqn -> jiyuan naming in the export folder
  2) Mirrors right_*.STL -> left_*.STL
  3) Mirrors URDF in-place (jiyuan.urdf becomes bilateral)
  4) Syncs export -> src/jiyuan
  5) Updates src/robot_description to the latest jiyuan model
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

export_dir="${1:-$HOME/Desktop/jiyuan}"
ros_root="$(cd "$(dirname "$0")/.." && pwd)"
ros_src="${ros_root}/src"
jiyuan_pkg="${ros_src}/jiyuan"
robot_desc="${ros_src}/robot_description"
rl_root="$(cd "${ros_root}/../rl" && pwd)"
mirror_tool="${rl_root}/utils/urdf2ros2/mirror_tool.py"

if [[ ! -d "${export_dir}" ]]; then
  echo "Error: export dir not found: ${export_dir}"
  exit 1
fi
if [[ ! -d "${jiyuan_pkg}" ]]; then
  echo "Error: jiyuan package not found: ${jiyuan_pkg}"
  exit 1
fi
if [[ ! -d "${robot_desc}" ]]; then
  echo "Error: robot_description not found: ${robot_desc}"
  exit 1
fi
if [[ ! -f "${mirror_tool}" ]]; then
  echo "Error: mirror_tool not found: ${mirror_tool}"
  exit 1
fi

echo "[1/5] Fix name jiyuqn -> jiyuan in export folder"
if [[ -f "${export_dir}/urdf/jiyuqn.urdf" ]]; then
  mv "${export_dir}/urdf/jiyuqn.urdf" "${export_dir}/urdf/jiyuan.urdf"
fi
if [[ -f "${export_dir}/urdf/jiyuqn.csv" ]]; then
  mv "${export_dir}/urdf/jiyuqn.csv" "${export_dir}/urdf/jiyuan.csv"
fi

for f in \
  "CMakeLists.txt" \
  "package.xml" \
  "launch/display.launch" \
  "launch/gazebo.launch" \
  "launch/display.launch.py" \
  "launch/gazebo.launch.py" \
  "urdf/jiyuan.urdf" \
  "urdf/jiyuan.csv"
do
  if [[ -f "${export_dir}/${f}" ]]; then
    perl -pi -e 's/jiyuqn/jiyuan/g' "${export_dir}/${f}"
  fi
done

echo "[2/5] Mirror STL: right_*.STL -> left_*.STL"
/usr/bin/python3 - <<PY
from pathlib import Path
import struct

meshes_dir = Path(r"${export_dir}") / "meshes"
right_files = sorted(list(meshes_dir.glob("right_*.STL")) + list(meshes_dir.glob("right_*.stl")))
if not right_files:
    raise SystemExit("No right_*.STL files found in " + str(meshes_dir))

def read_stl(filepath: Path):
    with open(filepath, "rb") as f:
        header = f.read(80)
        count = struct.unpack("<I", f.read(4))[0]
        tris = []
        for _ in range(count):
            normal = struct.unpack("<3f", f.read(12))
            v1 = struct.unpack("<3f", f.read(12))
            v2 = struct.unpack("<3f", f.read(12))
            v3 = struct.unpack("<3f", f.read(12))
            attr = f.read(2)
            tris.append({"normal": normal, "v1": v1, "v2": v2, "v3": v3, "attr": attr})
    return header, tris

def mirror_stl(tris):
    mirrored = []
    for tri in tris:
        v1 = list(tri["v1"])
        v2 = list(tri["v2"])
        v3 = list(tri["v3"])
        v1[0] *= -1
        v2[0] *= -1
        v3[0] *= -1
        normal = list(tri["normal"])
        normal[0] *= -1
        v2, v3 = v3, v2
        mirrored.append({
            "normal": tuple(normal),
            "v1": tuple(v1),
            "v2": tuple(v2),
            "v3": tuple(v3),
            "attr": tri["attr"],
        })
    return mirrored

def write_stl(filepath: Path, header, tris):
    with open(filepath, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            f.write(struct.pack("<3f", *tri["normal"]))
            f.write(struct.pack("<3f", *tri["v1"]))
            f.write(struct.pack("<3f", *tri["v2"]))
            f.write(struct.pack("<3f", *tri["v3"]))
            f.write(tri["attr"])

for right_path in right_files:
    left_path = right_path.with_name(right_path.name.replace("right_", "left_", 1))
    header, tris = read_stl(right_path)
    write_stl(left_path, header, mirror_stl(tris))
    print(f"  ✓ {right_path.name} -> {left_path.name}")
PY

echo "[3/5] Mirror URDF in-place (overwrite jiyuan.urdf)"
if [[ ! -f "${export_dir}/urdf/jiyuan.urdf" ]]; then
  echo "Error: ${export_dir}/urdf/jiyuan.urdf not found"
  exit 1
fi
tmp_urdf="${export_dir}/urdf/.jiyuan_bilateral.tmp.urdf"
/usr/bin/python3 "${mirror_tool}" mirror \
  "${export_dir}/urdf/jiyuan.urdf" \
  -o "${tmp_urdf}"
mv "${tmp_urdf}" "${export_dir}/urdf/jiyuan.urdf"

echo "[4/5] Sync export -> src/jiyuan"
rsync -a "${export_dir}/meshes/" "${jiyuan_pkg}/meshes/"
rsync -a "${export_dir}/urdf/" "${jiyuan_pkg}/urdf/"
if [[ -d "${export_dir}/textures" ]]; then
  rsync -a "${export_dir}/textures/" "${jiyuan_pkg}/textures/"
fi
if [[ -d "${export_dir}/mjcf" ]]; then
  rsync -a "${export_dir}/mjcf/" "${jiyuan_pkg}/mjcf/"
fi

echo "[5/5] Update src/robot_description"
rsync -a "${jiyuan_pkg}/meshes/" "${robot_desc}/meshes/"
cp "${jiyuan_pkg}/urdf/jiyuan.urdf" "${robot_desc}/urdf/assets.urdf"
cp "${jiyuan_pkg}/urdf/jiyuan.csv" "${robot_desc}/urdf/assets.csv"
perl -pi -e 's/package:\\/\\/jiyuan\\//package:\\/\\/robot_description\\//g' \
  "${robot_desc}/urdf/assets.urdf" "${robot_desc}/urdf/assets.csv"
if [[ -d "${jiyuan_pkg}/mjcf" ]]; then
  rsync -a "${jiyuan_pkg}/mjcf/" "${robot_desc}/mjcf/"
fi

echo ""
echo "Done."
echo "Next:"
echo "  cd ${ros_root}"
echo "  colcon build --packages-select robot_description"
echo "  source install/setup.zsh"
echo "  ros2 launch robot_description display.launch.py"
