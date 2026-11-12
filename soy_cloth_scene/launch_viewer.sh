#!/usr/bin/env bash
set -euo pipefail
scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$scene_dir/output/robot_live"
exec bash "$scene_dir/run.sh" --robot-wipe --hold-open --output "$scene_dir/output/robot_live" > "$scene_dir/output/viewer.log" 2>&1
