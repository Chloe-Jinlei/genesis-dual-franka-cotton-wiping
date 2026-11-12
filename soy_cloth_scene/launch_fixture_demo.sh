#!/usr/bin/env bash
set -euo pipefail
scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$scene_dir/output/wipe_live"
exec bash "$scene_dir/run.sh" --demo-wipe --output "$scene_dir/output/wipe_live" > "$scene_dir/output/wipe_live.log" 2>&1
