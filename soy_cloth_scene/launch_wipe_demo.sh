#!/usr/bin/env bash
set -euo pipefail
scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$scene_dir/launch_viewer.sh"
