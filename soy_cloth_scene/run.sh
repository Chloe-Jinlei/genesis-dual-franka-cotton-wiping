#!/usr/bin/env bash
set -euo pipefail
scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
genesis_root="$(dirname -- "$scene_dir")"
export GS_CACHE_FILE_PATH="$genesis_root/.cache/genesis"
export XDG_CACHE_HOME="$genesis_root/.cache"
export PYTHONDONTWRITEBYTECODE=1
exec "$genesis_root/.venv/bin/python" "$scene_dir/scene.py" "$@"
