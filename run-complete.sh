#!/usr/bin/env bash
# Reuse the archived production runner and write a new result directory.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin="$repo_root/.venv/bin/python"
if [[ ! -x "$python_bin" ]]; then
  printf 'Missing repository .venv. Run ./setup.sh first; see docs/REPRODUCING.md.\n' >&2
  exit 1
fi
runner="$repo_root/soy_cloth_scene/run_complete_contact_first.py"
test -f "$runner"
export PYOPENGL_PLATFORM=egl
export GS_CACHE_FILE_PATH="$repo_root/.cache/genesis"
export XDG_CACHE_HOME="$repo_root/.cache"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
cd "$repo_root"
has_output=false
is_help=false
for argument in "$@"; do
  case "$argument" in
    --output|--output=*) has_output=true ;;
    -h|--help) is_help=true ;;
  esac
done
extra_args=()
if [[ "$has_output" == false && "$is_help" == false ]]; then
  mkdir -p "$repo_root/soy_cloth_scene/output"
  output_dir="$(mktemp -d "$repo_root/soy_cloth_scene/output/complete_$(date -u +%Y%m%dT%H%M%SZ)_XXXXXX")"
  extra_args=(--output "$output_dir")
  printf 'New simulation output: %s\n' "$output_dir"
fi
exec "$python_bin" "$runner" "${extra_args[@]}" "$@"
