#!/usr/bin/env bash
set -euo pipefail
genesis_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (( $# == 0 )); then
  set -- --robot-wipe --hold-open
fi
exec bash "$genesis_root/soy_cloth_scene/run.sh" "$@"
