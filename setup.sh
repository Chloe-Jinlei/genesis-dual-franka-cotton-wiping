#!/usr/bin/env bash
# Install into this checkout only; system packages and drivers are not changed.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  printf 'Usage: %s [python3.12-or-absolute-python-path]\n' "$0"
  printf 'Creates .venv, installs the recorded package versions and the pinned Genesis source.\n'
  exit 0
fi
if (( $# > 1 )); then
  printf 'Expected at most one Python executable argument.\n' >&2
  exit 2
fi
python_bin="${1:-python3.12}"
for required_command in "$python_bin"; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    printf 'Missing command: %s. See docs/REPRODUCING.md.\n' "$required_command" >&2
    exit 1
  fi
done
"$python_bin" -c 'import sys; assert sys.version_info[:2] == (3,12), "Python 3.12 is required for this recorded environment"'
cd "$repo_root"
test -f requirements-lock.txt
test -f environment.json
expected_commit=ffd119c30118ebb9745fc009349efb2112a8cb76
if [[ ! -f genesis-world/pyproject.toml ]]; then
  if ! command -v git >/dev/null 2>&1 || [[ ! -f .gitmodules ]]; then
    printf 'Genesis source is missing. Use a Git clone with its submodule or extract the complete release.\n' >&2
    exit 1
  fi
  git submodule update --init -- genesis-world
fi
if [[ -e genesis-world/.git ]]; then
  if ! command -v git >/dev/null 2>&1; then
    printf 'Git is required to validate this submodule checkout.\n' >&2
    exit 1
  fi
  actual_commit="$(git -C genesis-world rev-parse HEAD)"
  if [[ "$actual_commit" != "$expected_commit" ]]; then
    printf 'Genesis must be at %s; found %s. Existing source was not reset.\n' "$expected_commit" "$actual_commit" >&2
    exit 1
  fi
  if [[ -n "$(git -C genesis-world status --porcelain --untracked-files=normal)" ]]; then
    printf 'Genesis has local changes. Preserve them separately before installing the recorded source.\n' >&2
    exit 1
  fi
fi
"$python_bin" - <<'PY'
import hashlib, json
from pathlib import Path
environment = json.loads(Path('environment.json').read_text())
actual = hashlib.sha256(Path('requirements-lock.txt').read_bytes()).hexdigest()
assert actual == environment['package_lock']['sha256'], 'Package lock differs from the environment record'
manifest_path = Path('packaging/genesis-source-manifest.json')
manifest = json.loads(manifest_path.read_text())
assert manifest['commit'] == 'ffd119c30118ebb9745fc009349efb2112a8cb76', 'Unexpected engine manifest revision'
assert manifest['version'] == '1.4.3', 'Unexpected engine manifest version'
assert manifest['files'], 'Engine source manifest is empty'
for name, expected in manifest['files'].items():
    relative = Path(name)
    assert not relative.is_absolute() and '..' not in relative.parts, f'Invalid manifest path: {name}'
    source = Path('genesis-world') / relative
    assert source.is_file() and not source.is_symlink(), f'Missing regular engine source file: {name}'
    assert source.stat().st_size == expected['bytes'], f'Engine source size changed: {name}'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected['sha256'], f'Engine source hash changed: {name}'
print(f'Verified {len(manifest["files"])} pinned Genesis source files.')
PY
mkdir -p "$repo_root/.cache/pip" "$repo_root/.cache/tmp"
export PIP_CACHE_DIR="$repo_root/.cache/pip"
export TMPDIR="$repo_root/.cache/tmp"
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
if [[ ! -x "$repo_root/.venv/bin/python" ]]; then
  if [[ -e "$repo_root/.venv" ]]; then
    printf '.venv exists but has no executable Python; it was not replaced.\n' >&2
    exit 1
  fi
  "$python_bin" -m venv "$repo_root/.venv"
fi
venv_python="$repo_root/.venv/bin/python"
"$venv_python" -c 'import sys; assert sys.version_info[:2] == (3,12), "Existing .venv must use Python 3.12"; assert sys.prefix != sys.base_prefix, "Expected a virtual environment"'
"$venv_python" -m pip --version
"$venv_python" -m pip install --index-url https://pypi.org/simple \
  --extra-index-url https://download.pytorch.org/whl/cu130 \
  -r "$repo_root/requirements-lock.txt"
# All runtime dependencies were installed from the lock. Build isolation may
# obtain build tools; --no-deps prevents changing the recorded runtime versions.
"$venv_python" -m pip install --index-url https://pypi.org/simple \
  --no-deps -e "$repo_root/genesis-world[render]"
"$venv_python" -m pip check
"$venv_python" - <<'PY'
import ctypes.util
import importlib.metadata as md
import shutil
from pathlib import Path
import torch
assert md.version('genesis-world') == '1.4.3'
assert torch.__version__ == '2.14.0+cu130', f'Expected recorded CUDA build, got {torch.__version__}'
print(f'Installed Genesis {md.version("genesis-world")}, Torch {torch.__version__}; CUDA available: {torch.cuda.is_available()}')
for font in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
    if not Path(font).is_file():
        print(f'Missing system font: {font}; see docs/REPRODUCING.md')
for command in ('ffmpeg', 'ffprobe'):
    if not shutil.which(command):
        print(f'Missing system command: {command}; see docs/REPRODUCING.md')
if not ctypes.util.find_library('EGL'):
    print('EGL library not found; see docs/REPRODUCING.md')
print('Installation checks do not validate rendering, native checkpoint restoration, or the full simulation.')
PY
printf 'Environment ready. Full simulation: %s/run-complete.sh\n' "$repo_root"
