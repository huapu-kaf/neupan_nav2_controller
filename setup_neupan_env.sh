#!/usr/bin/env bash
set -euo pipefail

# One-time environment bootstrap for NeuPAN python deps.
# - Creates/updates ./neupan_env
# - Installs python deps from src/neupan_nav2_controller/requirements.txt
#
# Usage:
#   ./scripts/setup_neupan_env.sh
#   PYTHON_BIN=/usr/bin/python3 ./scripts/setup_neupan_env.sh
#   VENV_DIR=/abs/path/to/neupan_env ./scripts/setup_neupan_env.sh

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$WS_DIR/neupan_env}"
REQ_FILE="$WS_DIR/src/neupan_nav2_controller/requirements.txt"

if [[ ! -f "$REQ_FILE" ]]; then
  echo "[ERROR] requirements.txt not found: $REQ_FILE" >&2
  exit 1
fi

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[INFO] Creating venv: $VENV_DIR"
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

echo "[INFO] Upgrading pip tooling"
python -m pip install -q --upgrade pip setuptools wheel

echo "[INFO] Installing NeuPAN python requirements"
# torch==...+cpu typically needs the CPU wheel index.
python -m pip install -r "$REQ_FILE" --extra-index-url https://download.pytorch.org/whl/cpu

echo "[INFO] Verifying imports & versions"
python - <<'PY'
import sys
pkgs = [
  'numpy','scipy','torch','yaml','cvxpy','cvxpylayers','diffcp','ecos'
]
print('python', sys.version)
for p in pkgs:
  try:
    m=__import__(p)
    ver=getattr(m, '__version__', '(no __version__)')
    print(f'{p} {ver}')
  except Exception as e:
    print(f'{p} import failed:', repr(e))
    raise
print('OK')
PY

echo "[INFO] Done. Next steps:" \
  && echo "  - source $WS_DIR/install/setup.bash" \
  && echo "  - source $VENV_DIR/bin/activate" \
  && echo "  - (optional) run ./scripts/complete_build.sh or ./scripts/neupan_build.sh"
#严格按照/src下放neupan插件    /scripts下放脚本