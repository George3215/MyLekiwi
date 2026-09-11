#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${MYLEKIWI_JETSON_PYTHON:-$ROOT/.venv/bin/python}"
if [[ ! -x "$PYTHON" && -x "$ROOT/../lerobot/.venv/bin/python" ]]; then
  PYTHON="$ROOT/../lerobot/.venv/bin/python"
fi
[[ -x "$PYTHON" ]] || { echo "No Jetson Python found; run: uv sync --extra jetson"; exit 1; }

export PYTHONPATH="$ROOT/third_party/lerobot/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON" "$@"
