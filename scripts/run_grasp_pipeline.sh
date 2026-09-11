#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
: "${MYLEKIWI_JETSON:?set MYLEKIWI_JETSON to user@jetson-ip}"
REMOTE_ROOT="${MYLEKIWI_REMOTE_ROOT:-MyLekiwi}"
LOCAL_PYTHON="${MYLEKIWI_PYTHON:-$ROOT/.venv/bin/python}"
JETSON_PYTHON="${MYLEKIWI_JETSON_PYTHON:-}"
SOCKET="/tmp/mylekiwi-ssh-%r@%h:%p"
SSH=(-o ControlMaster=auto -o ControlPersist=60 -o ControlPath="$SOCKET")

if [[ ! -x "$LOCAL_PYTHON" && -x "$ROOT/../.venv/bin/python" ]]; then
  LOCAL_PYTHON="$ROOT/../.venv/bin/python"
fi
mkdir -p "$ROOT/outputs/captures/latest" "$ROOT/outputs/plans/latest"

CAPTURE_OUTPUT=$(env -u LD_LIBRARY_PATH /usr/bin/ssh "${SSH[@]}" "$MYLEKIWI_JETSON" \
  "cd '$REMOTE_ROOT' && MYLEKIWI_JETSON_PYTHON='$JETSON_PYTHON' ./scripts/jetson_python.sh -m mylekiwi.capture_wrist_frame")
printf '%s\n' "$CAPTURE_OUTPUT"
REMOTE_IMAGE=$(printf '%s\n' "$CAPTURE_OUTPUT" | sed -n 's/^image=//p' | tail -n1)
test -n "$REMOTE_IMAGE" || { echo "Jetson returned no wrist image"; exit 1; }

# Runtime data contract: exactly one PNG Jetson -> 4090.
env -u LD_LIBRARY_PATH /usr/bin/scp "${SSH[@]}" \
  "$MYLEKIWI_JETSON:$REMOTE_IMAGE" "$ROOT/outputs/captures/latest/wrist.png"

cd "$ROOT"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HUB_DISABLE_PROGRESS_BARS=1
"$LOCAL_PYTHON" -m mylekiwi.plan_grasp
MODE=$("$LOCAL_PYTHON" -c 'import json; print(json.load(open("outputs/plans/latest/plan.json"))["execution_mode"])')

# Runtime data contract: exactly one JSON 4090 -> Jetson.
env -u LD_LIBRARY_PATH /usr/bin/ssh "${SSH[@]}" "$MYLEKIWI_JETSON" \
  "mkdir -p '$REMOTE_ROOT/outputs/plans/latest'"
env -u LD_LIBRARY_PATH /usr/bin/scp "${SSH[@]}" \
  "$ROOT/outputs/plans/latest/plan.json" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/outputs/plans/latest/plan.json"

if [[ "$MODE" != "offset_descent_then_center_grasp" ]]; then
  echo "No executable grasp: $MODE"
  exit 2
fi
echo "plan=$REMOTE_ROOT/outputs/plans/latest/plan.json"
