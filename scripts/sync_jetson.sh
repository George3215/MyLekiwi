#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
: "${MYLEKIWI_JETSON:?set MYLEKIWI_JETSON to user@jetson-ip}"
REMOTE_ROOT="${MYLEKIWI_REMOTE_ROOT:-MyLekiwi}"
SOCKET="/tmp/mylekiwi-ssh-%r@%h:%p"
SSH=(-o ControlMaster=auto -o ControlPersist=60 -o ControlPath="$SOCKET")

env -u LD_LIBRARY_PATH /usr/bin/ssh "${SSH[@]}" "$MYLEKIWI_JETSON" \
  "mkdir -p '$REMOTE_ROOT/mylekiwi' '$REMOTE_ROOT/third_party/lerobot/src'"
env -u LD_LIBRARY_PATH /usr/bin/scp "${SSH[@]}" \
  "$ROOT"/mylekiwi/*.py \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/mylekiwi/"
env -u LD_LIBRARY_PATH /usr/bin/scp "${SSH[@]}" \
  "$ROOT/pyproject.toml" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/pyproject.toml"
env -u LD_LIBRARY_PATH /usr/bin/scp -r "${SSH[@]}" \
  "$ROOT/configs" "$ROOT/robot" "$ROOT/scripts" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/"
env -u LD_LIBRARY_PATH /usr/bin/scp "${SSH[@]}" \
  "$ROOT/third_party/lerobot/pyproject.toml" \
  "$ROOT/third_party/lerobot/README.md" \
  "$ROOT/third_party/lerobot/LICENSE" \
  "$ROOT/third_party/lerobot/VENDORED_VERSION.md" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/third_party/lerobot/"
env -u LD_LIBRARY_PATH /usr/bin/scp -r "${SSH[@]}" \
  "$ROOT/third_party/lerobot/src/lerobot" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/third_party/lerobot/src/"
echo "Jetson runtime and pinned LeRobot source synced to $REMOTE_ROOT"
