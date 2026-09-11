#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
: "${MYLEKIWI_JETSON:?set MYLEKIWI_JETSON to user@jetson-ip}"
REMOTE_ROOT="${MYLEKIWI_REMOTE_ROOT:-MyLekiwi}"
SOCKET="/tmp/mylekiwi-ssh-%r@%h:%p"
SSH=(-o ControlMaster=auto -o ControlPersist=60 -o ControlPath="$SOCKET")

env -u LD_LIBRARY_PATH /usr/bin/ssh "${SSH[@]}" "$MYLEKIWI_JETSON" \
  "mkdir -p '$REMOTE_ROOT'"
env -u LD_LIBRARY_PATH /usr/bin/scp -r "${SSH[@]}" \
  "$ROOT/mylekiwi" "$ROOT/configs" "$ROOT/robot" \
  "$MYLEKIWI_JETSON:$REMOTE_ROOT/"
echo "Jetson code synced to $REMOTE_ROOT"
