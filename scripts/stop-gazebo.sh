#!/usr/bin/env bash
set -euo pipefail
LAB_DIR="$(cd "$(dirname "$0")/.." && pwd)"
STATE="$LAB_DIR/results/gazebo-session"
[[ -d "$STATE" ]] || exit 0
exec 9>"$STATE/demo.lock"
flock -n 9 || { echo 'Wait for the active flight to land before stopping Gazebo.'; exit 1; }
for role in gui server; do
  [[ -f "$STATE/$role.pid" ]] || continue
  pid="$(cat "$STATE/$role.pid")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null &&
     tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx 'GZ_PARTITION=px4-altitude-lab'; then
    kill -INT "$pid"
  fi
done
