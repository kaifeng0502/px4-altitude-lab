#!/usr/bin/env bash
# Ubuntu VM: retain the Gazebo window after the experiment lands.
set -eo pipefail
LAB_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTROLLER=pid
MPC_MODEL=calibrated
TIMING_MODE=scheduled
GROUND_INIT=settled
MOTION_PROFILE=gentle
BIAS_COMPENSATION=off
VIEW_ONLY=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --view-only) VIEW_ONLY=true; shift ;;
    --ground-init) GROUND_INIT="${2:?Expected legacy or settled}"; shift 2 ;;
    --motion-profile) MOTION_PROFILE="${2:?Expected standard or gentle}"; shift 2 ;;
    --bias-compensation) BIAS_COMPENSATION="${2:?Expected off or integral}"; shift 2 ;;
    --timing-mode) TIMING_MODE="${2:?Expected legacy or scheduled}"; shift 2 ;;
    --mpc-model) MPC_MODEL="${2:?Expected nominal or calibrated}"; shift 2 ;;
    --controller) CONTROLLER="${2:?Expected pid or mpc}"; shift 2 ;;
    *) echo 'Usage: run-gazebo.sh [--view-only] [--controller pid|mpc] [--mpc-model nominal|calibrated] [--timing-mode legacy|scheduled] [--ground-init legacy|settled] [--motion-profile standard|gentle] [--bias-compensation off|integral]'; exit 2 ;;
  esac
done
case "$CONTROLLER" in pid|mpc) ;; *) echo 'Controller must be pid or mpc'; exit 2 ;; esac
case "$MPC_MODEL" in nominal|calibrated) ;; *) echo 'MPC model must be nominal or calibrated'; exit 2 ;; esac
case "$TIMING_MODE" in legacy|scheduled) ;; *) echo 'Timing mode must be legacy or scheduled'; exit 2 ;; esac
case "$GROUND_INIT" in legacy|settled) ;; *) echo 'Ground init must be legacy or settled'; exit 2 ;; esac
case "$MOTION_PROFILE" in standard|gentle) ;; *) echo 'Motion profile must be standard or gentle'; exit 2 ;; esac
case "$BIAS_COMPENSATION" in off|integral) ;; *) echo 'Bias compensation must be off or integral'; exit 2 ;; esac
source /opt/ros/jazzy/setup.bash
source "$LAB_DIR/vendor/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_PARTITION=px4-altitude-lab
export GZ_IP=127.0.0.1
export QT_QPA_PLATFORM=xcb
# Ogre 2 runs through GLX/llvmpipe on this UTM VM. Disabling EGL vendor
# discovery is scoped to the GUI: Ogre's optional EGL device enumeration
# otherwise crashes in Mesa when software rendering meets a hardware device.
GUI_COMMAND=(env LIBGL_ALWAYS_SOFTWARE=1 QT_XCB_GL_INTEGRATION=xcb_glx
  __EGL_VENDOR_LIBRARY_FILENAMES=/dev/null gz sim -g --render-engine ogre2)
while IFS='=' read -r name value; do
  case "$name" in
    DISPLAY|XAUTHORITY|XDG_RUNTIME_DIR|DBUS_SESSION_BUS_ADDRESS) export "$name=$value" ;;
  esac
done < <(systemctl --user show-environment)
: "${DISPLAY:?Log into the Ubuntu desktop first}"
STATE="$LAB_DIR/results/gazebo-session"
mkdir -p "$STATE"
exec 9>"$STATE/demo.lock"
flock -n 9 || { echo 'An altitude experiment is already running.'; exit 1; }

# A dedicated partition avoids attaching to unrelated ROS/Gazebo worlds.
start_process() {
  local role="$1"
  shift
  local pidfile="$STATE/$role.pid"
  if [[ -f "$pidfile" ]]; then
    local pid
    pid="$(cat "$pidfile")"
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null &&
       tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx 'GZ_PARTITION=px4-altitude-lab'; then
      return
    fi
  fi
  nohup "$@" > "$STATE/$role.log" 2>&1 < /dev/null 9>&- &
  echo "$!" > "$pidfile"
}

# Reusing a removed/recreated model produced inconsistent sensor/ground-truth
# data in a trial. Every measured flight therefore gets a fresh owned server
# and GUI; --view-only preserves the existing session.
stop_owned() {
  local role="$1" pid
  [[ -f "$STATE/$role.pid" ]] || return 0
  pid="$(cat "$STATE/$role.pid")"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx 'GZ_PARTITION=px4-altitude-lab' || {
    echo "Refusing to stop an unverified $role process"; exit 1;
  }
  kill -INT "$pid"
  for attempt in $(seq 1 40); do
    kill -0 "$pid" 2>/dev/null || return 0
    # An adopted zombie has exited; it cannot hold sockets or render resources.
    [[ "$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)" == Z ]] && return 0
    sleep .1
  done
  kill -TERM "$pid"
  sleep 1
}
if ! $VIEW_ONLY; then
  flock -n "$LAB_DIR/results/px4-instance4.lock" -c true || {
    echo 'Wait for the active project PX4 flight to finish'; exit 1;
  }
  stop_owned gui
  stop_owned server
fi

# The basic X500 has no streaming camera; omit the optional unbuilt plugin.
python3 - "$PX4_GZ_SERVER_CONFIG" "$STATE/server.config" <<'PY'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1])
for parent in tree.iter():
    for child in list(parent):
        if 'GstCameraSystem' in child.attrib.get('filename', ''):
            parent.remove(child)
tree.write(sys.argv[2])
PY
export GZ_SIM_SERVER_CONFIG_PATH="$STATE/server.config"
SERVER_RUN=()
$VIEW_ONLY || SERVER_RUN=(-r)
start_process server gz sim "${SERVER_RUN[@]}" -s "$PX4_GZ_WORLDS/default.sdf"
ready=false
for attempt in $(seq 1 30); do
  if timeout 3 gz service -l | grep -qx '/world/default/create'; then
    ready=true
    break
  fi
  sleep 1
done
$ready || { echo "Gazebo startup failed: $STATE/server.log"; exit 1; }
start_process gui "${GUI_COMMAND[@]}"
$VIEW_ONLY && exit 0

# A view-only session may be paused. Resume only for an explicitly run flight.
response="$(gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false')"
[[ "$response" == *'data: true'* ]] || { echo 'Could not resume the simulation.'; exit 1; }
cd "$LAB_DIR"
LABEL="gazebo-$CONTROLLER"
[[ "$CONTROLLER" == mpc ]] && LABEL="${LABEL}-${MPC_MODEL}-${TIMING_MODE}-${GROUND_INIT}-${MOTION_PROFILE}-${BIAS_COMPENSATION}"
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_experiment.py --simulator gazebo --controller "$CONTROLLER" --mpc-model "$MPC_MODEL" --timing-mode "$TIMING_MODE" --ground-init "$GROUND_INIT" --motion-profile "$MOTION_PROFILE" --bias-compensation "$BIAS_COMPENSATION" --label "$LABEL"
.venv/bin/python scripts/analyze_latest.py
# Reopen the viewer if a VM rendering fault closed it; physics is independent.
start_process gui "${GUI_COMMAND[@]}"
echo 'Flight finished. Gazebo stays open; rerun this script for another flight.'
