#!/usr/bin/env bash
# Re-run an already configured VM and copy actual logs/plots back to this Mac.
set -euo pipefail
LAB_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VM_TARGET="${PX4_LAB_VM:-kai@192.168.64.2}"
SIMULATOR="${1:-sih}"
CONTROLLER="${2:-pid}"
TIMING_MODE="${4:-scheduled}"
GROUND_INIT="${5:-}"
if [[ -z "$GROUND_INIT" ]]; then
  if [[ "$SIMULATOR" == gazebo ]]; then GROUND_INIT=settled; else GROUND_INIT=legacy; fi
fi
MOTION_PROFILE="${6:-gentle}"
BIAS_COMPENSATION="${7:-off}"
case "$GROUND_INIT" in legacy|settled) ;; *) echo 'Ground init must be legacy or settled'; exit 2 ;; esac
case "$MOTION_PROFILE" in standard|gentle) ;; *) echo 'Motion profile must be standard or gentle'; exit 2 ;; esac
case "$BIAS_COMPENSATION" in off|integral) ;; *) echo 'Bias compensation must be off or integral'; exit 2 ;; esac
case "$TIMING_MODE" in legacy|scheduled) ;; *) echo 'Timing mode must be legacy or scheduled'; exit 2 ;; esac
MPC_MODEL="${3:-}"
if [[ -z "$MPC_MODEL" ]]; then
  if [[ "$SIMULATOR" == gazebo ]]; then MPC_MODEL=calibrated; else MPC_MODEL=nominal; fi
fi
case "$MPC_MODEL" in nominal|calibrated) ;; *) echo 'MPC model must be nominal or calibrated'; exit 2 ;; esac
case "$CONTROLLER" in pid|mpc) ;; *) echo 'Controller must be pid or mpc'; exit 2 ;; esac
case "$SIMULATOR" in sih|gazebo) ;; *) echo 'Usage: run-on-vm.sh [sih|gazebo] [pid|mpc] [nominal|calibrated] [legacy|scheduled] [legacy|settled] [standard|gentle] [off|integral]'; exit 2 ;; esac
SESSION_DIR="$(mktemp -d /tmp/px4-lab-session.XXXXXX)"
CONTROL_SOCKET="$SESSION_DIR/ssh"
cleanup() {
  ssh -S "$CONTROL_SOCKET" -O exit "$VM_TARGET" >/dev/null 2>&1 || true
  rmdir "$SESSION_DIR" 2>/dev/null || true
}
trap cleanup EXIT
ssh -M -S "$CONTROL_SOCKET" -o ControlPersist=600 -fN "$VM_TARGET"
rsync -a -e "ssh -S $CONTROL_SOCKET" "$LAB_DIR/scripts/" "$VM_TARGET:~/px4-altitude-lab/scripts/"
rsync -a -e "ssh -S $CONTROL_SOCKET" "$LAB_DIR/requirements-mpc.txt" "$VM_TARGET:~/px4-altitude-lab/"
rsync -a -e "ssh -S $CONTROL_SOCKET" "$LAB_DIR/config/" "$VM_TARGET:~/px4-altitude-lab/config/"
if [[ "$SIMULATOR" == gazebo ]]; then
  ssh -S "$CONTROL_SOCKET" "$VM_TARGET" "bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller $CONTROLLER --mpc-model $MPC_MODEL --timing-mode $TIMING_MODE --ground-init $GROUND_INIT --motion-profile $MOTION_PROFILE --bias-compensation $BIAS_COMPENSATION"
else
  ssh -S "$CONTROL_SOCKET" "$VM_TARGET" "source /opt/ros/jazzy/setup.bash; cd ~/px4-altitude-lab && OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_experiment.py --controller $CONTROLLER --mpc-model $MPC_MODEL --timing-mode $TIMING_MODE --ground-init $GROUND_INIT --motion-profile $MOTION_PROFILE --bias-compensation $BIAS_COMPENSATION --label sih-$CONTROLLER-$MPC_MODEL && .venv/bin/python scripts/analyze_latest.py"
fi
rsync -a -e "ssh -S $CONTROL_SOCKET" "$VM_TARGET:~/px4-altitude-lab/results/" "$LAB_DIR/results/"
printf 'Results saved to %s/results\n' "$LAB_DIR"
