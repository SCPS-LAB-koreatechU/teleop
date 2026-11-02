#!/usr/bin/env bash
# Launch the BHL 10-motor GUI.
#   ./run.sh                 live: can0 (left) + can1 (right) from hardware_joints.yaml
#   ./run.sh --sim           no hardware, fake motors
#   ./run.sh --rviz          + RViz showing the URDF follow the motors (implies --ros)
#   ./run.sh --sim --rviz    full dry run: fake motors, real 3D model
#   ./run.sh -h              all options
#
# --rviz starts robot_state_publisher + rviz2 (display.launch.py gui:=false) as
# children of this script and kills them again when the GUI window closes, so
# there is nothing to clean up by hand.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
WORKSPACE="${REPO}/ros2_ws"

# --rviz is ours; everything else is passed through to the GUI
WANT_RVIZ=0
MODEL="v1arm"
ARGS=()
while (($#)); do
  case "$1" in
    --rviz) WANT_RVIZ=1 ;;
    --rviz-model)
      [[ $# -ge 2 ]] || { echo "[FAIL] --rviz-model needs a model name" >&2; exit 1; }
      MODEL="$2"; shift ;;
    *) ARGS+=("$1") ;;
  esac
  shift
done
((WANT_RVIZ)) && ARGS+=("--ros")

# recoil CAN driver lives in the repo's lowlevel package (not pip-installed)
LOWLEVEL="${LOWLEVEL:-${REPO}/source/berkeley_humanoid_lite_lowlevel}"
if [[ -d "${LOWLEVEL}/berkeley_humanoid_lite_lowlevel" ]]; then
  export PYTHONPATH="${LOWLEVEL}${PYTHONPATH:+:${PYTHONPATH}}"
fi

# --ros/--rviz need rclpy: source Humble if it is there and not sourced yet
NEED_ROS=0
for a in ${ARGS[@]+"${ARGS[@]}"}; do [[ "$a" == "--ros" ]] && NEED_ROS=1; done
if ((NEED_ROS)) && [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
  set +u; source /opt/ros/humble/setup.bash; set -u
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"   # same domain as ros2_ws/run_demo.sh

RVIZ_PID=""
RVIZ_PGID=""
GUI_PID=""
# Kill anything this script started. Covers both directions: closing the GUI
# window must take RViz with it, and terminating this launcher (Ctrl+C, closing
# the terminal, a test harness) must not leave a GUI driving motors.
stop_one() {
  local pid="$1" sig="$2"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null || return 0
  kill "-${sig}" "${pid}" 2>/dev/null || true
  for _ in {1..30}; do kill -0 "${pid}" 2>/dev/null || return 0; sleep 0.1; done
  kill -KILL "${pid}" 2>/dev/null || true
}
# `ros2 launch` needs a moment to bring its nodes down, and killing it outright
# orphans rviz2 and robot_state_publisher -- so signal its whole process group
# and escalate slowly.
stop_group() {
  local pgid="$1"
  [[ -n "${pgid}" ]] || return 0
  pgrep -g "${pgid}" >/dev/null 2>&1 || return 0
  for sig in INT TERM KILL; do
    kill "-${sig}" "-${pgid}" 2>/dev/null || true
    for _ in {1..50}; do
      pgrep -g "${pgid}" >/dev/null 2>&1 || return 0
      sleep 0.1
    done
  done
}
cleanup() {
  stop_one "${GUI_PID}" TERM     # the GUI's own handler E-STOPs and idles the bus
  if [[ -n "${RVIZ_PGID}" ]]; then
    stop_group "${RVIZ_PGID}"
  else
    stop_one "${RVIZ_PID}" INT
  fi
}
trap cleanup EXIT INT TERM

if ((WANT_RVIZ)); then
  if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
    echo "[FAIL] ${WORKSPACE}/install/setup.bash missing - build the workspace first" >&2
    exit 1
  fi
  set +u; source "${WORKSPACE}/install/setup.bash"; set -u
  echo "[info] starting RViz (model:=${MODEL}), closing the GUI will close it again"
  # own process group, so shutting it down cannot orphan rviz2
  setsid ros2 launch berkeley_humanoid_lite_description display.launch.py \
    "model:=${MODEL}" gui:=false &
  RVIZ_PID=$!
  sleep 0.5
  RVIZ_PGID=$(ps -o pgid= -p "${RVIZ_PID}" 2>/dev/null | tr -d ' ')
  sleep 2   # let robot_state_publisher latch /robot_description before the GUI talks
fi

python3 "${HERE}/bhl_motor_gui.py" ${ARGS[@]+"${ARGS[@]}"} &
GUI_PID=$!
wait "${GUI_PID}"
STATUS=$?
GUI_PID=""          # already gone; cleanup only has RViz left to do
exit "${STATUS}"
