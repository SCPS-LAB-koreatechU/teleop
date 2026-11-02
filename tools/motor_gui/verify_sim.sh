#!/usr/bin/env bash
# Full simulated verification of the BHL motor GUI: control logic, safety rules,
# window layout, and the RViz link. No hardware, no CAN adapter.
#
#   ./verify_sim.sh            # everything
#   ./verify_sim.sh --no-ros   # skip the RViz stage (no ROS 2 / no display)
#
# Exit code 0 only if every stage passed.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
WORKSPACE="${REPO}/ros2_ws"
export PYTHONPATH="${REPO}/source/berkeley_humanoid_lite_lowlevel${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

WANT_ROS=1
[[ "${1:-}" == "--no-ros" ]] && WANT_ROS=0

RESULTS=()
FAILED=0
stage() {   # stage <name> <command...>
  local name="$1"; shift
  echo
  echo "=== ${name} ==============================================="
  if "$@"; then
    RESULTS+=("PASS  ${name}")
  else
    RESULTS+=("FAIL  ${name}")
    FAILED=1
  fi
}

stage "control logic and safety rules" python3 "${HERE}/sim_verify.py"
stage "built-in selftest"              python3 "${HERE}/bhl_motor_gui.py" --selftest

if [[ -n "${DISPLAY:-}" ]]; then
  stage "window layout" python3 "${HERE}/bhl_motor_gui.py" --layout-check
else
  echo "[skip] window layout: no DISPLAY"
  RESULTS+=("SKIP  window layout (no DISPLAY)")
fi

# --- RViz stage -------------------------------------------------------------
ros_stage() {
  local log; log="$(mktemp)"
  local rc=0
  if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
    echo "[FAIL] ${WORKSPACE}/install/setup.bash missing - build the workspace"
    return 1
  fi
  # shellcheck disable=SC1090,SC1091
  set +u; source /opt/ros/humble/setup.bash; source "${WORKSPACE}/install/setup.bash"; set -u

  echo "launching GUI + RViz (sim, demo motion) ..."
  "${HERE}/run.sh" --sim --rviz --demo >"${log}" 2>&1 &
  local pid=$!
  # shellcheck disable=SC2064
  trap "kill ${pid} 2>/dev/null" RETURN
  sleep 18

  # `--field name` prints one python list, which beats parsing the yaml block
  local names
  names=$(( $(timeout 10 ros2 topic echo --once --field name /joint_states 2>/dev/null \
              | tr -cd ',' | wc -c) + 1 ))
  echo "  /joint_states joints: ${names}"
  local urdf_joints
  urdf_joints=$(python3 - "${WORKSPACE}/src/berkeley_humanoid_lite_description/urdf/berkeley_humanoid_lite_v1arm.urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
print(sum(1 for j in ET.parse(sys.argv[1]).getroot().findall("joint") if j.get("type") != "fixed"))
PY
)
  if [[ "${names}" != "${urdf_joints}" ]]; then
    echo "  [FAIL] URDF has ${urdf_joints} movable joints, /joint_states carries ${names}"
    echo "         (missing joints get no TF and RViz shows RobotModel in error)"
    rc=1
  fi

  # The fingertip proves the whole chain is published, not just the arm joints.
  # Sample several times: the demo holds each pose for a few seconds, so two
  # samples an exact cycle apart would look (wrongly) identical.
  local seen=() sample
  for _ in 1 2 3 4 5; do
    sample=$(timeout 8 ros2 run tf2_ros tf2_echo world Index_Tip_1 2>/dev/null \
             | grep -m1 -A1 Translation | tr -d '\n')
    [[ -n "${sample}" ]] && seen+=("${sample}")
    sleep 1
  done
  if [[ ${#seen[@]} -eq 0 ]]; then
    echo "  [FAIL] no TF world -> Index_Tip_1 (fingertip of the right hand)"
    rc=1
  else
    local uniq
    uniq=$(printf '%s\n' "${seen[@]}" | sort -u | wc -l)
    echo "  fingertip TF present, ${uniq} distinct pose(s) in ${#seen[@]} samples"
    if [[ ${uniq} -lt 2 ]]; then
      echo "  [FAIL] the model never moved while the demo was running"
      rc=1
    fi
  fi

  # bracketed patterns so pgrep cannot match this script's own command line
  for proc in rviz2 robot_state_publisher; do
    pgrep -f "lib/[${proc:0:1}]${proc:1}/${proc}" >/dev/null \
      || { echo "  [FAIL] ${proc} is not running"; rc=1; }
  done

  echo "terminating the launcher (as Ctrl+C or closing the terminal would) ..."
  kill ${pid} 2>/dev/null
  sleep 12
  for proc in rviz2 robot_state_publisher; do
    if pgrep -f "lib/[${proc:0:1}]${proc:1}/${proc}" >/dev/null; then
      echo "  [FAIL] ${proc} survived the launcher"
      rc=1
    fi
  done
  if pgrep -f "[b]hl_motor_gui.py" >/dev/null; then
    echo "  [FAIL] a GUI process survived the launcher (it would keep driving motors)"
    rc=1
  fi
  [[ ${rc} -eq 0 ]] && echo "  RViz stage OK: joints published, model moves, everything closed"
  return ${rc}
}

if [[ ${WANT_ROS} -eq 1 && -n "${DISPLAY:-}" ]]; then
  stage "RViz link" ros_stage
else
  echo "[skip] RViz link"
  RESULTS+=("SKIP  RViz link")
fi

echo
echo "=== summary ================================================"
printf '%s\n' "${RESULTS[@]}"
echo
[[ ${FAILED} -eq 0 ]] && echo "SIM VERIFICATION PASSED" || echo "SIM VERIFICATION FAILED"
exit ${FAILED}
