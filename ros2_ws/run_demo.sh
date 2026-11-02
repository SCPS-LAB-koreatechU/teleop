#!/usr/bin/env bash
# Launch the arm MoveIt demo on an isolated ROS domain.
#
# A separate dexhand MoveIt session usually runs on this machine on the default
# domain (0), driving real hardware over /dev/ttyACM0. Sharing a domain with it
# is not merely noisy: RViz subscribes to /monitored_planning_scene, receives
# the dexhand scene, tries to apply dexhand joint variables to the humanoid
# model and aborts with
#
#   Variable 'R_Index_Yaw' is not known to model 'berkeley-humanoid-lite'
#
# Overriding ROS_DOMAIN_ID keeps the two discovery graphs apart.
#
#   ./run_demo.sh                 # RViz + move_group
#   ./run_demo.sh use_rviz:=false # headless, for check_ik.py
#   ./run_demo.sh hardware:=true  # real controllers, all in dry run (see demo.launch.py)
#
# Pass a different domain with ROS_DOMAIN_ID=NN ./run_demo.sh

# No `set -u`: the ROS setup scripts reference unbound variables by design.
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "workspace not built; run: colcon build --symlink-install" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"

# hardware:=true starts the DexHand drivers from the dexhand workspace; source
# it when present so the hand packages resolve. Harmless for pure simulation.
DEXHAND_WS="${DEXHAND_WS:-${HOME}/Desktop/dexhand_moveit_ws}"
if [[ -f "${DEXHAND_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${DEXHAND_WS}/install/setup.bash"
fi

# hardware:=true live:=true needs the recoil CAN driver, which lives in the
# repo's lowlevel package and is not pip-installed here.
LOWLEVEL="${LOWLEVEL:-${WORKSPACE}/../source/berkeley_humanoid_lite_lowlevel}"
if [[ -d "${LOWLEVEL}/berkeley_humanoid_lite_lowlevel" ]]; then
  export PYTHONPATH="${LOWLEVEL}${PYTHONPATH:+:${PYTHONPATH}}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
if [[ "${ROS_DOMAIN_ID}" == "0" ]]; then
  echo "refusing to run on domain 0; it collides with the dexhand session" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 launch berkeley_humanoid_lite_moveit_config demo.launch.py "$@"
