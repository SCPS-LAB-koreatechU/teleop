#!/usr/bin/env bash
# Fetch the third-party DexHand descriptions this workspace builds against.
#
# They are NOT committed here. Both are CC BY-NC-SA 4.0 while this repository is
# MIT, and ShareAlike would drag that licence across anything derived from them.
# Keeping them out and fetching on demand keeps the licences from mixing, and
# keeps ~15 MB of other people's STLs out of the history.
#
# What lands in vendor/:
#   dexhandv2_description  - V2 hand: the fingers this build actually uses
#   dexhand_v1_description - V1 forearm and wrist: the real wrist kinematics
#
# Run once after cloning, then generate:
#   ./fetch_vendor.sh
#   python3 src/berkeley_humanoid_lite_description/mirror_meshes.py
#   python3 src/berkeley_humanoid_lite_description/generate_urdf.py
#   colcon build --symlink-install

set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENDOR="${HERE}/vendor"
mkdir -p "${VENDOR}"

clone_or_update() {
  local name="$1" url="$2"
  local target="${VENDOR}/${name}"
  if [[ -d "${target}/.git" ]]; then
    echo "updating ${name}"
    git -C "${target}" pull --ff-only --quiet
  elif [[ -d "${target}" ]]; then
    echo "${name} exists but is not a git checkout; leaving it alone"
  else
    echo "cloning ${name}"
    git clone --depth 1 --quiet "${url}" "${target}"
  fi
}

clone_or_update dexhandv2_description \
  https://github.com/iotdesignshop/dexhandv2_description.git
clone_or_update dexhand_v1_description \
  https://github.com/iotdesignshop/dexhand_description.git

# The robot's own meshes (CC BY-SA 4.0) come from the upstream assets repo. The
# description package links to them as ../../../source/berkeley_humanoid_lite_assets.
ASSETS="${HERE}/../source/berkeley_humanoid_lite_assets"
if [[ -d "${ASSETS}/.git" ]]; then
  echo "updating berkeley_humanoid_lite_assets"
  git -C "${ASSETS}" pull --ff-only --quiet
elif [[ ! -d "${ASSETS}" ]]; then
  echo "cloning berkeley_humanoid_lite_assets"
  git clone --depth 1 --quiet https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets.git "${ASSETS}"
fi

# vendor/ holds ROS packages of its own. Without this colcon would try to build
# them alongside ours, which is neither wanted nor our problem.
touch "${VENDOR}/COLCON_IGNORE"

# The description package reaches the meshes through relative symlinks, so a
# clone anywhere works. An absolute path here would break on every other machine.
DESC="${HERE}/src/berkeley_humanoid_lite_description"
ln -sfn ../../vendor/dexhandv2_description/meshes "${DESC}/meshes_dexhand"
ln -sfn ../../vendor/dexhand_v1_description/meshes "${DESC}/meshes_v1"

echo
echo "vendor/ ready:"
du -sh "${VENDOR}"/* 2>/dev/null || true
echo
echo "Both are CC BY-NC-SA 4.0. Anything derived from them (including the"
echo "mirrored left-hand meshes) carries that licence, not this repo's MIT."
