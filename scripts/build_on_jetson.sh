#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_DL}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

echo "[ylhb] Building on Jetson"
echo "[ylhb] Workspace: ${WS_DIR}"

cd "${WS_DIR}"
source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

# Keep user-installed Python packages from shadowing ROS/Ubuntu build tooling.
# Runtime nodes may still use user packages; this only affects colcon build.
export PYTHONNOUSERSITE=1

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

echo "[ylhb] Build finished. Run: source ${WS_DIR}/install/setup.bash"
