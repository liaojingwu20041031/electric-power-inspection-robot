#!/usr/bin/env bash
set -euo pipefail

integration_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
ros_distro="${ROS_DISTRO:-humble}"

if [ ! -f "/opt/ros/${ros_distro}/setup.bash" ]; then
  echo "ROS 2 setup is missing: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 2
fi
if [ ! -f "${base_ws}/install/setup.bash" ]; then
  echo "Robot base workspace is not built: ${base_ws}" >&2
  exit 2
fi
if [ ! -f "${integration_ws}/install/setup.bash" ]; then
  echo "DG integration workspace is not built: ${integration_ws}" >&2
  exit 2
fi

# This entry point is for the cross-board graph. Do not allow a stale shell
# setting to silently isolate the process from RDK or the robot.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0

set +u
source "/opt/ros/${ros_distro}/setup.bash"
source "${base_ws}/install/setup.bash"
source "${integration_ws}/install/setup.bash"
set -u

cd "${integration_ws}"
exec ros2 launch ylhb_base dg_navigation_integration.launch.py "$@"
