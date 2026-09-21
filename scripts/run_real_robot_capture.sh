#!/usr/bin/env bash
set -euo pipefail

integration_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"

set +u
source "/opt/ros/${ros_distro}/setup.bash"
source "/home/nvidia/ros2_DL/install/setup.bash"
source "${integration_ws}/install/setup.bash"
set -u

cd "${integration_ws}"
exec python3 "${integration_ws}/scripts/real_robot_capture.py" "$@"
