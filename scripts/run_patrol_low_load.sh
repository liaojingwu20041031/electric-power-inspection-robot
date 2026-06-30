#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_DL}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MAP_PATH="${MAP_PATH:-${WS_DIR}/maps/my_map.yaml}"

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

cd "${WS_DIR}"
source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source_ros_setup "${WS_DIR}/install/setup.bash"
fi

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

ros2 launch ylhb_base bringup.launch.py "$@" &
pids+=("$!")
sleep 5

ros2 launch ylhb_base navigation.launch.py map:="${MAP_PATH}" &
pids+=("$!")
sleep 8

ros2 launch ylhb_mobile_bridge patrol_executor.launch.py auto_start:=false &
pids+=("$!")
sleep 3

ros2 topic pub --once /patrol/command std_msgs/msg/String "{data: start}"
wait
