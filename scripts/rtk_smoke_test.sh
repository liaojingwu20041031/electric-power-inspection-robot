#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_DL_logs}"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/humble/setup.bash

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "ERROR: missing $WS_DIR/install/setup.bash; build ylhb_base first." >&2
  exit 1
fi
source "$WS_DIR/install/setup.bash"

echo "+ ros2 pkg prefix ylhb_base"
ros2 pkg prefix ylhb_base

echo "+ ls -l /dev/rtk_4g"
ls -l /dev/rtk_4g

bringup_pid=""
cleanup() {
  if [ -n "$bringup_pid" ] && kill -0 "$bringup_pid" 2>/dev/null; then
    kill -INT "$bringup_pid" 2>/dev/null || true
    wait "$bringup_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "+ ros2 launch ylhb_base bringup.launch.py enable_rtk:=true"
ros2 launch ylhb_base bringup.launch.py enable_rtk:=true &
bringup_pid="$!"

echo "+ waiting for /gps/fix"
timeout 20 bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/nvidia/ros2_DL/install/setup.bash
  until ros2 topic list --no-daemon | grep -qx "/gps/fix"; do
    sleep 0.5
  done
'

echo "+ ros2 topic echo /gps/fix --once"
timeout 10 ros2 topic echo /gps/fix --once

echo "+ ros2 topic echo /gps/rtk_status --once"
timeout 10 ros2 topic echo /gps/rtk_status --once
