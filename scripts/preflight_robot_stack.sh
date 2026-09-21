#!/usr/bin/env bash
set -u

# Read-only runtime gate for the NVIDIA robot.  It deliberately does not
# start ROS nodes, configure CAN, publish a topic, or touch the filesystem.

WS_DIR="${WS_DIR:-${HOME}/ros2_DL}"
CAN_IFACE="${CAN_IFACE:-can1}"
BITRATE="${BITRATE:-500000}"
failures=0

pass_check() { printf '[PASS] %s\n' "$1"; }
fail_check() { printf '[FAIL] %s\n' "$1"; failures=$((failures + 1)); }
info_check() { printf '[INFO] %s\n' "$1"; }

if ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
  can_details="$(ip -details link show "${CAN_IFACE}" 2>/dev/null || true)"
  if grep -Eq 'state UP|<[^>]*UP[^>]*>' <<<"${can_details}" \
      && grep -Eq "bitrate[[:space:]]+${BITRATE}([[:space:]]|$)" <<<"${can_details}"; then
    pass_check "${CAN_IFACE} exists, is UP, and reports bitrate ${BITRATE}"
  else
    fail_check "${CAN_IFACE} is present but is not UP at bitrate ${BITRATE}"
  fi
else
  fail_check "CAN interface ${CAN_IFACE} is missing"
fi

for device in /dev/robot_lidar /dev/robot_imu; do
  if [[ -e "${device}" ]]; then
    pass_check "${device} exists"
  else
    fail_check "${device} is missing"
  fi
done

if ! command -v ros2 >/dev/null 2>&1; then
  fail_check "ros2 command is unavailable"
  printf '\nPreflight result: FAIL (%d check(s))\n' "${failures}"
  exit 1
fi

ros2_node_list="$(ros2 node list 2>/dev/null || true)"
if grep -qE '/?zlac8015d_canopen_controller([[:space:]]|$)' <<<"${ros2_node_list}"; then
  pass_check "ZLAC8015D controller node is visible"
else
  fail_check "ZLAC8015D controller node is not visible"
fi

topic_info() { timeout 4s ros2 topic info "$1" 2>/dev/null || true; }
publisher_count() { topic_info "$1" | awk '/Publisher count:/{print $3; exit}'; }
subscriber_count() { topic_info "$1" | awk '/Subscription count:/{print $3; exit}'; }

for topic in /odom /scan /imu/data /tf /tf_static; do
  count="$(publisher_count "${topic}")"
  if [[ "${count:-0}" =~ ^[1-9][0-9]*$ ]]; then
    pass_check "${topic} has ${count} publisher(s)"
  else
    fail_check "${topic} has no publisher"
  fi
done

cmd_subscribers="$(subscriber_count /cmd_vel)"
if [[ "${cmd_subscribers:-0}" =~ ^[1-9][0-9]*$ ]]; then
  pass_check "/cmd_vel has ${cmd_subscribers} subscriber(s)"
else
  fail_check "/cmd_vel has no bottom-controller subscriber"
fi

arbiter_publishers="$(publisher_count /cmd_vel)"
if [[ "${arbiter_publishers:-0}" =~ ^[1-9][0-9]*$ ]]; then
  info_check "/cmd_vel publisher count is ${arbiter_publishers}; inspect the node list above before motion"
else
  fail_check "/cmd_vel has no arbiter output publisher"
fi

if [[ -f "${WS_DIR}/maps/my_map.yaml" ]]; then
  pass_check "map file exists: ${WS_DIR}/maps/my_map.yaml"
else
  fail_check "map file is missing: ${WS_DIR}/maps/my_map.yaml"
fi

route_count="$(find "${WS_DIR}/maps" -maxdepth 1 -type f -name 'route_patrol_*.json' 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${route_count:-0}" =~ ^[1-9][0-9]*$ ]]; then
  pass_check "found ${route_count} patrol route file(s)"
else
  fail_check "no route_patrol_*.json found under ${WS_DIR}/maps"
fi

printf '\nPreflight result: %s (%d check(s))\n' \
  "$(if (( failures == 0 )); then printf PASS; else printf FAIL; fi)" "${failures}"
if (( failures != 0 )); then
  info_check "No motion command was published by this script. Fix the reported gate(s) first."
  exit 1
fi
