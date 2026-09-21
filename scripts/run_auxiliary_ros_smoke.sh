#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
ros_distro="${ROS_DISTRO:-humble}"
output_dir="${1:-${repo_root}/log/auxiliary_ros_smoke_$(date +%Y%m%d_%H%M%S)}"

for setup in "/opt/ros/${ros_distro}/setup.bash" "${base_ws}/install/setup.bash" "${repo_root}/install/setup.bash"; do
  if [ ! -f "${setup}" ]; then
    echo "Required ROS setup is missing: ${setup}" >&2
    exit 2
  fi
done

mkdir -p "${output_dir}"
output_dir="$(readlink -f "${output_dir}")"
event_csv="${output_dir}/synthetic_contract_event.csv"
mmwave_csv="${output_dir}/synthetic_contract_mmwave.csv"

cat > "${event_csv}" <<'CSV'
x,y,polarity,timestamp
100,100,1,1000000
101,100,1,1000100
102,100,0,1000200
100,101,1,1000300
101,101,0,1000400
102,101,1,1000500
100,100,1,1020000
101,100,1,1020100
102,100,0,1020200
100,101,1,1020300
101,101,0,1020400
102,101,1,1020500
CSV

cat > "${mmwave_csv}" <<'CSV'
timestamp_us,frame_id,track_id,x_m,y_m,z_m,radial_velocity_mps,snr_db,confidence
1000000,1,7,2.0,0.0,0.0,0.5,18.0,0.95
CSV

cat > "${output_dir}/README.txt" <<'TXT'
These CSV files are synthetic contract fixtures.
They prove only ROS message/interface plumbing and process cleanup.
They are not hardware measurements and cannot be used for teacher metrics.
No motion topic, service, or action is used by this smoke check.
TXT

set +u
source "/opt/ros/${ros_distro}/setup.bash"
source "${base_ws}/install/setup.bash"
source "${repo_root}/install/setup.bash"
set -u

declare -a process_groups=()

start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 &
  process_groups+=("$!")
}

stop_groups() {
  local pid
  for pid in "${process_groups[@]:-}"; do
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${process_groups[@]:-}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  process_groups=()
}

trap stop_groups EXIT INT TERM

start_group "${output_dir}/event_replay.log" ros2 run ylhb_base auxiliary_sensor_replay_node --node-name dg_smoke_event_replay --ros-args -p sensor_kind:=event_camera -p input_file:="${event_csv}" -p loop:=true -p replay_rate_hz:=20.0 -p source_kind:=SYNTHETIC_CONTRACT_FIXTURE -p data_topic:=/dg/smoke/event/events -p quality_topic:=/dg/smoke/event/input_quality

start_group "${output_dir}/event_feature.log" ros2 run ylhb_base event_feature_node --node-name dg_smoke_event_feature --ros-args -p events_topic:=/dg/smoke/event/events -p features_topic:=/dg/smoke/event/features -p quality_topic:=/dg/smoke/event/feature_quality

start_group "${output_dir}/mmwave_replay.log" ros2 run ylhb_base auxiliary_sensor_replay_node --node-name dg_smoke_mmwave_replay --ros-args -p sensor_kind:=mmwave -p input_file:="${mmwave_csv}" -p loop:=true -p replay_rate_hz:=20.0 -p source_kind:=SYNTHETIC_CONTRACT_FIXTURE -p data_topic:=/dg/smoke/mmwave/points -p quality_topic:=/dg/smoke/mmwave/quality

start_group "${output_dir}/lidar_mmwave.log" ros2 run ylhb_base lidar_robust_node --node-name dg_smoke_lidar_mmwave --ros-args -p scan_topic:=/dg/smoke/scan -p odom_topic:=/dg/smoke/odom_unused -p filtered_scan_topic:=/dg/smoke/lidar/filtered_scan -p stable_features_topic:=/dg/smoke/lidar/stable_features -p quality_topic:=/dg/smoke/lidar/quality -p mmwave_dynamic_mask_enabled:=true -p mmwave_points_topic:=/dg/smoke/mmwave/points -p mmwave_timeout_sec:=1.0

scan_message="{header: {frame_id: laser_link}, angle_min: -0.4, angle_max: 0.4, angle_increment: 0.1, time_increment: 0.0, scan_time: 0.1, range_min: 0.05, range_max: 8.0, ranges: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0], intensities: []}"
start_group "${output_dir}/scan_publisher.log" ros2 topic pub --rate 10 /dg/smoke/scan sensor_msgs/msg/LaserScan "${scan_message}"

capture_topic() {
  local topic="$1"
  local destination="$2"
  local topic_type=""
  local deadline=$((SECONDS + 15))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    topic_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
    if [ -n "${topic_type}" ]; then
      break
    fi
    sleep 0.2
  done
  if [ -z "${topic_type}" ]; then
    echo "Topic ${topic} was not discovered within 15 seconds" >&2
    return 1
  fi
  if ! timeout 15 ros2 topic echo "${topic}" --once >"${destination}" 2>&1; then
    echo "Timed out waiting for ${topic}; see ${destination}" >&2
    return 1
  fi
}

capture_topic /dg/smoke/event/input_quality "${output_dir}/event_input_quality.txt"
capture_topic /dg/smoke/event/feature_quality "${output_dir}/event_feature_quality.txt"
capture_topic /dg/smoke/mmwave/quality "${output_dir}/mmwave_quality.txt"
capture_topic /dg/smoke/lidar/quality "${output_dir}/lidar_quality.txt"

diagnostic_value() {
  local file="$1"
  local key="$2"
  awk -v requested_key="${key}" '
    $0 ~ "key: " requested_key "$" { found = 1; next }
    found && $0 ~ /^[[:space:]]*value:/ {
      sub(/^[[:space:]]*value:[[:space:]]*/, "")
      gsub(/[\047\042]/, "")
      print
      exit
    }
  ' "${file}"
}

require_diagnostic_value() {
  local file="$1"
  local key="$2"
  local expected="$3"
  local actual
  actual="$(diagnostic_value "${file}" "${key}")"
  if [ "${actual}" != "${expected}" ]; then
    echo "Diagnostic ${key} expected ${expected}, got ${actual:-MISSING}" >&2
    return 1
  fi
}

require_positive_diagnostic_value() {
  local file="$1"
  local key="$2"
  local actual
  actual="$(diagnostic_value "${file}" "${key}")"
  if ! awk -v value="${actual}" 'BEGIN { exit !(value + 0 > 0) }'; then
    echo "Diagnostic ${key} must be positive, got ${actual:-MISSING}" >&2
    return 1
  fi
}

require_diagnostic_value "${output_dir}/event_input_quality.txt" source_kind SYNTHETIC_CONTRACT_FIXTURE
require_diagnostic_value "${output_dir}/event_feature_quality.txt" repeatability_status COMPARED
require_diagnostic_value "${output_dir}/mmwave_quality.txt" source_kind SYNTHETIC_CONTRACT_FIXTURE
require_diagnostic_value "${output_dir}/lidar_quality.txt" auxiliary_dynamic_data_fresh True
require_diagnostic_value "${output_dir}/lidar_quality.txt" auxiliary_dynamic_reject_reason ACCEPTED
require_positive_diagnostic_value "${output_dir}/lidar_quality.txt" auxiliary_dynamic_point_count

stop_groups
trap - EXIT INT TERM
sleep 1
if ros2 node list 2>/dev/null | grep -q '/dg_smoke_'; then
  echo "Smoke nodes remained after process-group cleanup" >&2
  ros2 node list 2>/dev/null | grep '/dg_smoke_' >&2 || true
  exit 1
fi

(
  cd "${output_dir}"
  sha256sum synthetic_contract_event.csv synthetic_contract_mmwave.csv event_input_quality.txt event_feature_quality.txt mmwave_quality.txt lidar_quality.txt > manifest.sha256
)

cat > "${output_dir}/result.txt" <<EOF
AUXILIARY_ROS_SMOKE=PASS
source_kind=SYNTHETIC_CONTRACT_FIXTURE
motion_commands_sent=false
production_topics_modified=false
process_group_cleanup=PASS
output_dir=${output_dir}
EOF

cat "${output_dir}/result.txt"
