#!/usr/bin/env bash
# Capture EVS, APS and EVS+APS fused image topics on RDK X5.
# Safe by construction: this script never starts the robot base and never publishes /cmd_vel.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: run_three_view_capture.sh [duration_sec]

Environment overrides:
  DG_HVS_BRIDGE_BIN       bridge executable path
  DG_HVS_PARAMS_FILE      bridge parameter YAML path
  DG_HVS_TOOLKIT_ROOT     ShiMetaPi toolkit root
  DG_HVS_BAG_ROOT         output root (default: /root/dg202611_hvs_bags)
  DG_HVS_RECORD_RAW       true/false; also save EVS RAW8/EVT2/CSV and APS/fused files
EOF
}

duration_sec="${1:-30}"
if [[ "${duration_sec}" == "-h" || "${duration_sec}" == "--help" ]]; then
  usage
  exit 0
fi
if ! [[ "${duration_sec}" =~ ^[0-9]+$ ]] || (( duration_sec < 1 )); then
  echo "duration_sec must be a positive integer" >&2
  exit 2
fi

bridge_bin="${DG_HVS_BRIDGE_BIN:-/root/dg202611_hvs_verify_20260913_1228/build2/dg_hvs_bridge_node}"
params_file="${DG_HVS_PARAMS_FILE:-/root/dg202611_hvs_verify_20260913_1228/src/dg_hvs_bridge/config/rdk_x5_apx003cc.yaml}"
toolkit_root="${DG_HVS_TOOLKIT_ROOT:-/home/sunrise/dg202611_hvs_toolkit_20260912/shimetapi_hybrid_vision_toolkit-main}"
bag_root="${DG_HVS_BAG_ROOT:-/root/dg202611_hvs_bags}"
record_raw="${DG_HVS_RECORD_RAW:-false}"
tros_setup="${DG_RDK_TROS_SETUP:-/opt/tros/humble/setup.bash}"

if [[ ! -x "$bridge_bin" ]]; then
  echo "bridge executable is missing or not executable: $bridge_bin" >&2
  exit 3
fi
if [[ ! -f "$params_file" ]]; then
  echo "bridge parameter file is missing: $params_file" >&2
  exit 3
fi
if [[ ! -f "$tros_setup" ]]; then
  echo "TROS setup is missing: $tros_setup" >&2
  exit 3
fi
command -v ros2 >/dev/null 2>&1 || {
  # shellcheck disable=SC1090
  source "$tros_setup"
}
command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is unavailable after sourcing $tros_setup" >&2
  exit 3
}

# shellcheck disable=SC1090
source "$tros_setup"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export HV_TOOLKIT_ROOT="$toolkit_root"
export LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:/usr/hobot/lib:/app/multimedia_samples/sample_apx003cc/sample_hvs/dual_vc_vin_src/lib:${LD_LIBRARY_PATH:-}"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
session_dir="$bag_root/three_views_${stamp}"
mkdir -p "$session_dir"

{
  echo "capture_started_utc=$stamp"
  echo "capture_started_local=$(date --iso-8601=seconds 2>/dev/null || date)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "ros_domain_id=$ROS_DOMAIN_ID"
  echo "ros_localhost_only=$ROS_LOCALHOST_ONLY"
  echo "rmw_implementation=$RMW_IMPLEMENTATION"
  echo "duration_sec=$duration_sec"
  echo "bridge_bin=$bridge_bin"
  echo "params_file=$params_file"
  echo "toolkit_root=$HV_TOOLKIT_ROOT"
  echo "record_raw=$record_raw"
  echo "publishes_cmd_vel=false"
  echo "view_evs=/dg/event_camera/motion_mask"
  echo "view_aps=/dg/event_camera/aps/image_raw/compressed"
  echo "view_fused=/dg/event_camera/fused/image/compressed"
} > "$session_dir/SESSION.txt"

bridge_log="$session_dir/bridge.log"
bag_log="$session_dir/rosbag.log"
bridge_pid=""
bag_pid=""
bridge_status=0
bag_status=0

stop_child() {
  local pid="$1"
  local label="$2"
  local status=0
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    echo "Stopping $label (pid=$pid)"
    kill -INT "$pid" 2>/dev/null || true
  fi
  # Always wait for the child so the evidence records its real exit status,
  # including an early failure that happened before the stop request.
  set +e
  wait "$pid"
  status=$?
  set -e
  echo "${label}_exit=$status"
  return "$status"
}

cleanup() {
  local exit_status=$?
  set +e
  stop_child "$bag_pid" "rosbag"
  stop_child "$bridge_pid" "bridge"
  set -e
  exit "$exit_status"
}
trap cleanup EXIT INT TERM

bridge_args=(
  --ros-args
  --params-file "$params_file"
  -p recording_enabled:=false
  -p publish_features:=true
  -p max_events_per_message:=20000
  -p event_publish_rate_hz:=20.0
  -p preview_publish_rate_hz:=10.0
)
if [[ "$record_raw" == "true" || "$record_raw" == "1" || "$record_raw" == "yes" ]]; then
  raw_dir="$session_dir/raw"
  mkdir -p "$raw_dir"
  bridge_args+=(
    -p recording_enabled:=true
    -p record_directory:="$raw_dir"
    -p record_csv_enabled:=true
    -p record_aps_every_n:=3
  )
fi

"$bridge_bin" "${bridge_args[@]}" > "$bridge_log" 2>&1 &
bridge_pid=$!
sleep 2
if ! kill -0 "$bridge_pid" 2>/dev/null; then
  echo "bridge exited before capture; log follows:" >&2
  cat "$bridge_log" >&2 || true
  exit 4
fi

bag_path="$session_dir/rosbag"
ros2 bag record \
  -o "$bag_path" \
  /dg/event_camera/motion_mask \
  /dg/event_camera/aps/image_raw/compressed \
  /dg/event_camera/fused/image/compressed \
  /dg/event_camera/quality \
  /dg/event_camera/feature_quality \
  /dg/event_camera/camera_info \
  > "$bag_log" 2>&1 &
bag_pid=$!

sleep "$duration_sec"
set +e
stop_child "$bag_pid" "rosbag"
bag_status=$?
set -e
bag_pid=""
set +e
stop_child "$bridge_pid" "bridge"
bridge_status=$?
set -e
bridge_pid=""

find "$session_dir" -type f ! -name manifest.sha256 -print0 \
  | sort -z \
  | xargs -0 sha256sum > "$session_dir/manifest.sha256"

printf '%s\n' "$bag_status" > "$session_dir/rosbag.exit"
printf '%s\n' "$bridge_status" > "$session_dir/bridge.exit"
echo "HVS_THREE_VIEW_SESSION=$session_dir"
echo "HVS_THREE_VIEW_BAG=$bag_path"
echo "HVS_THREE_VIEW_DONE=1"
