#!/usr/bin/env bash
# Replay a measured Shimeta event CSV on RDK X5. No motion publishers are created.
set -euo pipefail

sidecar_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Load board-specific paths without embedding a login or home directory.
# shellcheck source=rdk_env.sh
[ -f "$sidecar_root/rdk_env.sh" ] && source "$sidecar_root/rdk_env.sh"
tros_setup="${DG_RDK_TROS_SETUP:-/opt/tros/humble/setup.bash}"
input_file="${1:-}"
rate_hz="${2:-50.0}"
loop="${3:-false}"

if [ -z "$input_file" ]; then
  echo "Usage: $0 <events.csv> [rate_hz] [true|false]" >&2
  exit 2
fi
if [ ! -f "$input_file" ]; then
  echo "Measured event CSV is missing: $input_file" >&2
  exit 2
fi
if [ ! -f "$tros_setup" ]; then
  echo "TROS setup is missing: $tros_setup" >&2
  exit 2
fi

set +u
source "$tros_setup"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0

echo "RDK_EVENT_MEASURED_REPLAY_START=1"
echo "INPUT_FILE=$(readlink -f "$input_file")"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RDK_USER=$DG_RDK_USER"
echo "RDK_HOST=$DG_RDK_HOST"
echo "RDK_WORKSPACE=$DG_RDK_WORKSPACE"
echo "RDK_RECORD_DIR=$DG_RDK_RECORD_DIR"
echo "PUBLISHES_MOTION_COMMANDS=false"

exec python3 "$sidecar_root/auxiliary_sensor_replay_node.py" \
  --node-name dg_rdk_event_measured_replay \
  --ros-args \
  -p sensor_kind:=event_camera \
  -p input_file:="$(readlink -f "$input_file")" \
  -p loop:="$loop" \
  -p replay_rate_hz:="$rate_hz" \
  -p source_kind:=MEASURED_REPLAY \
  -p frame_id:="${DG_EVENT_FRAME_ID:-event_camera_optical_frame}" \
  -p event_width:="${DG_EVENT_WIDTH:-768}" \
  -p event_height:="${DG_EVENT_HEIGHT:-608}" \
  -p event_window_us:="${DG_EVENT_WINDOW_US:-20000}" \
  -p data_topic:="${DG_EVENT_TOPIC:-/dg/event_camera/events}" \
  -p quality_topic:="${DG_EVENT_INPUT_QUALITY_TOPIC:-/dg/event_camera/quality}"
