#!/usr/bin/env bash
# Run only the event feature sidecar on RDK X5. No motion publishers are created.
set -euo pipefail

sidecar_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Load board-specific paths without embedding a login or home directory.
# shellcheck source=rdk_env.sh
[ -f "$sidecar_root/rdk_env.sh" ] && source "$sidecar_root/rdk_env.sh"
tros_setup="${DG_RDK_TROS_SETUP:-/opt/tros/humble/setup.bash}"
ros_domain_id="${ROS_DOMAIN_ID:-0}"
events_topic="${DG_EVENT_TOPIC:-/dg/event_camera/events}"
features_topic="${DG_EVENT_FEATURES_TOPIC:-/dg/event_camera/features}"
quality_topic="${DG_EVENT_QUALITY_TOPIC:-/dg/event_camera/feature_quality}"
expected_frame_id="${DG_EVENT_FRAME_ID:-event_camera_optical_frame}"
image_width="${DG_EVENT_WIDTH:-768}"
image_height="${DG_EVENT_HEIGHT:-608}"

if [ ! -f "$tros_setup" ]; then
  echo "TROS setup is missing: $tros_setup" >&2
  exit 2
fi
for required in event_feature_node.py event_feature_core.py pointcloud_reader.py ros_shutdown.py; do
  if [ ! -f "$sidecar_root/$required" ]; then
    echo "Sidecar file is missing: $sidecar_root/$required" >&2
    exit 2
  fi
done

set +u
source "$tros_setup"
set -u
export ROS_DOMAIN_ID="$ros_domain_id"
export ROS_LOCALHOST_ONLY=0

if ! ros2 topic list -t 2>/dev/null | grep -Fq "$events_topic [sensor_msgs/msg/PointCloud2]"; then
  echo "Event input topic is unavailable or has the wrong type: $events_topic" >&2
  echo "Start the vendor APX003CC/EVS publisher first." >&2
  exit 3
fi

echo "RDK_EVENT_SIDECAR_START=1"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RDK_USER=$DG_RDK_USER"
echo "RDK_HOST=$DG_RDK_HOST"
echo "RDK_WORKSPACE=$DG_RDK_WORKSPACE"
echo "EVENTS_TOPIC=$events_topic"
echo "FEATURES_TOPIC=$features_topic"
echo "QUALITY_TOPIC=$quality_topic"
echo "PUBLISHES_MOTION_COMMANDS=false"

exec python3 "$sidecar_root/event_feature_node.py" \
  --node-name dg_rdk_event_feature_node \
  --ros-args \
  -p events_topic:="$events_topic" \
  -p features_topic:="$features_topic" \
  -p quality_topic:="$quality_topic" \
  -p expected_frame_id:="$expected_frame_id" \
  -p image_width:="$image_width" \
  -p image_height:="$image_height"
