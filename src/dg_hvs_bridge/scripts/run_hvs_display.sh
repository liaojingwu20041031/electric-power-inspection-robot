#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${DG_HVS_VERIFY_ROOT:-$script_dir}"
toolkit_root="${DG_HVS_TOOLKIT_ROOT:-/home/sunrise/dg202611_hvs_toolkit_20260912/shimetapi_hybrid_vision_toolkit-main}"

set +u
source /opt/tros/humble/setup.bash
set -u
export AMENT_PREFIX_PATH="$root/install:${AMENT_PREFIX_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export HV_TOOLKIT_ROOT="$toolkit_root"
export LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:/usr/hobot/lib:/app/multimedia_samples/sample_apx003cc/sample_hvs/dual_vc_vin_src/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"

if [[ -z "${XAUTHORITY:-}" ]]; then
  xauthority="$(find /run/user -path '*/gdm/Xauthority' -type f 2>/dev/null | head -n 1 || true)"
  [[ -z "$xauthority" ]] || export XAUTHORITY="$xauthority"
fi

exec ros2 launch dg_hvs_bridge hvs_display.launch.py "$@"
