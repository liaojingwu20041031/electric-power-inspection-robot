#!/usr/bin/env bash
# Read-only hardware probe for APX003CC/EVS cameras on D-Robotics RDK X5.
# It does not scan I2C addresses, configure MIPI, load modules, modify /boot,
# restart the board, or publish ROS messages.
set -u

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Load board-specific paths without embedding a login or home directory.
# shellcheck source=rdk_env.sh
[ -f "$script_root/rdk_env.sh" ] && source "$script_root/rdk_env.sh"

output="${1:-${DG_RDK_RECORD_DIR}/rdk_x5_event_camera_probe_$(date +%Y%m%d_%H%M%S).txt}"
mkdir -p "$(dirname "$output")" 2>/dev/null || true

run() {
  echo
  echo "=== $1 ==="
  shift
  "$@" 2>&1 || echo "COMMAND_FAILED=$?"
}

{
  echo "DG-202611 RDK X5 event-camera read-only probe"
  echo "probe_started=$(date --iso-8601=seconds 2>/dev/null || date)"
  echo "probe_host=$(hostname 2>/dev/null || true)"
  echo "rdk_user=${DG_RDK_USER}"
  echo "rdk_host=${DG_RDK_HOST}"
  echo "rdk_workspace=${DG_RDK_WORKSPACE}"
  echo "rdk_record_dir=${DG_RDK_RECORD_DIR}"

  run "kernel" uname -a
  run "os_release" cat /etc/os-release
  run "device_tree_model" bash -c 'tr -d "\\0" </proc/device-tree/model 2>/dev/null || true'
  run "board_release_files" bash -c 'for f in /etc/board* /etc/hobot* /etc/drobotics*; do [ -f "$f" ] && { echo "--- $f"; sed -n "1,120p" "$f"; }; done'
  run "usb_devices" bash -c 'command -v lsusb >/dev/null && lsusb || echo "lsusb unavailable"'

  run "video_media_nodes" bash -c 'for pattern in /dev/video* /dev/media* /dev/v4l-subdev* /dev/event* /dev/i2c-*; do compgen -G "$pattern" || true; done'
  run "i2c_adapter_map" bash -c 'for node in /sys/class/i2c-adapter/i2c-*; do [ -e "$node" ] || continue; echo "--- $node"; printf "name="; cat "$node/name" 2>/dev/null || true; printf "device="; readlink -f "$node/device" 2>/dev/null || true; done'
  run "video_sysfs" bash -c 'for node in /sys/class/video4linux/video*; do [ -e "$node" ] || continue; echo "--- $node"; readlink -f "$node"; cat "$node/name" 2>/dev/null || true; done'
  run "media_sysfs" bash -c 'for node in /sys/class/media/media*; do [ -e "$node" ] || continue; echo "--- $node"; readlink -f "$node"; cat "$node/name" 2>/dev/null || true; done'
  run "active_apx_device_tree" bash -c 'find /sys/firmware/devicetree/base -iname "*apx003cc*" -print 2>/dev/null || true'

  run "apx_sensor_libraries" bash -c 'ls -l /usr/hobot/lib/sensor/libapx003cc.so* /usr/hobot/lib/sensor/*apx003cc*.json 2>/dev/null || echo "APX003CC sensor libraries not found"'
  run "apx_vendor_tools" bash -c '
    root=/app/multimedia_samples/sample_apx003cc
    if [ ! -d "$root" ]; then
      echo "APX003CC sample directory not found"
      exit 0
    fi
    echo "sample_root=$root"
    find "$root" -maxdepth 6 -type f \
      \( -name "evs_live_player.RAW8" -o -name "raw8_to_evt2" -o -name "raw8_to_evt2.cpp" -o -name "evt2_to_csv" -o -name "evt2_to_csv.cpp" \) \
      -printf "%M %u:%g %s %p\\n" 2>/dev/null | sort
  '
  run "apx_overlay_assets" bash -c '
    found=0
    for f in /boot/overlays/dtoverlay_cam?_apx003cc.dtbo /boot/overlays/dtoverlay_cam?_apx003cc.dts; do
      [ -f "$f" ] || continue
      found=1
      ls -l "$f"
      case "$f" in
        *.dts)
          grep -nE "target-path|compatible|reg[[:space:]]*=|reset-gpios|data-lanes|lane-rate|link-frequencies|status[[:space:]]*=" "$f" || true
          ;;
      esac
    done
    [ "$found" -eq 1 ] || echo "APX003CC overlay assets not found"
  '
  run "boot_overlay_configuration" bash -c '
    found=0
    for f in /boot/config.txt /boot/config/config.txt; do
      [ -f "$f" ] || continue
      found=1
      echo "--- $f"
      sed -n "1,240p" "$f"
    done
    [ "$found" -eq 1 ] || echo "boot overlay configuration file not found"
  '

  run "v4l2_devices" bash -c 'command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices || echo "v4l2-ctl unavailable"'
  run "media_graph" bash -c 'command -v media-ctl >/dev/null && media-ctl -p || echo "media-ctl unavailable"'
  run "gstreamer_camera_plugins" bash -c 'command -v gst-inspect-1.0 >/dev/null && gst-inspect-1.0 | grep -Ei "evs|event|v4l2|mipi|camera" || echo "gst-inspect unavailable"'
  run "kernel_camera_messages" bash -c 'dmesg --since "10 minutes ago" 2>&1 | grep -Ei "apx003cc|evs|event.?camera|mipi|csi|v4l2|video|camera" || true'
  run "loaded_camera_modules" bash -c 'lsmod 2>/dev/null | grep -Ei "apx003cc|evs|camera|v4l2|mipi|csi" || true'

  run "ros_environment" bash -c '
    if ! command -v ros2 >/dev/null 2>&1 && [ -f /opt/tros/humble/setup.bash ]; then
      source /opt/tros/humble/setup.bash
      echo "ros_setup=/opt/tros/humble/setup.bash"
    fi
    if command -v ros2 >/dev/null 2>&1; then
      echo "ros2=$(command -v ros2)"
      ros2 doctor --report 2>&1 || true
      ros2 topic list -t 2>&1 || true
    else
      echo "ros2 unavailable after checking TROS Humble"
    fi
  '
  run "event_camera_processes" bash -c 'ps -ef 2>/dev/null | grep -Ei "apx003cc|evs|event.?camera|camera" | grep -v grep || true'

  echo
  echo "=== probe_end ==="
  echo "probe_finished=$(date --iso-8601=seconds 2>/dev/null || date)"
} | tee "$output"

echo "PROBE_OUTPUT=$output"
