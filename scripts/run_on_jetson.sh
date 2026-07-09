#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_DL}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MODE="${1:-help}"

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

disable_display_sleep() {
  if ! command -v xset >/dev/null 2>&1; then
    echo "WARN: xset not found; cannot disable display sleep automatically." >&2
    return 0
  fi
  if ! xset q >/dev/null 2>&1; then
    echo "WARN: cannot access DISPLAY=${DISPLAY}; skip display sleep disable." >&2
    return 0
  fi
  xset s off >/dev/null 2>&1 || true
  xset s noblank >/dev/null 2>&1 || true
  xset -dpms >/dev/null 2>&1 || true
}

set_local_xauthority() {
  if [ -n "${XAUTHORITY:-}" ]; then
    return 0
  fi
  local uid
  uid="$(id -u)"
  local candidate
  for candidate in "/run/user/${uid}/gdm/Xauthority" "${HOME}/.Xauthority"; do
    if [ -r "${candidate}" ]; then
      export XAUTHORITY="${candidate}"
      return 0
    fi
  done
}

find_local_display() {
  local socket
  for socket in /tmp/.X11-unix/X*; do
    if [ -S "${socket}" ]; then
      printf ':%s\n' "${socket##*X}"
      return 0
    fi
  done
  printf ':0\n'
}

normalize_local_display() {
  if [ "${DISPLAY}" = "localhost:10.0" ] || [[ "${DISPLAY}" == localhost:* ]]; then
    export DISPLAY="$(find_local_display)"
    return 0
  fi
  if [[ "${DISPLAY}" == :* ]]; then
    local display_number="${DISPLAY#:}"
    display_number="${display_number%%.*}"
    if [ ! -S "/tmp/.X11-unix/X${display_number}" ]; then
      export DISPLAY="$(find_local_display)"
    fi
  fi
}

start_chinese_ime() {
  if [ "${ENABLE_CHINESE_IME:-true}" != "true" ]; then
    return 0
  fi
  export GTK_IM_MODULE="${GTK_IM_MODULE:-ibus}"
  export QT_IM_MODULE="${QT_IM_MODULE:-ibus}"
  export XMODIFIERS="${XMODIFIERS:-@im=ibus}"

  if ! command -v ibus-daemon >/dev/null 2>&1; then
    echo "WARN: ibus-daemon not found; Chinese IME is unavailable." >&2
    return 0
  fi
  ibus-daemon -drx >/dev/null 2>&1 || true
  if command -v ibus >/dev/null 2>&1; then
    if ! ibus engine pinyin >/dev/null 2>&1; then
      echo "WARN: cannot switch IBus engine to pinyin; install ibus-pinyin and check 'ibus list-engine'." >&2
    fi
  fi
}

require_ylhb_llm_executable() {
  local executable="$1"
  local path="${WS_DIR}/install/ylhb_llm/lib/ylhb_llm/${executable}"
  if [ -x "${path}" ]; then
    return 0
  fi
  echo "ERROR: missing ylhb_llm executable: ${executable}" >&2
  echo "Run: cd ${WS_DIR} && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --packages-select ylhb_llm" >&2
  exit 2
}

case "${MODE}" in
  bringup)
    shift || true
    uses_stm32=false
    for arg in "$@"; do
      if [ "${arg}" = "base_backend:=stm32" ]; then
        uses_stm32=true
        break
      fi
    done
    if [ "${uses_stm32}" != "true" ]; then
      echo "INFO: ZLAC backend uses PEAK PCAN-USB on SocketCAN can1; if can1 is not configured, run: ./scripts/setup_zlac_can.sh can1 500000" >&2
    fi
    exec ros2 launch ylhb_base bringup.launch.py "$@"
    ;;
  mapping)
    shift || true
    exec ros2 launch ylhb_base mapping.launch.py "$@"
    ;;
  navigation)
    shift || true
    exec ros2 launch ylhb_base navigation.launch.py "$@"
    ;;
  zed)
    shift || true
    exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i "$@"
    ;;
  zed_3d_capture)
    shift || true
    exec ros2 run ylhb_3d_mapping zed_svo_capture "$@"
    ;;
  zed_3d_reconstruct)
    shift || true
    exec ros2 run ylhb_3d_mapping zed_svo_reconstruct "$@"
    ;;
  perception)
    shift || true
    exec ros2 launch ylhb_perception perception.launch.py \
      model_path:="${WS_DIR}/src/ylhb_perception/models/yolo26.engine" \
      backend:=tensorrt \
      half:=true \
      "$@"
    ;;
  llm)
    shift || true
    require_ylhb_llm_executable inspection_agent_node
    require_ylhb_llm_executable base_motion_skill_node
    exec ros2 launch ylhb_llm llm.launch.py "$@"
    ;;
  inspection)
    shift || true
    for arg in "$@"; do
      case "${arg}" in
        enable_voice:=false|enable_voice_session:=false|enable_tts:=false)
          echo "ERROR: inspection mode is the formal robot console; voice session and TTS must stay enabled. Use llm mode for offline/debug launches." >&2
          exit 2
          ;;
      esac
    done
    export DISPLAY="${DISPLAY:-:0}"
    normalize_local_display
    set_local_xauthority
    disable_display_sleep
    start_chinese_ime
    require_ylhb_llm_executable inspection_agent_node
    require_ylhb_llm_executable base_motion_skill_node
    exec ros2 launch ylhb_llm llm.launch.py \
      enable_task_layer:=true \
      enable_display_ui:=true \
      enable_system_supervisor:=true \
      enable_keepout_navigation:=true \
      enable_voice:=true \
      enable_voice_session:=true \
      enable_capture_voice:=false \
      enable_tts:=true \
      audio_input_device:=plughw:CARD=Luna,DEV=0 \
      audio_output_device:=default \
      tts_voice:=Serena \
      display:="${DISPLAY}" \
      xauthority:="${XAUTHORITY:-}" \
      "$@"
    ;;
  teleop)
    shift || true
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard "$@"
    ;;
  *)
    cat <<EOF
Usage: $0 <mode> [ros arguments]

Modes:
  bringup      Start chassis backend, IMU, RPLidar, robot_state_publisher, EKF
  mapping      Start slam_toolbox mapping
  navigation   Start Nav2 with default map ${WS_DIR}/maps/my_map.yaml
  zed          Start ZED 2i wrapper
  zed_3d_capture  Record ZED SVO for later high-quality 3D reconstruction
  zed_3d_reconstruct  Reconstruct pointcloud.ply from a recorded ZED SVO
  perception   Start Jetson YOLO runtime with TensorRT engine
  llm          Start inspection AI task layer and voice I/O nodes
  inspection   Start inspection display UI and system supervisor
  teleop       Start keyboard teleop

Examples:
  $0 bringup base_backend:=zlac
  $0 bringup base_backend:=stm32
  $0 zed
  $0 perception model_path:=${WS_DIR}/src/ylhb_perception/models/yolo26.engine backend:=tensorrt imgsz:=960 half:=true
  $0 llm enable_voice:=false enable_tts:=false
  $0 llm enable_voice:=true enable_tts:=true audio_input_device:=plughw:CARD=Luna,DEV=0 audio_output_device:=plughw:CARD=Luna,DEV=0
  $0 inspection fullscreen:=true
  $0 navigation map:=${WS_DIR}/maps/my_map.yaml
  $0 navigation enable_keepout:=true keepout_mask:=${WS_DIR}/maps/keepout/keepout_mask_power_room_a.yaml
  $0 zed_3d_capture duration_sec:=0
  $0 zed_3d_reconstruct latest
  $0 zed_3d_reconstruct input:=latest profile:=quality_safe
  $0 zed_3d_reconstruct session:=capture_YYYYmmdd_HHMMSS
EOF
    ;;
esac
