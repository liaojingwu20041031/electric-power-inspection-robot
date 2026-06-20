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
  if [ -z "${XAUTHORITY:-}" ] && [ -f "${HOME}/.Xauthority" ]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
  if ! xset q >/dev/null 2>&1; then
    echo "WARN: cannot access DISPLAY=${DISPLAY}; skip display sleep disable." >&2
    return 0
  fi
  xset s off >/dev/null 2>&1 || true
  xset s noblank >/dev/null 2>&1 || true
  xset -dpms >/dev/null 2>&1 || true
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
    exec ros2 launch ylhb_llm llm.launch.py "$@"
    ;;
  inspection)
    shift || true
    export DISPLAY="${DISPLAY:-:0}"
    if [ "${DISPLAY}" = "localhost:10.0" ] || [[ "${DISPLAY}" == localhost:* ]]; then
      export DISPLAY=":0"
    fi
    disable_display_sleep
    start_chinese_ime
    exec ros2 launch ylhb_llm llm.launch.py \
      enable_task_layer:=true \
      enable_display_ui:=true \
      enable_system_supervisor:=true \
      enable_voice:=true \
      enable_voice_session:=true \
      enable_capture_voice:=false \
      enable_tts:=true \
      audio_input_device:=plughw:CARD=Luna,DEV=0 \
      audio_output_device:=default \
      tts_voice:=Serena \
      display:="${DISPLAY}" \
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
  navigation   Start Nav2 with default map ${WS_DIR}/src/my_map.yaml
  zed          Start ZED 2i wrapper
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
  $0 navigation map:=${WS_DIR}/src/my_map.yaml
EOF
    ;;
esac
