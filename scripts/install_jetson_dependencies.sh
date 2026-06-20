#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_DL}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/index-v4.yaml}"
export ROSDISTRO_INDEX_URL

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

echo "[ylhb] Installing Jetson native ROS 2 dependencies"
echo "[ylhb] Workspace: ${WS_DIR}"
echo "[ylhb] ROS distro: ${ROS_DISTRO}"

if [ ! -d "${WS_DIR}/src" ]; then
  echo "[ylhb] ERROR: ${WS_DIR}/src does not exist" >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-yaml \
  python3-opencv \
  ibus \
  ibus-gtk \
  ibus-gtk3 \
  ibus-pinyin \
  ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-rqt-image-view

if ! rosdep db >/dev/null 2>&1; then
  sudo rosdep init || true
fi
if ! rosdep update; then
  echo "[ylhb] WARN: rosdep update failed. Apt dependencies above were installed." >&2
  echo "[ylhb] WARN: Check ROSDISTRO_INDEX_URL=${ROSDISTRO_INDEX_URL} or rerun when network is available." >&2
fi

source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
if ! rosdep install --from-paths "${WS_DIR}/src" --ignore-src -r -y --rosdistro "${ROS_DISTRO}"; then
  echo "[ylhb] WARN: rosdep install failed. Continue because core apt dependencies were already requested explicitly." >&2
fi

python3 -m pip install --user --upgrade pip
python3 -m pip install --user "numpy<2" ultralytics

cat <<'EOF'
[ylhb] Dependency installation finished.
[ylhb] Notes:
  - ZED 2i requires the matching NVIDIA JetPack and ZED SDK installed from Stereolabs.
  - Put the PC-exported YOLO ONNX model at src/ylhb_perception/models/yolo26.onnx.
  - Compile ONNX to TensorRT once on the Jetson, then run src/ylhb_perception/models/yolo26.engine.
EOF
