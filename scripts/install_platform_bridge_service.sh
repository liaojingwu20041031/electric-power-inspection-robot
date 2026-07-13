#!/usr/bin/env bash
set -euo pipefail
install -d -m 700 /home/nvidia/.config/ylhb
test -f /home/nvidia/.config/ylhb/platform.env || install -m 600 /dev/null /home/nvidia/.config/ylhb/platform.env
# Set YLHB_CLOUD_ENABLED=true, HTTPS YLHB_CLOUD_BASE_URL and YLHB_CLOUD_ROBOT_TOKEN
# in the protected environment file before enabling outbound cloud connectivity.
sudo tee /etc/systemd/system/ylhb-mobile-bridge.service >/dev/null <<'EOF'
[Unit]
After=network-online.target
Wants=network-online.target
[Service]
User=nvidia
WorkingDirectory=/home/nvidia/ros2_DL
EnvironmentFile=/home/nvidia/.config/ylhb/platform.env
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_DL/install/setup.bash && ros2 run ylhb_mobile_bridge mobile_bridge_server --ros-args --params-file /home/nvidia/ros2_DL/src/ylhb_mobile_bridge/config/mobile_bridge.yaml'
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now ylhb-mobile-bridge.service
