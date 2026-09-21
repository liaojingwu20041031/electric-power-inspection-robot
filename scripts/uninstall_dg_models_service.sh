#!/usr/bin/env bash
set -euo pipefail

unit="dg202611-models.service"
sudo systemctl disable --now "${unit}" 2>/dev/null || true
sudo rm -f "/etc/systemd/system/${unit}"
sudo systemctl daemon-reload
echo "Removed ${unit}; release directories and /home/nvidia/ros2_DL were not changed."
