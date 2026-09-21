#!/usr/bin/env bash
set -euo pipefail

active_ws="${DG_INTEGRATION_WS:-/home/nvidia/dg202611_ws}"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
unit="dg202611-models.service"
unit_path="/etc/systemd/system/${unit}"

for required in \
  "${active_ws}/scripts/run_dg_models_service.sh" \
  "${active_ws}/install/setup.bash" \
  "${base_ws}/install/setup.bash"; do
  if [ ! -f "${required}" ]; then
    echo "Required runtime file is missing: ${required}" >&2
    exit 2
  fi
done

service_was_active=false
if systemctl is-active --quiet "${unit}"; then
  service_was_active=true
elif existing_processes="$(pgrep -af '[d]g_navigation_integration.launch.py' || true)" \
  && [ -n "${existing_processes}" ]; then
  echo "Refusing to start a second DG integration instance." >&2
  echo "Stop the existing manual instance first:" >&2
  echo "${existing_processes}" >&2
  exit 3
fi

temporary="$(mktemp)"
trap 'rm -f "${temporary}"' EXIT
cat >"${temporary}" <<EOF
[Unit]
Description=DG-202611 three-model observation and fusion layer
After=network-online.target ylhb-mobile-bridge.service
Wants=network-online.target
ConditionPathExists=${active_ws}/install/setup.bash

[Service]
Type=simple
User=nvidia
WorkingDirectory=${active_ws}
Environment=DG_BASE_WS=${base_ws}
ExecStart=${active_ws}/scripts/run_dg_models_service.sh
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
EOF

sudo install -o root -g root -m 0644 "${temporary}" "${unit_path}"
sudo systemctl daemon-reload
sudo systemctl enable "${unit}"
if ${service_was_active}; then
  sudo systemctl restart "${unit}"
else
  sudo systemctl start "${unit}"
fi
sudo systemctl --no-pager --full status "${unit}"
