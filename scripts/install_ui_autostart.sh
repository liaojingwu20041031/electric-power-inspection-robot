#!/usr/bin/env bash
set -euo pipefail

# The desktop autostart must launch the protected base UI. DG model services
# are installed separately as dg202611-models.service.
integration_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
base_runner="${base_ws}/scripts/run_on_jetson.sh"
AUTOSTART_DIR="${HOME}/.config/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/ylhb-inspection-ui.desktop"
LOG_DIR="${integration_ws}/runs/ui_autostart"

if [ ! -x "${base_runner}" ]; then
  echo "基础系统启动脚本不存在：${base_runner}" >&2
  exit 2
fi
if ! WS_DIR="${base_ws}" "${base_runner}" inspection_preflight; then
  echo "ERROR: 基础 UI 自启动预检失败；请先修复 ${base_ws}。" >&2
  exit 2
fi

mkdir -p "${AUTOSTART_DIR}" "${LOG_DIR}"
cat >"${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Type=Application
Name=YLHB Inspection Robot UI
Comment=Start electric power inspection robot console
Exec=${integration_ws}/scripts/start_inspection_ui_autostart.sh
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

echo "Installed: ${DESKTOP_FILE}"
echo "Runtime UI: ${base_ws}/scripts/run_on_jetson.sh inspection"
echo "Logs: ${LOG_DIR}"
echo "Manual test: ${integration_ws}/scripts/start_inspection_ui_autostart.sh"
echo "Uninstall: ${integration_ws}/scripts/uninstall_ui_autostart.sh"
