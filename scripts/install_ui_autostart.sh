#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSTART_DIR="${HOME}/.config/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/ylhb-inspection-ui.desktop"
LOG_DIR="${WS_DIR}/runs/ui_autostart"

if ! "${WS_DIR}/scripts/run_on_jetson.sh" inspection_preflight; then
  echo "ERROR: 自启动预检失败；修复后重试。缺少密钥时运行 ${WS_DIR}/scripts/configure_agent_env.sh" >&2
  exit 2
fi

mkdir -p "${AUTOSTART_DIR}" "${LOG_DIR}"
cat >"${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Type=Application
Name=YLHB Inspection Robot UI
Comment=Start electric power inspection robot console
Exec=${WS_DIR}/scripts/start_inspection_ui_autostart.sh
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

echo "Installed: ${DESKTOP_FILE}"
echo "Logs: ${LOG_DIR}"
echo "Manual test: ${WS_DIR}/scripts/start_inspection_ui_autostart.sh"
echo "Uninstall: ${WS_DIR}/scripts/uninstall_ui_autostart.sh"
