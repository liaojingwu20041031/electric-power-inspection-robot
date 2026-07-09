#!/usr/bin/env bash
set -euo pipefail

DESKTOP_FILE="${HOME}/.config/autostart/ylhb-inspection-ui.desktop"
if [ -f "${DESKTOP_FILE}" ]; then
  rm "${DESKTOP_FILE}"
  echo "Removed: ${DESKTOP_FILE}"
else
  echo "Not installed: ${DESKTOP_FILE}"
fi
