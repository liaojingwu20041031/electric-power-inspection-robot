#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WS_DIR
LOG_DIR="${WS_DIR}/runs/ui_autostart"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/inspection_ui_$(date +%Y%m%d_%H%M%S).log"

{
  echo "WS_DIR=${WS_DIR}"
  echo "DISPLAY=${DISPLAY:-}"
  echo "XAUTHORITY=${XAUTHORITY:-}"
  cd "${WS_DIR}"
  exec "${WS_DIR}/scripts/run_on_jetson.sh" inspection fullscreen:=true
} >>"${LOG_FILE}" 2>&1
