#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WS_DIR
LOG_DIR="${WS_DIR}/runs/ui_autostart"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/inspection_ui_$(date +%Y%m%d_%H%M%S).log"
STOPPED=false
CHILD_PID=""
trap 'STOPPED=true; [ -n "${CHILD_PID}" ] && kill -TERM "${CHILD_PID}" 2>/dev/null || true' INT TERM
running_stack() { pgrep -f 'inspection_agent_node|voice_session_node|voice_output_node|system_supervisor_node|inspection_display_ui_node' >/dev/null; }
wait_for_old_stack() { while running_stack; do sleep 1; done; }
restart_times=()
while [ "${STOPPED}" = false ]; do
  if running_stack; then
    echo "inspection 已运行，跳过重复启动" >>"${LOG_FILE}"
    exit 0
  fi
  echo "$(date -Is) starting full inspection stack DISPLAY=${DISPLAY:-}" >>"${LOG_FILE}"
  if [ "${YLHB_UI_INHIBIT_IDLE:-true}" = "true" ] && systemd-inhibit --help 2>&1 | grep -q -- '--what='; then
    systemd-inhibit --what=idle:sleep --why='YLHB inspection console' --mode=block "${WS_DIR}/scripts/run_on_jetson.sh" inspection fullscreen:=true mobile_bridge_managed_externally:=true >>"${LOG_FILE}" 2>&1 &
  else
    "${WS_DIR}/scripts/run_on_jetson.sh" inspection fullscreen:=true mobile_bridge_managed_externally:=true >>"${LOG_FILE}" 2>&1 &
  fi
  CHILD_PID=$!
  wait "${CHILD_PID}" || true
  CHILD_PID=""
  wait_for_old_stack
  [ "${STOPPED}" = false ] || break
  [ "${YLHB_INSPECTION_AUTO_RESTART:-true}" = "true" ] || break
  now=$(date +%s); kept=()
  for t in "${restart_times[@]}"; do [ $((now - t)) -lt 60 ] && kept+=("${t}"); done
  restart_times=("${kept[@]}")
  if [ "${#restart_times[@]}" -ge 3 ]; then echo "$(date -Is) inspection crash-loop limit reached; stop restarting" >>"${LOG_FILE}"; break; fi
  restart_times+=("${now}")
  sleep 4
done
