#!/usr/bin/env bash
set -euo pipefail

BACKUP_DIR="${HOME}/.config/ylhb/robot_console_kiosk_backup"
KEYS=("org.gnome.desktop.session idle-delay" "org.gnome.desktop.screensaver lock-enabled" "org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type" "org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout")
command -v gsettings >/dev/null 2>&1 || { echo "gsettings 不可用；未修改桌面设置"; exit 0; }
read_key() { gsettings get "$1" "$2" 2>/dev/null; }
has_key() { gsettings writable "$1" "$2" >/dev/null 2>&1 && read_key "$1" "$2" >/dev/null 2>&1; }
case "${1:-status}" in
  status) for item in "${KEYS[@]}"; do set -- ${item}; has_key "$1" "$2" && echo "$1 $2=$(read_key "$1" "$2")" || echo "$item: unsupported"; done ;;
  enable)
    mkdir -p "${BACKUP_DIR}"; : >"${BACKUP_DIR}/gsettings"
    for item in "${KEYS[@]}"; do set -- ${item}; has_key "$1" "$2" || continue; printf '%s\t%s\t%s\n' "$1" "$2" "$(read_key "$1" "$2")" >>"${BACKUP_DIR}/gsettings"; done
    has_key org.gnome.desktop.session idle-delay && gsettings set org.gnome.desktop.session idle-delay 0
    has_key org.gnome.desktop.screensaver lock-enabled && gsettings set org.gnome.desktop.screensaver lock-enabled false
    has_key org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type && gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
    has_key org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout && gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
    echo "已保存并应用当前 GNOME 会话支持的 kiosk 设置（未修改电池模式）" ;;
  disable)
    [ -f "${BACKUP_DIR}/gsettings" ] || { echo "未找到备份：${BACKUP_DIR}/gsettings"; exit 0; }
    while IFS=$'\t' read -r schema key value; do has_key "${schema}" "${key}" && gsettings set "${schema}" "${key}" "${value}"; done <"${BACKUP_DIR}/gsettings"
    echo "已恢复 kiosk 备份" ;;
  *) echo "Usage: $0 {status|enable|disable}"; exit 2 ;;
esac
