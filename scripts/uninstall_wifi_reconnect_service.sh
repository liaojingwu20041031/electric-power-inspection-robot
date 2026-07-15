#!/usr/bin/env bash
set -euo pipefail

[ "$EUID" -eq 0 ] || { printf '错误: 请使用 sudo 执行\n' >&2; exit 1; }
[ "${1:-}" = '' ] || [ "${1:-}" = '--purge-config' ] || { printf '用法: %s [--purge-config]\n' "$0" >&2; exit 2; }

systemctl disable --now ylhb-wifi-reconnect.timer 2>/dev/null || true
rm -f /etc/systemd/system/ylhb-wifi-reconnect.service \
  /etc/systemd/system/ylhb-wifi-reconnect.timer \
  /usr/local/libexec/ylhb-wifi-reconnect
if [ "${1:-}" = '--purge-config' ]; then
  rm -f /etc/ylhb/wifi-reconnect.env
  rmdir /etc/ylhb 2>/dev/null || true
fi
systemctl daemon-reload
systemctl reset-failed ylhb-wifi-reconnect.service 2>/dev/null || true
printf '已卸载 Wi-Fi 重连服务；NetworkManager profile 和 /home/nvidia 下的备份未恢复或删除。\n'
[ "${1:-}" = '--purge-config' ] || printf '保留配置: /etc/ylhb/wifi-reconnect.env\n'
