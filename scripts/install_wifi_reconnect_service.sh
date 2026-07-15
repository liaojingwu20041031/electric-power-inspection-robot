#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

[ "$EUID" -eq 0 ] || { printf '错误: 请使用 sudo -E 执行\n' >&2; exit 1; }
command -v nmcli >/dev/null 2>&1 || { printf '错误: 未找到 nmcli\n' >&2; exit 1; }

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONNECTION="${YLHB_WIFI_CONNECTION:-}"
INTERFACE="${YLHB_WIFI_INTERFACE:-}"
INTERVAL="${YLHB_WIFI_RECONNECT_INTERVAL_SEC:-10}"
UUID=''

[[ "$INTERVAL" =~ ^[1-9][0-9]*$ ]] || { printf '错误: 重连间隔必须是正整数\n' >&2; exit 1; }
if [ -n "$CONNECTION" ]; then
  UUID="$(nmcli -g connection.uuid connection show "$CONNECTION" 2>/dev/null)" || { printf '错误: 连接不存在: %s\n' "$CONNECTION" >&2; exit 1; }
else
  mapfile -t candidates < <(nmcli -t -f UUID,TYPE connection show --active | awk -F: '$2 == "802-11-wireless" {print $1}')
  [ "${#candidates[@]}" -gt 0 ] || mapfile -t candidates < <(nmcli -t -f UUID,TYPE connection show | awk -F: '$2 == "802-11-wireless" {print $1}')
  [ "${#candidates[@]}" -eq 1 ] || { printf '错误: 请设置 YLHB_WIFI_CONNECTION，拒绝自动选择多个候选\n' >&2; exit 1; }
  UUID="${candidates[0]}"
  CONNECTION="$(nmcli -g connection.id connection show uuid "$UUID")"
fi
[ "$(nmcli -g connection.type connection show uuid "$UUID")" = 802-11-wireless ] || { printf '错误: 目标不是 Wi-Fi profile\n' >&2; exit 1; }

if [ -z "$INTERFACE" ]; then
  INTERFACE="$(nmcli -g connection.interface-name connection show uuid "$UUID")"
fi
if [ -z "$INTERFACE" ]; then
  mapfile -t devices < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" {print $1}')
  [ "${#devices[@]}" -eq 1 ] || { printf '错误: 请设置 YLHB_WIFI_INTERFACE，拒绝自动选择多个接口\n' >&2; exit 1; }
  INTERFACE="${devices[0]}"
fi
nmcli -t -f DEVICE,TYPE device | awk -F: -v dev="$INTERFACE" '$1 == dev && $2 == "wifi" {found=1} END {exit !found}' || { printf '错误: 无线接口不存在: %s\n' "$INTERFACE" >&2; exit 1; }

case "$CONNECTION$INTERFACE" in *$'\n'*|*$'\r'*) printf '错误: profile 或接口包含换行\n' >&2; exit 1;; esac
env_quote() { local value="${1//\\/\\\\}"; value="${value//\"/\\\"}"; printf '"%s"' "$value"; }

install -d -o root -g root -m 0755 /usr/local/libexec /etc/ylhb
install -o root -g root -m 0755 "$SOURCE_DIR/wifi_reconnect_once.sh" /usr/local/libexec/ylhb-wifi-reconnect
env_tmp="$(mktemp)"
service_tmp="$(mktemp)"
timer_tmp="$(mktemp)"
trap 'rm -f "$env_tmp" "$service_tmp" "$timer_tmp"' EXIT
{
  printf 'YLHB_WIFI_CONNECTION='; env_quote "$CONNECTION"; printf '\n'
  printf 'YLHB_WIFI_INTERFACE='; env_quote "$INTERFACE"; printf '\n'
} > "$env_tmp"
cat > "$service_tmp" <<'EOF'
[Unit]
Description=YLHB Wi-Fi reconnect attempt
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
EnvironmentFile=/etc/ylhb/wifi-reconnect.env
ExecStart=/usr/local/libexec/ylhb-wifi-reconnect
TimeoutStartSec=30
EOF
cat > "$timer_tmp" <<EOF
[Unit]
Description=YLHB Wi-Fi reconnect timer

[Timer]
OnBootSec=20
OnUnitActiveSec=$INTERVAL
AccuracySec=2
Persistent=true
Unit=ylhb-wifi-reconnect.service

[Install]
WantedBy=timers.target
EOF
install -o root -g root -m 0600 "$env_tmp" /etc/ylhb/wifi-reconnect.env
install -o root -g root -m 0644 "$service_tmp" /etc/systemd/system/ylhb-wifi-reconnect.service
install -o root -g root -m 0644 "$timer_tmp" /etc/systemd/system/ylhb-wifi-reconnect.timer
systemctl daemon-reload
systemctl enable --now ylhb-wifi-reconnect.timer
printf '已安装 Wi-Fi 重连 timer；未立即手动激活或切换任何网络连接。\n'
