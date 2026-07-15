#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

LOCK_FILE=/run/ylhb-wifi-reconnect.lock
STATE_DIR="${XDG_RUNTIME_DIR:-/run}/ylhb-wifi-reconnect"
CONNECTION="${YLHB_WIFI_CONNECTION:-}"
INTERFACE="${YLHB_WIFI_INTERFACE:-}"

if [ "${1:-}" != "--locked" ]; then
  set +e
  flock -n -E 75 "$LOCK_FILE" "$0" --locked
  result=$?
  set -e
  [ "$result" -ne 75 ] || exit 2
  exit "$result"
fi

rate_limited() {
  local level="$1" key="$2" message="$3" now last_file last=0
  now="$(date +%s)"
  mkdir -p "$STATE_DIR" 2>/dev/null || true
  last_file="$STATE_DIR/$key"
  [ ! -r "$last_file" ] || read -r last < "$last_file" || last=0
  if [ $((now - last)) -ge 30 ]; then
    printf '%s: %s\n' "$level" "$message" >&2
    printf '%s\n' "$now" > "$last_file" 2>/dev/null || true
  fi
}

[ -n "$CONNECTION" ] && [ -n "$INTERFACE" ] || {
  rate_limited ERROR configuration 'YLHB_WIFI_CONNECTION 和 YLHB_WIFI_INTERFACE 必须设置'
  exit 1
}
command -v nmcli >/dev/null 2>&1 || {
  rate_limited ERROR nmcli-missing '未找到 nmcli'
  exit 1
}
nmcli general status >/dev/null 2>&1 || {
  rate_limited ERROR nm-offline 'NetworkManager 不在线'
  exit 1
}

[ "$(nmcli radio wifi 2>/dev/null || true)" != disabled ] || exit 0

device_row="$(nmcli -t -f DEVICE,TYPE,STATE device 2>/dev/null | awk -F: -v dev="$INTERFACE" '$1 == dev && $2 == "wifi" {print; exit}')"
[ -n "$device_row" ] || {
  rate_limited WARN interface-missing "无线接口不可用: $INTERFACE"
  exit 1
}
device_state="${device_row##*:}"
case "$device_state" in
  unmanaged|unavailable)
    rate_limited WARN "device-$device_state" "无线接口 $INTERFACE 状态为 $device_state，不重启 NetworkManager"
    exit 0
    ;;
esac

active_connection="$(nmcli -g GENERAL.CONNECTION device show "$INTERFACE" 2>/dev/null || true)"
if [ "$active_connection" = "$CONNECTION" ]; then
  if [ -z "$(nmcli -g IP4.ADDRESS device show "$INTERFACE" 2>/dev/null || true)" ]; then
    rate_limited INFO dhcp "连接已激活，$INTERFACE 正在获取 IPv4 地址"
  fi
  exit 0
fi
if [ -n "$active_connection" ] && [ "$active_connection" != "--" ]; then
  exit 0
fi

ssid="$(nmcli -g 802-11-wireless.ssid connection show "$CONNECTION" 2>/dev/null)" || {
  rate_limited ERROR profile "无法读取目标 Wi-Fi profile: $CONNECTION"
  exit 1
}
[ -n "$ssid" ] || {
  rate_limited ERROR ssid '目标 Wi-Fi profile 未绑定 SSID'
  exit 1
}
nmcli device wifi rescan ifname "$INTERFACE" >/dev/null 2>&1 || {
  rate_limited WARN scan "无线扫描失败: $INTERFACE"
  exit 1
}
visible_ssids="$(nmcli -g SSID device wifi list ifname "$INTERFACE" 2>/dev/null)" || {
  rate_limited WARN scan-list "无法读取扫描结果: $INTERFACE"
  exit 1
}
grep -Fqx -- "$ssid" <<< "$visible_ssids" || exit 0

if nmcli --wait 20 connection up "$CONNECTION" ifname "$INTERFACE" >/dev/null 2>&1; then
  printf '已请求 NetworkManager 激活 Wi-Fi profile %s（接口 %s）\n' "$CONNECTION" "$INTERFACE"
  exit 0
fi
rate_limited ERROR activate "激活 Wi-Fi profile 失败: $CONNECTION"
exit 1
