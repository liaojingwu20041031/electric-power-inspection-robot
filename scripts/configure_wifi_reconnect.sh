#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

BACKUP_PATH=/home/nvidia/.config/ylhb/wifi_reconnect_backup.json
AUTOCONNECT_PRIORITY="${YLHB_WIFI_AUTOCONNECT_PRIORITY:-100}"
RECONNECT_INTERVAL="${YLHB_WIFI_RECONNECT_INTERVAL_SEC:-10}"
MAKE_SYSTEM="${YLHB_WIFI_MAKE_SYSTEM_CONNECTION:-false}"
CONNECTION="${YLHB_WIFI_CONNECTION:-}"
INTERFACE="${YLHB_WIFI_INTERFACE:-}"
UUID=''

die() { printf '错误: %s\n' "$*" >&2; exit 1; }
usage() { printf '用法: %s {status|dry-run|apply|restore}\n' "$0"; }
property() { nmcli --wait 5 -g "$1" connection show uuid "$UUID"; }

require_tools() {
  command -v nmcli >/dev/null 2>&1 || die '未找到 nmcli'
  command -v python3 >/dev/null 2>&1 || die '未找到 python3'
}

validate_options() {
  [[ "$AUTOCONNECT_PRIORITY" =~ ^-?[0-9]+$ ]] || die 'YLHB_WIFI_AUTOCONNECT_PRIORITY 必须是整数'
  [[ "$RECONNECT_INTERVAL" =~ ^[1-9][0-9]*$ ]] || die 'YLHB_WIFI_RECONNECT_INTERVAL_SEC 必须是正整数'
  [[ "${MAKE_SYSTEM,,}" =~ ^(true|false)$ ]] || die 'YLHB_WIFI_MAKE_SYSTEM_CONNECTION 必须是 true 或 false'
}

resolve_connection() {
  local -a candidates=()
  if [ -n "$CONNECTION" ]; then
    UUID="$(nmcli --wait 5 -g connection.uuid connection show "$CONNECTION" 2>/dev/null)" || die "连接不存在: $CONNECTION"
    [ "$(nmcli --wait 5 -g connection.type connection show uuid "$UUID")" = 802-11-wireless ] || die "目标不是 Wi-Fi 连接: $CONNECTION"
    return
  fi
  mapfile -t candidates < <(nmcli -t -f UUID,TYPE connection show --active | awk -F: '$2 == "802-11-wireless" {print $1}')
  if [ "${#candidates[@]}" -eq 0 ]; then
    mapfile -t candidates < <(nmcli -t -f UUID,TYPE connection show | awk -F: '$2 == "802-11-wireless" {print $1}')
  fi
  [ "${#candidates[@]}" -gt 0 ] || die '没有已保存的 Wi-Fi 连接，请设置 YLHB_WIFI_CONNECTION'
  [ "${#candidates[@]}" -eq 1 ] || die '存在多个 Wi-Fi 候选，请设置 YLHB_WIFI_CONNECTION；不会根据 SSID 猜测 profile 名称'
  UUID="${candidates[0]}"
  CONNECTION="$(nmcli --wait 5 -g connection.id connection show uuid "$UUID")"
}

resolve_interface() {
  local bound active
  local -a devices=()
  if [ -n "$INTERFACE" ]; then
    nmcli -t -f DEVICE,TYPE device | awk -F: -v dev="$INTERFACE" '$1 == dev && $2 == "wifi" {found=1} END {exit !found}' || die "无线接口不存在: $INTERFACE"
    return
  fi
  bound="$(property connection.interface-name)"
  if [ -n "$bound" ]; then INTERFACE="$bound"; return; fi
  active="$(nmcli --wait 5 -g GENERAL.DEVICES connection show uuid "$UUID" | head -n 1)"
  if [ -n "$active" ] && [ "$active" != -- ]; then INTERFACE="$active"; return; fi
  mapfile -t devices < <(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" {print $1}')
  [ "${#devices[@]}" -gt 0 ] || die '未找到无线接口'
  [ "${#devices[@]}" -eq 1 ] || die '存在多个无线接口，请设置 YLHB_WIFI_INTERFACE'
  INTERFACE="${devices[0]}"
}

resolve_target() { require_tools; validate_options; resolve_connection; resolve_interface; }

show_status() {
  resolve_target
  local permissions timer_state ipv4
  permissions="$(property connection.permissions)"
  timer_state="$(systemctl is-active ylhb-wifi-reconnect.timer 2>/dev/null || true)"
  ipv4="$(nmcli -g IP4.ADDRESS device show "$INTERFACE" 2>/dev/null || true)"
  printf 'NetworkManager状态: %s\n' "$(nmcli -g STATE general 2>/dev/null || echo unavailable)"
  printf 'Wi-Fi radio状态: %s\n' "$(nmcli radio wifi 2>/dev/null || echo unavailable)"
  printf '目标连接profile: %s\n' "$CONNECTION"
  printf '目标连接UUID: %s\n' "$UUID"
  printf '目标SSID: %s\n' "$(property 802-11-wireless.ssid)"
  printf '无线接口: %s\n' "$INTERFACE"
  printf '设备状态: %s\n' "$(nmcli -g GENERAL.STATE device show "$INTERFACE" 2>/dev/null || echo unavailable)"
  printf '当前active连接: %s\n' "$(nmcli -g GENERAL.CONNECTION device show "$INTERFACE" 2>/dev/null || true)"
  printf 'IPv4地址: %s\n' "${ipv4:-未获取}"
  printf 'autoconnect: %s\n' "$(property connection.autoconnect)"
  printf 'autoconnect-retries: %s\n' "$(property connection.autoconnect-retries)"
  printf 'autoconnect-priority: %s\n' "$(property connection.autoconnect-priority)"
  printf 'permissions: %s\n' "${permissions:-系统级（无限制）}"
  printf 'powersave: %s\n' "$(property 802-11-wireless.powersave)"
  printf 'watchdog timer状态: %s\n' "${timer_state:-未安装}"
  printf '最近一次watchdog日志:\n'
  journalctl -u ylhb-wifi-reconnect.service -n 1 --no-pager 2>/dev/null || printf '无日志或无读取权限\n'
  if [[ "$permissions" == *user:nvidia:* ]]; then
    printf '警告: 连接仅允许 user:nvidia:；如需系统服务使用，请显式设置 YLHB_WIFI_MAKE_SYSTEM_CONNECTION=true\n' >&2
  fi
}

show_plan() {
  resolve_target
  printf '目标连接: %s\n目标UUID: %s\n无线接口: %s\n' "$CONNECTION" "$UUID" "$INTERFACE"
  printf '计划修改:\n'
  printf '  connection.autoconnect=yes\n'
  printf '  connection.autoconnect-retries=0\n'
  printf '  connection.autoconnect-priority=%s\n' "$AUTOCONNECT_PRIORITY"
  printf '  802-11-wireless.powersave=2\n'
  if [ "${MAKE_SYSTEM,,}" = true ]; then
    printf '  connection.permissions=（系统级，无限制）\n'
  else
    printf '  connection.permissions=保持当前值\n'
  fi
  printf 'watchdog间隔: %s 秒\n' "$RECONNECT_INTERVAL"
  printf '不会 down/up 连接、切换 Wi-Fi radio、重启 NetworkManager 或修改 IP/路由/DNS。\n'
}

create_backup() {
  if [ -f "$BACKUP_PATH" ]; then
    local saved_uuid
    saved_uuid="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["uuid"])' "$BACKUP_PATH")" || die '现有备份格式无效'
    [ "$saved_uuid" = "$UUID" ] || die "现有备份属于其他连接 UUID: $saved_uuid"
    printf '保留已有原始备份: %s\n' "$BACKUP_PATH"
    return
  fi
  install -d -m 700 "$(dirname "$BACKUP_PATH")"
  python3 - "$BACKUP_PATH" "$CONNECTION" "$UUID" "$INTERFACE" \
    "$(property connection.autoconnect)" \
    "$(property connection.autoconnect-retries)" \
    "$(property connection.autoconnect-priority)" \
    "$(property connection.permissions)" \
    "$(property 802-11-wireless.powersave)" <<'PY'
import datetime, json, pathlib, sys
keys = ('profile', 'uuid', 'interface', 'autoconnect', 'autoconnect-retries',
        'autoconnect-priority', 'permissions', 'powersave')
payload = dict(zip(keys, sys.argv[2:]))
payload['modified_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
path = pathlib.Path(sys.argv[1])
path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
path.chmod(0o600)
PY
  printf '已备份原始属性: %s\n' "$BACKUP_PATH"
}

apply_config() {
  [ "$EUID" -eq 0 ] || die 'apply 必须使用 sudo -E 执行'
  resolve_target
  create_backup
  args=(connection modify uuid "$UUID"
    connection.autoconnect yes
    connection.autoconnect-retries 0
    connection.autoconnect-priority "$AUTOCONNECT_PRIORITY"
    802-11-wireless.powersave 2)
  [ "${MAKE_SYSTEM,,}" != true ] || args+=(connection.permissions '')
  nmcli --wait 5 "${args[@]}"
  printf '持久自动重连属性已写入，但连接未重新激活。请由现场人员决定何时重新激活。\n'
}

restore_config() {
  [ "$EUID" -eq 0 ] || die 'restore 必须使用 sudo -E 执行'
  require_tools
  [ -f "$BACKUP_PATH" ] || die "备份不存在: $BACKUP_PATH"
  mapfile -t saved < <(python3 - "$BACKUP_PATH" <<'PY'
import json, sys
p = json.load(open(sys.argv[1], encoding='utf-8'))
for key in ('uuid', 'profile', 'autoconnect', 'autoconnect-retries',
            'autoconnect-priority', 'permissions', 'powersave'):
    print(p.get(key, ''))
PY
  )
  [ "${#saved[@]}" -eq 7 ] || die '备份格式无效'
  nmcli --wait 5 -g connection.id connection show uuid "${saved[0]}" >/dev/null 2>&1 || die "备份 UUID 对应连接不存在: ${saved[0]}"
  nmcli --wait 5 connection modify uuid "${saved[0]}" \
    connection.autoconnect "${saved[2]}" \
    connection.autoconnect-retries "${saved[3]}" \
    connection.autoconnect-priority "${saved[4]}" \
    connection.permissions "${saved[5]}" \
    802-11-wireless.powersave "${saved[6]}"
  printf '已按 UUID 恢复 Wi-Fi profile 原始属性（当前名称可能已变化），连接未重新激活。\n'
}

case "${1:-}" in
  status) show_status ;;
  dry-run) show_plan ;;
  apply) apply_config ;;
  restore) restore_config ;;
  *) usage; exit 2 ;;
esac
