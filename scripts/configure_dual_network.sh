#!/usr/bin/env bash
set -euo pipefail

PRIMARY_CONNECTION="${YLHB_NETWORK_PRIMARY_CONNECTION:-}"
SECONDARY_CONNECTION="${YLHB_NETWORK_SECONDARY_CONNECTION:-}"
PRIMARY_METRIC="${YLHB_NETWORK_PRIMARY_METRIC:-100}"
SECONDARY_METRIC="${YLHB_NETWORK_SECONDARY_METRIC:-600}"
BACKUP_PATH="${HOME}/.config/ylhb/dual_network_backup.json"

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '用法: %s {status|dry-run|apply|restore}\n' "$0"
}

require_nmcli() {
  command -v nmcli >/dev/null 2>&1 || die '未找到 nmcli'
  command -v ip >/dev/null 2>&1 || die '未找到 ip'
}

require_connections() {
  [ -n "$PRIMARY_CONNECTION" ] || die '必须设置 YLHB_NETWORK_PRIMARY_CONNECTION'
  [ -n "$SECONDARY_CONNECTION" ] || die '必须设置 YLHB_NETWORK_SECONDARY_CONNECTION'
  [ "$PRIMARY_CONNECTION" != "$SECONDARY_CONNECTION" ] || die '主连接和备用连接不能相同'
  nmcli --wait 5 -g GENERAL.NAME connection show "$PRIMARY_CONNECTION" >/dev/null 2>&1 || die "连接不存在: $PRIMARY_CONNECTION"
  nmcli --wait 5 -g GENERAL.NAME connection show "$SECONDARY_CONNECTION" >/dev/null 2>&1 || die "连接不存在: $SECONDARY_CONNECTION"
}

connection_device() {
  nmcli --wait 5 -g GENERAL.DEVICES connection show "$1" | head -n 1
}

connection_cidr() {
  nmcli --wait 5 -g IP4.ADDRESS connection show "$1" | head -n 1
}

subnets_overlap() {
  local first="$1" second="$2"
  python3 - "$first" "$second" <<'PY'
import ipaddress
import sys

try:
    first = ipaddress.ip_interface(sys.argv[1]).network
    second = ipaddress.ip_interface(sys.argv[2]).network
except ValueError:
    raise SystemExit(2)
raise SystemExit(0 if first.overlaps(second) else 1)
PY
}

check_subnet_conflict() {
  local primary_cidr secondary_cidr result
  primary_cidr="$(connection_cidr "$PRIMARY_CONNECTION")"
  secondary_cidr="$(connection_cidr "$SECONDARY_CONNECTION")"
  [ -n "$primary_cidr" ] || die "连接没有活动 IPv4 地址: $PRIMARY_CONNECTION"
  [ -n "$secondary_cidr" ] || die "连接没有活动 IPv4 地址: $SECONDARY_CONNECTION"
  if subnets_overlap "$primary_cidr" "$secondary_cidr"; then
    printf '网络子网冲突: %s 与 %s 处于重叠子网\n' "$primary_cidr" "$secondary_cidr" >&2
    return 1
  else
    result=$?
    [ "$result" -eq 1 ] || die '无法解析连接 IPv4 子网'
  fi
}

cloud_hostname() {
  python3 - <<'PY'
import os
from pathlib import Path
from urllib.parse import urlsplit

value = os.environ.get('YLHB_CLOUD_BASE_URL', '').strip()
if not value:
    path = Path.home() / '.config/ylhb/platform.env'
    if path.exists():
        for raw in path.read_text(encoding='utf-8').splitlines():
            line = raw.strip()
            if line.startswith('export '):
                line = line[7:].lstrip()
            if line.startswith('YLHB_CLOUD_BASE_URL='):
                value = line.split('=', 1)[1].strip().strip('"\'')
                break
try:
    print(urlsplit(value).hostname or '')
except ValueError:
    print('')
PY
}

show_connection() {
  local name="$1"
  printf '\n连接: %s\n' "$name"
  nmcli --wait 5 -f GENERAL.NAME,GENERAL.DEVICES,GENERAL.IP-IFACE,GENERAL.STATE,GENERAL.DEFAULT,IP4.ADDRESS,IP4.GATEWAY,IP4.ROUTE connection show "$name"
  nmcli --wait 5 -f ipv4.route-metric,ipv4.never-default,ipv6.route-metric,ipv6.never-default connection show "$name"
}

status() {
  require_nmcli
  printf '活动连接:\n'
  nmcli --wait 5 -t -f NAME,DEVICE,TYPE,STATE connection show --active
  while IFS= read -r name; do
    [ -n "$name" ] && show_connection "$name"
  done < <(nmcli --wait 5 -g NAME connection show --active)
  printf '\n当前 IPv4 路由:\n'
  ip -4 route
  local host cloud_ip default_count
  host="$(cloud_hostname)"
  if [ -n "$host" ]; then
    cloud_ip="$(getent ahostsv4 "$host" | awk 'NR == 1 {print $1}')"
    printf '\n云平台 hostname: %s\n' "$host"
    if [ -n "$cloud_ip" ]; then
      printf '云平台 IPv4: %s\n' "$cloud_ip"
      printf '当前云平台出口:\n'
      ip route get "$cloud_ip"
    else
      printf '云平台 DNS 当前不可解析\n'
    fi
  else
    printf '\n云平台 hostname: 未配置\n'
  fi
  default_count="$(ip -j -4 route show default | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
  if [ "$default_count" -ge 2 ]; then
    printf '备用默认路由: 存在（%s 条默认路由）\n' "$default_count"
  else
    printf '备用默认路由: 不存在\n'
  fi
  if [ -n "$PRIMARY_CONNECTION" ] && [ -n "$SECONDARY_CONNECTION" ]; then
    require_connections
    if check_subnet_conflict; then
      printf '子网冲突: 无\n'
    else
      printf '子网冲突: 有\n'
    fi
  else
    printf '子网冲突: 设置两个连接名称后可检查\n'
  fi
}

print_modify_command() {
  local name="$1" metric="$2"
  printf 'nmcli connection modify %q ipv4.route-metric %q ipv4.never-default no ipv6.route-metric %q ipv6.never-default no\n' "$name" "$metric" "$metric"
}

dry_run() {
  require_nmcli
  require_connections
  check_subnet_conflict || die '检测到同子网冲突，拒绝生成应用计划'
  printf '计划修改（不会执行）:\n'
  print_modify_command "$PRIMARY_CONNECTION" "$PRIMARY_METRIC"
  print_modify_command "$SECONDARY_CONNECTION" "$SECONDARY_METRIC"
  printf '不会修改 IP、网关、DNS、Wi-Fi 密码、自动连接、防火墙、SSH 或 NetworkManager 服务。\n'
  printf '不会自动 down/up 连接；应用后由现场人员本地重新激活。\n'
}

read_connection_values() {
  local property
  for property in ipv4.route-metric ipv4.never-default ipv6.route-metric ipv6.never-default; do
    nmcli --wait 5 -g "$property" connection show "$1"
  done
}

create_backup() {
  if [ -f "$BACKUP_PATH" ]; then
    printf '保留已有备份: %s\n' "$BACKUP_PATH"
    return
  fi
  local primary_values secondary_values
  primary_values="$(read_connection_values "$PRIMARY_CONNECTION")"
  secondary_values="$(read_connection_values "$SECONDARY_CONNECTION")"
  install -d -m 700 "$(dirname "$BACKUP_PATH")"
  PRIMARY_CONNECTION="$PRIMARY_CONNECTION" SECONDARY_CONNECTION="$SECONDARY_CONNECTION" \
  PRIMARY_VALUES="$primary_values" SECONDARY_VALUES="$secondary_values" \
  python3 - "$BACKUP_PATH" <<'PY'
import json
import os
import sys
from pathlib import Path

def values(name):
    rows = os.environ[name].splitlines()
    return {
        'ipv4.route-metric': rows[0] if len(rows) > 0 else '',
        'ipv4.never-default': rows[1] if len(rows) > 1 else '',
        'ipv6.route-metric': rows[2] if len(rows) > 2 else '',
        'ipv6.never-default': rows[3] if len(rows) > 3 else '',
    }

payload = {
    'primary': {'name': os.environ['PRIMARY_CONNECTION'], **values('PRIMARY_VALUES')},
    'secondary': {'name': os.environ['SECONDARY_CONNECTION'], **values('SECONDARY_VALUES')},
}
path = Path(sys.argv[1])
path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
path.chmod(0o600)
PY
  printf '已备份当前路由属性: %s\n' "$BACKUP_PATH"
}

apply_config() {
  require_nmcli
  require_connections
  check_subnet_conflict || die '检测到同子网冲突，拒绝修改路由配置'
  create_backup
  nmcli --wait 5 connection modify "$PRIMARY_CONNECTION" ipv4.route-metric "$PRIMARY_METRIC" ipv4.never-default no ipv6.route-metric "$PRIMARY_METRIC" ipv6.never-default no
  nmcli --wait 5 connection modify "$SECONDARY_CONNECTION" ipv4.route-metric "$SECONDARY_METRIC" ipv4.never-default no ipv6.route-metric "$SECONDARY_METRIC" ipv6.never-default no
  printf '路由属性已写入，但连接未重新激活。\n'
  printf '请在 Jetson 本地现场重新激活连接，避免远程 SSH 会话立即中断。\n'
}

restore_config() {
  require_nmcli
  [ -f "$BACKUP_PATH" ] || die "备份不存在: $BACKUP_PATH"
  mapfile -t rows < <(python3 - "$BACKUP_PATH" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding='utf-8'))
for role in ('primary', 'secondary'):
    item = payload[role]
    print(item['name'])
    print(item.get('ipv4.route-metric', ''))
    print(item.get('ipv4.never-default', ''))
    print(item.get('ipv6.route-metric', ''))
    print(item.get('ipv6.never-default', ''))
PY
  )
  [ "${#rows[@]}" -eq 10 ] || die '备份格式无效'
  nmcli --wait 5 -g GENERAL.NAME connection show "${rows[0]}" >/dev/null 2>&1 || die "连接不存在: ${rows[0]}"
  nmcli --wait 5 -g GENERAL.NAME connection show "${rows[5]}" >/dev/null 2>&1 || die "连接不存在: ${rows[5]}"
  nmcli --wait 5 connection modify "${rows[0]}" ipv4.route-metric "${rows[1]}" ipv4.never-default "${rows[2]}" ipv6.route-metric "${rows[3]}" ipv6.never-default "${rows[4]}"
  nmcli --wait 5 connection modify "${rows[5]}" ipv4.route-metric "${rows[6]}" ipv4.never-default "${rows[7]}" ipv6.route-metric "${rows[8]}" ipv6.never-default "${rows[9]}"
  printf '已恢复备份路由属性，但连接未重新激活。\n'
  printf '请在 Jetson 本地现场重新激活连接。\n'
}

case "${1:-}" in
  status) status ;;
  dry-run) dry_run ;;
  apply) apply_config ;;
  restore) restore_config ;;
  *) usage; exit 2 ;;
esac
