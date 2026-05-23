#!/bin/bash
set -euo pipefail

echo "================================================="
echo "  [比赛用] 小车硬件 USB/CAN 一键持久恢复安装器  "
echo "================================================="

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: 请用 sudo 运行: sudo ./src/bind_usb.sh" >&2
  exit 1
fi

GUARD_PATH="/usr/local/sbin/robot-hardware-guard.sh"
SERVICE_PATH="/etc/systemd/system/robot-hardware-guard.service"
OLD_SERVICE_PATH="/etc/systemd/system/robot-ch340-guard.service"
OLD_GUARD_PATH="/usr/local/sbin/robot-ch340-guard.sh"
CH341_MODULE_PATH="/home/nvidia/drivers/ch341/ch341ser_linux/driver/ch341.ko"
PCAN_MODULE_PATH="/home/nvidia/drivers/pcan/peak-linux-driver-8.15.2/driver/pcan.ko"

lower_file() {
  path="$1"
  [ -r "$path" ] || return 1
  tr '[:upper:]' '[:lower:]' < "$path"
}

tty_matches_usb_id() {
  tty="$1"
  expected_vendor="$2"
  expected_product="$3"
  sys_path="$(readlink -f "/sys/class/tty/${tty##*/}/device" 2>/dev/null || true)"
  current="$sys_path"
  while [ -n "$current" ] && [ "$current" != "/" ]; do
    vendor="$(lower_file "$current/idVendor" 2>/dev/null || true)"
    product="$(lower_file "$current/idProduct" 2>/dev/null || true)"
    if [ "$vendor" = "$expected_vendor" ] && [ "$product" = "$expected_product" ]; then
      return 0
    fi
    parent="$(dirname "$current")"
    [ "$parent" != "$current" ] || break
    current="$parent"
  done
  return 1
}

first_tty_by_usb_id() {
  expected_vendor="$1"
  expected_product="$2"
  shift 2

  for pattern in "$@"; do
    for tty in $pattern; do
      [ -e "$tty" ] || continue
      if tty_matches_usb_id "$tty" "$expected_vendor" "$expected_product"; then
        printf '%s\n' "$tty"
        return 0
      fi
    done
  done
  return 1
}

interface_driver() {
  interface="$1"
  if [ -L "/sys/bus/usb/devices/$interface/driver" ]; then
    basename "$(readlink -f "/sys/bus/usb/devices/$interface/driver")"
  else
    printf '%s\n' "none"
  fi
}

list_usb_interfaces_by_id() {
  expected_vendor="$1"
  expected_product="$2"

  for device in /sys/bus/usb/devices/*; do
    [ -d "$device" ] || continue
    vendor="$(lower_file "$device/idVendor" 2>/dev/null || true)"
    product="$(lower_file "$device/idProduct" 2>/dev/null || true)"
    [ "$vendor" = "$expected_vendor" ] && [ "$product" = "$expected_product" ] || continue

    for interface in "$device":*; do
      [ -d "$interface" ] || continue
      [ -r "$interface/bInterfaceNumber" ] || continue
      basename "$interface"
    done
  done
}

print_conflict_hints() {
  if pgrep -x brltty >/dev/null 2>&1 \
    || systemctl is-active --quiet brltty.service 2>/dev/null \
    || dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q 'install ok installed'; then
    echo "WARN: 检测到 brltty 仍安装或运行，可能抢占 USB 串口。建议执行: sudo apt purge -y brltty"
  fi

  if pgrep -x ModemManager >/dev/null 2>&1 \
    || systemctl is-active --quiet ModemManager.service 2>/dev/null; then
    echo "WARN: 检测到 ModemManager 仍在运行，可能抢占串口。建议执行: sudo systemctl disable --now ModemManager"
  fi
}

install_hardware_guard() {
  echo "- 正在安装统一硬件 guard: $GUARD_PATH"
  cat > "$GUARD_PATH" <<EOF
#!/bin/bash
set -u

LOG_PATH="/var/log/robot-hardware-guard.log"
CH341_MODULE_PATH="$CH341_MODULE_PATH"
PCAN_MODULE_PATH="$PCAN_MODULE_PATH"
IMU_VID="1a86"
IMU_PID="7523"
LIDAR_VID="10c4"
LIDAR_PID="ea60"
PCAN_VID="0c72"
PCAN_PID="000c"
CAN_IFACE="can1"
CAN_BITRATE="500000"

log() {
  printf '%s %s\n' "\$(date '+%F %T')" "\$*" >> "\$LOG_PATH"
}

lower_file() {
  path="\$1"
  [ -r "\$path" ] || return 1
  tr '[:upper:]' '[:lower:]' < "\$path"
}

list_usb_interfaces_by_id() {
  expected_vendor="\$1"
  expected_product="\$2"

  for device in /sys/bus/usb/devices/*; do
    [ -d "\$device" ] || continue
    vendor="\$(lower_file "\$device/idVendor" 2>/dev/null || true)"
    product="\$(lower_file "\$device/idProduct" 2>/dev/null || true)"
    [ "\$vendor" = "\$expected_vendor" ] && [ "\$product" = "\$expected_product" ] || continue

    for interface in "\$device":*; do
      [ -d "\$interface" ] || continue
      [ -r "\$interface/bInterfaceNumber" ] || continue
      basename "\$interface"
    done
  done
}

interface_driver() {
  interface="\$1"
  if [ -L "/sys/bus/usb/devices/\$interface/driver" ]; then
    basename "\$(readlink -f "/sys/bus/usb/devices/\$interface/driver")"
  else
    printf '%s\n' "none"
  fi
}

bind_usb_interface() {
  interface="\$1"
  driver_name="\$2"
  current_driver="\$(interface_driver "\$interface")"

  [ "\$current_driver" = "\$driver_name" ] && return 0

  if [ "\$current_driver" != "none" ] && [ -w "/sys/bus/usb/drivers/\$current_driver/unbind" ]; then
    printf '%s\n' "\$interface" > "/sys/bus/usb/drivers/\$current_driver/unbind" 2>/dev/null \
      && log "unbound \$interface from \$current_driver" \
      || log "failed to unbind \$interface from \$current_driver"
  fi

  if [ -w "/sys/bus/usb/drivers/\$driver_name/bind" ]; then
    printf '%s\n' "\$interface" > "/sys/bus/usb/drivers/\$driver_name/bind" 2>/dev/null \
      && log "bound \$interface to \$driver_name" \
      || log "failed to bind \$interface to \$driver_name"
  else
    log "\$driver_name bind path is unavailable for \$interface"
  fi
}

ensure_ch341_driver() {
  [ -d /sys/bus/usb/drivers/usb_ch341 ] && return 0

  if [ ! -f "\$CH341_MODULE_PATH" ]; then
    log "usb_ch341 driver is not registered and module is missing: \$CH341_MODULE_PATH"
    return 1
  fi

  output="\$(/sbin/insmod "\$CH341_MODULE_PATH" 2>&1)"
  status=\$?
  if [ "\$status" -eq 0 ]; then
    log "loaded usb_ch341 module from \$CH341_MODULE_PATH"
    return 0
  fi

  case "\$output" in
    *"File exists"*|*"file exists"*) log "usb_ch341 module already loaded: \$output" ;;
    *) log "failed to insmod \$CH341_MODULE_PATH: \$output" ;;
  esac

  [ -d /sys/bus/usb/drivers/usb_ch341 ]
}

ensure_cp210x_driver() {
  /sbin/modprobe cp210x 2>/dev/null || log "failed to modprobe cp210x"
  [ -d /sys/bus/usb/drivers/cp210x ]
}

ensure_pcan_driver() {
  [ -d /sys/bus/usb/drivers/pcan ] && return 0

  /sbin/modprobe pcan 2>/dev/null && {
    log "loaded pcan module with modprobe"
    [ -d /sys/bus/usb/drivers/pcan ] && return 0
  }

  if [ ! -f "\$PCAN_MODULE_PATH" ]; then
    log "pcan driver is not registered and module is missing: \$PCAN_MODULE_PATH"
    return 1
  fi

  output="\$(/sbin/insmod "\$PCAN_MODULE_PATH" 2>&1)"
  status=\$?
  if [ "\$status" -eq 0 ]; then
    log "loaded pcan module from \$PCAN_MODULE_PATH"
    return 0
  fi

  case "\$output" in
    *"File exists"*|*"file exists"*) log "pcan module already loaded: \$output" ;;
    *) log "failed to insmod \$PCAN_MODULE_PATH: \$output" ;;
  esac

  [ -d /sys/bus/usb/drivers/pcan ]
}

tty_matches_usb_id() {
  tty="\$1"
  expected_vendor="\$2"
  expected_product="\$3"
  sys_path="\$(readlink -f "/sys/class/tty/\${tty##*/}/device" 2>/dev/null || true)"
  current="\$sys_path"
  while [ -n "\$current" ] && [ "\$current" != "/" ]; do
    vendor="\$(lower_file "\$current/idVendor" 2>/dev/null || true)"
    product="\$(lower_file "\$current/idProduct" 2>/dev/null || true)"
    if [ "\$vendor" = "\$expected_vendor" ] && [ "\$product" = "\$expected_product" ]; then
      return 0
    fi
    parent="\$(dirname "\$current")"
    [ "\$parent" != "\$current" ] || break
    current="\$parent"
  done
  return 1
}

first_tty_by_usb_id() {
  expected_vendor="\$1"
  expected_product="\$2"
  shift 2

  for pattern in "\$@"; do
    for tty in \$pattern; do
      [ -e "\$tty" ] || continue
      if tty_matches_usb_id "\$tty" "\$expected_vendor" "\$expected_product"; then
        printf '%s\n' "\$tty"
        return 0
      fi
    done
  done
  return 1
}

refresh_tty_alias() {
  alias_path="\$1"
  vid="\$2"
  pid="\$3"
  shift 3

  tty="\$(first_tty_by_usb_id "\$vid" "\$pid" "\$@" || true)"
  [ -n "\$tty" ] || return 1

  current_target="\$(readlink "\$alias_path" 2>/dev/null || true)"
  if [ "\$current_target" != "\${tty##*/}" ]; then
    ln -sfn "\${tty##*/}" "\$alias_path" \
      && log "refreshed \$alias_path -> \${tty##*/}" \
      || log "failed to refresh \$alias_path -> \${tty##*/}"
  fi

  chmod 0666 "\$tty" 2>/dev/null || log "failed to chmod 0666 \$tty"
  chgrp dialout "\$tty" 2>/dev/null || log "failed to chgrp dialout \$tty"
  chgrp dialout "\$alias_path" 2>/dev/null || true
  return 0
}

netdev_matches_usb_id() {
  netdev="\$1"
  expected_vendor="\$2"
  expected_product="\$3"
  sys_path="\$(readlink -f "/sys/class/net/\$netdev/device" 2>/dev/null || true)"
  current="\$sys_path"
  while [ -n "\$current" ] && [ "\$current" != "/" ]; do
    vendor="\$(lower_file "\$current/idVendor" 2>/dev/null || true)"
    product="\$(lower_file "\$current/idProduct" 2>/dev/null || true)"
    if [ "\$vendor" = "\$expected_vendor" ] && [ "\$product" = "\$expected_product" ]; then
      return 0
    fi
    parent="\$(dirname "\$current")"
    [ "\$parent" != "\$current" ] || break
    current="\$parent"
  done
  return 1
}

first_pcan_netdev() {
  if [ -e "/sys/class/net/\$CAN_IFACE" ] && netdev_matches_usb_id "\$CAN_IFACE" "\$PCAN_VID" "\$PCAN_PID"; then
    printf '%s\n' "\$CAN_IFACE"
    return 0
  fi

  for netdev_path in /sys/class/net/can*; do
    [ -e "\$netdev_path" ] || continue
    netdev="\${netdev_path##*/}"
    if netdev_matches_usb_id "\$netdev" "\$PCAN_VID" "\$PCAN_PID"; then
      printf '%s\n' "\$netdev"
      return 0
    fi
  done
  return 1
}

can_iface_needs_config() {
  details="\$(/sbin/ip -details link show "\$CAN_IFACE" 2>/dev/null || true)"
  [ -n "\$details" ] || return 0

  printf '%s\n' "\$details" | grep -q 'state UP' || return 0
  printf '%s\n' "\$details" | grep -q "bitrate \$CAN_BITRATE" || return 0
  printf '%s\n' "\$details" | grep -q 'restart-ms 100' || return 0
  printf '%s\n' "\$details" | grep -q '<BERR-REPORTING>' || return 0
  return 1
}

ensure_pcan_can1() {
  netdev="\$(first_pcan_netdev || true)"
  [ -n "\$netdev" ] || return 1

  if [ "\$netdev" != "\$CAN_IFACE" ] && [ ! -e "/sys/class/net/\$CAN_IFACE" ]; then
    /sbin/ip link set "\$netdev" down 2>/dev/null || true
    /sbin/ip link set "\$netdev" name "\$CAN_IFACE" 2>/dev/null \
      && log "renamed \$netdev to \$CAN_IFACE" \
      || log "failed to rename \$netdev to \$CAN_IFACE"
  fi

  [ -e "/sys/class/net/\$CAN_IFACE" ] || return 1
  can_iface_needs_config || return 0

  /sbin/ip link set "\$CAN_IFACE" down 2>/dev/null || true
  /sbin/ip link set "\$CAN_IFACE" type can bitrate "\$CAN_BITRATE" berr-reporting on restart-ms 100 2>/dev/null \
    && log "configured \$CAN_IFACE bitrate \$CAN_BITRATE" \
    || log "failed to configure \$CAN_IFACE bitrate \$CAN_BITRATE"
  /sbin/ip link set "\$CAN_IFACE" up 2>/dev/null \
    && log "set \$CAN_IFACE up" \
    || log "failed to set \$CAN_IFACE up"
}

run_once() {
  ensure_ch341_driver || true
  while IFS= read -r interface; do
    [ -n "\$interface" ] || continue
    bind_usb_interface "\$interface" usb_ch341
  done < <(list_usb_interfaces_by_id "\$IMU_VID" "\$IMU_PID")
  refresh_tty_alias /dev/robot_imu "\$IMU_VID" "\$IMU_PID" /dev/ttyCH341USB* /dev/ttyUSB* || true

  ensure_cp210x_driver || true
  while IFS= read -r interface; do
    [ -n "\$interface" ] || continue
    bind_usb_interface "\$interface" cp210x
  done < <(list_usb_interfaces_by_id "\$LIDAR_VID" "\$LIDAR_PID")
  refresh_tty_alias /dev/robot_lidar "\$LIDAR_VID" "\$LIDAR_PID" /dev/ttyUSB* || true

  ensure_pcan_driver || true
  while IFS= read -r interface; do
    [ -n "\$interface" ] || continue
    bind_usb_interface "\$interface" pcan
  done < <(list_usb_interfaces_by_id "\$PCAN_VID" "\$PCAN_PID")
  ensure_pcan_can1 || true
}

log "robot-hardware-guard started"
while true; do
  run_once
  sleep 0.5
done
EOF
  chmod 0755 "$GUARD_PATH"
}

install_hardware_service() {
  echo "- 正在安装 systemd 服务: $SERVICE_PATH"
  cat > "$SERVICE_PATH" <<EOF
[Unit]
Description=Robot LiDAR IMU PCAN persistent hardware guard
After=systemd-udevd.service network-pre.target

[Service]
Type=simple
ExecStart=$GUARD_PATH
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
EOF
}

print_conflict_hints

echo "- 正在写入 CP210x (激光雷达) 规则为 /dev/robot_lidar"
echo 'SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="robot_lidar"' > /etc/udev/rules.d/99-robot-lidar.rules

echo "- 正在写入 CH340 (IMU) 规则为 /dev/robot_imu"
{
  echo 'SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="robot_imu"'
  echo 'SUBSYSTEM=="tty", KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="robot_imu"'
} > /etc/udev/rules.d/99-robot-imu.rules

install_hardware_guard
install_hardware_service

echo "- 正在停用旧 CH340 guard（如果存在）..."
systemctl disable --now robot-ch340-guard.service >/dev/null 2>&1 || true
[ -f "$OLD_SERVICE_PATH" ] && mv -f "$OLD_SERVICE_PATH" "${OLD_SERVICE_PATH}.disabled" || true
[ -f "$OLD_GUARD_PATH" ] && mv -f "$OLD_GUARD_PATH" "${OLD_GUARD_PATH}.disabled" || true

echo "- 正在重新加载 udev 规则和 systemd..."
udevadm control --reload-rules 2>/dev/null || service udev reload
udevadm trigger 2>/dev/null || true
systemctl daemon-reload

echo "- 正在启用并启动 robot-hardware-guard.service..."
systemctl enable --now robot-hardware-guard.service
sleep 2

echo "================================================="
echo "统一硬件 guard 已安装：LiDAR / IMU / PCAN 会自动恢复。"
echo "日志: /var/log/robot-hardware-guard.log"
echo "================================================="
echo "+ lsusb -t"
lsusb -t || true
echo "+ ls -l /dev/robot_lidar /dev/robot_imu /dev/ttyUSB* /dev/ttyCH341USB*"
ls -l /dev/robot_lidar /dev/robot_imu /dev/ttyUSB* /dev/ttyCH341USB* 2>/dev/null || true
echo "+ 当前 USB interface driver:"
for interface in $(list_usb_interfaces_by_id 10c4 ea60; list_usb_interfaces_by_id 1a86 7523; list_usb_interfaces_by_id 0c72 000c); do
  printf '%s -> %s\n' "$interface" "$(interface_driver "$interface")"
done
echo "+ ip -details link show can1"
ip -details link show can1 || true
echo "+ systemctl status robot-hardware-guard.service --no-pager"
systemctl status robot-hardware-guard.service --no-pager || true
echo "================================================="
