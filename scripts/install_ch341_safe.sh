#!/usr/bin/env bash
set -euo pipefail

base_dir="${HOME}/drivers/ch341"
driver_dir="${base_dir}/ch341ser_linux"
zip_path="${HOME}/Downloads/CH341SER_LINUX.ZIP"
built_module=""
build_dir=""

usage() {
  cat <<'USAGE'
Usage: scripts/install_ch341_safe.sh [--precheck|--build|--test-load|--install|--rollback]

安全边界:
  - 默认不永久安装，不修改 /boot，不修改 initrd，不修改 ROS2 功能代码。
  - 优先尝试内核自带 usbserial/ch341；缺失时才编译 WCH 官方 ch341ser_linux。
  - --test-load 只临时加载构建出的模块；--install 才复制到 /lib/modules 并 depmod。

源码来源:
  - https://github.com/WCHSoftGroup/ch341ser_linux
  - https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html

阶段:
  --precheck   检查内核、现有模块、CH340 USB 枚举、源码和编译环境
  --build      下载或复用 WCH 源码并编译 .ko
  --test-load  尝试加载 usbserial/ch341；必要时编译并临时 insmod WCH 模块
  --install    先 test-load，通过后输入精确 YES 才安装到当前内核模块目录
  --rollback   卸载 ch34x/ch341，删除本脚本安装的模块目录和 modules-load 配置
USAGE
}

section() {
  printf '\n==== %s ====\n' "$1"
}

die() {
  printf 'ERROR: %s\n' "$1" >&2
  exit 1
}

run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

warn_run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  "$@" || printf 'WARN: command failed, continuing\n' >&2
}

kernel_release() {
  uname -r
}

module_available() {
  local name="$1"
  modprobe -n -v "${name}" >/dev/null 2>&1
}

list_ch340_ttys() {
  local tty sys_path current vendor product

  shopt -s nullglob
  for tty in /dev/ttyUSB* /dev/ttyCH341USB*; do
    sys_path="$(readlink -f "/sys/class/tty/${tty##*/}/device" 2>/dev/null || true)"
    current="${sys_path}"
    while [ -n "${current}" ] && [ "${current}" != "/" ]; do
      vendor=""
      product=""
      [ -r "${current}/idVendor" ] && vendor="$(tr '[:upper:]' '[:lower:]' < "${current}/idVendor")"
      [ -r "${current}/idProduct" ] && product="$(tr '[:upper:]' '[:lower:]' < "${current}/idProduct")"
      if [ "${vendor}" = "1a86" ] && [ "${product}" = "7523" ]; then
        printf '%s\n' "${tty}"
        break
      fi
      current="$(dirname "${current}")"
    done
  done
  shopt -u nullglob
}

list_ch340_interfaces() {
  local device interface vendor product

  for device in /sys/bus/usb/devices/*; do
    vendor=""
    product=""
    [ -r "${device}/idVendor" ] && vendor="$(tr '[:upper:]' '[:lower:]' < "${device}/idVendor")"
    [ -r "${device}/idProduct" ] && product="$(tr '[:upper:]' '[:lower:]' < "${device}/idProduct")"
    if [ "${vendor}" != "1a86" ] || [ "${product}" != "7523" ]; then
      continue
    fi

    for interface in "${device}":*; do
      [ -d "${interface}" ] || continue
      [ -r "${interface}/bInterfaceNumber" ] || continue
      basename "${interface}"
    done
  done
}

ch340_interface_driver() {
  local interface="$1"
  local driver_link="/sys/bus/usb/devices/${interface}/driver"

  if [ -L "${driver_link}" ]; then
    basename "$(readlink -f "${driver_link}")"
  else
    printf 'none\n'
  fi
}

show_ch340_interface_drivers() {
  local interface driver found=0

  while IFS= read -r interface; do
    [ -n "${interface}" ] || continue
    found=1
    driver="$(ch340_interface_driver "${interface}")"
    printf 'CH340 interface %s driver: %s\n' "${interface}" "${driver}"
  done < <(list_ch340_interfaces)

  if [ "${found}" -eq 0 ]; then
    printf '未发现 1a86:7523 CH340 USB interface。\n'
  fi
}

rebind_ch340_usbfs_to_wch() {
  local interface driver rebound=1

  if [ ! -d /sys/bus/usb/drivers/usb_ch341 ]; then
    printf 'WARN: usb_ch341 driver is not registered; cannot bind CH340 interface to WCH driver yet.\n' >&2
    return 1
  fi

  while IFS= read -r interface; do
    [ -n "${interface}" ] || continue
    driver="$(ch340_interface_driver "${interface}")"
    if [ "${driver}" = "usb_ch341" ] || [ "${driver}" = "ch341" ]; then
      printf 'CH340 interface %s is already bound to %s.\n' "${interface}" "${driver}"
      rebound=0
      continue
    fi
    if [ "${driver}" != "usbfs" ]; then
      printf 'CH340 interface %s driver is %s; not rebinding automatically.\n' "${interface}" "${driver}"
      continue
    fi

    section "Rebind CH340 interface ${interface}: usbfs -> usb_ch341"
    printf '%s\n' "${interface}" | sudo tee /sys/bus/usb/drivers/usbfs/unbind >/dev/null
    printf '%s\n' "${interface}" | sudo tee /sys/bus/usb/drivers/usb_ch341/bind >/dev/null
    rebound=0
  done < <(list_ch340_interfaces)

  return "${rebound}"
}

sysfs_has_ch340_usb() {
  local device vendor product

  for device in /sys/bus/usb/devices/*; do
    vendor=""
    product=""
    [ -r "${device}/idVendor" ] && vendor="$(tr '[:upper:]' '[:lower:]' < "${device}/idVendor")"
    [ -r "${device}/idProduct" ] && product="$(tr '[:upper:]' '[:lower:]' < "${device}/idProduct")"
    if [ "${vendor}" = "1a86" ] && [ "${product}" = "7523" ]; then
      return 0
    fi
  done
  return 1
}

show_ch340_status() {
  section "CH340 USB and tty status"
  local found_usb=1

  if sysfs_has_ch340_usb; then
    printf 'sysfs: found 1a86:7523 CH340 USB device.\n'
    found_usb=0
  fi

  if command -v lsusb >/dev/null 2>&1; then
    if lsusb | grep -i -E "1a86:7523|ch340|ch341|QinHeng"; then
      found_usb=0
    fi
  else
    printf 'WARN: lsusb 不存在，跳过 USB 枚举检查。\n' >&2
  fi

  if [ "${found_usb}" -ne 0 ]; then
    printf '未发现 1a86:7523 CH340 USB 设备。\n'
  fi

  show_ch340_interface_drivers

  local ttys
  ttys="$(list_ch340_ttys || true)"
  if [ -n "${ttys}" ]; then
    printf 'CH340 tty:\n%s\n' "${ttys}"
  else
    printf '未发现绑定到 1a86:7523 的 /dev/ttyUSB* 或 /dev/ttyCH341USB*。\n'
  fi

  ls -l /dev/robot_imu /dev/ttyUSB* /dev/ttyCH341USB* 2>/dev/null || true
}

precheck() {
  section "Kernel release"
  run uname -r

  section "Kernel build directory"
  local krel
  krel="$(kernel_release)"
  if [ -d "/lib/modules/${krel}/build" ]; then
    run ls -ld "/lib/modules/${krel}/build"
  else
    printf 'WARN: missing kernel build directory: /lib/modules/%s/build\n' "${krel}" >&2
    printf '      编译 WCH 驱动前需要安装匹配当前内核的 headers/build tree。\n' >&2
  fi

  section "Existing kernel modules"
  if module_available usbserial; then
    printf 'usbserial module is available.\n'
  else
    printf 'WARN: usbserial module is not available via modprobe.\n' >&2
  fi
  if module_available ch341; then
    printf 'kernel ch341 module is available; --test-load can use the fast path.\n'
  else
    printf 'kernel ch341 module is not available; WCH source build is needed.\n'
  fi

  show_ch340_status

  section "WCH source"
  if [ -d "${driver_dir}" ]; then
    printf 'Reusing WCH source directory: %s\n' "${driver_dir}"
  elif [ -f "${zip_path}" ]; then
    printf 'Found WCH zip package: %s\n' "${zip_path}"
  else
    printf 'WCH source is not local yet. --build will clone:\n'
    printf '  https://github.com/WCHSoftGroup/ch341ser_linux\n'
    printf 'or place CH341SER_LINUX.ZIP at:\n'
    printf '  %s\n' "${zip_path}"
  fi

  section "Build tools"
  for tool in make gcc; do
    if command -v "${tool}" >/dev/null 2>&1; then
      printf '%s: %s\n' "${tool}" "$(command -v "${tool}")"
    else
      printf 'WARN: %s not found in PATH.\n' "${tool}" >&2
    fi
  done
}

ensure_source() {
  mkdir -p "${base_dir}"

  if [ -d "${driver_dir}" ]; then
    printf 'Reusing WCH source directory: %s\n' "${driver_dir}"
    return
  fi

  if [ -f "${zip_path}" ]; then
    section "Extract WCH zip"
    command -v unzip >/dev/null 2>&1 || die "unzip not found; install unzip or use git source"
    run unzip -q "${zip_path}" -d "${base_dir}"
    if [ -d "${driver_dir}" ]; then
      return
    fi
    local extracted
    extracted="$(find "${base_dir}" -maxdepth 2 -type f -iname 'Makefile' -printf '%h\n' | head -n 1)"
    [ -n "${extracted}" ] || die "解压后未找到包含 Makefile 的 WCH 源码目录"
    run mv "${extracted}" "${driver_dir}"
    return
  fi

  section "Clone WCH source"
  command -v git >/dev/null 2>&1 || die "git not found and ${zip_path} does not exist"
  run git clone --depth 1 https://github.com/WCHSoftGroup/ch341ser_linux.git "${driver_dir}"
}

select_build_dir() {
  if [ -f "${driver_dir}/Makefile" ]; then
    build_dir="${driver_dir}"
    return
  fi
  if [ -f "${driver_dir}/driver/Makefile" ]; then
    build_dir="${driver_dir}/driver"
    return
  fi

  build_dir="$(find "${driver_dir}" -maxdepth 3 -type f -iname 'Makefile' -printf '%h\n' | head -n 1)"
  [ -n "${build_dir}" ] || die "未找到 WCH 源码 Makefile"
}

build_driver() {
  precheck
  ensure_source
  select_build_dir

  section "Build WCH CH341/CH340 serial driver"
  (cd "${build_dir}" && make -n clean >/dev/null 2>&1 && run make clean) || true
  (cd "${build_dir}" && run make)

  section "Built kernel modules"
  built_module="$(find "${build_dir}" -name "*.ko" -print | head -n 1)"
  [ -n "${built_module}" ] || die "编译完成但未找到 .ko 模块"
  run modinfo "${built_module}"
}

load_fast_path() {
  section "Try kernel usbserial/ch341 fast path"
  warn_run sudo modprobe usbserial
  if sudo modprobe ch341; then
    printf 'Loaded kernel ch341 module.\n'
    return 0
  fi
  printf 'kernel ch341 module could not be loaded; will use WCH source build if available.\n'
  return 1
}

verify_tty_after_replug() {
  show_ch340_status
  rebind_ch340_usbfs_to_wch || true
  sleep 1
  show_ch340_status

  if list_ch340_ttys | grep -q .; then
    printf '\nCH340 已生成 tty。WCH 官方驱动通常生成 /dev/ttyCH341USB*；内核原生 ch341 通常生成 /dev/ttyUSB*。\n'
    printf '接下来运行 src/bind_usb.sh 生成 /dev/robot_imu。\n'
    return 0
  fi

  printf '\n请拔插 CH340 IMU USB-TTL 模块，然后按 Enter 继续...'
  read -r _

  rebind_ch340_usbfs_to_wch || true
  sleep 1
  show_ch340_status

  if list_ch340_ttys | grep -q .; then
    printf '\nCH340 已生成 tty。WCH 官方驱动通常生成 /dev/ttyCH341USB*；内核原生 ch341 通常生成 /dev/ttyUSB*。\n'
    printf '接下来运行 src/bind_usb.sh 生成 /dev/robot_imu。\n'
    return 0
  fi

  die "CH340 已尝试加载驱动但仍未生成 /dev/ttyUSB* 或 /dev/ttyCH341USB*；请检查 dmesg 中 ch34/ch341 报错"
}

insmod_wch_module() {
  local output

  if output="$(sudo insmod "${built_module}" 2>&1)"; then
    [ -n "${output}" ] && printf '%s\n' "${output}"
    return 0
  fi

  if printf '%s\n' "${output}" | grep -qi 'File exists'; then
    printf 'WCH module is already loaded; continuing with bind/tty verification.\n'
    return 0
  fi

  printf '%s\n' "${output}" >&2
  die "failed to insmod WCH module: ${built_module}"
}

test_load() {
  if load_fast_path; then
    verify_tty_after_replug
    return
  fi

  build_driver

  section "Temporary insmod WCH module"
  printf '+ sudo insmod %q\n' "${built_module}"
  insmod_wch_module
  verify_tty_after_replug
}

install_driver() {
  test_load

  [ -n "${built_module}" ] || {
    printf '\n系统自带 ch341 已可用，不需要安装 WCH 外置模块。\n'
    return 0
  }

  local krel install_dir module_name
  krel="$(kernel_release)"
  install_dir="/lib/modules/${krel}/extra/wch-ch341"
  module_name="$(basename "${built_module}" .ko)"

  printf '\n将安装模块到: %s/%s.ko\n' "${install_dir}" "${module_name}"
  printf '并写入: /etc/modules-load.d/wch-ch341.conf\n'
  printf '只有确认临时加载已生成 /dev/ttyCH341USB* 或 /dev/ttyUSB* 后，输入精确 YES 才永久安装: '
  local answer
  read -r answer
  if [ "${answer}" != "YES" ]; then
    printf '未输入 YES，退出，不安装。\n'
    return 0
  fi

  section "Install WCH module"
  run sudo mkdir -p "${install_dir}"
  run sudo cp "${built_module}" "${install_dir}/${module_name}.ko"
  printf '%s\n' "${module_name}" | sudo tee /etc/modules-load.d/wch-ch341.conf >/dev/null
  run sudo depmod -a
  warn_run sudo modprobe "${module_name}"

  printf '\n安装完成。请重新插拔 IMU，然后执行:\n'
  printf '  lsmod | grep -E "ch341|ch34x"\n'
  printf '  lsusb\n'
  printf '  dmesg | grep -i ch34\n'
  printf '  ls -l /dev/robot_imu /dev/ttyCH341USB* /dev/ttyUSB*\n'
}

rollback() {
  section "Rollback WCH CH341 changes"
  warn_run sudo rmmod ch34x
  warn_run sudo rmmod ch341
  run sudo rm -f /etc/modules-load.d/wch-ch341.conf
  run sudo rm -rf "/lib/modules/$(kernel_release)/extra/wch-ch341"
  run sudo depmod -a
}

case "${1:-}" in
  --precheck)
    precheck
    ;;
  --build)
    build_driver
    ;;
  --test-load)
    test_load
    ;;
  --install)
    install_driver
    ;;
  --rollback)
    rollback
    ;;
  -h|--help)
    usage
    ;;
  "")
    usage
    exit 1
    ;;
  *)
    usage >&2
    die "未知参数: $1"
    ;;
esac
