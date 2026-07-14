#!/usr/bin/env bash

mobile_bridge_unit="ylhb-mobile-bridge.service"

mobile_bridge_unit_exists() {
  systemctl list-unit-files "${mobile_bridge_unit}" --no-legend 2>/dev/null | grep -q "^${mobile_bridge_unit}"
}

resolve_mobile_bridge_owner() {
  local requested="${YLHB_MOBILE_BRIDGE_OWNER:-auto}" explicit=""
  for arg in "$@"; do
    case "${arg}" in
      mobile_bridge_managed_externally:=true) explicit="systemd" ;;
      mobile_bridge_managed_externally:=false) explicit="supervisor" ;;
    esac
  done
  [ -z "${explicit}" ] || requested="${explicit}"
  case "${requested}" in
    auto)
      if systemctl is-active --quiet "${mobile_bridge_unit}" 2>/dev/null; then
        requested="systemd"
      elif systemctl is-enabled --quiet "${mobile_bridge_unit}" 2>/dev/null; then
        requested="systemd"
        echo "ERROR: ${mobile_bridge_unit} is enabled but not active; refusing to start a second Mobile Bridge." >&2
      elif mobile_bridge_unit_exists; then
        requested="supervisor"
      else
        requested="supervisor"
      fi
      ;;
    systemd|supervisor) ;;
    *) echo "ERROR: YLHB_MOBILE_BRIDGE_OWNER must be auto, systemd or supervisor." >&2; return 2 ;;
  esac
  if [ "${requested}" = "systemd" ] && ! systemctl is-active --quiet "${mobile_bridge_unit}" 2>/dev/null; then
    echo "ERROR: Mobile Bridge owner is systemd but ${mobile_bridge_unit} is not active." >&2
  fi
  if [ "${requested}" = "supervisor" ] && systemctl is-active --quiet "${mobile_bridge_unit}" 2>/dev/null; then
    echo "ERROR: ${mobile_bridge_unit} is active; refusing supervisor ownership to avoid a duplicate Mobile Bridge." >&2
    return 2
  fi
  printf '%s\n' "${requested}"
}
