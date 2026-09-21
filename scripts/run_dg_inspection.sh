#!/usr/bin/env bash
set -euo pipefail

# Production console policy: use one, known-good UI and Mobile Bridge owner.
# The DG workspace owns model/sensor sidecars; it must not overlay ylhb_llm.
integration_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"
base_runner="${base_ws}/scripts/run_on_jetson.sh"
base_ui="${base_ws}/install/ylhb_llm/lib/ylhb_llm/inspection_display_ui_node"

if [ ! -x "${base_runner}" ]; then
  echo "基础系统启动脚本不存在：${base_runner}" >&2
  exit 2
fi
if [ ! -f "${base_ws}/install/setup.bash" ]; then
  echo "基础系统未编译：${base_ws}" >&2
  exit 2
fi
if [ ! -x "${base_ui}" ]; then
  echo "基础系统 UI 不存在：${base_ui}" >&2
  echo "请先在基础工作区完成 ylhb_llm 构建。" >&2
  exit 2
fi

existing_stack="$(pgrep -af '[i]nspection_display_ui_node|[s]ystem_supervisor_node' || true)"
if [ -n "${existing_stack}" ]; then
  echo "拒绝启动第二套 inspection UI/Supervisor：已有运行实例。" >&2
  echo "${existing_stack}" >&2
  exit 3
fi

# Mobile Bridge is owned by systemd in production. Do not let this wrapper
# silently create a second bridge through Supervisor.
for arg in "$@"; do
  case "${arg}" in
    mobile_bridge_managed_externally:=*|auto_start_mobile_bridge:=*)
      echo "生产入口不允许覆盖 Mobile Bridge 所有权参数：${arg}" >&2
      echo "请只保留 ylhb-mobile-bridge.service 作为唯一 owner。" >&2
      exit 2
      ;;
  esac
done

export WS_DIR="${base_ws}"
export DG_INTEGRATION_WS="${integration_ws}"
exec "${base_runner}" inspection \
  mobile_bridge_managed_externally:=true \
  auto_start_mobile_bridge:=false \
  "$@"
