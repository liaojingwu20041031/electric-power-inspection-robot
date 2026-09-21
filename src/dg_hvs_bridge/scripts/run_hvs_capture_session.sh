#!/usr/bin/env bash
set -euo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Load board-specific paths when this script is copied to an RDK bundle.
# shellcheck source=rdk_env.sh
[ -f "$script_root/../../../scripts/rdk_env.sh" ] && source "$script_root/../../../scripts/rdk_env.sh"

duration_sec="${1:-60}"
root_dir="${2:-${DG_RDK_RECORD_DIR:-$HOME/dg_hvs_evidence}}"
case "${duration_sec}" in
  ''|*[!0-9]*) echo "duration_sec must be a positive integer" >&2; exit 2 ;;
esac
if [ "${duration_sec}" -le 0 ]; then
  echo "duration_sec must be a positive integer" >&2
  exit 2
fi

host_utc="$(date -u +%Y%m%dT%H%M%SZ)"
session_dir="${root_dir}/${host_utc}_hvs"
mkdir -p "${session_dir}"

{
  echo "host_utc=${host_utc}"
  echo "board_date=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "duration_sec=${duration_sec}"
  echo "publishes_cmd_vel=false"
  echo "rdk_user=${DG_RDK_USER:-unknown}"
  echo "rdk_host=${DG_RDK_HOST:-unknown}"
  echo "rdk_workspace=${DG_RDK_WORKSPACE:-unknown}"
  echo "rdk_record_dir=${root_dir}"
} > "${session_dir}/SESSION.txt"

set +e
timeout -s INT -k 10 "${duration_sec}" \
  ros2 launch dg_hvs_bridge hvs_bridge.launch.py \
  recording_enabled:=true record_directory:="${session_dir}" \
  > "${session_dir}/bridge.log" 2>&1
exit_code=$?
set -e
printf '%s\n' "${exit_code}" > "${session_dir}/bridge.exit"

find "${session_dir}" -type f ! -name manifest.sha256 -print0 \
  | sort -z \
  | xargs -0 sha256sum > "${session_dir}/manifest.sha256"

echo "HVS_SESSION_DIR=${session_dir}"
echo "HVS_BRIDGE_EXIT=${exit_code}"
if [ "${exit_code}" -ne 0 ] && [ "${exit_code}" -ne 124 ]; then
  exit "${exit_code}"
fi
