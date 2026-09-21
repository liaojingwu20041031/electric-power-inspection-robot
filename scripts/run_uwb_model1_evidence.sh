#!/usr/bin/env bash
set -euo pipefail

integration_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WS_DIR="${WS_DIR:-${integration_ws}}"

scenario="${1:-static}"
case "${scenario}" in
  static)
    filename="static_raw_20260905_165208.csv"
    ;;
  move)
    filename="move_raw_20260905_170425.csv"
    ;;
  anchor_fault)
    filename="anchor_fault.csv"
    ;;
  *)
    echo "Usage: $0 {static|move|anchor_fault} [ROS launch arguments]" >&2
    exit 2
    ;;
esac
shift || true

input_file="${integration_ws}/src/ylhb_base/data/uwb/20260905/${filename}"
if [ ! -f "${input_file}" ]; then
  echo "Measured UWB input is missing: ${input_file}" >&2
  exit 2
fi

echo "Starting motion-free UWB -> MODEL-1 evidence replay" >&2
echo "Scenario: ${scenario}" >&2
echo "Input: ${input_file}" >&2
echo "Output frame: uwb_replay_test (identity test transform, not map alignment)" >&2

exec "${integration_ws}/scripts/run_on_jetson.sh" \
  dg_uwb_evidence \
  input_file:="${input_file}" \
  output_frame_id:=uwb_replay_test \
  "$@"
