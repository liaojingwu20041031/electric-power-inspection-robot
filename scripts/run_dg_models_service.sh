#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Production remains observation-only until the supervised recovery-motion
# acceptance test is completed on the robot.
exec "${script_dir}/run_dg_integration.sh" \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_uwb_replay:=false \
  cmd_vel_output_topic:=/dg/test_cmd_vel \
  mission_gate_enabled:=true \
  mission_status_timeout_sec:=3.0 \
  mission_activation_grace_sec:=2.0
