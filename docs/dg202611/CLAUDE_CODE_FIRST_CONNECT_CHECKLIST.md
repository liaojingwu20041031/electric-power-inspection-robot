# DG-202611 Claude Code First-Connect Checklist

First connection is read-only: **READ, VERIFY, REPORT**.

## Connect

```powershell
ssh weiyang@192.168.3.130
```

- Port: `22`
- Hostname expected: `ubuntu103`
- Authentication used previously: interactive password; never record it
- Key authentication: `UNKNOWN_NOT_VERIFIED`

## Verify environment

```bash
hostname
whoami
hostname -I
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
cd /home/weiyang/dg202611_ws
printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\nDISPLAY=%s\nROS_LOCALHOST_ONLY=%s\n' \
  "${ROS_DISTRO:-NOT_SET}" "${ROS_DOMAIN_ID:-NOT_SET}" \
  "${RMW_IMPLEMENTATION:-NOT_SET}" "${DISPLAY:-NOT_SET}" \
  "${ROS_LOCALHOST_ONLY:-NOT_SET}"
python3 --version
command -v colcon
git --version
```

Expected: Ubuntu 22.04.5 LTS, ROS 2 Humble, Python 3.10.12, `/usr/bin/colcon`, Git 2.34.1. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and SSH `DISPLAY` were not set; `ROS_LOCALHOST_ONLY=0`.

## Verify repository

```bash
cd /home/weiyang/dg202611_ws/src/electric-power-inspection-robot
git status --short --branch
git branch --show-current
git rev-parse HEAD
git log --oneline --decorate -15
git remote -v
```

Expected branch: `dg202611-synthetic-validation`. The functional validation HEAD is `fd2c60d54401480fd40590857be55b9443c43d49`; takeover HEAD should be one documentation-only handoff commit above it. Expected worktree: clean. Expected remote: `origin` on the GitHub repository. Local commits are not pushed. If anything differs, stop and report—do not reset or clean.

## Read the contracts

```bash
sed -n '1,260p' docs/dg202611/CLAUDE_CODE_HANDOFF.md
sed -n '1,260p' src/dg_synthetic_validation/docs/runtime_contract.md
```

## Locate evidence without rerunning it

```bash
ls -ld /home/weiyang/dg202611_ws/results/synthetic/{S01_nominal_20260828-145833,S02_gnss_gradual_degradation_and_outage_20260828-145858,S03_lidar_geometry_degradation_20260828-145928,S04_gnss_and_lidar_concurrent_degradation_20260828-145956}
sed -n '1,160p' /home/weiyang/dg202611_ws/results/synthetic/summary.md
sed -n '1,20p' /home/weiyang/dg202611_ws/results/synthetic/summary.csv
test -d /home/weiyang/dg202611_ws/results/synthetic/S01_nominal_20260827-205016 && echo HISTORICAL_FAIL_PRESERVED
```

Expected: latest S01–S04 are PASS; S02–S04 have an out-of-scope relocalization warning; S05–S08 are not run. Preserve every existing result directory.

## Verify safety and entry points

- Read build/test commands in the handoff; do not execute them on first connect.
- Test output is `/dg/test_cmd_vel`; real robot command is `/cmd_vel`.
- The synthetic injector must not publish `/dg/*` algorithm outputs.
- No test chain may publish to `/cmd_vel`.
- Monitor is read-only: `ros2 run dg_synthetic_validation monitor_scenario`.
- RViz belongs in a VMware desktop terminal; SSH has no `DISPLAY`.
- `tmux` is not installed; do not install it during takeover.

## Return report and stop

Report actual host/user/IP, ROS environment, repository/branch/HEAD/status, handoff/runtime-contract readability, latest evidence paths and PASS/warning state, safety-contract understanding, and every discrepancy. Do not modify files, commit, build, test, run scenarios, install packages, push, start Gazebo, run S05–S08, or connect/control a robot.
