# DG-202611 Codex → Claude Code Formal Handoff

## 1. Scope and truth boundary

This is a handoff-only record for the DG-202611 development line. It does not authorize new development.

- Fixed competition baseline: `c41adca7f9bddb4240a1a4855437518b36d9fe13`
- ROS 2 runtime-integration base: `096bf66377e0dbaa3ed812782222ea95a2439c7a`
- Synthetic-validation base: `f7676ee3d9f53bae33d384f5bb506eaa144df095`
- Validated functional-code HEAD before this documentation-only handoff commit: `fd2c60d54401480fd40590857be55b9443c43d49`
- Branch: `dg202611-synthetic-validation`
- Remote: `origin https://github.com/liaojingwu20041031/electric-power-inspection-robot.git`
- Push state at handoff: local commits have not been pushed.

The repository HEAD at takeover is expected to be one documentation-only commit above `fd2c60d`. Verify it with `git log`; the functional code under test remains `fd2c60d`. If the branch, ancestry, or worktree differs, stop and report the actual state. Do not reset, merge, rebase, clean, or overwrite user changes.

Evidence classification:

- `ENGINEERING_FOUNDATION`
- `SOFTWARE_POC`
- `ROS2_RUNTIME_VALIDATED`
- `SYNTHETIC_VALIDATED` for S01–S04 only
- `NOT_YET_GAZEBO_VALIDATED`
- `NOT_YET_TARGET_HARDWARE_VALIDATED`
- `NOT_YET_REAL_ROBOT_VALIDATED`
- `NOT_YET_COMPETITION_METRIC_VALIDATED`

Do not describe the localization system as completed. These results are synthetic software-behavior evidence, not real-robot, Gazebo, accuracy, reliability, or competition-performance evidence.

## 2. Connection and host

The Ubuntu VM is `Ubuntu103` in VMware Workstation on the Windows host.

| Field | Verified value |
|---|---|
| SSH user | `weiyang` |
| SSH host/IP | `192.168.3.130` |
| SSH port | `22` (default) |
| SSH alias | No named alias; Windows SSH config has an IP host stanza for `192.168.3.130` |
| Authentication | Interactive password authentication was used; no password is stored here |
| Key authentication | `UNKNOWN_NOT_VERIFIED` |
| Ubuntu hostname | `ubuntu103` |
| Ubuntu user | `weiyang` |
| Home | `/home/weiyang` |
| Current VM IP | `192.168.3.130` |

Safe connection commands:

```powershell
ssh weiyang@192.168.3.130
# The IP stanza also supplies the user:
ssh 192.168.3.130
```

Never place a password, private key, token, or API key in this repository or in logs.

Codex previously operated mainly through background SSH. Output from that SSH terminal is not mirrored into the VMware Ubuntu GUI. Background SSH is appropriate for Git, build, tests, headless scenarios, and log inspection. The VMware desktop terminal is appropriate for RViz2, GUI tools, and direct user observation. Failure to open RViz from a headless SSH session with no `DISPLAY` is not evidence that the RViz configuration is invalid.

## 3. Environment and ROS 2 setup

Verified on 2026-08-28:

| Component | Value |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS distribution | Humble |
| Python | 3.10.12 |
| colcon | `/usr/bin/colcon` |
| Git | 2.34.1 |
| Workspace | `/home/weiyang/dg202611_ws` |
| Repository | `/home/weiyang/dg202611_ws/src/electric-power-inspection-robot` |
| `ROS_DOMAIN_ID` | `NOT_SET` (default) |
| `RMW_IMPLEMENTATION` | `NOT_SET` (ROS 2 Humble default) |
| `DISPLAY` over the verified SSH session | `NOT_SET` |
| `ROS_LOCALHOST_ONLY` | `0` |

Always initialize explicitly after SSH login:

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
cd /home/weiyang/dg202611_ws
```

Do not rely on `.bashrc`. It currently sources `/opt/ros/humble/setup.bash`, `~/ros2_ws/install/local_setup.bash`, and `~/dev_ws/install/local_setup.sh`, but it does not source the DG workspace. If package conflicts appear, inspect `AMENT_PREFIX_PATH`, `COLCON_PREFIX_PATH`, `ROS_DOMAIN_ID`, and `RMW_IMPLEMENTATION`, and use a clean shell with explicit setup order.

## 4. Git verification and history

Run these read-only checks first:

```bash
cd /home/weiyang/dg202611_ws/src/electric-power-inspection-robot
git status --short --branch
git branch --show-current
git rev-parse HEAD
git log --oneline --decorate -15
git remote -v
```

Relevant history before the handoff documentation commit:

```text
fd2c60d fix(test): prewarm ROS graph before scenario clock
c74e4f5 fix(test): prewarm synthetic sensor inputs
e1fdbc7 fix(test): apply startup grace to nominal health
1a3fb4b fix(test): scope relocalization checks by scenario
618122f fix(test): ignore bounded startup relocalization noise
bd33a76 fix(test): terminate integration cleanly
b8b00ba fix(test): stop ROS processes before context shutdown
380cdd3 feat(test): add validation monitor and plots
d707193 fix(ros2): normalize diagnostic status levels
f7676ee feat(test): add DG synthetic degradation validation
096bf66 fix(ros2): repair DG navigation runtime integration
c41adca feat(fusion): add fault-aware adaptive measurement weighting
```

## 5. Verified build and test entry points

Targeted build command already verified successful:

```bash
cd /home/weiyang/dg202611_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --packages-select ylhb_interfaces ylhb_base dg_synthetic_validation
```

The command emitted an underlay override warning for `ylhb_interfaces` but built the three selected packages successfully. Do not treat a full-repository build as the DG validation entry point: the repository also contains ZED, CUDA, TensorRT, and `ylhb_perception` target/hardware components whose unavailable dependencies can create unrelated failures.

Previously verified tests (recorded results; do not rerun during first takeover):

```bash
/usr/bin/python3 -m pytest -q \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/ylhb_base/test
# 253 passed

/usr/bin/python3 -m pytest -q \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/test
# 4 passed

python3 -m py_compile \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/dg_synthetic_validation/scenario_runner.py \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/dg_synthetic_validation/result_writer.py \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/dg_synthetic_validation/monitor_node.py \
  /home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/dg_synthetic_validation/plot_results.py
# PASS

cd /home/weiyang/dg202611_ws
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
colcon test --packages-select ylhb_base dg_synthetic_validation
colcon test-result --verbose
```

The recorded full selected-package colcon result is `666 tests, 0 errors, 335 failures, 10 skipped`; it is **not PASS**. The failures are predominantly existing flake8/pep257/style findings and must not be represented as core DG behavioral-test failures. Conversely, the targeted pass counts must not be represented as a full-repository pass.

## 6. Competition ROS 2 nodes and topic contract

All seven nodes were built, registered, and jointly launched under ROS 2 Humble.

| Capability | Package / executable | Node name | Main inputs | Main outputs |
|---|---|---|---|---|
| GNSS Quality | `ylhb_base` / `gnss_quality_node` | `dg_gnss_quality_node` | `/gps/fix`, `/gps/rtk_status` | `/dg/gnss/quality`, `/dg/gnss/accepted_fix` |
| LiDAR Robust | `ylhb_base` / `lidar_robust_node` | `dg_lidar_robust_node` | `/scan`, `/odom` | `/dg/lidar/filtered_scan`, `/dg/lidar/stable_features`, `/dg/lidar/quality` |
| Scan-to-Map Relocalization | `ylhb_base` / `scan_map_relocalization_node` | `dg_scan_map_relocalization_node` | `/map`, `/scan`, `/initialpose`, `/dg/relocalization/seed` | `/scan_match_pose`, `/dg/relocalization/match_quality` |
| Active Relocalization | `ylhb_base` / `active_relocalization_node` | `dg_active_relocalization_node` | `/scan`, `/odom`, `/amcl_pose`, `/dg/lidar/quality`, `/dg/gnss/quality`, `/dg/relocalization/match_quality`, `/dg/relocalization/manual_takeover` | `/cmd_vel_recovery`, `/dg/relocalization/status`, `/dg/relocalization/seed` |
| Multisource Fusion | `ylhb_base` / `multisource_fusion_node` | `dg_multisource_fusion_node` | `/odom`, `/dg/gnss/accepted_fix`, `/dg/gnss/quality`, `/dg/lidar/quality`, `/amcl_pose`, `/scan_match_pose`, `/dg/relocalization/match_quality` | `/dg/fusion/odom`, `/dg/fusion/pose`, `/dg/fusion/status` |
| Navigation Health | `ylhb_base` / `navigation_health_node` | `dg_navigation_health_node` | `/dg/lidar/quality`, `/dg/gnss/quality`, `/dg/relocalization/match_quality`, `/dg/relocalization/status`, `/amcl_pose`, `/odom`, `/scan` | `/dg/navigation/status` |
| cmd_vel Arbiter | `ylhb_base` / `cmd_vel_arbiter_node` | `dg_cmd_vel_arbiter_node` | `/cmd_vel_nav`, `/cmd_vel_recovery`, `/dg/navigation/status`, `/dg/relocalization/manual_takeover` | Launch-configured output; `/dg/test_cmd_vel` in validation, `/cmd_vel` by production default |

Read the authoritative runtime contract before changing or running anything:

`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot/src/dg_synthetic_validation/docs/runtime_contract.md`

It defines the injector/algorithm boundary, topics, evaluator fields, thresholds, safety output remapping, and synthetic-evidence limitations.

## 7. DiagnosticStatus compatibility fix

ROS 2 Humble Python exposes `diagnostic_msgs/msg/DiagnosticStatus.level` as `bytes` in this runtime (`OK=b'\x00'`, `WARN=b'\x01'`, `ERROR=b'\x02'`). The old `int(status.level)` path raised `ValueError`.

- Helper: `src/ylhb_base/scripts/diagnostic_level.py`
- Function: `normalize_diagnostic_level`
- Adapters: `gnss_quality_node.py`, `multisource_fusion_node.py`, `navigation_health_node.py`
- Test: `src/ylhb_base/test/test_diagnostic_level.py`
- Compatibility-test result: 18 related tests passed; included in the 253 targeted `ylhb_base` tests.

```text
CORE_ALGORITHM_SEMANTICS_CHANGED: NO
ALGORITHM_THRESHOLDS_CHANGED: NO
STATE_MACHINE_CHANGED: NO
```

## 8. Synthetic-validation framework and exact commands

Package executables include `synthetic_injector_node`, `synthetic_evaluator_node`, `run_scenario`, `run_all_s01_s04`, and `monitor_scenario`.

The following are the exact individual scenario commands. They are recorded for future authorized use; **do not execute them during first takeover**.

```bash
cd /home/weiyang/dg202611_ws
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash

ros2 run dg_synthetic_validation run_scenario S01 \
  --repo-root /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --results-root /home/weiyang/dg202611_ws/results/synthetic

ros2 run dg_synthetic_validation run_scenario S02 \
  --repo-root /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --results-root /home/weiyang/dg202611_ws/results/synthetic

ros2 run dg_synthetic_validation run_scenario S03 \
  --repo-root /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --results-root /home/weiyang/dg202611_ws/results/synthetic

ros2 run dg_synthetic_validation run_scenario S04 \
  --repo-root /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --results-root /home/weiyang/dg202611_ws/results/synthetic
```

The runner is headless and writes rosbag, CSV, timeline, result files, logs, and static PNG plots. `run_all_s01_s04` also exists, but individual runs are preferred when diagnosing a specific scenario.

Live read-only monitor:

```bash
ros2 run dg_synthetic_validation monitor_scenario
```

It only subscribes; it does not publish algorithm inputs, command the robot, or change state. It displays elapsed time plus GNSS, LiDAR, Fusion, Navigation, Relocalization, and `/dg/test_cmd_vel`. The scenario and phase are supplied by the external run and are not live monitor fields; report them as `N/A`, not inferred data. Missing values appear as `NO_DATA`.

RViz2 configuration:

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
rviz2 -d /home/weiyang/dg202611_ws/install/dg_synthetic_validation/share/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz
```

Run RViz2 in an Ubuntu VMware desktop terminal. The verified SSH session had `DISPLAY=NOT_SET`; a Qt/xcb GUI error there is a headless-session limitation, not an RViz configuration failure.

## 9. Final S01–S04 results

Result root: `/home/weiyang/dg202611_ws/results/synthetic/`

| Scenario | Latest directory | Result | Verified observations and limits |
|---|---|---|---|
| S01 nominal | `S01_nominal_20260828-145833` | PASS, 79 samples, no warnings | GNSS and LiDAR healthy, finite Fusion output, Navigation did not fail, no active relocalization after warm-up, safe test output. This is not positioning-accuracy evidence. |
| S02 GNSS degradation/outage | `S02_gnss_gradual_degradation_and_outage_20260828-145858` | PASS, 159 samples, 1 warning | GNSS `GOOD → DEGRADED` at about 4.107 s, then `REJECTED` at about 8.107 s; rejected fixes were not accepted; LiDAR remained healthy. This validates the software gate with synthetic data, not real satellite blockage. |
| S03 LiDAR geometry degradation | `S03_lidar_geometry_degradation_20260828-145928` | PASS, 119 samples, 1 warning | LiDAR `GOOD → DEGRADED` at about 4.108 s, then `REJECTED` at about 8.108 s; geometry score covers 1.0 down to 0.0 and crosses the 0.2 threshold. This is synthetic scan geometry, not real LiDAR performance. |
| S04 concurrent degradation | `S04_gnss_and_lidar_concurrent_degradation_20260828-145956` | PASS, 159 samples, 1 warning | GNSS and LiDAR reached `GOOD/DEGRADED/REJECTED`; Fusion produced finite adaptive fields and observed modes `LIDAR_AIDED`, `DEAD_RECKONING`, and `NOMINAL`; Navigation observed `RECOVERING/DEGRADED`. This demonstrates a synthetic software response chain, not localization accuracy. |

S02–S04 warning: `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE`. It does not prove that the Active Relocalization closed loop succeeds. S05–S08 are `NOT_RUN`.

Each latest scenario directory contains:

```text
rosbag/
samples.csv
timeline.csv
result.json
result.md
plots/gnss_quality.png
plots/lidar_quality.png
plots/fusion_confidence.png
plots/state_timeline.png
```

Summary files:

- `/home/weiyang/dg202611_ws/results/synthetic/summary.csv`
- `/home/weiyang/dg202611_ws/results/synthetic/summary.md`

Historical blocker evidence must be preserved without deletion, overwrite, or modification:

`/home/weiyang/dg202611_ws/results/synthetic/S01_nominal_20260827-205016/`

This run preserves the first real ROS 2 synthetic-runtime evidence that exposed the `DiagnosticStatus.level` compatibility blocker. Its original rosbag, CSV, timeline, and logs are immutable evidence. PNGs generated later from the historical CSV are permitted additions already present.

## 10. Safety contract

```text
TEST_OUTPUT: /dg/test_cmd_vel
REAL_ROBOT_CMD: /cmd_vel
```

S01–S04 recorded zero `/cmd_vel` publishers at both scenario start and end. Test control must never be sent to `/cmd_vel`. Before any future control-related work, run:

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /dg/test_cmd_vel -v
```

If a synthetic/test chain publishes to `/cmd_vel`, stop immediately and report it.

The synthetic injector may publish only sensor/base inputs:

- `/gps/fix`
- `/gps/rtk_status`
- `/scan`
- `/odom`
- `/amcl_pose`
- `/map`
- `/initialpose`

`DIRECT_DG_OUTPUTS_FAKED_BY_INJECTOR: NO`. Never make a scenario pass by directly publishing `/dg/gnss/quality`, `/dg/lidar/quality`, `/dg/fusion/status`, `/dg/navigation/status`, or `/dg/relocalization/status`; those must come from the real competition nodes.

## 11. Visible user workflow

`tmux` is not installed and no tmux session exists. Do not install it as part of takeover. If the owner later approves installation, use a stable session:

```bash
tmux new -s dg202611
tmux attach -t dg202611
tmux ls
```

Detach without stopping: press `Ctrl+B`, then `D`.

For future authorized experiments, keep automation and observation separate:

- Terminal A: experiment command, or `tmux attach -t dg202611` after tmux is explicitly approved and installed.
- Terminal B: source both ROS setups, then run `ros2 run dg_synthetic_validation monitor_scenario`.
- Terminal C: source both ROS setups, then start RViz2 with the installed configuration above.
- Background SSH: Git, build, test, headless scenario, logs.
- VMware desktop terminals: monitor and RViz visible to the user.

Gazebo is not part of the current phase and must not be installed or started.

## 12. Engineering foundation, POCs, and open gaps

Repository documentation/configuration records this engineering foundation: Jetson Orin Nano Super, Ubuntu 22.04/ROS 2 Humble, differential drive, ZLAC8015D V4, RPLidar, HiPNUC IMU, WTRTK980 RTK/GNSS, ZED 2i, SLAM Toolbox, AMCL, and Nav2.

Current `src/ylhb_base/config/ekf.yaml` is a two-dimensional wheel-odometry plus IMU EKF with `world_frame: odom`. It does not establish GNSS fusion or 3D/Z performance.

Code/POC presence:

- GNSS quality gate
- LiDAR robust processing, stable-feature extraction, dynamic-candidate handling, and geometry quality
- Scan-to-Map matching and recovery logic
- Active Relocalization state machine
- Multisource Fusion side channel and adaptive observation weighting
- Navigation Health
- cmd_vel Arbiter
- Synthetic Validation Framework

Validated: code presence, targeted ROS 2 build/discovery/joint launch, and S01–S04 synthetic state-response behavior.

Not completed or not proven:

- S05–S08: `NOT_RUN`
- Gazebo: `NOT_RUN`
- Real robot: `NOT_RUN`
- Jetson target integration: `NOT_DONE`
- ZED SDK/VIO integration: `NOT_DONE`
- mmWave radar: `NOT_DONE`
- Optical flow: `NOT_DONE`
- Event camera: `NOT_DONE`
- BeiDou short-message: `NOT_IMPLEMENTED`
- Official XY error <20 cm: `NOT_PROVEN`
- Official Z error <20 cm: `NOT_PROVEN`
- Feature repeatability ≥95%: `NOT_PROVEN`
- Relocalization success >95%: `NOT_PROVEN`

Z/elevation remains an `OPEN_TECHNICAL_GAP`: 2D wheel+IMU EKF and 2D LiDAR/AMCL cannot prove Z performance. Future candidates include ZED 2i VIO, RTK altitude, a 3D estimator, 3D LiDAR, or another height observation, but no option has been selected.

Teacher-proposed “conventional localization + innovative localization + BeiDou” is `NEXT_STAGE_TECHNOLOGY_SELECTION_CONTEXT`, not an implementation directive. Candidate technologies are ZED 2i VIO/3D localization, mmWave radar, optical-flow/ground-texture assistance, and research reserves such as event/ToF/polarization sensing. Do not install them during first takeover.

Recommended future order, subject to external approval:

1. Phase A — S05–S08 Active Relocalization synthetic closed-loop validation.
2. Phase B — ZED 2i/VIO technical verification and interface design.
3. Phase C — 3D/Z-axis localization POC.
4. Phase D — mmWave radar technology selection.
5. Phase E — Gazebo/sensor simulation.
6. Phase F — Jetson target integration.
7. Phase G — real robot.
8. Phase H — official metric validation.

Before Phase A, explicitly account for the current out-of-scope relocalization warnings and inspect any `/initialpose` QoS warning. Do not change code until a separately approved task defines the expected closed-loop behavior.

## 13. First takeover rule

The first Claude Code interaction is **READ, VERIFY, REPORT only**. It must connect, verify environment/Git, read this handoff and `runtime_contract.md`, locate summary/latest evidence, inspect the documented build/test entry points and safety contract, and return a takeover report. It must not modify code or documents, commit, rerun scenarios/tests/builds, install software, push, start S05–S08/Gazebo, or connect/control a robot.
