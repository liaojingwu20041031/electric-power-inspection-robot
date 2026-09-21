# DG-202611 Round B — 03 Implementation Log

Chronological record of what was investigated, decided, and produced in Round B.
Prefix for every relative path:
`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot`

Round B changed no core algorithm file. `CORE_THRESHOLD_CHANGE`,
`CORE_STATE_MACHINE_CHANGE`, and `CORE_ALGORITHM_SEMANTIC_CHANGE` are all
`NOT ALLOWED` and all remained `NO`.

## 1. Read the real state machine

Read `src/ylhb_base/scripts/active_relocalization_core.py` end to end rather than
relying on the handoff summary.

Result: ten states, listed at lines 15-24. `LOST` and `SEARCHING` do not exist.
Earlier informal descriptions using those names were wrong and are corrected in
`02_architecture_and_design.md`. Full transition table and verified defaults are
recorded there.

Two facts from this read shaped the rest of the round:

- `command_linear_x` is always `0.0` (line 673) — recovery is pure rotation
- `STOPPING` has no timeout — a bad stop condition hangs the machine forever

## 2. Found the trigger/degraded split

Read `assess_health`, lines 192-304, and separated the two reason lists.

`trigger_reasons` can start a recovery: `AMCL_COVARIANCE_HIGH`,
`AMCL_COVARIANCE_INVALID`, `AMCL_STALE`, `SCAN_MATCH_QUALITY_LOW`,
`SCAN_MATCH_INVALID`, `SCAN_STALE`.

`degraded_reasons` cannot: `LIDAR_QUALITY_LOW`, `LIDAR_QUALITY_INVALID`,
`GNSS_DEGRADED`, `GNSS_REJECTED`, `ODOM_STALE`.

This is the key finding of Round B. S02–S04 degrade GNSS and LiDAR only, so they
can never trigger active relocalization, and the recorded
`RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` warning is not closed-loop
success. Recorded in `02_architecture_and_design.md` section 3 and in
`06_results_and_evidence.md`.

## 3. Traced the arbiter chain

Read `src/ylhb_base/scripts/navigation_health_core.py:199-217,253-262` and
`src/ylhb_base/scripts/cmd_vel_arbiter_core.py:84-109`.

Confirmed that five relocalization states collapse to `RECOVERING`, that
`RECOVERING` is evaluated before the AMCL-`REJECTED` branch, and that the arbiter
forwards recovery only while `navigation_state == RECOVERING`. This establishes
the minimum observable chain any S05–S08 run must show.

## 4. Diagnosed why the old synthetic input could never work

Four independent blockers were identified in the S01–S04 injector. Each one alone
is sufficient to prevent a closed loop. Full detail, including the failure trail,
is in `04_debug_and_failures.md`; summary here:

| # | Blocker | Effect |
|---|---|---|
| 1 | `_make_amcl` covariance pinned at `0.05`, below `max_covariance=0.5` | `AMCL_COVARIANCE_HIGH` could never fire, so no trigger path existed at all |
| 2 | `_make_odom` `twist.linear.x` pinned at `0.05`, above `stop_velocity=0.03` | `STOPPING` could never confirm, and it has no timeout, so it would hang indefinitely |
| 3 | odom yaw pinned at `0` with no motion model | `ACTIVE_SCAN` could never reach its target angle, ending in `SEGMENT_TIMEOUT` after 8 s |
| 4 | no TF publisher anywhere in the integration launch | `scan_map_relocalization_node._scan_to_base_points` always failed with `TF_FAILED`, so no candidate could ever be produced |

Additionally the injector's analytic scan was `range + 0.12*sin(3a) +
0.04*cos(7a)`, a wobbling circle geometrically inconsistent with the published
map. Even with TF present, a match was impossible.

Blockers 1–3 are why `DECISION_001` (the plant) was needed. Blocker 4 is why the
injector boundary had to widen to `/tf` and `/tf_static`.

## 5. Investigated TF ownership

Searched the whole DG chain for TF use. Result: the only consumer is
`src/ylhb_base/scripts/scan_map_relocalization_node.py:452`, looking up
`base_footprint <- laser`. `src/ylhb_base/scripts/multisource_fusion_node.py:196-197`
explicitly refuses to own `map -> odom`.

Approved `odom -> base_footprint` (dynamic) and `base_footprint -> laser`
(static). Denied `map -> odom`. Written up as ten questions in
`MAP_ODOM_TF_JUSTIFICATION.md`.

Consequence recorded: RViz fixed frame is `odom`
(`src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz:74`) and the map
is drawn by a visualization-only `MarkerArray` on `/dg_validation_viz/markers`,
outside `/dg/*`
(`src/dg_synthetic_validation/dg_synthetic_validation/visualization_markers_node.py:7,39`).

## 6. Offline verification of the real matcher

Called the real `refine_pose_near_seed` from
`src/ylhb_base/scripts/scan_map_relocalization_node.py` against ray-cast
geometry, with thresholds untouched (`min_score=0.45`, `min_inlier_ratio=0.45`,
`max_mean_distance=0.30`, verified at lines 178-180 and 251-253).

| Case | Result | score | inlier | mean | recovered yaw |
|---|---|---|---|---|---|
| no rotation | `ACCEPTED` | 0.975 | 1.000 | 0.009 | — |
| true yaw +10 deg from a 0 deg seed | `ACCEPTED` | 0.975 | 1.000 | 0.009 | 10.00 deg |
| true yaw -10 deg from a 0 deg seed | `ACCEPTED` | 0.975 | 1.000 | 0.009 | -10.00 deg |

180 scan points, about 0.15 s per match against a `segment_timeout` of 8 s, so
there is ample margin. The distance field built in 0.05 s.

Also verified: the new ray-cast generator produces 360 finite beams, 0.850 m to
the interior wall and 3.900 m to the border from the origin; the analytic
generator remains bit-identical to the historical S01–S04 waveform, so S01–S04
evidence is unaffected.

```text
CONCLUSION: no threshold, state machine, or algorithm semantic change is needed.
```

The +-10 deg cases are not arbitrary: they match `segment_deltas = (+10, -20,
+10)`, so they verify the matcher over the exact rotations the recovery performs.

## 7. Designed the two synthetic components

`SYNTHETIC_KINEMATIC_PLANT` (`DECISION_001`) and `SYNTHETIC_AMCL_SURROGATE`
(`DECISION_002`) were specified as two separately named components. The first
attempt folded the surrogate into the plant; that was corrected. See
`04_debug_and_failures.md` section 6.

Both specifications are recorded in `02_architecture_and_design.md` sections 7
and 8, including the deliberate pose bias of about 0.15 m / 8 deg that sits
inside the matcher's real +-0.20 m / +-10 deg coarse window.

## 8. Probed GUI capture on this host

Established that the Ubuntu103 desktop session is Wayland (GNOME) and that
automatic screen capture is not available. Three methods were tried and all
failed. Detail in `04_debug_and_failures.md` section 8.

```text
GUI_CAPTURE_READY: MANUAL_ONLY
```

Automatic data plots are unaffected and are still produced.

## 9. Landed the two code-side decision deltas

The earlier half of Round B recorded `DECISION_002` and `DECISION_005` as decisions
whose code side had not been landed. Both are now present on the target and were
confirmed by read-back, not by a transfer summary.

| Decision | Target evidence |
|---|---|
| `002` — surrogate is a separate component | `scenario_schema.py` declares `class AmclSurrogateConfig` alongside `class PlantConfig`; `PlantConfig` no longer carries `amcl_follows_plant` or `covariance_after_match`, and its docstring now ends "localization observation is a separate concern, see AmclSurrogateConfig" |
| `005` — no `map -> odom` | `synthetic_injector_node.py` `_static_transforms` returns `base_footprint -> sensor` only, with a docstring stating `map->odom is deliberately absent` |

Corroborated in every precheck artefact: `PERFECT_MAP_ODOM_PUBLISHED: NO`.

```text
CODE_SIDE_LANDING_OF_DECISION_002: TARGET_WORKTREE_VERIFIED
CODE_SIDE_LANDING_OF_DECISION_005: TARGET_WORKTREE_VERIFIED
```

## 10. Second staging-target divergence, and the gate that follows from it

```text
REPEATED_FAILURE_CLASS: STAGING_TARGET_DIVERGENCE
OCCURRENCE_COUNT:       2
```

Occurrence 1: the `map -> odom` removal and the surrogate separation were reported
done while the target still held the old code. Occurrence 2: the `InputState` class
was reported added while a structural `grep` on the target evaluator returned
nothing.

Both times the implementation existed only in a local staging directory, completed
after an earlier sync, and status was reported from local state. A file-transfer
command that exits 0 and a subagent summary are **not** verification. Full record,
including the mandatory four-part gate, in `04_debug_and_failures.md` section 9.

## 11. Made the INPUT evidence real, then proved it round-trips

`InputState` reached the target, and two accidental duplications were found and
removed: an older incomplete `InputState` without `tf_base_to_sensor_available`
shadowed by a newer complete one, and a duplicated `quaternion_yaw`. Python accepts
shadowed definitions silently, so the file imported cleanly while being unable to
record the field two scenario criteria depend on.

Verified after the fix:

```text
InputState definitions:     1
quaternion_yaw definitions: 1
INPUT columns non-empty:    15/15  (real SampleRecorder, write then re-read samples.csv)
quaternion_yaw(40 deg):     40.00 degrees
```

Why this mattered beyond tidiness: with the columns empty,
`s06_plant_yaw_responded_to_command` would have failed for lack of evidence and the
failure would have been read as "the synthetic plant does not respond to commands".
The correct classification is `EVIDENCE_PIPELINE_BUG`, not `PLANT_FAILURE`. See
`04_debug_and_failures.md` section 10.

## 12. Closed the empty-plot evidence hole

`plot_results` `save()` was appending a figure path to `files` even when zero data
series had been plotted. Two live blank paths were reproduced: the `state_timeline`
figure, and `save()` figures plotting a populated value column against an all-`None`
`elapsed_time`.

Implemented: data artists tagged `dg-data-series`, threshold lines tagged
`dg-reference-line`, and one write choke point that refuses `savefig` when the
counted finite-point series is zero. Tagging is required because `len(ax.lines)`
counts `axhline` threshold lines drawn from hard-coded algorithm defaults, so a panel
with only threshold lines would otherwise self-certify as populated.

```text
NEW_TESTS: 14
RULE:      FILE_EXISTS != VALID_EVIDENCE
```

Verified by read-back on the target: `DATA_SERIES_GID = "dg-data-series"` and
`REFERENCE_LINE_GID = "dg-reference-line"` in `plot_results.py`.

## 13. Fixed the forbidden-claim scanner, twice

Defect 1: patterns wrapped CJK phrases in `\b` anchors. Python `re` has no word
boundary between two CJK characters, so the anchored patterns never matched
mid-sentence and the scanner passed the claims it exists to block. Missed phrases
confirmed: `定位精度20厘米`, `使用Gazebo仿真采集`.

Defect 2, worse: the scanner flagged its own **mandatory** disclaimers, reporting
`NOT REAL ROBOT DATA` and `非真实机器人数据` as claims. A guard that fails on the
markers it requires is a guard someone switches off.

```text
AFTER FIX: POSITIVE 11/11 caught, NEGATIVE 10/10 not flagged
```

## 14. Repaired the RViz configuration

Three displays were dead. `/dg/fusion/pose` and `/amcl_pose` are
`PoseWithCovarianceStamped` but were configured as `rviz_default_plugins/Pose`, which
consumes `PoseStamped`; a third display targeted the `map` frame, unreachable with no
`map -> odom`. All three would have appeared configured and rendered nothing.

```text
RULE: a .rviz file parsing as valid YAML proves neither that RViz accepts each
display's field set nor that the message types match.
```

Verified by read-back of `src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz`:
`Grid`, `Raw Scan` (`/scan`), `Filtered Scan` (`/dg/lidar/filtered_scan`), `Odometry`
(`/odom`), `TF`, `Validation Markers` (`/dg_validation_viz/markers`),
`Fixed Frame: odom`, default view `rviz_default_plugins/TopDownOrtho`.

## 15. Packaging: executables and scenario files

Verified with `ros2 pkg executables dg_synthetic_validation` against the installed
workspace, which is a runtime check rather than a reading of `setup.py`:

```text
capture_evidence          monitor_scenario         run_all_s01_s04
run_scenario              synthetic_evaluator_node synthetic_injector_node
visualization_markers_node write_evidence_manifest
EXECUTABLES: 8
```

`visualization_markers_node` therefore has an installed executable, closing the open
issue that previously forced `USER_VISIBLE_WORKFLOW.md` to list only two commands.

All eight scenario YAMLs install to share. Verified in
`install/dg_synthetic_validation/share/dg_synthetic_validation/config/`: `S01.yaml`
through `S08.yaml`.

## 16. Ran the S05–S08 headless precheck

Four prechecks were executed and all four `PASS`.

```text
RUN_CLASS:              PRECHECK
NOT_FINAL_EVIDENCE:     TRUE
NOT_FOR_DOCUMENT_CLAIM: TRUE
EVIDENCE_LEVEL:         SYNTHETIC SOFTWARE VALIDATION PRECHECK
LOCATION:               results/synthetic/precheck/
```

Those four tokens are recorded inside `metadata.json` **and** `result.json` **and**
prominently at the top of `result.md` — not merely in the directory name. A directory
name is not a label: it is lost the moment a figure or a table is copied out of it.

Results, run paths, and the criteria actually evaluated are in
`06_results_and_evidence.md`. Two findings worth carrying here:

- S06 and S07 incidentally reached `RECOVERED` although neither is chartered to test
  recovery, because `_handle_verifying` requires candidate quality plus pose
  stability and does **not** require healthy covariance
- S07 and S08 reported bit-identical match quality, which is deterministic and
  expected, and must never be read as two independent corroborating measurements

## 17. Reproduced the regression figures

First attempt used bare `/usr/bin/python3` and failed at collection with
`ModuleNotFoundError: No module named 'rclpy'`, because the ROS environment was not
sourced. That is an invocation error, not a code defect, and the historical figure
was **not** quoted as reproduced while it stood.

After sourcing `/opt/ros/humble/setup.bash` and the workspace install:

```text
TARGETED_FUNCTIONAL_TESTS  ylhb_base:                253 passed
PREVIOUS_253_PASS_REPRODUCED:                        YES
SYNTHETIC_PACKAGE_TESTS    dg_synthetic_validation:   33 passed
FULL_COLCON_TEST:                                    NOT_RUN this round
```

The three categories are never merged. See `06_results_and_evidence.md` section 6.

## 18. Visualization layout fix — landed and verified

The user opened the GUI and found the RViz text markers severely overlapping, making the
default view unusable as evidence. The fix is visualization-only: separated regions, one
marker per line with a unique id and position, uniform text sizing, and a
screenshot-ready default `TopDownOrtho` view. It touches no algorithm, threshold, or
state machine.

```text
MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED
```

Verification, with provenance kept explicit: `py_compile` on the node,
`colcon build --packages-select dg_synthetic_validation`, the static layout audit
(24 text markers, 5 separated regions, 0.70 m minimum pairwise separation against a
0.44 m worst-case line height, about 0.26 m clearance), the display-type audit and the
`VISUALIZATION_NODE_BOUNDARY` grep were performed and reported by the coordinator.
Re-confirmed independently by read-back here: source and installed
`rviz/dg_synthetic_validation.rviz` both parse as YAML with 6 displays,
`Fixed Frame: odom`, `TopDownOrtho`, `Scale: 32`, `Y: -2.1`; the node parses via
`ast.parse`; exactly one `create_publisher` on `/dg_validation_viz/markers` with zero TF
symbols and `/cmd_vel` only as a graph publisher-count query; `ros2 pkg executables`
still lists 8; `git status --porcelain src/ylhb_base` still returns 0 lines.

Because the new view reached the **installed** config, RViz started from the install path
gets the fixed layout rather than only the source tree.

Two limits remain, both recorded rather than closed: `RVIZ_DISPLAY_FIELD_ACCEPTANCE` is
`NOT_YET_VERIFIED`, because a YAML parse cannot tell an accepted field set from one RViz
silently drops, and `EVIDENCE_TEXT_LEGIBILITY_AT_1280x800` is
`USER_JUDGEMENT_REQUIRED`. See `07_open_issues.md` sections 2.3 and 2.4.

Recorded alongside it, because it will otherwise be mistaken for a defect: idle RViz
showing no `Raw Scan`, `Filtered Scan` or `Odometry` data, and the monitor showing
`NO_DATA`, is the correct idle behaviour when no scenario is running. It must never be
"fixed" by fabricating a publisher.

## 19. Verified state of the remote worktree at time of writing

Read-only observations, each from a direct read of the target.

| Observation | Verified state |
|---|---|
| `config/` scenario files | `S01.yaml` .. `S08.yaml` present; `S05`–`S08` also installed to share |
| landed scenario names | S05 `amcl_covariance_ramp_localization_trigger`, S06 `closed_loop_active_scan_recovery_motion`, S07 `real_candidate_from_real_seed_request`, S08 `multiframe_verification_and_control_handoff` |
| `setup.py` `console_scripts` | 8 entries, all 8 confirmed by `ros2 pkg executables` |
| `AmclSurrogateConfig` | present and separate from `PlantConfig` (`DECISION_002` landed) |
| `map -> odom` | absent from `_static_transforms` (`DECISION_005` landed) |
| `ylhb_base` | `git status --porcelain src/ylhb_base` returns 0 lines — clean and unmodified |
| repository HEAD | `57ec8ec docs(dg): add Claude Code development handoff`, branch `dg202611-synthetic-validation` |
| Round B work | modified / untracked only; nothing committed |

```text
GIT_COMMITTED: NO
GIT_PUSHED:    NO
```

All four landed scenarios match their specified intent, S08 included: the protocol
specifies multi-frame recovery verification for S08, and that is what
`multiframe_verification_and_control_handoff` implements. What no scenario exercises is
the `FAILED` / `MANUAL_REQUIRED` half of the machine — a coverage gap rather than a spec
deviation, recorded in `07_open_issues.md` section 5.

## 20. Environment context

```text
VM rebooted between sessions:  YES (uptime 11 minutes at 2026-08-29 11:37,
                               versus a prior session on 2026-08-28)
