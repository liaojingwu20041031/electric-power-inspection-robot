# DG-202611 Round B — 08 Handoff Summary

```text
ROUND:            B
NATURE:           investigation, design, implementation-support, and record round
SCENARIOS RUN:    S05-S08 PRECHECK only, all four PASS
FINAL EVIDENCE:   NONE (NO_FINAL_EVIDENCE_SCENARIO_EXECUTED = YES)
EVIDENCE_CLASS:   SYNTHETIC_SOFTWARE_VALIDATION
GIT_COMMITTED:    NO
GIT_PUSHED:       NO
```

## 1. Document set

Extends the existing `docs/dg202611/` directory, which previously held
`CLAUDE_CODE_HANDOFF.md` and `CLAUDE_CODE_FIRST_CONNECT_CHECKLIST.md`. Those two are
unchanged.

| File | Contents |
|---|---|
| `01_goal_and_scope.md` | Goal, evidence class, run-class and status vocabulary, forbidden claims, frozen constraints, environment context |
| `02_architecture_and_design.md` | The real state machine, trigger split, arbiter chain, plant, surrogate, TF, `DECISION_001`–`008`, field-role model, evidence-validity contracts |
| `03_implementation_log.md` | What was investigated and produced, in order |
| `04_debug_and_failures.md` | The unsanitised failure trail, 20 entries |
| `05_test_protocol.md` | S05–S08 protocols, precheck criteria, verification rules |
| `06_results_and_evidence.md` | Precheck results with run paths, the three test categories, and the corrected FINAL-evidence accounting (this column previously read "final evidence `NOT_YET_RUN`", which contradicted disk) |
| `07_open_issues.md` | Verified open items, and the items closed this round |
| `08_handoff_summary.md` | This file |
| `MAP_ODOM_TF_JUSTIFICATION.md` | The ten-question `map -> odom` decision record |
| `EXPERIMENT_EVIDENCE_POLICY.md` | Permanent forward-looking evidence rule |
| `USER_VISIBLE_WORKFLOW.md` | Terminals A/B/C and the visible-evidence checkpoint |
| `MANUAL_CAPTURE_CHECKLIST.md` | The manual capture path on this host |
| `GAZEBO_SIMULATION_READINESS_REPORT.md` | Read-only audit; readiness `BLOCKED` |

## 2. The two findings that matter most

**One.** Active relocalization has two separate health reason lists
(`src/ylhb_base/scripts/active_relocalization_core.py:192-304`). Only
`trigger_reasons` can start a recovery. GNSS and LiDAR degradation land in
`degraded_reasons`.

```text
S02-S04-style GNSS/LiDAR degradation can NEVER trigger active relocalization.
```

So the S02–S04 warning `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` is **not**
closed-loop success. The S05 precheck is the contrast case: its trigger reason is
`AMCL_COVARIANCE_HIGH`, recorded per sample.

**Two.** An evidence pipeline can fail silently in more ways than the system under
test can. This round found a blank plot filed as a figure, a black frame that passed a
content check, RViz displays that rendered nothing while appearing configured, a
forbidden-claim scanner that matched nothing, and an INPUT recorder that would have
turned a working plant into a red S06.

```text
FILE_EXISTS        != VALID_EVIDENCE
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

## 3. What Round B established

- The state machine has exactly ten states. `LOST` and `SEARCHING` do not exist
- Recovery motion is pure rotation: `command_linear_x` is always `0.0`; observed as
  max `|cmd_linear_x| = 0.000000` in the S06 precheck
- `STOPPING` has no timeout, so a bad stop condition hangs silently
- Four independent blockers made the old synthetic input incapable of a closed loop
- The only TF consumer in the DG chain is `scan_map_relocalization_node.py:452`,
  looking up `base_footprint <- laser`
- The unmodified matcher recovers +-10 deg yaw offsets from consistent synthetic
  geometry; offline `ACCEPTED`, `score=0.975`, `inlier=1.000`, `mean=0.009`
- S05–S08 precheck all `PASS`: the trigger path, the stop confirmation, the commanded
  rotation, a real candidate attributable to a real seed request, and multi-frame
  verification followed by a control handoff
- S06 and S07 incidentally reach `RECOVERED`; their **navigation** ends at
  `LOCALIZATION_SUSPECT`, and S08 is the only run whose navigation reaches `RECOVERED`
- Consequently **no** threshold, state machine, or algorithm semantic change is needed
- Automatic GUI capture is impossible on this Wayland host; `GUI_CAPTURE_READY:
  MANUAL_ONLY`

## 4. The repeated process failure, and the gate that closes it

```text
REPEATED_FAILURE_CLASS: STAGING_TARGET_DIVERGENCE
OCCURRENCE_COUNT:       2
```

Both times, work was completed in a local staging directory after an earlier sync and
then reported from local state. Occurrence 1: `map -> odom` removal and the surrogate
separation. Occurrence 2: the `InputState` class, while a structural `grep` on the
target returned nothing — which would have produced a false S06 `PLANT_FAILURE` from 15
empty INPUT columns.

Mandatory before any `TARGET_WORKTREE_VERIFIED` claim: target read-back, structural
`grep`, `py_compile` on the target copy, and a runtime sanity check where a data path
is involved.

```text
NOT VERIFICATION: a file-transfer command that exited 0.
NOT VERIFICATION: a subagent summary stating the work was done.
```

Status vocabulary, never used interchangeably: `LOCAL_STAGING_ONLY`,
`WRITTEN_TO_TARGET_WORKTREE`, `TARGET_WORKTREE_VERIFIED`, `GIT_COMMITTED`,
`GIT_PUSHED`.

## 5. What the next round must do

| Item | State |
|---|---|
| RViz marker layout | landed; `MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED`, installed config included |
| `RVIZ_DISPLAY_FIELD_ACCEPTANCE` | `NOT_YET_VERIFIED` — a YAML parse cannot prove RViz honours a display's field set; only opening RViz settles it |
| `EVIDENCE_TEXT_LEGIBILITY_AT_1280x800` | `USER_JUDGEMENT_REQUIRED` — body text about 9-10 px cap height at `Scale: 32`; the visible extent was deliberately not cropped to enlarge it |
| S05–S08 final evidence | `RUN` — corrected; this cell read `NOT_YET_RUN`, which contradicted `results/synthetic/final/`. Seven FINAL directories, all PASS; S06 has two (canonical `..._20260829-165043`). Manual capture landed for S06 canonical, S07 and S08; S05 GUI remains `NOT_CAPTURED` |
| `FAILED` / `MANUAL_REQUIRED` coverage | `NOT_YET_COVERED` — a coverage gap, not a spec deviation; spec in `05_test_protocol.md` section 11, not scheduled |
| commit | nothing is committed; `GIT_COMMITTED: NO`, `GIT_PUSHED: NO` |
| `FULL_COLCON_TEST` | `NOT_RUN` this round |

Closed this round, each by target read-back: `DECISION_002` and `DECISION_005` landed,
`S05.yaml`–`S08.yaml` present and installed, and `visualization_markers_node` has an
installed executable. Audit trail in `07_open_issues.md` section 1.

## 6. Status tokens carried forward

```text
S05-S08 PRECHECK:             PASS x4, RUN_CLASS=PRECHECK, NOT_FINAL_EVIDENCE=TRUE
S05-S08 FINAL EVIDENCE:       RUN — 7 FINAL runs, all PASS, RUN_CLASS=FINAL
                              (CORRECTED: was NOT_YET_RUN)
                              S05 canonical ..._20260829-160001
                              S06 canonical ..._20260829-165043 (2 FINAL runs)
                              S07 ..._20260829-171148
                              S08 ..._20260829-171722
TARGETED_FUNCTIONAL_TESTS:    ylhb_base 253 passed (reproduced with ROS sourced)
SYNTHETIC_PACKAGE_TESTS:      dg_synthetic_validation 33 passed
FULL_COLCON_TEST:             NOT_RUN this round; previously 666 / 0 errors /
                              335 failures / 10 skipped, NOT PASS
Gazebo / real robot:          NOT_RUN
XY / Z error, repeatability, relocalization success: NOT_PROVEN
Z/elevation:                  OPEN_TECHNICAL_GAP
CRITERIA_PRE_REGISTERED:      NO — post-hoc criteria evolution risk is disclosed
                              in 06_results_and_evidence.md. Verified there: the
                              three criteria commits did NOT by themselves turn
                              S01 green; S01 still FAILED at e1fdbc7 and at
                              c74e4f5, and reached green at fd2c60d, a rig
                              start-up change that removed the transient at
                              source. One criteria commit (1a3fb4b) moved S01
                              from PASS to FAIL.
NAVIGATION_RESUMED_EVIDENCED: NO
SAFE_CONTROL_HANDOFF_EVIDENCED: YES, as a DERIVED_ASSERTION
GROUND_TRUTH_LEAK=NO:         a hardcoded string literal corroborated by
                              independent code audit, never a measurement
CHECK_TOTALS:                 11 counted for S01; 14 counted + 1
                              non-discriminating for S06-class. The previously
                              published 12/12 and 15/15 each included one check
                              that could not fail.
```

The three test categories are never merged. The 335 failures are pre-existing
flake8/pep257 style findings and are not core DG behavioural failures. The targeted
passes are not a whole-repository pass.

## 7. Rules that survive this round

- Core thresholds, state machine, and algorithm semantics are frozen; `ylhb_base` is
  git-clean and unmodified
- No synthetic `/cmd_vel_nav`, no synthetic `/dg/*` output, no perfect `map -> odom`,
  no candidate or `RECOVERED` injection, no scripted yaw shortcut
- Recovery output reaches `/dg/test_cmd_vel` only; `/cmd_vel` has `0` publishers at
  scenario start and end, observed in all four precheck runs
- After `RECOVERED`, `NAV_SOURCE_STALE` is expected and acceptable, and is described
  only as safe control handoff in synthetic software validation:
  `SAFE_CONTROL_HANDOFF_EVIDENCED = YES`, `NAVIGATION_RESUMED_EVIDENCED = NO`,
  and the handoff is a `DERIVED_ASSERTION` because `cmd_vel_source` and
  `safety_cmd_vel_publishers` were empty in every FINAL sample
- The matcher accepted exactly one candidate (`match_message_count = 1`). Multi-cycle
  verification is stability confirmation of that single match, never repeated
  independent matching
- S05–S08 evidence the software state chain, not localization accuracy: the true pose
  is authored by the test rig and the surrogate reads synthetic ground truth
- A scan-to-map `mean_distance` is never a positioning error
- A deterministic figure repeated across two runs is one measurement, not two
- An assertion resting on an INPUT column must separate
  `INPUT_EVIDENCE_INCOMPLETE` from a behavioural failure
- Failures are never deleted from the record
- A uniform or black frame is a failure, never a screenshot; an empty figure is never
  filed; an empty idle view is never fixed by fabricating a publisher
