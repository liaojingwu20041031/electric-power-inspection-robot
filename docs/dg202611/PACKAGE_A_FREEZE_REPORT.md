# DG-202611 Package A — Freeze Report

```
EVIDENCE_CLASS              SYNTHETIC_SOFTWARE_VALIDATION
REAL_ROBOT_DATA             FALSE
COMPETITION_METRIC_EVIDENCE FALSE
GAZEBO_EVIDENCE             FALSE
RERUN_PERFORMED             NO
```

Independent audit verdict entering this round:
`PASS_WITH_REQUIRED_BOOKKEEPING_FIXES`, `RERUN_REQUIRED = NO`.

This round changed **no** algorithm, threshold, state machine or scenario
physics. It repaired provenance, bookkeeping and claim wording, and established
the first real Git anchor for the code that produced the evidence.

## 1. Git provenance — what can and cannot be proven

The runs were executed from a `--symlink-install` workspace, so the running code
was the working tree, and the working tree was uncommitted. That has a
consequence which must not be glossed over:

> **The exact source state that produced each run cannot be proven from Git
> history, because it was never committed at the time.** It can only be
> reconstructed from file mtimes and the `git_status` field each run recorded in
> its own `metadata.json`.

Provenance fields, to be read together:

```
historical_recorded_build_commit       as recorded per run (57ec8ec for most,
                                       fd2c60d for the canonical S01-S04 runs)
runtime_worktree_commit_at_execution   RECONSTRUCTED — no commit existed
freeze_anchor_commit                   see section 2
runtime_provenance_status              RECONSTRUCTED_FROM_WORKTREE_METADATA_AND_MTIME
```

The `build_commit` value inside each `result.json` records the HEAD at execution
time. For most runs that is `57ec8ec`, which is a **docs-only** commit and
therefore does **not** identify the running code. Historical `metadata.json`
files were **not** rewritten — falsifying them to point at a commit that did not
exist would destroy the audit trail. The wording that treated `build_commit` as
"the version of the executing code" has been corrected in the process docs
instead.

## 2. Freeze anchor

```
freeze_anchor_commit   see FREEZE_ANCHOR line at the end of this file
branch                 dg202611-synthetic-validation
parent                 57ec8ec
```

Committed: 10 modified + 22 new files = 32 items, all under
`src/dg_synthetic_validation/` and `docs/dg202611/`. Run artefacts were not
committed and could not have been — `results/` lives outside the repository.

**Disclosed inclusion:** `docs/dg202611/GAZEBO_SIMULATION_READINESS_REPORT.md` is
in this commit. It is a read-only audit *of the Package A repository*, written
before any Gazebo development, and its substantive findings are Package A
findings (the `base_controller` kinematics config risk, and the absence of
mass/COM/inertia in any URDF). It is not Gazebo P0 development output; no URDF,
world, launch file or simulation result is included. If MASTER prefers it on the
Gazebo line, moving it is a one-file change.

## 3. Canonical runs

Ledger: `PACKAGE_A_RUN_INVENTORY.md` — 41 directories, 40 `result.json`,
31 PASS, 9 FAIL. One directory
(`S02_gnss_gradual_degradation_and_outage_20260828-145410`) is INCOMPLETE with no
`result.json`; it is retained, not deleted.

| scenario | canonical run | note |
|---|---|---|
| S01 | `S01_nominal_20260828-145833` | build `fd2c60d`, 9 checks, 79 samples |
| S02 | `S02_..._20260828-145858` | |
| S03 | `S03_..._20260828-145928` | |
| S04 | `S04_..._20260828-145956` | |
| S05 | `S05_..._20260829-160001` | two earlier FINALs are DUPLICATE_VALID_RERUN |
| S06 | `S06_..._20260829-165043` | 399 samples; `-161050` is SUPERSEDED_FINAL, also PASS |
| S07 | `S07_..._20260829-171148` | |
| S08 | `S08_..._20260829-171722` | |

The `S06 FINAL EVIDENCE: NOT_YET_RUN` claim was stale and is corrected. It
contradicted both the disk and the same file's own appendix, and it applied to all
four of S05-S08, not only S06. Separately, `S06_RERUN = NO` forbade *further*
reruns for screenshot purposes; it never meant that no FINAL run existed. Both
statements now read consistently.

## 4. Corrections made to previously published claims

**Check totals were inflated.** `relocalization_not_unexpectedly_active` was
hardcoded `True` for every scenario except S01 yet still counted, so the published
`12/12`, `15/15`, `14/14` and `13/13` each contained one check that could not
fail. The code now counts it for S01 only and reports it elsewhere under
`non_discriminating_checks`. Recomputed from real samples: S05 11+1, S06 14+1,
S07 13+1, S08 12+1, and **S01 has 0 non-discriminating** because there the check
is genuine. No verdict changes.

**Multi-frame verification was overstated.** `match_message_count = 1` — the real
Scan-to-Map matcher produced exactly **one** accepted candidate. The repeated
identical match values are a latch of that single match. Any wording implying
three consecutive independent matcher successes is forbidden. Correct form: after
a single accepted candidate match, the system confirmed candidate-pose stability
across multiple consecutive verification cycles, with `verification_count`
advancing 0 to 3.

**Ground-truth surrogate boundary.** S05-S08 candidate poses originate from an
AMCL surrogate that reads synthetic ground truth. The supervisor under test has
no ground-truth leak, but `GROUND_TRUTH_LEAK=NO` as it appears in the evidence is
a hardcoded string literal, so it stands on independent code audit and not on
measurement. Package A evidences supervisor logic, trigger, candidate handling,
verification flow, state transitions and safe handoff. It does **not** evidence
real scan-matcher performance, real relocalization accuracy, or competition
success rate.

**Safe handoff.** `SAFE_CONTROL_HANDOFF_EVIDENCED` is supported by the state
transitions. `NAVIGATION_RESUMED_EVIDENCED = NO`. `cmd_vel_source` and
`safety_cmd_vel_publishers` were empty in the samples, so the handoff conclusion
rests on a `DERIVED_ASSERTION`, not a published source field.

**Criteria were not pre-registered.** Post-hoc criteria evolution risk exists and
is disclosed in `PACKAGE_A_FAILURE_HISTORY.md`. The honest detail matters in both
directions: three commits (`618122f`, `1a3fb4b`, `e1fdbc7`) changed the evaluation
layer, and S01 actually **regressed to FAIL at `1a3fb4b`**, a criteria commit.
S01 first passed at `fd2c60d`, and the two commits immediately preceding it
(`c74e4f5`, `fd2c60d`) touch only `scenario_runner.py` — test-rig start-up, not
criteria. So the loosened criteria were real and consequential but **not
sufficient**; the last step to green removed a start-up transient at its source.
Neither "criteria were pre-registered" nor "criteria were loosened until it
passed" is an accurate description.

## 5. QoS issue closed as a misdiagnosis

```
INITIALPOSE_QOS_FINAL_VERDICT   COMPATIBLE
OLD_ISSUE_STATUS                CLOSED_AS_MISDIAGNOSIS
```

Verified in `scan_map_relocalization_node.py`: `initial_pose_qos_profile()`
returns RELIABLE + TRANSIENT_LOCAL and is used on the **publisher**; the
`/initialpose` **subscription** uses the default `10`, i.e. RELIABLE + VOLATILE.
A TRANSIENT_LOCAL publisher against a VOLATILE subscriber is compatible — the
incompatible pairing is the reverse. The earlier diagnosis had the direction
backwards and is retained as a superseded record rather than deleted.

Two further corrections: the warning text previously quoted was itself wrong (the
real line is `New publisher discovered ... offering incompatible QoS`), and the
process that emitted it was **`rosbag2_recorder`** — a recording-side
subscription, not a DG node and not the injector. It is therefore not evidence of
loss on any supervisor data path. The specific incompatible publisher is recorded
`UNDETERMINED`; a hypothesis about the injector's unconditional VOLATILE
`/initialpose` publisher is recorded as unproven, not as a conclusion.

The real defect in the S05 context was missing TF / `base_footprint`, not QoS.

## 6. Open issues carried forward, not closed by P0 passing

- `REAL_BASE_KINEMATICS_CONFIG_MISMATCH` — severity HIGH,
  `REAL_ROBOT_CONFIGURATION_RISK`. `base_controller` never receives
  `base_kinematics.yaml` (the YAML keys on `zlac8015d_canopen_controller`), so it
  runs on the C++ default `wheel_track = 0.25` against the measured `0.4008`, and
  declares no `wheel_radius` at all. Not a failure mode — the current default
  path. Which backend the physical vehicle runs was not determined. Needs a
  separate real-vehicle audit task; must not be fixed opportunistically from a
  simulation or synthetic line.
- `FAILED` / `MANUAL_REQUIRED` branches carry no synthetic evidence. All four of
  S05-S08 take the success path, so the recovery machine's timeout handling,
  retry logic and exhaustion path are unvalidated.
- `S05_GUI_CAPTURE: NOT_CAPTURED` — a capture omission, **not** `NOT_OBSERVED`.
  The states occurred and are in `samples.csv` and `timeline.csv`.
- S06 transient states `SUSPECTED`, `TRIGGERED`, `STOPPING`,
  `WAITING_CANDIDATE`, `VERIFYING` were not photographed; each lasted less than
  the 0.4 s capture interval. Same distinction applies.
- `/dg/fusion/pose` has no RViz visualization path; fusion state reaches CSV,
  plots and Monitor but not the RViz frame.
- Marker staleness incident: a frozen post-run display was initially mistaken for
  live state. Fixed with staleness ageing; the incident is recorded.
- GNSS antenna offset remains a `0 0 0` placeholder.

## 7. Verification performed this round

No scenario was re-run. Checks executed: `git status`, full 32-item scope review,
synthetic package test suite (33 passed), recomputation of check totals against
canonical `samples.csv`, canonical path existence, ledger reconciliation to
41/40/31/9, and greps for `NOT_YET_RUN`, `S06_RERUN`, `navigation resumed`,
misleading multi-frame matching wording and the old QoS direction.

```
PACKAGE_A_RUNTIME_COMPLETE   YES
PACKAGE_A_FREEZE_READY       YES
COMPETITION_METRIC_EVIDENCE  NO
```


---

FREEZE_ANCHOR: 02af370d99ece040605ec1bee585866e5760f9c6
BRANCH:        dg202611-synthetic-validation
PARENT:        57ec8ecff90d4e383e8c5197105bc8d182474245
FILES:         35 (10 modified, 25 added)
TREE_STATE:    clean after commit
