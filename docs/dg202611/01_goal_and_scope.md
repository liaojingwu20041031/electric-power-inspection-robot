# DG-202611 Round B — 01 Goal and Scope

## 1. Purpose of this document set

`docs/dg202611/01_*.md` through `08_*.md` are the numbered development-process
record for **Round B** of the DG-202611 line. They extend, and do not replace,
`CLAUDE_CODE_HANDOFF.md` and `CLAUDE_CODE_FIRST_CONNECT_CHECKLIST.md`.
Where the handoff and these records disagree, the handoff describes the state at
takeover and these records describe the Round B findings on top of it.

Companion non-numbered records in the same round:

- `MAP_ODOM_TF_JUSTIFICATION.md` — the `map -> odom` transform decision record
- `EXPERIMENT_EVIDENCE_POLICY.md` — the permanent forward-looking evidence rule
- `USER_VISIBLE_WORKFLOW.md` — how a human actually watches a run
- `MANUAL_CAPTURE_CHECKLIST.md` — the manual capture path on this specific host
- `GAZEBO_SIMULATION_READINESS_REPORT.md` — read-only audit,
  `GAZEBO_SIMULATION_READINESS = BLOCKED`

## 2. Round B goal

Make the Active Relocalization closed loop **structurally reachable** under
synthetic software validation, so that S05–S08 can be designed, prechecked, and later
run for final evidence, **without modifying any core algorithm, threshold, or state
machine**.

Round B is an investigation, design, implementation-support, and record round. It is
explicitly not a final-evidence round.

```text
S05-S08 PRECHECK:        RUN, all four PASS
S05-S08 FINAL EVIDENCE:  RUN  (CORRECTED — this line read NOT_YET_RUN, which
                         contradicted disk. Seven FINAL directories exist under
                         results/synthetic/final/, all PASS. S06 in particular
                         has TWO FINAL runs, canonical ..._20260829-165043.
                         See 06_results_and_evidence.md section 5.)
```

Round B's own charter was "not a final-evidence round", and that remains an
accurate description of Round B's *intent*. It is not a statement about what
exists on disk now, and the two must not be conflated.

See `06_results_and_evidence.md` for both, and for the distinction between them.

The driving finding of Round B is that the historical S02–S04 warning
`RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` cannot be read as closed-loop
success, because the degradation modelled by S02–S04 is incapable of triggering
active relocalization by construction. See `02_architecture_and_design.md`
section 3 and `04_debug_and_failures.md`.

## 3. Evidence classification for everything in Round B

```text
EVIDENCE_CLASS: SYNTHETIC_SOFTWARE_VALIDATION
```

Round B inherits the handoff labels and adds nothing above them:

- `ENGINEERING_FOUNDATION`
- `SOFTWARE_POC`
- `ROS2_RUNTIME_VALIDATED`
- `SYNTHETIC_VALIDATED` for S01–S04 only
- `NOT_YET_GAZEBO_VALIDATED`
- `NOT_YET_TARGET_HARDWARE_VALIDATED`
- `NOT_YET_REAL_ROBOT_VALIDATED`
- `NOT_YET_COMPETITION_METRIC_VALIDATED`

Status tokens used throughout these records:

| Token | Meaning |
|---|---|
| `NOT_YET_RUN` | The activity is defined but has not been executed |
| `NOT_YET_VERIFIED` | The value or behaviour is unknown; it is never guessed |
| `PRECHECK` | A run that exercises the chain but is not evidence |
| `NOT_FINAL_EVIDENCE` | Explicitly excluded from any document claim |

### 3.1 Run-class vocabulary

```text
RUN_CLASS:              PRECHECK
NOT_FINAL_EVIDENCE:     TRUE
NOT_FOR_DOCUMENT_CLAIM: TRUE
EVIDENCE_LEVEL:         SYNTHETIC SOFTWARE VALIDATION PRECHECK
```

The four S05–S08 runs of this round carry exactly those four tokens, written inside
`metadata.json`, inside `result.json`, and prominently at the top of `result.md`.

```text
RULE: a directory name is not a label. Run class lives in the artefacts, because a
figure or a number copied out of a directory leaves the directory name behind.
```

A precheck says the chain runs and the criteria evaluate against real recorded
columns. It does not say a run is evidence: it was headless, it did not pass the
`VISIBLE_EVIDENCE_CHECKPOINT`, and it has no manual visible evidence.

### 3.2 Reporting-status vocabulary

These five terms are distinct and are never used interchangeably. They exist because
files were twice reported as "pushed / verified" while they existed only in a local
staging directory — see `04_debug_and_failures.md` section 9.

| Term | Meaning |
|---|---|
| `LOCAL_STAGING_ONLY` | The edit exists locally only. The target has never seen it |
| `WRITTEN_TO_TARGET_WORKTREE` | The bytes were written to the target path |
| `TARGET_WORKTREE_VERIFIED` | The target path was read back and confirmed |
| `GIT_COMMITTED` | Committed in the target repository |
| `GIT_PUSHED` | Pushed to a remote |

```text
REPEATED_FAILURE_CLASS: STAGING_TARGET_DIVERGENCE
OCCURRENCE_COUNT:       2
```

No file may be called `TARGET_WORKTREE_VERIFIED` without all of: a target read-back, a
structural `grep` for the symbol that was supposed to appear, `py_compile` on the
target copy, and — where a data path is involved — a runtime sanity check that the
value actually lands in the artefact.

```text
NOT VERIFICATION: a file-transfer command that exited 0.
NOT VERIFICATION: a subagent summary stating the work was done.
```

```text
GIT_COMMITTED: NO
GIT_PUSHED:    NO
NO_FINAL_EVIDENCE_SCENARIO_EXECUTED: YES
```

Every statement about the target in these records is a **timestamped observation**,
valid as of the read that produced it, not a standing fact.

### 3.3 Evidence-validity rules

```text
FILE_EXISTS        != VALID_EVIDENCE
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

Three separate defects in this round produced an artefact that existed and carried no
measurement: a blank plot filed into the manifest, a black frame that passed a
channel-spread check, and RViz displays that rendered nothing while appearing
configured. Detail in `04_debug_and_failures.md` sections 11, 13 and 14.

One more rule of the same family:

```text
RULE: an assertion that depends on an INPUT column must distinguish
INPUT_EVIDENCE_INCOMPLETE from a genuine behavioural failure, because those two
conclusions point at opposite root causes.
```

## 4. Forbidden claims

None of the following may appear anywhere in the Round B record, in a result
file, in a commit message, or in a report derived from them:

- XY error `<20 cm`, Z error `<20 cm`
- feature repeatability `>=95%`
- relocalization success `>95%`
- "real robot", "Gazebo", "competition performance"
- "navigation resumed", "Nav2 resumed navigation", "robot continued mission"
- any reading of a scan-to-map `mean_distance` as a positioning error
- "three consecutive independent matcher successes", or any phrasing implying
  repeated independent scan-to-map matching. The matcher accepted exactly **one**
  candidate (`match_message_count = 1`); the repeated verification values are a
  latch of that single match. See `06_results_and_evidence.md` section 2.4
- any presentation of `12/12` or `15/15` check totals as if every check were
  discriminating. One check in each could not fail; corrected counts are 11 for
  S01 and 14 counted + 1 non-discriminating for S06-class
- any presentation of `GROUND_TRUTH_LEAK=NO` as a measurement. It is a hardcoded
  string literal corroborated by independent code audit

```text
NAVIGATION_RESUMED_EVIDENCED:    NO
SAFE_CONTROL_HANDOFF_EVIDENCED:  YES, as a DERIVED_ASSERTION
```

Round B produces software-state evidence at `PRECHECK` level. It produces no accuracy,
reliability, or performance evidence of any kind.

The forbidden-claim scanner in `evidence_manifest.py` enforces this on captured
evidence. It had two defects of its own this round, both fixed: `\b`-anchored CJK
patterns that never matched, and a rule that flagged its own mandatory disclaimers.
See `04_debug_and_failures.md` section 12.

## 5. In scope

- Read and record the real Active Relocalization state machine as implemented
- Record the trigger/degraded reason split and its consequence for S02–S04
- Record the Navigation Health and cmd_vel Arbiter chain that gates recovery
- Record the four concrete blockers that made the old synthetic input unusable
- Record the TF investigation and the `map -> odom` decision
- Record the synthetic kinematic plant and the synthetic AMCL surrogate designs
- Record the offline verification of the real scan matcher
- Define the S05–S08 test protocol and record the S05–S08 precheck outcomes
- Record the evidence-pipeline defects found and fixed this round
- Record the GUI capture reality on this host
- Write the permanent evidence-retention policy

## 6. Out of scope for Round B

- S05–S08 **final evidence** runs — out of scope for *Round B's* charter. They
  were executed later and do exist; this line previously read `NOT_YET_RUN`,
  which was a status claim rather than a scope statement. Corrected: see
  `06_results_and_evidence.md` section 5
- `colcon build`; `FULL_COLCON_TEST` (`NOT_RUN` this round)
- Software installation, git commit, git push
- Gazebo, Jetson target integration, ZED/VIO, mmWave, optical flow
- Any Z/elevation work; Z remains an `OPEN_TECHNICAL_GAP`
- Any localization-accuracy statement

## 7. Frozen constraints

```text
CORE_THRESHOLD_CHANGE:            NOT ALLOWED
CORE_STATE_MACHINE_CHANGE:        NOT ALLOWED
CORE_ALGORITHM_SEMANTIC_CHANGE:   NOT ALLOWED
SYNTHETIC_CMD_VEL_NAV:            NOT ALLOWED
SYNTHETIC_DG_OUTPUT:              NOT ALLOWED
PERFECT_MAP_ODOM:                 NOT ALLOWED
CANDIDATE_OR_RECOVERED_INJECTION: NOT ALLOWED
SCRIPTED_YAW_SHORTCUT:            NOT ALLOWED
```

All verified as holding this round:

| Constraint | Verified state |
|---|---|
| `ylhb_base` | git-clean and UNMODIFIED (`git status --porcelain src/ylhb_base` returns 0 lines) |
| core state machine | unchanged |
| core thresholds | unchanged |
| `map -> odom` | absent from `_static_transforms`; `PERFECT_MAP_ODOM_PUBLISHED: NO` in every precheck artefact |
| `/cmd_vel_nav` | no synthetic publisher |
| `/dg/*` | no synthetic output |
| candidate / `RECOVERED` | no direct injection |
| recovery yaw | no scripted shortcut; the plant turns because it is commanded |

Packaging, verified by a runtime query rather than by reading `setup.py`:

```text
ros2 pkg executables dg_synthetic_validation  ->  8 executables
config/S01.yaml .. config/S08.yaml            ->  all 8 install to share
```

Safety contract, unchanged from the handoff and re-affirmed for Round B:

```text
TEST_OUTPUT:     /dg/test_cmd_vel
REAL_ROBOT_CMD:  /cmd_vel
```

Recovery output only ever reaches `/dg/test_cmd_vel`. The real `/cmd_vel` must
have **0 publishers at scenario start and 0 publishers at scenario end**. A
non-zero count is a safety failure and stops the run. All four precheck runs recorded
`0` at start and `0` at end.

## 8. Accepted end-of-loop behaviour

After `RECOVERED` the arbiter selects `NAV` and then emits zero with
`NAV_SOURCE_STALE`, because Nav2 is intentionally disabled and nothing publishes
`/cmd_vel_nav`.

```text
STATUS: EXPECTED_AND_ACCEPTABLE
PERMITTED_WORDING: "safe control handoff verified in synthetic software validation"
SAFE_CONTROL_HANDOFF_EVIDENCED: YES  — the state transitions support it
NAVIGATION_RESUMED_EVIDENCED:   NO
HANDOFF_EVIDENCE_STRENGTH:      DERIVED_ASSERTION
```

It must never be written as navigation resuming or a mission continuing.
See `DECISION_007` in `02_architecture_and_design.md` section 9.

The `DERIVED_ASSERTION` qualifier is required: `cmd_vel_source` and
`safety_cmd_vel_publishers` were empty in every sample of the S06 canonical, S07
and S08 FINAL runs, so no directly published source field witnesses which
controller held the output. The conclusion is derived from state transitions plus
the `cmd_vel_source_inferred` column. Detail in `07_open_issues.md` section 9.1.

Observed in the S08 precheck: post-`RECOVERED` max `|cmd_angular_z|` is `0.000000`,
i.e. recovery released control.

## 9. Environment context

```text
VM rebooted between sessions:  YES (uptime 11 minutes at 2026-08-29 11:37,
                               versus a prior session on 2026-08-28)
Disk state:                    fully persisted across the reboot
Long-running processes lost:   NONE — none had been started
```

The Xwayland `XAUTHORITY` file is `/run/user/1000/.mutter-Xwaylandauth.*` with a
**random suffix that changes across reboots**. It must be globbed, never hard-coded.
This was observed directly: the suffix recorded during the earlier capture probe is
not the suffix present after the reboot.
