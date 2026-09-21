# DG-202611 Round B — 06 Results and Evidence

```text
ROUND_B_PRECHECK_RESULTS:             S05, S06, S07, S08 — all four PASS
ROUND_B_FINAL_EVIDENCE:               RUN — all four scenarios, 7 FINAL runs
NO_FINAL_EVIDENCE_SCENARIO_EXECUTED:  NO
PACKAGE_A_RUNTIME_EXECUTION_COMPLETE: YES
EVIDENCE_CLASS:                       SYNTHETIC_SOFTWARE_VALIDATION
GIT_COMMITTED:                        NO
GIT_PUSHED:                           NO
```

### Corrected stale header — why the two lines above changed

```text
SUPERSEDED HEADER LINES, kept so the correction stays auditable:
  ROUND_B_FINAL_EVIDENCE:               NOT_YET_RUN
  NO_FINAL_EVIDENCE_SCENARIO_EXECUTED:  YES
```

This header was written **before the final batch ran and was never updated**.
That is the whole defect: it is a stale header, not a disagreement about what
happened. The staleness made this file contradict two things at once —
`results/synthetic/final/`, where all seven directories carry
`run_class = FINAL` and `not_final_evidence = false`; and **its own later
appendix**, which already documented three of those runs while the header still
denied any existed.

It is recorded as a corrected stale header rather than silently rewritten,
because a header that went stale through an un-updated edit is a process fact
worth keeping visible.

Canonical FINAL runs to cite, all `result=PASS`, `run_class=FINAL`, `errors=[]`:

```text
S05  S05_amcl_covariance_ramp_localization_trigger_20260829-160001   CANONICAL
     ..._20260829-141521 and ..._20260829-150349  DUPLICATE_VALID_RERUN
S06  S06_closed_loop_active_scan_recovery_motion_20260829-165043     CANONICAL, 399 samples
     ..._20260829-161050  SUPERSEDED_FINAL, 392 samples, also PASS
S07  S07_real_candidate_from_real_seed_request_20260829-171148
S08  S08_multiframe_verification_and_control_handoff_20260829-171722
```

Sections 1 through 4 below describe the four **precheck** runs, and everything
said about the precheck class remains true: a precheck is not citable in a
proposal, report, or presentation. The FINAL runs are a different run class and
that restriction does not transfer to them.

## 1. What a `PRECHECK` result is, and is not

```text
RUN_CLASS:              PRECHECK
NOT_FINAL_EVIDENCE:     TRUE
NOT_FOR_DOCUMENT_CLAIM: TRUE
EVIDENCE_LEVEL:         SYNTHETIC SOFTWARE VALIDATION PRECHECK
```

A precheck answers one question: does the chain run end to end and do the criteria
evaluate against real recorded columns. It is not final evidence, because it was run
headless, without the `VISIBLE_EVIDENCE_CHECKPOINT` in `USER_VISIBLE_WORKFLOW.md`, and
without the manual capture step that gives a run its visible evidence.

The four tokens above are written **inside** `metadata.json`, **inside** `result.json`,
and prominently at the top of `result.md`. They are deliberately not left to the
directory name:

```text
RULE: a directory name is not a label. It is lost the moment a figure, a number,
or a table is copied out of the directory.
```

## 2. S05–S08 precheck results

Run root: `/home/weiyang/dg202611_ws/results/synthetic/precheck/`

| Scenario | Verdict | Samples | Run directory |
|---|---|---|---|
| S05 | `PASS` | 239 | `S05_amcl_covariance_ramp_localization_trigger_20260829-120010` |
| S06 | `PASS` | 387 | `S06_closed_loop_active_scan_recovery_motion_20260829-120206` |
| S07 | `PASS` | 159 | `S07_real_candidate_from_real_seed_request_20260829-120415` |
| S08 | `PASS` | 239 | `S08_multiframe_verification_and_control_handoff_20260829-120513` |

Each directory holds `metadata.json`, `result.json`, `result.md`, `samples.csv`,
`timeline.csv`, `field_roles.csv`, `scenario.yaml`, `plots/`, `rosbag/`, `logs/`.

```text
ALL FOUR: RUN_CLASS=PRECHECK, NOT_FINAL_EVIDENCE=TRUE, NOT_FOR_DOCUMENT_CLAIM=TRUE
```

### 2.1 S05 — trigger and escalation

```text
CRITERIA: s05_sut_reported_amcl_health_degraded          True
          s05_normal_suspected_triggered_in_order        True
          s05_trigger_reason_is_a_real_trigger_path      True
          s05_navigation_health_escalated                True
```

Observed:

- `reloc_amcl_health`: `GOOD -> BAD`
- relocalization: `NORMAL -> SUSPECTED -> TRIGGERED` in order, terminating in `STOPPING`
- `reloc_trigger_reason`: `AMCL_COVARIANCE_HIGH`, a `trigger_reasons` member
- `navigation_state`: `NOMINAL -> DEGRADED -> NOMINAL -> LOCALIZATION_SUSPECT -> RECOVERING`

S05 terminates in `STOPPING`, which is consistent with its charter and with the
recorded input: S05 runs with the plant disabled, and `odom_linear_x` is a constant
`0.05` across all 239 samples (verified in `samples.csv`, and declared in every phase
of `config/S05.yaml`). `0.05 > stop_velocity = 0.03`, so the stop is never confirmed.
This is blocker 2's mechanism, retained deliberately in a scenario whose charter ends
at trigger and escalation.

```text
tf_base_to_sensor_available: EMPTY for S05, and that is CORRECT.
```

S05 runs with the plant disabled, so no TF buffer exists and there is nothing to
report. This is an **honest absence**, not a gap in the evidence. Recording it as
empty rather than `False` matters: `False` would assert that a lookup was attempted
and failed.

### 2.2 S06 — closed-loop recovery motion

```text
CRITERIA: s06_stopping_confirmed_then_active_scan   True
          s06_navigation_health_recovering          True
          s06_supervisor_commanded_rotation         True
          s06_arbiter_forwarded_recovery            True
          s06_test_cmd_vel_carried_rotation         True
          s06_recovery_is_pure_rotation             True
          s06_plant_yaw_responded_to_command        True
```

Observed:

- `odom_yaw` span **9.28 degrees**. It stops inside the real `1.5` degree tolerance of
  the `10` degree segment target, so the sweep converged rather than overshooting
- **9** samples of non-zero supervisor angular command, matched **1:1** on
  `/cmd_vel_recovery` and `/dg/test_cmd_vel`
- `cmd_vel_source_inferred`: `ZERO -> RECOVERY -> ZERO`
- max `|cmd_linear_x|` = **0.000000**, matching `command_linear_x` hard-coded to `0.0`
  in the core — recovery is pure rotation
- `tf_base_to_sensor_available`: `True`
- `reloc_active_segment` reached `1`

The 1:1 match is the part that carries weight: it shows the command observed on the
test topic is the command the supervisor issued, not a value the rig produced.

### 2.3 S07 — a real candidate from a real seed request

```text
CRITERIA: s07_seed_requested_by_real_supervisor        True
          s07_real_matcher_published_quality           True
          s07_candidate_accepted_by_real_matcher       True
          s07_candidate_meets_unmodified_thresholds    True
          s07_candidate_used_real_scan_points          True
          s07_candidate_followed_the_seed_request      True
```

Ordering, which is the whole point of the scenario:

```text
first seed request:  6.510 s
first accepted match: 6.610 s
```

The candidate demonstrably **followed** a real supervisor seed request. A candidate
that appeared before the request would be a candidate the rig produced.

Accepted match, as computed by the real matcher:

```text
score:         0.9795765221706485
inlier_ratio:  1.0
mean_distance: 0.007222222329841719
used_points:   180
reason:        ACCEPTED
```

### 2.4 S08 — multi-cycle / multi-frame stability verification and control handoff

```text
CRITERIA: s08_waiting_verifying_recovered_in_order      True
          s08_multi_frame_verification_completed        True
          s08_recovered_observed                        True
          s08_navigation_health_reported_recovered      True
          s08_safe_control_handoff_observed             True
```

Observed:

- `reloc_verification_count` reached **3**, the real `verification_samples` default
- `WAITING_CANDIDATE -> VERIFYING -> RECOVERED` in order
- `navigation_state` reached `RECOVERED`
- post-`RECOVERED` max `|cmd_angular_z|` = **0.000000**, so recovery released control

#### What the verification count actually shows — required wording

```text
MATCHER_ACCEPTED_CANDIDATES:  match_message_count = 1   (exactly ONE)
VERIFICATION_CYCLES:          reloc_verification_count  0 -> 1 -> 2 -> 3
REPEATED_INDEPENDENT_MATCHING: NO
```

Verified in both the S07 and S08 precheck and FINAL `samples.csv`:
`reloc_verification_count` does progress `0 -> 1 -> 2 -> 3`, while
`match_message_count` never exceeds **1**. The real Scan-to-Map matcher produced
exactly one accepted candidate. The repeated identical match values across the
verification window are a **latch of that single match**, not repeated
independent scan-to-map matching.

The permitted description, to be used in this shape:

> After a single accepted candidate match, the system confirmed candidate-pose
> stability across multiple consecutive verification cycles, with
> `verification_count` advancing from 0 to 3.

```text
FORBIDDEN: "three consecutive independent matcher successes"
FORBIDDEN: any phrasing implying repeated independent scan-to-map matching
REQUIRED:  where "multi-frame" is kept it must read as
           "multi-cycle / multi-frame stability verification"
```

#### The handoff claim and its evidence strength

```text
SAFE_CONTROL_HANDOFF_EVIDENCED:  YES  (state transitions support it)
NAVIGATION_RESUMED_EVIDENCED:    NO
HANDOFF_EVIDENCE_STRENGTH:       DERIVED_ASSERTION
```

The zero after `RECOVERED` is reportable only as safe control handoff in synthetic
software validation. It is never navigation resuming, Nav2 resuming, or a mission
continuing (`DECISION_007`).

The strength qualifier is required because `cmd_vel_source` and
`safety_cmd_vel_publishers` were **empty in every sample** of the S06 canonical,
S07 and S08 FINAL runs. No directly published source field witnesses which
controller held the output; the only populated column is
`cmd_vel_source_inferred`. The handoff conclusion is therefore a
`DERIVED_ASSERTION` from state transitions plus an inferred source, not a direct
measurement. Detail in `07_open_issues.md` section 9.1.

### 2.5 Safety, all four runs

```text
real /cmd_vel publishers at start: 0
real /cmd_vel publishers at end:   0
```

Recorded per run as `real_cmd_vel_unpublished_at_start` and
`real_cmd_vel_is_unpublished`, both `True` in all four.

## 3. Two cross-scenario observations that must travel with the numbers

### 3.1 S06 and S07 incidentally reached `RECOVERED`

Both ran the full sequence

```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
       -> WAITING_CANDIDATE -> VERIFYING -> RECOVERED
```

even though neither is chartered to test recovery. This is real behaviour of the
unmodified code, not a defect: `_handle_verifying` requires candidate quality plus
pose stability and does **not** require healthy covariance.

The consequence must be recorded with it, or the runs look inconsistent:

| Scenario | `converge_on_match` | Relocalization end | Navigation end |
|---|---|---|---|
| S06 | false | `RECOVERED` | `LOCALIZATION_SUSPECT` |
| S07 | false | `RECOVERED` | `LOCALIZATION_SUSPECT` |
| S08 | true | `RECOVERED` | `RECOVERED` |

With `converge_on_match` false the surrogate covariance stays high, and in
`navigation_health_core` the AMCL-`REJECTED` branch outranks `RECOVERED` in the
priority order. S08 is the only one whose navigation reaches `RECOVERED`.

Corroborated by the surrogate-mode column rather than asserted:

```text
S06  synthetic_amcl_surrogate_mode: BIASED_NO_CONVERGENCE_CONFIGURED
S07  synthetic_amcl_surrogate_mode: BIASED_NO_CONVERGENCE_CONFIGURED
S08  synthetic_amcl_surrogate_mode: BIASED -> CONVERGED
```

The S08 transition is caused by a real accepted `/scan_match_pose`, not by a timer.

### 3.2 S07 and S08 reported identical match quality

```text
S07 and S08: score 0.9795765221706485, inlier_ratio 1.0,
             mean_distance 0.007222222329841719
```

Identical, deterministically, because both runs use identical synthetic geometry and
identical seed construction.

```text
RULE: these are ONE measurement reported twice.
They are NOT two independent corroborating measurements.
```

Recorded explicitly so no later reader treats the repetition as replication.

## 4. What S07's `mean_distance` does and does not mean

```text
CLASSIFICATION: SYNTHETIC_MATCH_QUALITY_RESULT
```

`mean_distance = 0.00722` is a **scan-to-map residual against synthetic grid
geometry**. It is not a 7 mm positioning error, and it must never be used to support a
20 cm metric claim of any kind.

The deeper limitation, which no amount of passing changes:

```text
The true pose is AUTHORED BY THE TEST RIG.
GROUND_TRUTH_USED_BY_SUT:       NO
GROUND_TRUTH_USED_BY_SURROGATE: YES
```

So S05–S08 evidence the **software state chain**, not localization accuracy. Recorded
in every precheck `result.md` as
`GROUND_TRUTH_USED_FOR_ASSERTION: NO_ONLY_FOR_INPUT_AND_BIAS_CONSTRUCTION` and
`GROUND_TRUTH_LEAKAGE_TO_MATCHER: NO`.

### 4.1 Ground-truth surrogate boundary — required wherever an S05–S08 candidate pose is discussed

```text
CANDIDATE_POSE_ORIGIN:  an AMCL SURROGATE that READS synthetic ground truth
SUPERVISOR_UNDER_TEST:  no ground-truth leak
```

The candidate pose in S05–S08 is not the product of a real localizer. It
originates from the synthetic AMCL surrogate, and that surrogate reads the rig's
synthetic ground truth to build its seed and bias. Separately, the supervisor
under test does not read ground truth.

How the leak token must be presented:

```text
GROUND_TRUTH_LEAK=NO  IS   a hardcoded string literal in the writer
                           (result_writer.py:331 emits
                           "GROUND_TRUTH_LEAKAGE_TO_MATCHER": "NO")
GROUND_TRUTH_LEAK=NO  IS   corroborated by independent CODE AUDIT
GROUND_TRUTH_LEAK=NO  IS   NOT a measurement or a runtime check
```

A literal that is written unconditionally cannot fail, so it evidences nothing by
itself. The claim rests on the code audit; the artefact string only records the
audit's conclusion.

What Package A does evidence:

```text
supervisor logic    trigger    candidate handling
verification flow   state transitions    safe handoff
```

What Package A does **not** evidence:

```text
real scan-matcher performance
real relocalization accuracy
competition success rate
```

What S07 does legitimately show:

- the real Scan-to-Map node and the real `refine_pose_near_seed` executed
- the candidate was **not** injected
- real `score`, `inlier_ratio` and `mean_distance` were computed from real scan points
- the real, unmodified accept/reject thresholds executed: `0.45 / 0.45 / 0.30`

An offline check adds the discriminating detail: the **seed alone fails** those
thresholds (`score 0.244`, `inlier 0.378`) while the refined match passes. The
accepted candidate is therefore the matcher's search product, not a confirmation of
the seed it was handed. That is the difference between S07 and the tautology recorded
in `04_debug_and_failures.md` section 7.

## 5. Final evidence — CORRECTED, the FINAL runs exist

This section previously read `S05 FINAL EVIDENCE: NOT_YET_RUN` through
`S08 FINAL EVIDENCE: NOT_YET_RUN`, with a table of `NOT_YET_RUN` cells. That
contradicted disk. The superseded block is kept for audit:

```text
SUPERSEDED — DO NOT CITE
S05 FINAL EVIDENCE: NOT_YET_RUN
S06 FINAL EVIDENCE: NOT_YET_RUN
S07 FINAL EVIDENCE: NOT_YET_RUN
S08 FINAL EVIDENCE: NOT_YET_RUN
```

Corrected, read back from `results/synthetic/final/`. All seven directories
carry `result=PASS`, `run_class=FINAL`, `errors=[]`:

```text
S05 FINAL EVIDENCE: RUN — 3 runs, all PASS (canonical ..._20260829-160001)
S06 FINAL EVIDENCE: RUN — 2 runs, both PASS (canonical ..._20260829-165043)
S07 FINAL EVIDENCE: RUN — 1 run, PASS
S08 FINAL EVIDENCE: RUN — 1 run, PASS
```

| Scenario | Final-evidence directory | Samples | Visible evidence | Manifest |
|---|---|---|---|---|
| S05 | `S05_..._20260829-160001` (**CANONICAL**, 2 further valid reruns) | 239 | `NOT_CAPTURED` — a capture omission, not `NOT_OBSERVED` | none |
| S06 | `S06_..._20260829-165043` (**CANONICAL**) | 399 | 5 frames | `evidence/evidence_manifest.md` |
| S06 | `S06_..._20260829-161050` (`SUPERSEDED_FINAL`) | 392 | none | none |
| S07 | `S07_..._20260829-171148` | 159 | 6 frames | `evidence/evidence_manifest.md` |
| S08 | `S08_..._20260829-171722` | 239 | 6 frames | `evidence/evidence_manifest.md` |

Per-run inventory detail is owned by `PACKAGE_A_RUN_INVENTORY.md` and is not
duplicated here. The S05 accounting is in the appendix below; S06 has its own
appendix section.

A final-evidence run requires, in addition to a passing precheck: the
`VISIBLE_EVIDENCE_CHECKPOINT` passed, terminals A and B visibly updating on the VMware
desktop, manual screenshots captured and checked frame by frame, and a verified
evidence manifest. Automatic GUI capture is unavailable on this host
(`GUI_CAPTURE_READY: MANUAL_ONLY`), so the manual step cannot be skipped.

Still outstanding for a final-evidence run, though no longer the marker overlap: that
layout fix has landed and is `TARGET_WORKTREE_VERIFIED`, including in the installed
config. What remains is `RVIZ_DISPLAY_FIELD_ACCEPTANCE` (`NOT_YET_VERIFIED` — only
opening RViz settles whether every display's field set is honoured) and
`EVIDENCE_TEXT_LEGIBILITY_AT_1280x800` (`USER_JUDGEMENT_REQUIRED`). See
`07_open_issues.md` section 2 and `04_debug_and_failures.md` section 17.

## 6. Test status — three separate categories

These three categories must **never** be merged into a single pass/fail statement.

### 6.1 `TARGETED_FUNCTIONAL_TESTS`

```text
ylhb_base:                    253 passed
PREVIOUS_253_PASS_REPRODUCED: YES
```

Reproduced this round with `/opt/ros/humble/setup.bash` and the workspace install
sourced. The first attempt used bare `/usr/bin/python3` and failed at collection with
`ModuleNotFoundError: No module named 'rclpy'`; that was an invocation error, not a
code defect, and the figure was not quoted as reproduced until it actually was
(`04_debug_and_failures.md` section 16).

These are targeted passes on selected packages. They are **not** a whole-repository
pass.

### 6.2 `SYNTHETIC_PACKAGE_TESTS`

```text
dg_synthetic_validation: 33 passed
```

Includes the 14 new tests covering the plot write choke point and the gid-based series
counting.

### 6.3 `FULL_COLCON_TEST`

```text
STATUS THIS ROUND: NOT_RUN
PREVIOUSLY RECORDED: 666 tests, 0 errors, 335 failures, 10 skipped — NOT PASS
```

The previously recorded figure is **not** a pass, and its failures are predominantly
pre-existing flake8/pep257 **style** findings. It must never be presented as core DG
behavioural failure, and the targeted passes in 6.1 and 6.2 must never be presented as
a whole-repository pass. Both misreadings are available in opposite directions from the
same three numbers, which is why the categories stay separate.

## 7. Offline matcher verification

An offline call into the real `refine_pose_near_seed` from
`src/ylhb_base/scripts/scan_map_relocalization_node.py` against ray-cast geometry,
with thresholds untouched (`min_score=0.45`, `min_inlier_ratio=0.45`,
`max_mean_distance=0.30`).

| Case | Result | score | inlier | mean | recovered yaw |
|---|---|---|---|---|---|
| no rotation | `ACCEPTED` | 0.975 | 1.000 | 0.009 | — |
| true yaw +10 deg from a 0 deg seed | `ACCEPTED` | 0.975 | 1.000 | 0.009 | 10.00 deg |
| true yaw -10 deg from a 0 deg seed | `ACCEPTED` | 0.975 | 1.000 | 0.009 | -10.00 deg |

```text
scan points per match:  180
time per match:         ~0.15 s   (segment_timeout is 8 s, ample margin)
distance field build:   0.05 s
ray-cast generator:     360 finite beams
                        0.850 m to the interior wall from the origin
                        3.900 m to the border from the origin
analytic generator:     bit-identical to the historical S01-S04 waveform
```

```text
CONCLUSION: no threshold, state machine, or algorithm semantic change is needed.
```

Scope, stated explicitly: it shows that the **unmodified** matcher can recover a
+-10 deg yaw offset from consistent synthetic geometry within its own thresholds. It is
not a localization-accuracy result, not a success-rate result, and not evidence about
real LiDAR data.

## 8. Historical S01–S04 evidence and the warning that must not be over-read

S01–S04 remain as recorded in the handoff: all four `PASS`, with `SYNTHETIC_VALIDATED`
scope. Round B did not modify `results/` other than by writing the four new precheck
directories.

S02–S04 carry the warning:

```text
RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE
```

Round B established why this must **not** be read as closed-loop success:

- S02–S04 degrade GNSS and LiDAR only
- `GNSS_DEGRADED`, `GNSS_REJECTED`, `LIDAR_QUALITY_LOW`, `LIDAR_QUALITY_INVALID` and
  `ODOM_STALE` are `degraded_reasons`, which **cannot** trigger recovery
  (`src/ylhb_base/scripts/active_relocalization_core.py:192-304`)
- the injector additionally pinned AMCL covariance at `0.05`, below
  `max_covariance = 0.5`, so `AMCL_COVARIANCE_HIGH` could never fire either

So in S02–S04 no trigger path existed at all. The warning records bounded activity on
a status topic, nothing more.

The S05 precheck is the contrast case: its trigger reason is `AMCL_COVARIANCE_HIGH`,
recorded per sample, and the escalation is visible in `navigation_state`. That is what
a trigger looks like in the evidence, and it is not what S02–S04 contain.

## 9. GUI capture readiness

```text
GUI_CAPTURE_READY: MANUAL_ONLY   (frozen this round)
```

The desktop session is GNOME on Wayland. `ffmpeg -f x11grab` exits `0` but yields
`pblack:99` because mutter's guard window covers the rootless Xwayland root; `xwd
-root` fails `BadMatch` on `X_GetImage`; `org.gnome.Shell.Screenshot` over D-Bus
returns `AccessDenied`. Detail in `04_debug_and_failures.md` sections 8 and 14.

```text
FILE_EXISTS       != VALID_SCREENSHOT
CAPTURE_AVAILABLE != VALID_EVIDENCE
RULE: a uniform or black frame is FAILURE and is never filed as a screenshot.
```

The capture helper itself had a false positive: the first implementation judged frames
by flat-colour and channel spread only, and the guard-window black frame carries a
channel spread of about 11, enough to pass. A brightness floor was added and validated
against known-positive and known-negative inputs.

Automatic data plots are unaffected and are still produced — subject now to the write
choke point that refuses a figure with zero data series.

## 10. Claims that are not supported by anything in this round

```text
XY error < 20 cm:              NOT_PROVEN
Z error < 20 cm:               NOT_PROVEN
feature repeatability >= 95%:  NOT_PROVEN
relocalization success > 95%:  NOT_PROVEN
Gazebo validation:             NOT_RUN
real robot validation:         NOT_RUN
competition performance:       NOT_PROVEN
```

Four `PASS` prechecks do not move any line in that table. They are software-state
evidence at `PRECHECK` level, produced against synthetic input whose true pose is
authored by the test rig.

Z/elevation remains an `OPEN_TECHNICAL_GAP`, unchanged by Round B.

---

## S05 FINAL evidence accounting (added 2026-08-29)

Three FINAL runs of S05 exist under `results/synthetic/final/`.  All three are
valid and produced identical verdicts; the duplication is a capture artefact,
not a data disagreement.  Exactly one is designated canonical so that a later
document cites a single run.

| run_id | status | reason |
|---|---|---|
| `S05_..._20260829-141521` | DUPLICATE_VALID_RERUN | first FINAL run; the marker node still carried the pre-staleness code, so no trustworthy GUI capture was possible |
| `S05_..._20260829-150349` | DUPLICATE_VALID_RERUN | staleness fix was built but the already-running marker node had not been restarted, so the fix was not active in the GUI |
| `S05_..._20260829-160001` | **CANONICAL** | marker node restarted and confirmed; cite this run |

All three: `result=PASS`, `run_class=FINAL`, 12 checks true as recorded in the
artefacts — but see "Corrected check accounting" below, because one of those 12
could not fail and the honest figure is **11 counted checks** — 0 errors,
0 warnings, 239 samples, `reloc: NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING`,
`trigger_reason=AMCL_COVARIANCE_HIGH`, real `/cmd_vel` publishers 0 at start
and end, 9 plots generated with `scan_map_quality.png` correctly skipped
(S05 runs without the plant, so no candidate data exists).

### S05_GUI_CAPTURE: NOT_CAPTURED

State this precisely, because the two readings are not equivalent:

- The states WERE produced by the software.  `TRIGGERED` and `STOPPING` are in
  `samples.csv` and `timeline.csv` for all three runs.
- The live GUI screenshot was simply not taken during the measured window.

This is a **capture omission**, NOT `NOT_OBSERVED`.  `NOT_OBSERVED` is reserved
for a state the system never entered, and must never be used to describe a
state that occurred but went unphotographed.  No screenshot may be staged or
recreated after the fact to fill this gap.

The one S05-era screenshot that was taken shows an all-`STALE` / `NO_DATA`
post-run screen.  It is filed as evidence of the marker staleness fix working,
NOT as evidence of any S05 result: every S05-specific field in it reads STALE or
NO_DATA.  It is worth keeping for a second reason — the per-source ages differ
(sensor inputs ~91.3 s versus algorithm outputs ~86.7 s, a ~4.6 s gap), which
truthfully exposes that the injector stops at the scenario duration while the
integration launch is torn down a few seconds later.  The previous frozen-value
display could not have shown that.

### Why S05 was not re-run again for a screenshot

S05's substantive finding is a data result, not a visual one: during the LiDAR
`DEGRADED` window (6.1 s to 12.1 s in the canonical run) `relocalization_state`
remained `NORMAL` throughout.  That is the direct counter-evidence to reading
the historical S02-S04 `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` warning
as closed-loop success, and it is verifiable from the CSV rather than from a
picture.  A screenshot adds no evidential weight to it.

---

## S06 FINAL evidence accounting (correction)

```text
S06 FINAL EVIDENCE:  RUN — 2 runs, both PASS
SUPERSEDED CLAIM:    "S06 FINAL EVIDENCE: NOT_YET_RUN"  — contradicted disk
```

| run_id | samples | status | reason |
|---|---|---|---|
| `S06_closed_loop_active_scan_recovery_motion_20260829-165043` | 399 | **`CANONICAL`** | executed after the marker-staleness fix was actually active, so its GUI state is trustworthy; cite this run |
| `S06_closed_loop_active_scan_recovery_motion_20260829-161050` | 392 | `SUPERSEDED_FINAL` | valid FINAL data, but captured before that fix was live; do not cite |

Both: `result=PASS`, `run_class=FINAL`, `errors=[]`. The relationship matters and
is stated so neither run is misread: `161050` is **not** a failure and **not** a
discarded run. It is valid data superseded for capture reasons only, and it is
retained under the immutability rule in `EXPERIMENT_EVIDENCE_POLICY.md` section 6.
Only the canonical run carries an `evidence/` directory and manifest.

### Reconciling `S06_RERUN = NO`

These two statements have been read as contradictory. They are not:

```text
S06_RERUN = NO   means      no FURTHER S06 rerun for screenshot purposes
S06_RERUN = NO   does NOT   mean that no S06 FINAL run exists
```

`S06_RERUN = NO` is recorded inside the canonical run's own
`evidence/evidence_manifest.md`, directly alongside
`S06_CANONICAL_FINAL_EVIDENCE = YES`. A directive that lives inside a FINAL run's
manifest cannot mean that no FINAL run happened. It forbids chasing a better
screenshot; it says nothing about execution.

## Corrected check accounting (correction)

```text
CLASSIFICATION:  NON_DISCRIMINATING_CHECK
FIXED IN CODE:   YES — by the lead engineer, result_writer.py ~488-506
```

`relocalization_not_unexpectedly_active` was hardcoded `True` for every scenario
except S01, yet still occupied a slot in the reported check totals. It could not
fail, so it inflated apparent coverage. The code now counts it for S01 only and
reports it for every other scenario under `non_discriminating_checks`, with a
stated reason and the `NON_DISCRIMINATING_CHECK` classification.

| Figure | Previously published | Corrected |
|---|---|---|
| S01-class | `12/12` | **11 counted checks** |
| S06-class | `15/15` | **14 counted checks + 1 non-discriminating** |

The previously published `12/12` and `15/15` totals each included one check that
could not fail, and must not be presented as if every check were discriminating.

Two boundaries on this correction:

- The check counts stored **inside** the existing FINAL `result.json` artefacts
  predate the code change and still carry the old shape. Those artefacts are
  immutable audited evidence and are not rewritten. The corrected accounting is
  documentation-side only.
- The `15/15` figures appearing in `03_implementation_log.md` and
  `04_debug_and_failures.md` are a **different measurement** — non-empty INPUT
  columns in `samples.csv`, verified by runtime round-trip. They are not check
  totals and are correct as written.

## Evidence manifest caption fields that render empty (correction)

This concerns how a reader should interpret the manifests under
`results/synthetic/final/*/evidence/`. The manifests themselves are audited
artefacts and are **not** edited; neither is any screenshot.

Some caption fields render with nothing after the label, for example:

```text
... wz 0.0，/cmd_vel 发布者 。场景时刻 4.34s。
                          ^ renders empty
```

```text
EMPTY CAPTION FIELD MEANS:      the field was NOT PUBLISHED by any topic
EMPTY CAPTION FIELD DOES NOT MEAN: the value was lost, dropped, or omitted
```

The blank is `NOT_APPLICABLE`, not `MISSING`. Verified cause: the caption's
`/cmd_vel 发布者` slot is filled from the `safety_cmd_vel_publishers` column,
and that column is empty in **every** sample of the S06 canonical (399/399), S07
(159/159) and S08 (239/239) FINAL runs, because no topic published it. The
labeller writes exactly what the column held, which is the correct behaviour
under `EXPERIMENT_EVIDENCE_POLICY.md` section 5: a missing observation is
recorded as empty and never inferred or interpolated.

The same reading applies to any empty `cmd 来源` slot. Note that in the retained
manifests `cmd 来源` is populated (`ZERO` / `RECOVERY`) because it is filled from
`cmd_vel_source_inferred`, an **inferred** column — while the direct
`cmd_vel_source` column is empty in all samples. Where an empty `cmd 来源`
appears, it means no source field was published, not that a known value went
missing.

```text
RULE: a blank caption field is never back-filled, and never re-captioned to look
complete. It is read as "not published", and the reader is told so here.
```

## Criteria-evolution disclosure — S01 and the road to green

```text
CRITERIA_PRE_REGISTERED:              NO
POST_HOC_CRITERIA_EVOLUTION_RISK:     EXISTS — criteria were edited during development
CRITERIA_CHANGES_SUFFICIENT_TO_PASS:  NO — verified below
```

State the uncomfortable half first: the S01–S08 pass criteria were **not
pre-registered**. They were written and revised while the rig was being brought
up, so the structural risk of post-hoc criteria evolution is real and is disclosed
here rather than left for a reader to discover.

What the artefacts actually show, though, is more specific than "criteria were
loosened, then it passed" — and the difference matters, because that shorter
reading is harsher than the evidence supports.

Three commits changed the **evaluation layer**. Verified by `git show --stat`:
each touches `result_writer.py` and nothing else, so each is a genuine criteria
change, not a rig change:

```text
618122f  fix(test): ignore bounded startup relocalization noise   result_writer.py only
1a3fb4b  fix(test): scope relocalization checks by scenario        result_writer.py only
e1fdbc7  fix(test): apply startup grace to nominal health          result_writer.py only
```

Two further commits changed the **test rig's start-up**, not the criteria. Again
verified: each touches `scenario_runner.py` and nothing else:

```text
c74e4f5  fix(test): prewarm synthetic sensor inputs                scenario_runner.py only
fd2c60d  fix(test): prewarm ROS graph before scenario clock        scenario_runner.py only
```

The S01 verdict at each build, read from `build_commit` in each run's
`result.json` under `results/synthetic/`:

| S01 run | `build_commit` | Layer changed | Verdict | Failures |
|---|---|---|---|---|
| `S01_nominal_20260828-145033` | `618122f` | criteria | `PASS` | — |
| `S01_nominal_20260828-145355` | `1a3fb4b` | criteria | `FAIL` | `UNEXPECTED_RELOCALIZATION_STATE`, `NAVIGATION_NOT_FAILED` |
| `S01_nominal_20260828-145527` | `e1fdbc7` | criteria | `FAIL` | `UNEXPECTED_RELOCALIZATION_STATE` |
| `S01_nominal_20260828-145657` | `c74e4f5` | rig start-up | `FAIL` | `UNEXPECTED_RELOCALIZATION_STATE` |
| `S01_nominal_20260828-145833` | `fd2c60d` | rig start-up | `PASS` | — |

Two things follow, and both must be stated together:

- The criteria changes were **real and they mattered**. They are not dismissed
  here, and the lack of pre-registration is not excused by anything below.
- The criteria changes were **not sufficient**. S01 was still `FAIL` at
  `e1fdbc7`, the last of the three, and still `FAIL` at `c74e4f5`. The step that
  actually reached green was `fd2c60d`, a rig change that removed the start-up
  transient **at its source** by prewarming the ROS graph before the scenario
  clock started — rather than teaching the evaluator to tolerate it.

One further detail cuts against the "loosen until green" reading: `1a3fb4b`, a
criteria commit, moved S01 from `PASS` to `FAIL`. The criteria sequence was not
monotonically permissive.

```text
CONCLUSION TO CARRY: criteria evolution during development is disclosed and is a
real risk. It is NOT what turned S01 green. The last step to green removed the
transient at its source in the rig, which is the stronger of the two positions
and is the one the artefacts support.
```
