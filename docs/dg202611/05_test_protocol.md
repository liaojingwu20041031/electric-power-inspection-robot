# DG-202611 Round B — 05 Test Protocol for S05–S08

```text
PRECHECK_STATUS:       RUN for S05, S06, S07, S08 — all four PASS
FINAL_EVIDENCE_STATUS: RUN for S05, S06, S07, S08 — all FINAL runs PASS
                       (CORRECTED: this line read "NOT_YET_RUN for S05, S06,
                       S07, S08", which contradicted results/synthetic/final/.
                       S06 has TWO FINAL runs, canonical ..._20260829-165043.
                       See 06_results_and_evidence.md section 5.)
EVIDENCE_CLASS:        SYNTHETIC_SOFTWARE_VALIDATION
```

## 1. Scenario identity status — read this first

```text
SCENARIO_DEFINITION_STATUS: LANDED AND VERIFIED ON THE TARGET
```

`src/dg_synthetic_validation/config/` now contains `S01.yaml` through `S08.yaml`, and
all eight install to
`install/dg_synthetic_validation/share/dg_synthetic_validation/config/` (verified by
read-back). The protocols below were written before the YAMLs existed, so each
section states its **proposed** intent alongside the **landed** charter.

| ID | Landed scenario name | Round B protocol intent | Match |
|---|---|---|---|
| S05 | `amcl_covariance_ramp_localization_trigger` | trigger and stop confirmation | matches |
| S06 | `closed_loop_active_scan_recovery_motion` | active scan sweep and seed handoff | matches, widened to the full closed-loop motion |
| S07 | `real_candidate_from_real_seed_request` | closed loop to `RECOVERED` | narrowed: the charter is candidate attribution, not recovery |
| S08 | `multiframe_verification_and_control_handoff` | multi-frame recovery verification | matches |

```text
ALL FOUR LANDED SCENARIOS MATCH THEIR SPECIFIED INTENT.
```

S08 in particular: the Round B protocol specifies multi-frame recovery verification —
candidate, continuous observation, multi-frame consistency and recovery confirmation,
recovery success or failure, Localization Health returning to a navigable state,
recovery control released, and the arbiter's permitted control path handed back. That
is what the landed `S08.yaml` implements, and section 7 below records it as such.

The protocol's separate clause about entering `FAILED` / `MANUAL_REQUIRED` "under
failure conditions" is a statement about what the state machine may legitimately do.
It is not the assignment of a failure-branch scenario to S08, and it must not be read
as one.

```text
COVERAGE_GAP: the FAILED and MANUAL_REQUIRED branches are exercised by no scenario.
STATUS: NOT_YET_COVERED — a coverage gap, NOT a spec deviation.
```

Specified as future work in section 11, tracked in `07_open_issues.md`, and
deliberately **not** scheduled for this round.

All state names used below are the real ones. `LOST` and `SEARCHING` do not exist
and must never appear.

## 2. Rules that apply to every scenario

Preconditions common to S05–S08:

```text
Nav2:                  disabled
cmd_vel_output_topic:  /dg/test_cmd_vel
/cmd_vel publishers:   0 at scenario start AND 0 at scenario end
core thresholds:       unmodified
core state machine:    unmodified
injector boundary:     /gps/fix /gps/rtk_status /scan /odom /amcl_pose /map
                       /initialpose /tf /tf_static
TF edges present:      odom -> base_footprint (dynamic)
                       base_footprint -> laser (static)
TF edges absent:       map -> odom
plant:                 SYNTHETIC_KINEMATIC_PLANT enabled
surrogate:             SYNTHETIC_AMCL_SURROGATE enabled
scan generator:        raycast
```

The trigger must come from a `trigger_reasons` member. GNSS or LiDAR degradation
alone **cannot** trigger recovery, so a scenario built on it can only ever produce
`degraded_reasons` and will never enter `SUSPECTED` for a localization cause. See
`02_architecture_and_design.md` section 3.

## 3. The pass criterion that is explicitly rejected

```text
NOT AN ACCEPTABLE PASS CRITERION:
  "relocalization activity was observed"
```

This is stated as a rule because it is exactly how the historical S02–S04 warning
`RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` has been misread. Observing
activity on `/dg/relocalization/status` proves only that a node is publishing. A
pass requires the **named state transitions** in the correct order, with the
arbiter behaviour that accompanies them. Anything less is not a pass.

Equally rejected: a pass claimed from a run whose visible evidence is a black or
uniform frame. See `04_debug_and_failures.md` section 8.

## 4. S05 — trigger and stop confirmation

```text
PROPOSED_INTENT: reach ACTIVE_SCAN from NORMAL via a real trigger reason
PRECHECK: PASS   FINAL_EVIDENCE: RUN, PASS (3 runs; canonical ..._20260829-160001)
LANDED_SCENARIO: config/S05.yaml, amcl_covariance_ramp_localization_trigger
```

`PRECONDITIONS`
- Section 2 common preconditions
- Plant starts at its declared initial pose, at rest
- Surrogate publishes `/amcl_pose` with a healthy covariance during the warm-up phase

`INPUTS`
- Surrogate covariance ramped above `max_covariance = 0.5` to raise
  `AMCL_COVARIANCE_HIGH`, a `trigger_reasons` member
- Plant commanded only by the arbiter's `/dg/test_cmd_vel`; no scripted motion

`EXPECTED_STATE_TRANSITIONS`
```text
NORMAL -> SUSPECTED      on the 2nd consecutive triggerable sample
SUSPECTED -> TRIGGERED   on the 3rd consecutive triggerable sample
TRIGGERED -> STOPPING    next tick
STOPPING -> ACTIVE_SCAN  once odom is fresh, |linear| <= 0.03, |angular| <= 0.03, yaw finite
```
Navigation Health: `LOCALIZATION_SUSPECT` while `SUSPECTED`, then `RECOVERING`
from `TRIGGERED` onward.

`REQUIRED_OBSERVATIONS`
- `/dg/relocalization/status`: `state` and `trigger_reason` per sample
- `/dg/navigation/status`: `overall_state`, `relocalization_state`, `reasons`
- `/dg/test_cmd_vel`: zero output through `SUSPECTED` and `TRIGGERED`
- `/odom`: velocity crossing below `0.03` before `ACTIVE_SCAN` is entered
- `/cmd_vel` publisher count at start and end

`PASS_CRITERIA`
- All four transitions above observed, in order, with `trigger_reason` naming a
  `trigger_reasons` member
- `STOPPING` confirmed only after the velocity condition is genuinely met
- `command_linear_x` is `0.0` in every recovery sample
- `/cmd_vel` publisher count is `0` at start and end

`FAIL_CRITERIA`
- `SUSPECTED` never reached, or reached with only `degraded_reasons`
- `STOPPING` entered and never confirmed (note: no timeout exists, so this
  presents as a silent hang, not an error)
- Any non-zero `/dg/test_cmd_vel` linear component
- Any `/cmd_vel` publisher at any point

`OUT_OF_SCOPE`
- Whether the pose was actually lost; the covariance is synthetic
- Localization accuracy of any kind
- Candidate quality, which S05 does not reach

## 5. S06 — active scan sweep and seed publication

```text
PROPOSED_INTENT: exercise the segment sweep and the seed handoff
PRECHECK: PASS   FINAL_EVIDENCE: RUN, 2 runs, both PASS
                 CANONICAL         ..._20260829-165043  (399 samples)
                 SUPERSEDED_FINAL  ..._20260829-161050  (392 samples)
LANDED_SCENARIO: config/S06.yaml, closed_loop_active_scan_recovery_motion
```

`FINAL_EVIDENCE` for S06 previously read `NOT_YET_RUN`, which contradicted disk.
The canonical run is the later one, executed after the marker-staleness fix was
actually active; `161050` is valid FINAL data captured before that fix was live
and is retained, not discarded. `S06_RERUN = NO` forbids further reruns for
screenshot purposes and does **not** mean no FINAL run exists. Full accounting in
`06_results_and_evidence.md`.

`PRECONDITIONS`
- Section 2 common preconditions
- S05 behaviour reachable, i.e. the trigger and stop path works

`INPUTS`
- Same trigger input as S05
- Plant free to rotate under arbiter command, so yaw actually changes

`EXPECTED_STATE_TRANSITIONS`
```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
ACTIVE_SCAN -> WAITING_CANDIDATE   after reaching segment_target_yaw
                                   within 1.5 deg and settling 0.5 s
```

`REQUIRED_OBSERVATIONS`
- Measured yaw against `segment_target_yaw` for the first segment of
  `segment_deltas = (+10, -20, +10)` degrees
- `/dg/test_cmd_vel`: angular magnitude `0.18 rad/s`, linear exactly `0.0`
- A seed published on `/dg/relocalization/seed` while in `WAITING_CANDIDATE`
- Elapsed time per segment against `segment_timeout = 8.0 s`

`PASS_CRITERIA`
- `ACTIVE_SCAN` reaches its target within tolerance and settles, then
  `WAITING_CANDIDATE` is entered
- At least one seed observed on `/dg/relocalization/seed`
- Rotation is pure: linear component `0.0` throughout
- No segment ends in `SEGMENT_TIMEOUT`

`FAIL_CRITERIA`
- `SEGMENT_TIMEOUT` on any segment
- Yaw never converges to within `1.5 deg` of target
- No seed published while in `WAITING_CANDIDATE`
- Total rotation exceeding `max_total_rotation = 60 deg`

`OUT_OF_SCOPE`
- Whether the seed is correct; S06 tests the handoff, not the pose
- Match acceptance, which is S07

## 6. S07 — closed loop to `RECOVERED`

```text
PROPOSED_INTENT: the real scan matcher recovers a genuinely wrong pose,
                 carrying the state machine to RECOVERED
PRECHECK: PASS   FINAL_EVIDENCE: RUN, PASS (..._20260829-171148, 159 samples)
LANDED_SCENARIO: config/S07.yaml, real_candidate_from_real_seed_request
```

This is the scenario that was **structurally impossible** before Round B: blocker
4 meant the matcher always returned `TF_FAILED`, so no candidate could exist.

`PRECONDITIONS`
- Section 2 common preconditions, with TF genuinely present and the
  `base_footprint <- laser` lookup succeeding
- Ray-cast scan generator active, so scan geometry is consistent with `/map`
- Surrogate pose bias active: about 0.15 m and 8 deg, **inside** the matcher's real
  +-0.20 m / +-10 deg coarse window
- Matcher thresholds untouched: `min_score=0.45`, `min_inlier_ratio=0.45`,
  `max_mean_distance=0.30`

`INPUTS`
- Trigger as in S05
- A seed derived from a **biased** `/amcl_pose`, so the seed handed to the matcher is
  genuinely wrong

`EXPECTED_STATE_TRANSITIONS`
```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
ACTIVE_SCAN -> WAITING_CANDIDATE
WAITING_CANDIDATE -> VERIFYING   on an accepted, in-threshold candidate
VERIFYING -> RECOVERED           after 3 consecutive samples with
                                 position jump <= 0.5 m and yaw jump <= 20 deg
```
`RECOVERED` is absorbing. Navigation Health passes `RECOVERED` through; the
arbiter then selects `NAV` and emits zero with `NAV_SOURCE_STALE`.

`REQUIRED_OBSERVATIONS`
- `/dg/relocalization/match_quality`: `accepted`, `score`, `inlier_ratio`,
  `mean_distance`, `reason` per attempt
- `/scan_match_pose` published on acceptance
- The seed pose, the matched pose, and the plant's true pose recorded separately,
  so the bias and its correction are both visible
- `VERIFYING` sample count and the measured position/yaw jumps
- Arbiter reason string after `RECOVERED`
- `/cmd_vel` publisher count at start and end

`PASS_CRITERIA`
- The full transition chain above observed in order, ending in `RECOVERED`
- The accepted match satisfies the **unmodified** thresholds
- The matched pose is closer to plant truth than the biased seed was, i.e. the
  matcher demonstrably corrected the seed rather than passing it through
- After `RECOVERED`, the arbiter emits zero with `NAV_SOURCE_STALE`
- `/cmd_vel` publisher count `0` at start and end

`FAIL_CRITERIA`
- `TF_FAILED` in any match attempt
- No candidate accepted, so `VERIFYING` never entered
- `RECOVERED` reached while the seed was already correct, which would make the run a
  tautology and void it as evidence (see `04_debug_and_failures.md` section 7)
- Any threshold, state machine, or algorithm semantic altered to obtain the result
- Arbiter forwarding a non-zero command after `RECOVERED`

`OUT_OF_SCOPE`
- Localization accuracy. The surrogate reads synthetic ground truth, so S07
  evidences the **software state chain** only.
- Any XY or Z error claim, feature repeatability, or relocalization success rate
- Real LiDAR behaviour, real AMCL behaviour, real robot behaviour

## 7. S08 — multi-frame recovery verification and control handoff

```text
PROTOCOL_INTENT: multi-cycle / multi-frame stability verification, then control handoff
LANDED_SCENARIO: config/S08.yaml, multiframe_verification_and_control_handoff
MATCH:           the landed scenario implements the specified intent
PRECHECK: PASS   FINAL_EVIDENCE: RUN, PASS (..._20260829-171722, 239 samples)
```

The specified chain, in order: a candidate, continuous observation of it, multi-frame
consistency and recovery confirmation, recovery success or failure, Localization Health
returning to a navigable state, recovery control released, and the arbiter's permitted
control path handed back.

`PRECONDITIONS`
- Section 2 common preconditions
- The same synthetic input base as S07, plus surrogate convergence
- Surrogate convergence is caused **only** by a real accepted `/scan_match_pose` from
  the real matcher, never scheduled by time

`INPUTS`
- Trigger as in S05
- A biased seed inside the matcher's real coarse window, as in S07
- `converge_on_match` enabled, so covariance returns to a healthy value **after** a real
  acceptance

`EXPECTED_STATE_TRANSITIONS`
```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
ACTIVE_SCAN -> WAITING_CANDIDATE
WAITING_CANDIDATE -> VERIFYING   on an accepted, in-threshold candidate
VERIFYING -> RECOVERED           after verification_samples = 3 consecutive samples
                                 with position jump <= 0.5 m and yaw jump <= 20 deg
```
Navigation Health passes `RECOVERED` through; the arbiter then selects `NAV` and, with
Nav2 disabled and nothing publishing `/cmd_vel_nav`, emits zero with
`NAV_SOURCE_STALE`.

`REQUIRED_OBSERVATIONS`
- `reloc_verification_count` progression up to the real `verification_samples` default
- The measured position and yaw jumps for each verification sample
- `synthetic_amcl_surrogate_mode` transition, and the accepted `/scan_match_pose` that
  causes it
- `navigation_state` reaching `RECOVERED`
- `cmd_angular_z` after `RECOVERED`, to show recovery released control
- `/cmd_vel` publisher count at start and end

`PASS_CRITERIA`
- `WAITING_CANDIDATE -> VERIFYING -> RECOVERED` observed in order
- Stability verification completes over multiple consecutive **verification
  cycles**, not a single sample. This is a multi-cycle / multi-frame stability
  check on one candidate pose, **not** repeated independent matching — see the
  wording rule below
- Navigation Health reports `RECOVERED`
- After `RECOVERED`, recovery output is zero — control was released, not held
- Surrogate convergence is attributable to a real accepted match, not to elapsed time
- `/cmd_vel` publisher count `0` at start and end

`FAIL_CRITERIA`
- `RECOVERED` reached without the multi-frame verification count being satisfied
- Any injected candidate or injected `RECOVERED` state
- Surrogate convergence occurring on a timer rather than on a real acceptance
- Non-zero recovery output after `RECOVERED`
- Any `/cmd_vel` publisher at any point

`OUT_OF_SCOPE`
- Localization accuracy; the surrogate reads synthetic ground truth
- Any statement that navigation resumed. The permitted wording for the end state is
  "safe control handoff verified in synthetic software validation"
- Any claim of repeated independent scan-to-map matching. Verified in S07 and S08:
  `reloc_verification_count` advances `0 -> 1 -> 2 -> 3` while
  `match_message_count = 1`. The matcher accepted exactly one candidate, and the
  repeated identical match values are a **latch** of that single match.
  Required wording: "after a single accepted candidate match, the system confirmed
  candidate-pose stability across multiple consecutive verification cycles, with
  `verification_count` advancing from 0 to 3". Forbidden: "three consecutive
  independent matcher successes", or anything equivalent
- The `FAILED` and `MANUAL_REQUIRED` branches, which S08 does not exercise. See
  section 11

## 8. Reporting rules for S05–S08 results

- Report the three test categories separately and never merge them; see
  `06_results_and_evidence.md` section 6
- Report the run class with every number. A `PRECHECK` figure is never a document
  claim, whatever it says
- After `RECOVERED`, `NAV_SOURCE_STALE` may be described as "safe control handoff
  verified in synthetic software validation", and never as navigation resuming,
  Nav2 resuming, or a mission continuing. Record
  `SAFE_CONTROL_HANDOFF_EVIDENCED = YES` and
  `NAVIGATION_RESUMED_EVIDENCED = NO`, and qualify the handoff as a
  `DERIVED_ASSERTION`: `cmd_vel_source` and `safety_cmd_vel_publishers` were empty
  in every FINAL sample, so no directly published source field witnesses it
- Every S05–S08 statement carries `SYNTHETIC_SOFTWARE_VALIDATION`
- A missing observation is recorded as empty or `NO_DATA`, never inferred. In an
  evidence-manifest caption an empty field means "the field was not published by
  any topic", never "the value was lost"
- A repeated deterministic figure is one measurement, not two
- Where an S05–S08 candidate pose is discussed, state that it originates from an
  AMCL surrogate that reads synthetic ground truth, that the supervisor under test
  has no ground-truth leak, and that `GROUND_TRUTH_LEAK=NO` is a hardcoded string
  literal corroborated by independent code audit rather than a measurement


## 9. Precheck outcomes and the criteria actually evaluated

```text
RUN_CLASS: PRECHECK   NOT_FINAL_EVIDENCE: TRUE   NOT_FOR_DOCUMENT_CLAIM: TRUE
RUN ROOT:  /home/weiyang/dg202611_ws/results/synthetic/precheck/
```

The criterion names below are the ones in `result.json` and `result.md`. Full observed
values are in `06_results_and_evidence.md` section 2; only the criteria are listed
here, because the protocol's job is to say what is checked.

### 9.1 S05 — `PASS`, 239 samples

```text
s05_sut_reported_amcl_health_degraded
s05_normal_suspected_triggered_in_order
s05_trigger_reason_is_a_real_trigger_path
s05_navigation_health_escalated
```

S05 terminates in `STOPPING`, not `ACTIVE_SCAN`: the plant is disabled and
`odom_linear_x` is a constant `0.05` in every phase, above `stop_velocity = 0.03`, so
the stop is never confirmed. The proposed protocol in section 4 expected
`STOPPING -> ACTIVE_SCAN`; the landed charter ends at trigger and escalation, and the
stop confirmation is exercised by S06 instead.

```text
tf_base_to_sensor_available: EMPTY for S05, and CORRECT — the plant is disabled,
so no TF buffer exists. An honest absence, not a gap.
```

### 9.2 S06 — `PASS`, 387 samples

```text
s06_stopping_confirmed_then_active_scan
s06_navigation_health_recovering
s06_supervisor_commanded_rotation
s06_arbiter_forwarded_recovery
s06_test_cmd_vel_carried_rotation
s06_recovery_is_pure_rotation
s06_plant_yaw_responded_to_command
```

`s06_plant_yaw_responded_to_command` is the criterion that motivated the assertion
rule in section 10.2: it requires `odom_yaw` to move by more than 1 degree, so an
empty `INPUT` column fails it for lack of evidence and the failure reads as a plant
defect.

### 9.3 S07 — `PASS`, 159 samples

```text
s07_seed_requested_by_real_supervisor
s07_real_matcher_published_quality
s07_candidate_accepted_by_real_matcher
s07_candidate_meets_unmodified_thresholds
s07_candidate_used_real_scan_points
s07_candidate_followed_the_seed_request
```

`s07_candidate_followed_the_seed_request` is an ordering check, not a quality check:
the first seed request precedes the first accepted match. A candidate that arrived
before a request would be a candidate the rig produced.

### 9.4 S08 — `PASS`, 239 samples

```text
s08_waiting_verifying_recovered_in_order
s08_multi_frame_verification_completed
s08_recovered_observed
s08_navigation_health_reported_recovered
s08_safe_control_handoff_observed
```

S08 is the only run whose navigation reaches `RECOVERED`, and the only one that shows a
control handoff. The `FAILED` and `MANUAL_REQUIRED` branches are exercised by no
scenario at all; that is a coverage gap, recorded in section 11, and not a shortfall
against S08's specified intent.

### 9.5 Common criteria, all four runs

```text
samples_present  integration_alive  finite_fusion_outputs
real_cmd_vel_unpublished_at_start   real_cmd_vel_is_unpublished
relocalization_activity_observed    relocalization_states_are_known
```

`relocalization_activity_observed` appears in this list and is **still not** a pass
criterion in the sense rejected in section 3. It records that the status topic
carried traffic. The pass is carried by the named per-scenario criteria above it.

`relocalization_not_unexpectedly_active` was previously listed here as a common
criterion for all four runs. That was wrong: it was hardcoded `True` for every
scenario except S01 while still occupying a slot in the reported check total, so
it could not fail. It is removed from the common-criteria list above and
reclassified:

```text
CLASSIFICATION:  NON_DISCRIMINATING_CHECK
S01:             a COUNTED check — recovery activity there is a genuine failure
S02-S08:         reported under non_discriminating_checks with a stated reason,
                 NOT counted in the check total
FIXED IN CODE:   YES — result_writer.py ~488-506
```

Consequence for previously published totals:

```text
was 12/12 for S01-class  ->  11 counted checks
was 15/15 for S06-class  ->  14 counted checks + 1 non-discriminating
```

Neither old total may be presented as if all of its checks were discriminating.
The counts stored inside existing FINAL `result.json` artefacts predate the code
change and are not rewritten — those artefacts are immutable.

### 9.6 What a precheck does not establish

```text
VISIBLE_EVIDENCE_CHECKPOINT: NOT PASSED for any of the four runs
MANUAL SCREENSHOTS:          NONE
EVIDENCE MANIFEST:           NONE
```

A precheck is headless. Final evidence additionally requires the checkpoint in
`USER_VISIBLE_WORKFLOW.md` section 4, manual capture per `MANUAL_CAPTURE_CHECKLIST.md`,
and a verified manifest.

## 10. Verification rules for anything reported from a run

### 10.1 Target verification before a claim

No file, fix, or scenario may be reported as implemented or verified without all of:
target read-back, a structural `grep` for the symbol that was supposed to appear,
`py_compile` on the target copy, and — where a data path is involved — a runtime
sanity check that the value lands in the artefact.

```text
NOT VERIFICATION: a file-transfer command that exited 0.
NOT VERIFICATION: a subagent summary stating the work was done.
```

The class has already recurred twice (`04_debug_and_failures.md` section 9), so this
is a gate, not a recommendation.

### 10.2 Incomplete evidence is not a behavioural failure

```text
RULE: an assertion that depends on an INPUT column MUST distinguish
      INPUT_EVIDENCE_INCOMPLETE from a genuine behavioural failure.
```

The two verdicts point at opposite root causes — the recorder versus the system under
test — and a threshold comparison cannot tell an absent column from a stationary
robot. See `04_debug_and_failures.md` section 10.

### 10.3 Artefacts must carry content, not just exist

```text
FILE_EXISTS        != VALID_EVIDENCE
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

A figure is written only when a tagged data series with at least one finite point was
drawn; threshold lines do not count. A frame is filed only when it passes both the
flat-colour test and the brightness floor. An RViz view is evidence only if the
display types match the topic message types and the frame is reachable.

### 10.4 Cross-run repetition is not corroboration

S07 and S08 reported bit-identical match quality
(`0.9795765221706485 / 1.0 / 0.007222222329841719`), deterministically, from identical
synthetic geometry and identical seed construction. That is one measurement reported
twice and must never be presented as two independent measurements.

### 10.5 Match residuals are not accuracy

`mean_distance` is a scan-to-map residual against synthetic grid geometry, classified
`SYNTHETIC_MATCH_QUALITY_RESULT`. It is not a positioning error in metres and may
never support a 20 cm metric claim. The true pose is authored by the test rig, and the
AMCL surrogate reads synthetic plant ground truth
(`GROUND_TRUTH_USED_BY_SUT: NO`, `GROUND_TRUTH_USED_BY_SURROGATE: YES`), so S05–S08
evidence the software state chain only.

### 10.6 The ground-truth surrogate boundary

```text
CANDIDATE_POSE_ORIGIN:  an AMCL SURROGATE that READS synthetic ground truth
SUPERVISOR_UNDER_TEST:  no ground-truth leak
GROUND_TRUTH_LEAK=NO:   a hardcoded string literal, corroborated by independent
                        code audit — NOT a measurement
```

What S05–S08 (Package A) **do** evidence: supervisor logic, trigger, candidate
handling, verification flow, state transitions, safe handoff.

What they **do not** evidence: real scan-matcher performance, real relocalization
accuracy, competition success rate.

## 11. Future work, not scheduled — `FAILED` / `MANUAL_REQUIRED` coverage

```text
COVERAGE_GAP: FAILED and MANUAL_REQUIRED branches untested
STATUS:       NOT_YET_COVERED (a coverage gap, NOT a spec deviation)
SCHEDULED:    NO — explicitly not this round
```

S05–S08 all take the success path, so the recovery machine's failure handling and retry
logic carry no synthetic evidence at all. This section retains the specification so the
gap is actionable rather than merely noted. It is not assigned to S08, and S08 is not
in deviation for lacking it.

Untested paths:

```text
_fail() reasons:  ATTEMPT_TIMEOUT  SEGMENT_TIMEOUT  MAX_TOTAL_ROTATION
                  ODOM_STALE_OR_MISSING  YAW_MISSING
                  WAITING_FOR_CANDIDATE_TIMEOUT
retry path:       FAILED -> STOPPING while attempt_id < max_attempts = 2
exhaustion path:  FAILED -> MANUAL_REQUIRED at MAX_ATTEMPTS_EXCEEDED
manual takeover:  the MANUAL_REQUIRED latch
```

`PROPOSED_APPROACH`
- Withhold a usable candidate, for example by degrading scan geometry **after** the seed
  request, so the real matcher legitimately rejects what it is given
- Let candidate exhaustion carry the machine through both attempts into
  `MANUAL_REQUIRED`

```text
RULE: this must be achieved by degrading INPUT only.
A FAILED or MANUAL_REQUIRED state is NEVER injected.
```

Injecting either state would fabricate the exact output under test, in the same way a
synthetic `/cmd_vel_nav` would fabricate the arbiter's input (`DECISION_006`).

`EXPECTED_STATE_TRANSITIONS`
```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
ACTIVE_SCAN -> WAITING_CANDIDATE -> ACTIVE_SCAN   per failed segment
ACTIVE_SCAN -> FAILED                             when segments are exhausted
FAILED -> STOPPING                                retry while attempt_id < max_attempts = 2
FAILED -> MANUAL_REQUIRED                         when attempts are exhausted
```

`REQUIRED_OBSERVATIONS`
- `reloc_failure_reason` per failed segment and per failed attempt
- `attempt_id` progression, bounded by `max_attempts = 2`
- Navigation Health `FAILED` then `MANUAL_REQUIRED`, both passed through
- `/dg/test_cmd_vel` forced to zero throughout both states
- `/cmd_vel` publisher count at start and end

`PASS_CRITERIA`
- Segment stepping, then `FAILED`, then a bounded retry, then `MANUAL_REQUIRED`
- Every rejection is the real matcher's own verdict against unmodified thresholds
- Arbiter output is zero for the whole of `FAILED` and `MANUAL_REQUIRED`
- `attempt_id` never exceeds `max_attempts = 2`
- `/cmd_vel` publisher count `0` at start and end

`FAIL_CRITERIA`
- Retry looping without bound, or `MANUAL_REQUIRED` never reached
- Any non-zero arbiter output during `FAILED` or `MANUAL_REQUIRED`
- A silent hang in `STOPPING`, which has no timeout
- Any threshold altered to force a rejection
- Any `/cmd_vel` publisher at any point

`OUT_OF_SCOPE`
- Whether a real robot would have failed under comparable conditions
- Any reliability or success-rate figure
