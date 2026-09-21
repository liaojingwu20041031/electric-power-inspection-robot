# DG-202611 Package A — Run Inventory

```text
EVIDENCE_CLASS = SYNTHETIC_SOFTWARE_VALIDATION
REAL_ROBOT_DATA = FALSE
COMPETITION_METRIC_EVIDENCE = FALSE
```

```text
PURPOSE:      complete ledger of every run directory on disk, not only the passes
RUN_ROOT:     /home/weiyang/dg202611_ws/results/synthetic
DIRECTORIES:  41
SOURCE:       result.json, metadata.json, result.md, plots/, logs/, rosbag/,
              evidence/ read directly from each run directory
METHOD:       every value below is read from an artefact. Nothing is inferred
              from a directory name and nothing is carried over between runs.
```

This file is bookkeeping. It designates which run a later document may cite and
records the runs it may not, so that a passing run can never be quoted without
the failed and superseded runs beside it. It makes no claim about accuracy,
repeatability, relocalization success rate, real-robot behaviour, Gazebo, or any
competition metric.

## 1. Reconciliation

```text
run directories                 41
result.json present             40
result.json absent               1
PASS                            31
FAIL                             9
```

Arithmetic, stated explicitly so a later reader can re-derive it:

```text
directories       = 30 (S01-S04, run root) + 7 (final/) + 4 (precheck/)   = 41
result.json       = 41 - 1 interrupted run without result.json            = 40
verdicts          = 31 PASS + 9 FAIL                                      = 40
PASS by group     = 20 (S01-S04) + 7 (final/) + 4 (precheck/)             = 31
FAIL by group     =  9 (S01-S04) + 0 (final/) + 0 (precheck/)             =  9
directories/group = 30 (S01-S04) + 7 (final/) + 4 (precheck/)             = 41
```

Per scenario, directories / result.json / PASS / FAIL:

| Scenario | Dirs | result.json | PASS | FAIL |
|---|---|---|---|---|
| S01 nominal | 11 | 11 | 6 | 5 |
| S02 gnss_gradual_degradation_and_outage | 7 | 6 | 5 | 1 |
| S03 lidar_geometry_degradation | 6 | 6 | 4 | 2 |
| S04 gnss_and_lidar_concurrent_degradation | 6 | 6 | 5 | 1 |
| S05 (3 FINAL + 1 PRECHECK) | 4 | 4 | 4 | 0 |
| S06 (2 FINAL + 1 PRECHECK) | 3 | 3 | 3 | 0 |
| S07 (1 FINAL + 1 PRECHECK) | 2 | 2 | 2 | 0 |
| S08 (1 FINAL + 1 PRECHECK) | 2 | 2 | 2 | 0 |
| **Total** | **41** | **40** | **31** | **9** |

The single directory without `result.json`:

```text
S02_gnss_gradual_degradation_and_outage_20260828-145410   INTERRUPTED
```

## 2. Status vocabulary used in the ledger

```text
CANONICAL             the one run per scenario a later document may cite
DUPLICATE_VALID_RERUN a valid run of the same scenario, superseded for citation
SUPERSEDED_FINAL      a FINAL-class run replaced by a later FINAL run
HISTORICAL_FAIL       a FAIL run, retained permanently as evidence
PRECHECK              RUN_CLASS=PRECHECK, NOT_FINAL_EVIDENCE=TRUE
INCOMPLETE            the run did not reach result-writing
```

```text
RULE: DUPLICATE_VALID_RERUN does not mean byte-identical. For S01-S04 it means a
valid PASS produced under an EARLIER criteria set than the canonical run. The
criteria state column carries that difference; the status word alone does not.
```

## 3. Criteria state, as recorded in the artefacts

There is no criteria version field in these runs. What each run does carry is the
set of check keys its `result.json` evaluated, and a `build_commit` in
`metadata.json`. Both are read values, so the criteria state below is evidence,
not a reconstruction. No git commit is guessed per run.

Check sets observed (names exactly as recorded in `result.json`):

| Token | Checks | Contents |
|---|---|---|
| `NO_CRITERIA_EVALUATED` | 1 | `runner_exception` only; the run aborted before criteria ran |
| `S01_SET_8` | 8 | `samples_present`, `integration_alive`, `real_cmd_vel_is_unpublished`, `finite_fusion_outputs`, `relocalization_not_unexpectedly_active`, `gnss_nominal_seen`, `lidar_nominal_seen`, `navigation_not_failed` |
| `S01_SET_8_GRACE` | 8 | `S01_SET_8` plus `evaluation_notes.relocalization_startup_grace_sec = 2.0` |
| `S01_SET_9` | 9 | `S01_SET_8_GRACE` plus `relocalization_activity_observed` |
| `S02_SET_8` | 8 | `samples_present`, `integration_alive`, `real_cmd_vel_is_unpublished`, `finite_fusion_outputs`, `relocalization_not_unexpectedly_active`, `gnss_good_to_degraded`, `gnss_rejection_seen`, `gnss_rejected_fix_not_accepted` |
| `S02_SET_9` | 9 | `S02_SET_8` plus `relocalization_activity_observed` |
| `S03_SET_8` | 8 | common four plus `relocalization_not_unexpectedly_active`, `lidar_good_seen`, `lidar_degraded_score_seen`, `lidar_state_degraded_or_rejected` |
| `S03_SET_9` | 9 | `S03_SET_8` plus `relocalization_activity_observed` |
| `S04_SET_8` | 8 | common four plus `relocalization_not_unexpectedly_active`, `gnss_rejection_seen`, `lidar_degradation_seen`, `navigation_degradation_seen` |
| `S04_SET_9` | 9 | `S04_SET_8` plus `relocalization_activity_observed` |
| `S05_SET_12` | 12 | 7 shared closed-loop checks plus 4 `s05_*` checks (`sut_reported_amcl_health_degraded`, `normal_suspected_triggered_in_order`, `trigger_reason_is_a_real_trigger_path`, `navigation_health_escalated`) |
| `S06_SET_15` | 15 | 7 shared closed-loop checks plus 7 `s06_*` checks |
| `S07_SET_14` | 14 | 7 shared closed-loop checks plus 6 `s07_*` checks |
| `S08_SET_13` | 13 | 7 shared closed-loop checks plus 5 `s08_*` checks |

The seven shared closed-loop checks present in every S05-S08 run:
`samples_present`, `integration_alive`, `real_cmd_vel_unpublished_at_start`,
`real_cmd_vel_is_unpublished`, `finite_fusion_outputs`,
`relocalization_states_are_known`, `relocalization_activity_observed`, together
with `relocalization_not_unexpectedly_active`.

`build_commit` values recorded in `metadata.json`, in run order. Subjects are read
from `git log -1` on branch `dg202611-synthetic-validation`; the mapping is not
inferred:

| build_commit | Subject | Runs built with it |
|---|---|---|
| `096bf66` | fix(ros2): repair DG navigation runtime integration | the two 20260827 S01 runs |
| `f7676ee` | feat(test): add DG synthetic degradation validation | 1436xx-1437xx sweep |
| `380cdd3` | feat(test): add validation monitor and plots | 1442xx-1443xx sweep |
| `b8b00ba` | fix(test): stop ROS processes before context shutdown | 1444xx-1445xx sweep |
| `bd33a76` | fix(test): terminate integration cleanly | 1447xx-1448xx sweep |
| `618122f` | fix(test): ignore bounded startup relocalization noise | 1450xx-1451xx sweep |
| `1a3fb4b` | fix(test): scope relocalization checks by scenario | S01 145355, S02 145410 |
| `e1fdbc7` | fix(test): apply startup grace to nominal health | S01 145527 |
| `c74e4f5` | fix(test): prewarm synthetic sensor inputs | S01 145657 |
| `fd2c60d` | fix(test): prewarm ROS graph before scenario clock | the canonical S01-S04 sweep |
| `57ec8ec` | docs(dg): add Claude Code development handoff | all 11 S05-S08 runs |

```text
NOTE: 618122f, 1a3fb4b and e1fdbc7 touch only result_writer.py, the evaluation
layer. c74e4f5 and fd2c60d touch only scenario_runner.py, the test rig start-up
sequence. The distinction matters and is kept in PACKAGE_A_FAILURE_HISTORY.md.
```

## 4. Ledger — S01 nominal

Directory prefix for every row: `results/synthetic/`. Timestamps are
`metadata.json:started_at_utc`. Verdict is `result.json:result`.

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S01 | `S01_nominal_20260827-204855` | 2026-08-27T12:48:55Z | present | FAIL | `096bf66` | `NO_CRITERIA_EVALUATED` | HISTORICAL_FAIL | not superseded; retained permanently | 0 samples; `RUNNER_EXCEPTION` DiagnosticStatus level type error; no `samples.csv`, no `plots/`; rosbag and `logs/` retained |
| S01 | `S01_nominal_20260827-205016` | 2026-08-27T12:50:16Z | present | FAIL | `096bf66` | `S01_SET_8` | HISTORICAL_FAIL | not superseded; retained permanently | 101 samples; `gnss_nominal_seen` false and `relocalization_not_unexpectedly_active` false; reference case named in `EXPERIMENT_EVIDENCE_POLICY.md` section 6 |
| S01 | `S01_nominal_20260828-143601` | 2026-08-28T06:36:01Z | present | PASS | `f7676ee` | `S01_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145833` | 76 samples; 8/8 true; predates the S01 9-check set; `git_status` records uncommitted result_writer/scenario_runner edits at run time |
| S01 | `S01_nominal_20260828-144227` | 2026-08-28T06:42:27Z | present | PASS | `380cdd3` | `S01_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145833` | 79 samples; 8/8 true; clean `git_status` |
| S01 | `S01_nominal_20260828-144455` | 2026-08-28T06:44:55Z | present | PASS | `b8b00ba` | `S01_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145833` | 79 samples; 8/8 true |
| S01 | `S01_nominal_20260828-144747` | 2026-08-28T06:47:47Z | present | PASS | `bd33a76` | `S01_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145833` | 79 samples; 8/8 true; transient GNSS `GOOD -> REJECTED -> GOOD` inside the first 0.8 s |
| S01 | `S01_nominal_20260828-145033` | 2026-08-28T06:50:33Z | present | PASS | `618122f` | `S01_SET_8_GRACE` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145833` | 79 samples; first run carrying `evaluation_notes.relocalization_startup_grace_sec = 2.0` and `startup_samples_retained = true` |
| S01 | `S01_nominal_20260828-145355` | 2026-08-28T06:53:55Z | present | FAIL | `1a3fb4b` | `S01_SET_9` | HISTORICAL_FAIL | not superseded; retained permanently | 79 samples; `relocalization_not_unexpectedly_active` and `navigation_not_failed` both false; `Navigation: RECOVERING -> FAILED @ 0.230 s` recorded in `important_transitions` |
| S01 | `S01_nominal_20260828-145527` | 2026-08-28T06:55:27Z | present | FAIL | `e1fdbc7` | `S01_SET_9` | HISTORICAL_FAIL | not superseded; retained permanently | 79 samples; `relocalization_not_unexpectedly_active` false only; `Relocalization: FAILED -> STOPPING @ 0.237 s` |
| S01 | `S01_nominal_20260828-145657` | 2026-08-28T06:56:57Z | present | FAIL | `c74e4f5` | `S01_SET_9` | HISTORICAL_FAIL | not superseded; retained permanently | 79 samples; `relocalization_not_unexpectedly_active` false; `important_transitions` empty, so the activity is in the per-sample state column rather than in a transition |
| S01 | `S01_nominal_20260828-145833` | 2026-08-28T06:58:33Z | present | PASS | `fd2c60d` | `S01_SET_9` | **CANONICAL** | supersedes the five earlier S01 PASS runs for citation | 79 samples; `relocalization_activity_observed` false and `relocalization_not_unexpectedly_active` true, i.e. no relocalization activity at all in nominal; 0 errors, 0 warnings; the run named in `results/synthetic/summary.csv` |

```text
S01 note: relocalization_activity_observed is informational, not a gate. In the
canonical S01 run it is FALSE and the run still PASSES, because for a nominal
scenario the absence of relocalization activity is the expected reading.
```

## 5. Ledger — S02 gnss_gradual_degradation_and_outage

Directory prefix: `results/synthetic/`. Scenario duration 16.0 s, seed 20261102.

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S02 | `S02_..._20260828-143653` | 2026-08-28T06:36:53Z | present | PASS | `f7676ee` | `S02_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145858` | 159 samples; 8/8 true |
| S02 | `S02_..._20260828-144243` | 2026-08-28T06:42:43Z | present | PASS | `380cdd3` | `S02_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145858` | 159 samples; 8/8 true |
| S02 | `S02_..._20260828-144511` | 2026-08-28T06:45:11Z | present | PASS | `b8b00ba` | `S02_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145858` | 159 samples; 8/8 true; startup GNSS `REJECTED -> RECOVERING -> GOOD` inside 0.5 s |
| S02 | `S02_..._20260828-144803` | 2026-08-28T06:48:03Z | present | PASS | `bd33a76` | `S02_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145858` | 157 samples; 8/8 true; startup `Navigation: RECOVERING -> FAILED -> RECOVERING` inside 0.7 s |
| S02 | `S02_..._20260828-145049` | 2026-08-28T06:50:49Z | present | FAIL | `618122f` | `S02_SET_8` | HISTORICAL_FAIL | not superseded; retained permanently | 131 samples; `relocalization_not_unexpectedly_active` false; relocalization `NORMAL -> STOPPING` at 10.65 s, i.e. after the GNSS `REJECTED` step at 8.10 s, not in the startup window |
| S02 | `S02_..._20260828-145410` | 2026-08-28T06:54:10Z | **absent** | none recorded | `1a3fb4b` | not evaluated | INCOMPLETE | not superseded; retained permanently | run interrupted; `metadata.json`, `scenario.yaml`, `logs/`, `rosbag/` (2.8 MB) retained; no `samples.csv`, no `timeline.csv`, no `plots/`, no `result.md`; recorder log shows recording stopped about 12 s into a 16 s scenario |
| S02 | `S02_..._20260828-145858` | 2026-08-28T06:58:58Z | present | PASS | `fd2c60d` | `S02_SET_9` | **CANONICAL** | supersedes the four earlier S02 PASS runs for citation | 159 samples; 9/9 true; 0 errors; carries warning `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE`; only two transitions recorded, GNSS `GOOD -> DEGRADED` at 4.107 s and `DEGRADED -> REJECTED` at 8.107 s; the run named in `results/synthetic/summary.csv` |

## 6. Ledger — S03 lidar_geometry_degradation

Directory prefix: `results/synthetic/`. Scenario duration 12.0 s.

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S03 | `S03_..._20260828-143723` | 2026-08-28T06:37:23Z | present | PASS | `f7676ee` | `S03_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145928` | 119 samples; 8/8 true |
| S03 | `S03_..._20260828-144306` | 2026-08-28T06:43:06Z | present | PASS | `380cdd3` | `S03_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145928` | 117 samples; 8/8 true |
| S03 | `S03_..._20260828-144533` | 2026-08-28T06:45:33Z | present | PASS | `b8b00ba` | `S03_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145928` | 119 samples; 8/8 true |
| S03 | `S03_..._20260828-144826` | 2026-08-28T06:48:26Z | present | FAIL | `bd33a76` | `S03_SET_8` | HISTORICAL_FAIL | not superseded; retained permanently | 119 samples; `relocalization_not_unexpectedly_active` false; startup churn `Navigation: FAILED -> NOMINAL -> FAILED -> RECOVERING` and relocalization `STOPPING -> NORMAL -> STOPPING` inside the first 0.8 s |
| S03 | `S03_..._20260828-145111` | 2026-08-28T06:51:11Z | present | FAIL | `618122f` | `S03_SET_8` | HISTORICAL_FAIL | not superseded; retained permanently | 119 samples; `relocalization_not_unexpectedly_active` false; `Relocalization: FAILED -> STOPPING @ 0.206 s`, then repeated `Navigation: NOMINAL <-> RECOVERING` oscillation from 1.8 s to 2.5 s |
| S03 | `S03_..._20260828-145928` | 2026-08-28T06:59:28Z | present | PASS | `fd2c60d` | `S03_SET_9` | **CANONICAL** | supersedes the three earlier S03 PASS runs for citation | 119 samples; 9/9 true; 0 errors; warning `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE`; only two transitions, LiDAR `GOOD -> DEGRADED` at 4.108 s and `DEGRADED -> REJECTED` at 8.108 s; the run named in `results/synthetic/summary.csv` |

## 7. Ledger — S04 gnss_and_lidar_concurrent_degradation

Directory prefix: `results/synthetic/`. Scenario duration 16.0 s, seed 20261104.

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S04 | `S04_..._20260828-143751` | 2026-08-28T06:37:51Z | present | PASS | `f7676ee` | `S04_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145956` | 159 samples; 8/8 true |
| S04 | `S04_..._20260828-144324` | 2026-08-28T06:43:24Z | present | PASS | `380cdd3` | `S04_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145956` | 159 samples; 8/8 true; long fusion `LIDAR_AIDED <-> NOMINAL <-> DEAD_RECKONING` chatter recorded from 0.7 s |
| S04 | `S04_..._20260828-144552` | 2026-08-28T06:45:52Z | present | PASS | `b8b00ba` | `S04_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145956` | 159 samples; 8/8 true |
| S04 | `S04_..._20260828-144844` | 2026-08-28T06:48:44Z | present | FAIL | `bd33a76` | `S04_SET_8` | HISTORICAL_FAIL | not superseded; retained permanently | 157 samples; `relocalization_not_unexpectedly_active` false; run opens in `Navigation: LOCALIZATION_SUSPECT` and `Relocalization: FAILED -> STOPPING @ 0.324 s` |
| S04 | `S04_..._20260828-145130` | 2026-08-28T06:51:30Z | present | PASS | `618122f` | `S04_SET_8` | DUPLICATE_VALID_RERUN | superseded for citation by `20260828-145956` | 157 samples; 8/8 true; first S04 run carrying the 2.0 s startup grace in `evaluation_notes`; relocalization `NORMAL -> STOPPING -> NORMAL` at 1.11-1.21 s falls inside that grace window |
| S04 | `S04_..._20260828-145956` | 2026-08-28T06:59:56Z | present | PASS | `fd2c60d` | `S04_SET_9` | **CANONICAL** | supersedes the four earlier S04 PASS runs for citation | 159 samples; 9/9 true; 0 errors; warning `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE`; 20 transitions recorded including relocalization `NORMAL <-> STOPPING` at 1.1 s, 2.2 s and 4.2 s; the run named in `results/synthetic/summary.csv` |

## 8. Ledger — FINAL runs, S05 to S08

Directory prefix: `results/synthetic/final/`. Every row carries
`run_class = FINAL`, `not_final_evidence = false`,
`not_for_document_claim = false` inside both `result.json` and `metadata.json`,
and `run_class_markers.EVIDENCE_LEVEL = "SYNTHETIC SOFTWARE VALIDATION"`.
All seven were built at `57ec8ec`.

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S05 | `S05_..._20260829-141521` | 2026-08-29T06:15:21Z | present | PASS | `57ec8ec` | `S05_SET_12` (12/12 true) | DUPLICATE_VALID_RERUN | superseded for citation by `20260829-160001` | 239 samples, 24.0 s; 9 PNG plots plus `SKIPPED_PLOTS.txt`; no `evidence/` directory; reason for supersession is recorded in `06_results_and_evidence.md`: the marker node still carried pre-staleness code |
| S05 | `S05_..._20260829-150349` | 2026-08-29T07:03:49Z | present | PASS | `57ec8ec` | `S05_SET_12` (12/12 true) | DUPLICATE_VALID_RERUN | superseded for citation by `20260829-160001` | 239 samples; no `evidence/` directory; reason recorded in `06_results_and_evidence.md`: the staleness fix was built but the running marker node had not been restarted |
| S05 | `S05_..._20260829-160001` | 2026-08-29T08:00:01Z | present | PASS | `57ec8ec` | `S05_SET_12` (12/12 true) | **CANONICAL** | supersedes the two earlier S05 FINAL runs for citation | 239 samples; `reloc: NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING` with `trigger_reason = AMCL_COVARIANCE_HIGH`; no `evidence/` directory, so `S05_GUI_CAPTURE: NOT_CAPTURED` stands as a capture omission, never as `NOT_OBSERVED` |
| S06 | `S06_..._20260829-161050` | 2026-08-29T08:10:50Z | present | PASS | `57ec8ec` | `S06_SET_15` (15/15 true) | SUPERSEDED_FINAL | superseded for citation by `20260829-165043` | 392 samples, 40.0 s; PASS, and it stays on the record as PASS; no `evidence/` directory, which is the artefact-level difference from the canonical run |
| S06 | `S06_..._20260829-165043` | 2026-08-29T08:50:43Z | present | PASS | `57ec8ec` | `S06_SET_15` (15/15 true) | **CANONICAL** | supersedes `20260829-161050` for citation | 399 samples; `evidence/` holds 5 labelled PNG captures plus `evidence_manifest.csv` and `evidence_manifest.md`; 10 PNG plots including `scan_map_quality.png` |
| S07 | `S07_..._20260829-171148` | 2026-08-29T09:11:48Z | present | PASS | `57ec8ec` | `S07_SET_14` (14/14 true) | **CANONICAL** | only S07 FINAL run | 159 samples, 16.0 s; `evidence/` holds 6 labelled PNG captures, `evidence_manifest.csv`, `evidence_manifest.md` and the `dg_label_captures.py` used to label them |
| S08 | `S08_..._20260829-171722` | 2026-08-29T09:17:22Z | present | PASS | `57ec8ec` | `S08_SET_13` (13/13 true) | **CANONICAL** | only S08 FINAL run | 239 samples, 24.0 s; `evidence/` holds 6 labelled PNG captures, both manifests, `dg_capture_loop.sh` and `dg_label_captures.py` |

## 9. Ledger — PRECHECK runs, S05 to S08

Directory prefix: `results/synthetic/precheck/`. All four were built at `57ec8ec`
and all four record, inside `result.json` and `metadata.json`:

```text
RUN_CLASS:              PRECHECK
NOT_FINAL_EVIDENCE:     TRUE
NOT_FOR_DOCUMENT_CLAIM: TRUE
EVIDENCE_LEVEL:         SYNTHETIC SOFTWARE VALIDATION PRECHECK
```

```text
THESE FOUR RUNS ARE EXPLICITLY NOT_FINAL_EVIDENCE.
They may not be cited in a proposal, report, presentation, or claim of any kind.
A precheck shows the chain runs end to end and the criteria evaluate against real
recorded columns. That is all it shows.
```

| Scenario | run_dir | started_at_utc | result_json | pass_fail | build_commit | criteria_version_or_state | canonical_status | superseded_status | notes |
|---|---|---|---|---|---|---|---|---|---|
| S05 | `precheck/S05_..._20260829-120010` | 2026-08-29T04:00:10Z | present | PASS | `57ec8ec` | `S05_SET_12` (12/12 true) | PRECHECK | not a citation candidate at any time | 239 samples; `NOT_FINAL_EVIDENCE=TRUE`; headless, no `evidence/` directory |
| S06 | `precheck/S06_..._20260829-120206` | 2026-08-29T04:02:06Z | present | PASS | `57ec8ec` | `S06_SET_15` (15/15 true) | PRECHECK | not a citation candidate at any time | 387 samples; `NOT_FINAL_EVIDENCE=TRUE`; note 387 here versus 399 in the canonical S06 FINAL run |
| S07 | `precheck/S07_..._20260829-120415` | 2026-08-29T04:04:15Z | present | PASS | `57ec8ec` | `S07_SET_14` (14/14 true) | PRECHECK | not a citation candidate at any time | 159 samples; `NOT_FINAL_EVIDENCE=TRUE` |
| S08 | `precheck/S08_..._20260829-120513` | 2026-08-29T04:05:13Z | present | PASS | `57ec8ec` | `S08_SET_13` (13/13 true) | PRECHECK | not a citation candidate at any time | 239 samples; `NOT_FINAL_EVIDENCE=TRUE` |

## 10. Canonical designation for S01-S04, and why

S01-S04 have no `final/` directory and carry no `RUN_CLASS` field at all, so the
canonical run had to be determined from the artefacts. The designation is:

| Scenario | Canonical run |
|---|---|
| S01 | `S01_nominal_20260828-145833` |
| S02 | `S02_gnss_gradual_degradation_and_outage_20260828-145858` |
| S03 | `S03_lidar_geometry_degradation_20260828-145928` |
| S04 | `S04_gnss_and_lidar_concurrent_degradation_20260828-145956` |

Four independent reasons from the artefacts, each checkable on its own:

1. **The runner's own summary points at exactly these four.**
   `results/synthetic/summary.csv` and `summary.md` list one row per scenario, and
   the `artifact_directory` field in each row is one of the four above. No other
   S01-S04 run appears in either file.
2. **They are the latest PASS per scenario.** For each of S01-S04 no later
   directory of that scenario exists.
3. **They are the only complete four-scenario sweep at the last build.** All four
   record `build_commit = fd2c60d`, and `fd2c60d` is the last commit before the
   S05-S08 work began at `57ec8ec`. Every earlier sweep either contains a FAIL, is
   missing a scenario, or both.
4. **They are the only S01-S04 runs evaluated against the 9-check sets.** Each of
   the four carries its scenario's `*_SET_9`, which is the widest criteria set any
   S01-S04 run was evaluated against, and each passes it with 0 errors.

```text
RULE: designating a canonical run does not delete or downgrade the others. The
five earlier S01 passes, four earlier S02 passes, three earlier S03 passes and
four earlier S04 passes remain valid runs and remain on this ledger. Canonical
means "cite this one", not "the others did not happen".
```

## 11. The one incomplete run

```text
DIR:      results/synthetic/S02_gnss_gradual_degradation_and_outage_20260828-145410
STATUS:   INCOMPLETE
VERDICT:  none. There is no verdict, and none may be assigned to it later.
```

Present: `metadata.json`, `scenario.yaml`, `logs/integration.log`,
`logs/rosbag.log`, `rosbag/metadata.yaml`, `rosbag/rosbag_0.db3` (2.8 MB).
Absent: `result.json`, `result.md`, `samples.csv`, `timeline.csv`, `plots/`.

`logs/rosbag.log` records `Recording stopped` about 12 s after the recorder
subscribed, against a 16.0 s scenario duration in `scenario.yaml`, so the run was
cut short before the evaluator wrote a result. The next run in the sequence,
`S01_nominal_20260828-145527`, starts 77 s later.

```text
This directory counts as a run directory and does NOT count as a verdict. That is
the whole reason 41 and 40 differ. An interrupted run is neither a PASS that went
unrecorded nor a FAIL: it is an absence, and it is recorded as an absence.
```

## 12. Schema drift across the run set, for anyone reading raw JSON

Three drifts exist between the 2026-08-27 runs and the 2026-08-29 runs. None of
them changes any verdict, and all three will trip a naive reader of `result.json`:

| Field | 2026-08-27 runs | 2026-08-28 runs | 2026-08-29 runs |
|---|---|---|---|
| `performance_claim` | string `NOT_COMPETITION_PERFORMANCE_EVIDENCE` | boolean `false`, with the string moved to `performance_claim_label` | as 2026-08-28 |
| `run_class` | absent | absent | `FINAL` or `PRECHECK`, plus `run_class_markers` |
| `authenticity_marker` | absent | `THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION` | same |

```text
CAUTION: performance_claim = false means "this is NOT competition performance
evidence". Read as a bare boolean it inverts. Read performance_claim_label.
```

S01-S04 runs carry no `run_class` field. Their absence of a `FINAL` marker is not
a defect of those runs; the field did not exist yet. It does mean the S01-S04
canonical designation in section 10 rests on the four artefact-based reasons given
there and not on any in-file run-class marker.

## 13. Claims this inventory does not support

```text
POSITIONING_ACCURACY_CLAIM:   NOT_SUPPORTED, and deliberately not restated here
FEATURE_REPEATABILITY_CLAIM:  NOT_SUPPORTED, and deliberately not restated here
RELOCALIZATION_SUCCESS_RATE:  NOT_SUPPORTED, and deliberately not restated here
GAZEBO:                       no Gazebo, no physics simulator, NOT_RUN
PHYSICAL_VEHICLE:             no real robot, no physical robot, NOT_RUN
COMPETITION_PERFORMANCE:      not competition performance, NOT_PROVEN
```

The three suppressed lines are the metric targets whose numeric forms are written
out once, as unproven, in `06_results_and_evidence.md` section 10. They are not
repeated here, because a numeral copied out of a ledger row travels without the
row.

31 PASS verdicts across 41 directories move none of those lines. Every run in this
ledger is synthetic software validation whose input, and whose true pose, are
authored by the test rig. Scenario and check names containing the word "real" (for
example `s07_seed_requested_by_real_supervisor`) refer to real code paths in the
unmodified software; no real robot and no real sensor data are involved anywhere in
this set.

## 14. Discrepancies found against existing documents

Reported, not edited. These files belong to other owners.

1. `docs/dg202611/06_results_and_evidence.md` header block states
   `ROUND_B_FINAL_EVIDENCE: NOT_YET_RUN` and
   `NO_FINAL_EVIDENCE_SCENARIO_EXECUTED: YES`, and its section 5 lists all four of
   S05-S08 as `FINAL EVIDENCE: NOT_YET_RUN`. Seven directories under
   `results/synthetic/final/` carry `run_class = FINAL` and
   `not_final_evidence = false` inside `result.json`. The same file's appendix,
   added later, documents three of them. The header and section 5 are stale
   relative to both the disk and the file's own appendix.
2. `results/synthetic/summary.csv` and `summary.md` describe only the four
   canonical S01-S04 runs. They are correct for what they cover, and they are not
   a summary of the run set: 37 of the 41 directories do not appear in them.

```text
NO RUN DIRECTORY IN THIS LEDGER MAY BE DELETED, MOVED, OR REWRITTEN.
This includes the 9 FAIL runs, the interrupted run, the superseded FINAL runs and
the 4 prechecks. Their failure history is recorded in
docs/dg202611/PACKAGE_A_FAILURE_HISTORY.md.
```

---

## Appendix — check totals in this ledger predate the non-discriminating fix

Added during Package A bookkeeping closeout, after the ledger tables above were
written, then corrected once the figures were recomputed from the canonical runs'
actual `samples.csv` rather than from a synthetic test row.

The `criteria_version_or_state` column throughout this file quotes the totals each
run's own `result.json` recorded at execution time.  Those are reproduced
faithfully and are **not** edited, because a ledger of historical runs must report
what those runs actually recorded.

For every scenario **except S01**, those totals included one check that could not
fail.  `relocalization_not_unexpectedly_active` was computed as
`not relocation_active if scenario_id == "S01" else True` — hardcoded `True`
outside S01 while still occupying a slot in the counted total.  The evaluation
code has since been corrected: it is a counted check for S01 only, and elsewhere
it is reported under `non_discriminating_checks` with its reason, classified
`NON_DISCRIMINATING_CHECK`.

Recomputed by re-evaluating each canonical run's recorded `samples.csv` with the
corrected code:

| scenario | canonical run | checks recorded at execution | counted after fix | non-discriminating after fix |
|---|---|---|---|---|
| S01 | `S01_nominal_20260828-145833` | 9 | 11 | **0** |
| S05 | `S05_..._20260829-160001` | 12 | 11 | 1 |
| S06 | `S06_..._20260829-165043` | 15 | 14 | 1 |
| S07 | `S07_..._20260829-171148` | 14 | 13 | 1 |
| S08 | `S08_..._20260829-171722` | 13 | 12 | 1 |

All five re-evaluate to `result = PASS`, so the fix changes reported coverage and
changes no verdict.

Two things this table must not be misread as:

1. **S01 was never inflated.** For S01 the check is a genuine assertion — the
   nominal control case must not activate recovery — so its recorded 9 were 9
   real checks and its non-discriminating count is 0.
2. **The "counted after fix" column is not a like-for-like comparison.** S01
   rises from 9 to 11 because the current evaluation code carries checks that did
   not exist when S01 ran (for example `relocalization_states_are_known` and
   `real_cmd_vel_unpublished_at_start`, both added for S05-S08). That is newer
   criteria applied to older data, not a correction of the S01 run.

Consequence for citation: quoting `15/15` or `12/12` as evidence of coverage
breadth requires this caveat, because one of those slots could not fail. A reader
comparing "15/15" against "14 discriminating + 1 inert" would otherwise
overestimate what was asserted.

No run needs re-running. The inert check never masked a failure — it could only
inflate the apparent count, and every genuine assertion was evaluated normally.
