# DG-202611 Package A — Failure History

```text
EVIDENCE_CLASS = SYNTHETIC_SOFTWARE_VALIDATION
REAL_ROBOT_DATA = FALSE
COMPETITION_METRIC_EVIDENCE = FALSE
```

```text
SCOPE:        the 9 FAIL runs among the 41 run directories under
              /home/weiyang/dg202611_ws/results/synthetic
COMPANION:    docs/dg202611/PACKAGE_A_RUN_INVENTORY.md holds the full ledger
SOURCE:       result.json errors and checks, metadata.json build_commit,
              logs/integration.log, and git log on branch
              dg202611-synthetic-validation
```

```text
STANDING RULE
No failure record may be deleted. A failed run is evidence. It is not noise, not a
false start, and not something a later pass replaces. The 9 directories below stay
on disk with their rosbag, CSV, logs and result.json intact, permanently.
```

## 1. The nine failures at a glance

| # | run_dir | Scenario | build_commit | Recorded errors | Classification | State |
|---|---|---|---|---|---|---|
| F1 | `S01_nominal_20260827-204855` | S01 | `096bf66` | `RUNNER_EXCEPTION: AssertionError: The 'level' field must be of type 'bytes' or 'ByteString' with length 1` | TOOLING_OR_EVIDENCE_BUG | RESOLVED |
| F2 | `S01_nominal_20260827-205016` | S01 | `096bf66` | `UNEXPECTED_RELOCALIZATION_STATE`, `GNSS_NOMINAL_SEEN` | TOOLING_OR_EVIDENCE_BUG plus CRITERION_CHANGE | RESOLVED |
| F3 | `S01_nominal_20260828-145355` | S01 | `1a3fb4b` | `UNEXPECTED_RELOCALIZATION_STATE`, `NAVIGATION_NOT_FAILED` | CRITERION_CHANGE plus ENVIRONMENT | SUPERSEDED |
| F4 | `S01_nominal_20260828-145527` | S01 | `e1fdbc7` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE plus ENVIRONMENT | SUPERSEDED |
| F5 | `S01_nominal_20260828-145657` | S01 | `c74e4f5` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE plus ENVIRONMENT | SUPERSEDED |
| F6 | `S02_..._20260828-145049` | S02 | `618122f` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE | SUPERSEDED |
| F7 | `S03_..._20260828-144826` | S03 | `bd33a76` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE | SUPERSEDED |
| F8 | `S03_..._20260828-145111` | S03 | `618122f` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE | SUPERSEDED |
| F9 | `S04_..._20260828-144844` | S04 | `bd33a76` | `UNEXPECTED_RELOCALIZATION_STATE` | CRITERION_CHANGE | SUPERSEDED |

```text
FAIL runs in final/:     0
FAIL runs in precheck/:  0
All 9 failures are S01-S04, all on 2026-08-27 and 2026-08-28.
```

Classification vocabulary, used with one meaning only:

```text
CRITERION_CHANGE          the criterion that failed was later changed or scoped
RUNTIME_BUG               a defect in the software under test
TOOLING_OR_EVIDENCE_BUG   a defect in the test rig or the evidence chain
GENUINE_SUT_BEHAVIOUR     the software did the wrong thing and still would
ENVIRONMENT               host, ROS graph, timing or start-up ordering
```

```text
RESOLVED    root cause identified and a fix landed
SUPERSEDED  a later run of the same scenario passed under the current criteria,
            and the failing criterion no longer exists in that form
OPEN        neither of the above
```

## 2. F1 — the DiagnosticStatus level type error

```text
RUN:        results/synthetic/S01_nominal_20260827-204855
VERDICT:    FAIL          SAMPLES: 0          CHECKS: 1 (runner_exception only)
ERROR:      RUNNER_EXCEPTION: AssertionError: The 'level' field must be of type
            'bytes' or 'ByteString' with length 1
CLASS:      TOOLING_OR_EVIDENCE_BUG
STATE:      RESOLVED
```

Probable cause, verified against the code as it stands: ROS 2 Humble's generated
Python message exposes `diagnostic_msgs/msg/DiagnosticStatus.level`, a `uint8`, as
a one-byte `bytes` value, not an `int`. The synthetic injector assigned an integer,
and `rclpy` rejected the assignment before any sample was collected. The run died
at 0 samples, so no criteria were evaluated at all: `result.json` contains the
single key `runner_exception`.

What the artefacts confirm and what they do not:

```text
CONFIRMED:  0 samples, no samples.csv, no plots/, no timeline.csv
CONFIRMED:  logs/integration.log records SIGINT immediately after the seven DG
            nodes report ready, then all seven dying with rclpy context errors
NOT IN LOG: the AssertionError itself. It was raised in the runner process, and
            integration.log captures only the launch process output.
```

What changed afterwards:

```text
src/dg_synthetic_validation/.../synthetic_injector_node.py
    status.level = bytes([max(0, min(255, int(phase.diagnostic_level)))])
    with an inline comment recording why the cast is required

d707193  fix(ros2): normalize diagnostic status levels   2026-08-28 14:41
    src/ylhb_base/scripts/diagnostic_level.py            new, normalize_diagnostic_level()
    src/ylhb_base/scripts/gnss_quality_node.py           reader normalised, default=3
    src/ylhb_base/scripts/navigation_health_node.py      reader normalised, default=3
    src/ylhb_base/scripts/multisource_fusion_node.py     reader normalised, default=3
    src/ylhb_base/test/test_diagnostic_level.py          new unit test
    src/ylhb_base/docs/diagnostic_level_compatibility.md new note
```

Both sides needed the fix: the injector had to write the byte form, and the three
consuming nodes had to accept `bytes`, `int` or `str` and fail closed on anything
else. `d707193` is an ancestor of `HEAD` and lands 2026-08-28 14:41, so both
2026-08-27 runs predate it.

```text
NOTE ON CLASSIFICATION: the write-side defect is in the test rig, which makes this
TOOLING_OR_EVIDENCE_BUG. The read-side half of the same fix is in src/ylhb_base,
which is software under test. Neither half is a behavioural defect in the
localization or fusion logic.
```

## 3. F2 — the same type bug, seen through the GNSS gate

```text
RUN:        results/synthetic/S01_nominal_20260827-205016
VERDICT:    FAIL          SAMPLES: 101          CHECKS: 8, of which 6 true
ERRORS:     UNEXPECTED_RELOCALIZATION_STATE, GNSS_NOMINAL_SEEN
FALSE:      gnss_nominal_seen, relocalization_not_unexpectedly_active
CLASS:      TOOLING_OR_EVIDENCE_BUG (gnss_nominal_seen)
            plus CRITERION_CHANGE (relocalization)
STATE:      RESOLVED
```

This run did collect 101 samples, 81 s after F1 and at the same `build_commit`
`096bf66`. The runner code lived in an untracked package at that point
(`metadata.json:git_status` reads `?? src/dg_synthetic_validation/`), so the rig
could change between the two runs without the recorded build commit changing. That
is why F1 aborted and F2 did not.

`gnss_nominal_seen` false is consistent with the same `DiagnosticStatus.level`
defect, on the read side. `src/ylhb_base/scripts/gnss_quality_node.py` derives its
verdict from the incoming diagnostic level; with the level unreadable the gate
could not report `GOOD`, so no nominal GNSS sample was ever seen in a scenario whose
entire first phase is nominal GNSS. After `d707193` that read path calls
`normalize_diagnostic_level(selected.level, default=3)`, which accepts the byte
form and fails closed rather than mis-reading.

```text
The GNSS gate never reporting GOOD across an 8 s nominal run is not a fusion
result and was never treated as one. It is an unreadable input field.
```

The second error, `UNEXPECTED_RELOCALIZATION_STATE`, is the same criterion that
produces F3 to F9 and is treated with them in section 4.

```text
RETENTION: this directory is the reference case named in
EXPERIMENT_EVIDENCE_POLICY.md section 6. Its rosbag, CSV, timeline and logs are
immutable. It is the first ROS 2 synthetic-runtime evidence of the compatibility
blocker, and its value comes precisely from being a FAIL.
```

## 4. F3 to F9 — `UNEXPECTED_RELOCALIZATION_STATE`

```text
RUNS:   S01 145355, S01 145527, S01 145657,
        S02 145049, S03 144826, S03 145111, S04 144844
CHECK:  relocalization_not_unexpectedly_active = false in all seven
CLASS:  CRITERION_CHANGE, with ENVIRONMENT as a contributing cause in the S01 runs
STATE:  SUPERSEDED
```

Probable cause: the relocalization supervisor reacted to degraded inputs and to
start-up transients, while the then-current criterion treated any relocalization
activity as a failure regardless of scenario or of when in the run it occurred. The
recorded transitions show the two distinct shapes:

| Run | What the transitions show |
|---|---|
| S01 145355 | `Navigation: RECOVERING -> FAILED @ 0.230 s`, `Relocalization: FAILED -> NORMAL` at the same instant, `Fusion: DEAD_RECKONING -> INITIALIZING`; all inside the first third of a second |
| S01 145527 | `Relocalization: FAILED -> STOPPING @ 0.237 s`, then GNSS `GOOD -> RECOVERING -> GOOD` by 0.537 s |
| S01 145657 | `important_transitions` is empty; the activity is in the per-sample state column only |
| S03 144826 | `Navigation: FAILED -> NOMINAL -> FAILED -> RECOVERING` and `Relocalization: STOPPING -> NORMAL -> STOPPING` inside 0.8 s |
| S03 145111 | `Relocalization: FAILED -> STOPPING @ 0.206 s`, then `Navigation: NOMINAL <-> RECOVERING` oscillating from 1.8 s to 2.5 s |
| S04 144844 | run opens in `Navigation: LOCALIZATION_SUSPECT`; `Relocalization: FAILED -> STOPPING @ 0.324 s` |
| S02 145049 | the outlier: relocalization `NORMAL -> STOPPING` at **10.65 s**, after the GNSS `REJECTED` step at 8.10 s, so this one is not a start-up transient |

Two different things are therefore inside one error token. Six of the seven are
start-up transients in the first second; S02 145049 is activity in the degraded
phase. The criterion did not distinguish them.

What changed afterwards. Three commits touch only
`src/dg_synthetic_validation/dg_synthetic_validation/result_writer.py`, the
evaluation layer:

| Commit | Date | Subject | Effect on the criteria |
|---|---|---|---|
| `618122f` | 2026-08-28 14:50 | fix(test): ignore bounded startup relocalization noise | introduces `evaluation_notes.relocalization_startup_grace_sec = 2.0` with `startup_samples_retained = true`; first visible in the 1450xx sweep |
| `1a3fb4b` | 2026-08-28 14:53 | fix(test): scope relocalization checks by scenario | adds the ninth check `relocalization_activity_observed`, splitting "activity happened" from "activity is a failure"; first visible in S01 145355 |
| `e1fdbc7` | 2026-08-28 14:55 | fix(test): apply startup grace to nominal health | extends the grace window to the nominal health check; first visible in S01 145527 |

```text
startup_samples_retained = true is the part that keeps this honest. The grace
window changes how the first 2.0 s are JUDGED. It does not delete those samples
from samples.csv, so the transients above remain readable in every run.
```

Two further commits are **not** criteria changes and must not be described as
such. Both touch only `scenario_runner.py`, the rig start-up sequence:

| Commit | Date | Subject |
|---|---|---|
| `c74e4f5` | 2026-08-28 14:56 | fix(test): prewarm synthetic sensor inputs |
| `fd2c60d` | 2026-08-28 14:58 | fix(test): prewarm ROS graph before scenario clock |

This distinction is load-bearing. S01 still FAILED at `e1fdbc7` (F4) and still
FAILED at `c74e4f5` (F5), after two of the three criteria commits had landed. S01
first passed at `fd2c60d`, a rig start-up fix. So the criteria changes alone did
not turn S01 green: the ROS-graph prewarm did, by removing the start-up transient
rather than by excusing it. That is why `ENVIRONMENT` sits beside
`CRITERION_CHANGE` on F3, F4 and F5.

Status of the criterion in the canonical runs:

| Canonical run | `relocalization_activity_observed` | `relocalization_not_unexpectedly_active` | Warning |
|---|---|---|---|
| S01 145833 | false | true | none |
| S02 145858 | true | true | `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` |
| S03 145928 | true | true | `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` |
| S04 145956 | true | true | `RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` |

The activity that failed F6 to F9 is still recorded in the canonical S02-S04 runs.
It is reported as a warning rather than suppressed.

```text
RULE: RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE is a warning about bounded
activity on a status topic. It is not closed-loop recovery success, and the runs
carrying it are not evidence of recovery. See 06_results_and_evidence.md section 8.
```

## 5. F3's second error — `NAVIGATION_NOT_FAILED`

```text
RUN:    results/synthetic/S01_nominal_20260828-145355
CHECK:  navigation_not_failed = false
CLASS:  CRITERION_CHANGE plus ENVIRONMENT
STATE:  SUPERSEDED
```

The only run of the nine where a second gating check failed. `important_transitions`
records `Navigation: RECOVERING -> FAILED @ 0.230 s`, so navigation was already in
`RECOVERING` when the scenario clock started and reached `FAILED` 230 ms in. In a
nominal scenario the criterion is correct to object; what it could not do was tell a
node that had not finished starting from a node that had genuinely failed.
`e1fdbc7` applied the same 2.0 s grace window to the nominal health check, and
`c74e4f5` and `fd2c60d` removed the transient at source by prewarming the sensor
inputs and the ROS graph before the scenario clock starts.

```text
Neither the criterion change nor the prewarm asserts that navigation was healthy in
this run. It was in FAILED at 0.230 s and the samples.csv for this run still says so.
```

## 6. Post-hoc criteria evolution — the honest statement

```text
CRITERIA_PRE_REGISTERED:            NO
POST_HOC_CRITERIA_EVOLUTION_RISK:   PRESENT AND ACKNOWLEDGED
```

The criteria for S01-S04 were not pre-registered. They were written, run, found to
be wrong about start-up transients and about scenario scope, and then changed, all
inside the same development session. Three of those changes (`618122f`, `1a3fb4b`,
`e1fdbc7`) landed between 14:50 and 14:55 on 2026-08-28, in among the runs they
affect. That is post-hoc criteria evolution, and it is a real weakness of this
evidence set. It is recorded here rather than smoothed over.

What the timestamps also settle, and this is a separate statement:

```text
Last criteria change:            e1fdbc7, 2026-08-28 14:55
Canonical S01-S04 runs started:  14:58:33, 14:58:58, 14:59:28, 14:59:56
```

The canonical S01-S04 runs postdate the final criteria change. No criterion was
changed after a canonical run in order to make that run pass.

Both statements stand together. The second does not cancel the first: criteria that
evolved during development, in response to failures, are weaker evidence than
criteria fixed in advance, even when every cited run was produced under the final
version of them. Anyone quoting the canonical S01-S04 results should carry that
qualification with the number.

## 7. What remains open

```text
OPEN FAILURES: none of the nine
```

All nine are RESOLVED or SUPERSEDED as marked in section 1. No failure in this set
is an unexplained defect awaiting diagnosis. Three limitations are not failures and
are not closed by anything above:

1. The seven `UNEXPECTED_RELOCALIZATION_STATE` failures were resolved partly by
   changing the criteria and partly by changing the rig's start-up. Neither route
   involved a change to the localization, fusion or supervisor logic in
   `src/ylhb_base`. Nothing here evidences a behavioural fix to the software under
   test, because no behavioural defect in it was found.
2. `S02_..._20260828-145049` failed on relocalization activity at 10.65 s, in the
   degraded phase and outside any start-up window. The canonical S02 run reports
   activity of that kind as a warning rather than a failure. The activity is
   retained and readable; the decision to treat it as out of scope rather than as a
   failure is a criteria decision, not a measurement.
3. No FAIL run exists under `final/` or `precheck/`. Eleven runs at
   `build_commit = 57ec8ec` all passed. Eleven passes at one build is a narrow base,
   and it says nothing about behaviour at any other build.

## 8. Retention

```text
NO FAILURE RECORD MAY BE DELETED. A FAILED RUN IS EVIDENCE.
```

The nine directories, and the interrupted
`S02_gnss_gradual_degradation_and_outage_20260828-145410`, stay on disk with their
`result.json`, `samples.csv`, `timeline.csv`, `rosbag/` and `logs/` unmodified. This
applies to `S01_nominal_20260827-204855` in particular, which holds 0 samples and no
plots: a run that produced no data still records that the chain could not run, and
that is the only artefact proving when the compatibility blocker was live.

```text
Plots regenerated later from a preserved CSV are a permitted addition.
Modifying a CSV, a result.json, or a rosbag is not.
```
