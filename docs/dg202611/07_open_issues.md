# DG-202611 Round B — 07 Open Issues

Every item is a verified observation, not a guess. Items marked
`NOT_YET_VERIFIED` are unknown and are not filled in speculatively.

Prefix for every relative path:
`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot`

## 1. Closed this round, recorded so the closure is auditable

These items were open earlier in Round B. Each was closed by a **target read-back**,
not by a transfer summary — the distinction matters, because the same class of
mistaken closure happened twice this round (`04_debug_and_failures.md` section 9).

| Was open | Now | Target evidence |
|---|---|---|
| `map -> odom` identity transform still emitted (`DECISION_005` not landed) | CLOSED | `synthetic_injector_node.py` `_static_transforms` returns `base_footprint -> sensor` only; `PERFECT_MAP_ODOM_PUBLISHED: NO` in every precheck artefact |
| AMCL surrogate folded into `PlantConfig` (`DECISION_002` not landed) | CLOSED | `scenario_schema.py` declares `AmclSurrogateConfig` separately; `PlantConfig` no longer carries `amcl_follows_plant` or `covariance_after_match` |
| S05–S08 scenario definitions absent | CLOSED | `config/S05.yaml` .. `S08.yaml` present and installed to share |
| `visualization_markers_node` had no installed executable | CLOSED | `ros2 pkg executables dg_synthetic_validation` lists it among 8 executables |

## 2. RViz visualization — the layout fix landed; two residual limits

### 2.1 The overlap defect, and its resolution

```text
DEFECT:  RViz text markers severely overlapped in the default view
STATUS:  RESOLVED
MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED
```

When the user first opened the GUI the text markers overlapped badly, making the default
view unusable as evidence: overlapping text cannot be read, and a screenshot of it cannot
be captioned honestly. The fix is visualization-only. No algorithm, threshold, or state
machine was touched.

Evidence, split by who observed it, because that distinction is the whole point of the
verification gate in `01_goal_and_scope.md` section 3.2:

| Check | Result | Observed by |
|---|---|---|
| `visualization_markers_node.py` and `rviz/dg_synthetic_validation.rviz` written to target and read back | PASS | coordinator, own read-back |
| `py_compile` on the node | PASS | coordinator |
| `colcon build --packages-select dg_synthetic_validation` | PASS | coordinator |
| the new view reached the **installed** config | PASS | coordinator, re-confirmed here |
| `.rviz` YAML parse, 6 displays | PASS | re-confirmed here on both source and installed copies |
| node parses as Python (`ast.parse`) | PASS | re-confirmed here |
| `VISUALIZATION_NODE_BOUNDARY` | PASS | coordinator, re-confirmed here |
| post-change synthetic package tests | 33 passed | coordinator |
| `ros2 pkg executables dg_synthetic_validation` | 8 | re-confirmed here |
| `git status --porcelain src/ylhb_base` | 0 lines | re-confirmed here |

Read back here directly from the target:

```text
source and installed rviz/dg_synthetic_validation.rviz:
  YAML parse OK, 6 displays, Fixed Frame: odom,
  View rviz_default_plugins/TopDownOrtho, Scale: 32, X: 0, Y: -2.1
```

So RViz launched from the install path gets the fixed layout, not just the source tree.

Layout, as reported by the coordinator's static audit: 24 text markers, one line per
marker, unique id and position, across 5 separated regions; minimum pairwise separation
0.70 m against a 0.44 m worst-case line height, i.e. about 0.26 m clearance. Corroborated
here from the module constants read off the target: `ROW_STEP = 0.70` pitch, a single
uniform `BODY_HEIGHT = 0.40` for every state line, `BANNER_HEIGHT = 0.60`, and one marker
per `TextLine` with its own namespace and id base per region
(`dg_banner`, `dg_left_column`, `dg_right_column`, `dg_candidate_block`,
`dg_motion_block`, `dg_view_notes`).

Text regions sit outside the map box, so they never overlay scan or map geometry:

```text
MAP_KEEPOUT = 4.45   against a map and scan spanning x,y in [-4, +4]
text columns at x = -+9.30, bottom blocks at x = -+5.80, banner at y = 6.90,
footer notes at y = -10.10
```

`VISUALIZATION_NODE_BOUNDARY`, re-verified here by grep on the target: exactly one
`create_publisher`, on `/dg_validation_viz/markers`, outside `/dg/*`; zero TF symbols;
`/cmd_vel` referenced only through a graph publisher-count query, with the node's own
comment recording "Graph query only. This node never publishes /cmd_vel". The single
`LOST|SEARCHING` hit is the comment above `VALID_STATES` stating that those two names
must never be rendered.

One correction to the earlier record, from this read-back: the dead pose displays were
**removed**, not re-typed. The `.rviz` carries an explicit `# REMOVED:` block naming all
three (`/dg/fusion/pose`, `/amcl_pose`, `/scan_match_pose`) and the two independent
reasons. `/amcl_pose` and `/scan_match_pose` are redrawn as arrows in `odom` by the marker
node under its declared display-only map-origin assumption. `/dg/fusion/pose` has **no
visualization path at all**, which the `.rviz` states in the same block. See
`04_debug_and_failures.md` section 13.

### 2.2 What is not a defect here

Idle RViz showing no `Raw Scan`, no `Filtered Scan` and no `Odometry` data, and the
monitor showing `NO_DATA`, is the **correct and expected** idle behaviour when no
scenario is running.

```text
RULE: an empty idle view is never repaired by fabricating a publisher.
```

### 2.3 `RVIZ_DISPLAY_FIELD_ACCEPTANCE`

```text
RVIZ_DISPLAY_FIELD_ACCEPTANCE: NOT_YET_VERIFIED
```

YAML parsing proves the `.rviz` is well-formed. It does **not** prove that RViz honours
every display's field set. A display whose fields RViz silently drops still parses
perfectly, so the parse cannot distinguish "accepted" from "quietly ignored".

Only opening RViz settles this. It is distinct from the display **type** audit, which is
complete: the message-type mismatches are gone and the unreachable-frame display was
removed rather than propped up. What remains unverified is field acceptance, not typing.

### 2.4 `EVIDENCE_TEXT_LEGIBILITY_AT_1280x800`

```text
EVIDENCE_TEXT_LEGIBILITY_AT_1280x800: USER_JUDGEMENT_REQUIRED
```

At the VM's 1280x800 with `Scale: 32`, body text renders at roughly 9-10 px cap height,
and the banner is about 1.7x larger. That is legible on screen, but small for a printed
proposal figure.

Two remedies, both for the owner to choose on screen:

| Remedy | Effect |
|---|---|
| maximise the RViz window before capture | more pixels for the same scene |
| raise the VM display resolution | same, and helps every later capture |

```text
NOT DONE: the visible extent was NOT reduced to enlarge the text.
```

Cropping the view to make text bigger would push the text regions out of frame, which
defeats the purpose of the layout. This is a presentation trade-off for the owner to
judge against a real screen, not a defect to be silently tuned away.

## 3. Nothing is committed

```text
GIT_COMMITTED: NO
GIT_PUSHED:    NO
STATUS: OPEN — deliberate, no commit was authorised this round
```

Verified: `HEAD` is `57ec8ec docs(dg): add Claude Code development handoff` on branch
`dg202611-synthetic-validation`. All Round B work is present as modified or untracked
files. `git status --porcelain src/ylhb_base` returns 0 lines, so `ylhb_base` is clean
and unmodified.

Consequence: every statement in these records about the target is a timestamped
observation, and an uncommitted worktree can diverge again.

## 4. Final-evidence scenarios — CORRECTED, they have been executed

The heading and status block previously read "No final-evidence scenario has
been executed" with `S05-S08 FINAL EVIDENCE: NOT_YET_RUN`. That contradicted
disk and is corrected here:

```text
SUPERSEDED:  NO_FINAL_EVIDENCE_SCENARIO_EXECUTED: YES
SUPERSEDED:  S05-S08 FINAL EVIDENCE:              NOT_YET_RUN
```

```text
CORRECTED
S05-S08 PRECHECK:        PASS (RUN_CLASS=PRECHECK)
S05-S08 FINAL EVIDENCE:  RUN — seven FINAL run directories exist under
                         results/synthetic/final/, all result=PASS,
                         run_class=FINAL, 0 errors
S06 FINAL EVIDENCE:      RUN, 2 runs, both PASS (see below)
```

S06, stated explicitly because it was the flatly contradicted case:

| S06 FINAL run | Samples | Designation |
|---|---|---|
| `S06_closed_loop_active_scan_recovery_motion_20260829-165043` | 399 | **`CANONICAL`** — cite this run |
| `S06_closed_loop_active_scan_recovery_motion_20260829-161050` | 392 | `SUPERSEDED_FINAL` — valid data, do not cite |

The relationship between the two, so neither is misread: `161050` ran **before
the marker-staleness fix was live**, so its GUI state could not be trusted for
capture; `165043` ran **after that fix was actually active**, which is why it is
the run to cite. `161050` is not a failed or discarded run — it is valid FINAL
data that was superseded for capture reasons only, and it is retained under the
immutability rule.

Reconciling the `S06_RERUN = NO` directive, which has been read as a
contradiction of the above:

```text
S06_RERUN = NO   means: no FURTHER S06 rerun for screenshot purposes.
S06_RERUN = NO   does NOT mean: no S06 FINAL run exists.
```

Both statements are true at once. `S06_RERUN = NO` is recorded inside the
canonical run's own `evidence/evidence_manifest.md` alongside
`S06_CANONICAL_FINAL_EVIDENCE = YES`, which is only coherent if a FINAL run
exists. The directive forbids re-running to chase a better screenshot; it says
nothing about whether the run happened.

Run-level detail for the other scenarios belongs to
`PACKAGE_A_RUN_INVENTORY.md` and is not duplicated here.

What remains true and unchanged: the four **precheck** runs are headless, did
not pass the `VISIBLE_EVIDENCE_CHECKPOINT`, have no manual screenshots and no
evidence manifest. A precheck run may not be cited in a proposal, report, or
presentation. That restriction applies to the precheck class, not to the FINAL
runs above.

## 5. Coverage gap — the `FAILED` and `MANUAL_REQUIRED` branches are untested

```text
COVERAGE_GAP: FAILED and MANUAL_REQUIRED branches untested
STATUS:       NOT_YET_COVERED (a coverage gap, NOT a spec deviation)
```

All four landed scenarios match their specified intent, S08 included: the Round B
protocol specifies multi-frame recovery verification for S08, and that is what
`config/S08.yaml` (`multiframe_verification_and_control_handoff`) implements. The
protocol's separate clause about entering `FAILED` / `MANUAL_REQUIRED` "under failure
conditions" describes what the state machine may legitimately do; it does not assign a
failure-branch scenario to S08.

What is untested:

```text
_fail() reasons:  ATTEMPT_TIMEOUT  SEGMENT_TIMEOUT  MAX_TOTAL_ROTATION
                  ODOM_STALE_OR_MISSING  YAW_MISSING
                  WAITING_FOR_CANDIDATE_TIMEOUT
retry path:       FAILED -> STOPPING while attempt_id < max_attempts = 2
exhaustion path:  FAILED -> MANUAL_REQUIRED at MAX_ATTEMPTS_EXCEEDED
manual takeover:  the MANUAL_REQUIRED latch
```

Why it matters: S05–S08 all currently take the success path, so the recovery machine's
failure handling and retry logic carry **no synthetic evidence at all**. A state machine
whose only recorded runs succeed has an untested half.

Proposed future work: a scenario that withholds a usable candidate — for example
degrading scan geometry **after** the seed request, so the real matcher legitimately
rejects — to drive candidate exhaustion through both attempts into `MANUAL_REQUIRED`.

```text
RULE: this must be achieved by degrading INPUT only.
A FAILED or MANUAL_REQUIRED state is NEVER injected.
```

Specification retained in `05_test_protocol.md` section 11.

```text
SCHEDULED: NO — explicitly not this round.
```

## 6. `FULL_COLCON_TEST` not run this round

```text
STATUS THIS ROUND: NOT_RUN
PREVIOUSLY RECORDED: 666 tests, 0 errors, 335 failures, 10 skipped — NOT PASS
```

The previously recorded failures are predominantly pre-existing flake8/pep257 **style**
findings. They must never be presented as core DG behavioural failure, and the
targeted passes (`ylhb_base` 253, `dg_synthetic_validation` 33) must never be presented
as a whole-repository pass. The three categories stay separate
(`06_results_and_evidence.md` section 6).

## 7. Visible evidence depends on a manual step

```text
GUI_CAPTURE_READY: MANUAL_ONLY   (frozen)
STATUS: OPEN, ACCEPTED
```

The desktop session is GNOME on Wayland (logind seat0/tty2), with `Xwayland :0
-rootless` at 1280x800. All three automatic methods fail: `ffmpeg -f x11grab` exits `0`
and yields `pblack:99` because mutter's guard window covers the rootless root; `xwd
-root` fails `BadMatch` on `X_GetImage`; `org.gnome.Shell.Screenshot` over D-Bus
returns `AccessDenied`.

```text
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

The capture helper's own false positive is fixed: a channel-spread-only check passed
the guard-window black frame (spread about 11), so a brightness floor was added and
validated against known-positive and known-negative inputs.

Two environment facts that will bite an automation attempt:

- `XAUTHORITY` is `/run/user/1000/.mutter-Xwaylandauth.*` with a **random suffix that
  changes across reboots**; it must be globbed, never hard-coded
- the GNOME screenshot directory must be **confirmed after the first capture** and
  never assumed. `~/Pictures/Screenshots` did not exist at audit time; GNOME 42 creates
  it on the first capture

```text
RVIZ_IS_MAPPED_XWAYLAND_CLIENT: NOT_YET_VERIFIED
```

Automatic data plots are unaffected, subject to the write choke point that refuses a
figure with zero data series.

## 8. `STOPPING` has no timeout

```text
STATUS: OPEN, ACCEPTED, NOT TO BE "FIXED" IN CORE
```

`STOPPING` confirms only when odom is fresh and both velocity magnitudes are at or
below `0.03` and yaw is finite. There is no timeout
(`src/ylhb_base/scripts/active_relocalization_core.py`, `STOPPING` handling).

Observed rather than hypothetical: the S05 precheck terminates in `STOPPING`, because
S05 runs with the plant disabled and `odom_linear_x` is a constant `0.05` in every
phase, above `stop_velocity = 0.03`. In S05 that is the charter, and the run ends on
duration. In a scenario that expected to progress, the same condition would present as
a **silent hang** rather than a diagnosable failure.

Recorded as an observation, not a defect to patch: `DECISION_008` forbids core
state-machine changes. Any scenario that must pass through `STOPPING` watches for the
hang signature explicitly.

## 9. Nav2 disabled, so the post-recovery path terminates in zero output

```text
STATUS: EXPECTED_AND_ACCEPTABLE (DECISION_007)
```

Nothing publishes `/cmd_vel_nav`, so after `RECOVERED` the arbiter selects `NAV` and
emits zero with `NAV_SOURCE_STALE`. `DECISION_006` forbids fabricating a synthetic
`/cmd_vel_nav` to paper over this. Observed in the S08 precheck as post-`RECOVERED`
max `|cmd_angular_z| = 0.000000`.

It is only reportable as "safe control handoff verified in synthetic software
validation". It is never reportable as navigation resuming, Nav2 resuming, or a mission
continuing.

### 9.1 Exactly what the handoff claim rests on

```text
SAFE_CONTROL_HANDOFF_EVIDENCED:  YES  — supported by the recorded state transitions
NAVIGATION_RESUMED_EVIDENCED:    NO   — forbidden claim, nothing evidences it
HANDOFF_EVIDENCE_STRENGTH:       DERIVED_ASSERTION
```

`SAFE_CONTROL_HANDOFF_EVIDENCED` is permitted: the state transitions recorded in
`samples.csv` and `timeline.csv` reach `RECOVERED` and recovery output goes to
zero, which is what the claim asserts.

`NAVIGATION_RESUMED` is not permitted in any form and is recorded as
`NAVIGATION_RESUMED_EVIDENCED = NO`.

The limitation on the handoff conclusion, verified in the FINAL run samples:

| Field | S06 canonical | S07 | S08 |
|---|---|---|---|
| `cmd_vel_source` | empty in 399/399 | empty in 159/159 | empty in 239/239 |
| `safety_cmd_vel_publishers` | empty in 399/399 | empty in 159/159 | empty in 239/239 |

Neither field was populated in any sample, so no directly published source field
witnesses which controller held the output. The non-empty column is
`cmd_vel_source_inferred` (`ZERO` / `RECOVERY`, plus `UNKNOWN` in S06), and its
name states what it is. The handoff conclusion therefore rests on a
`DERIVED_ASSERTION` built from state transitions and inferred source, not on a
direct measurement of the commanding source.

### 9.2 `relocalization_not_unexpectedly_active` was a non-discriminating check — now fixed in code

```text
CLASSIFICATION:  NON_DISCRIMINATING_CHECK
STATUS:          FIXED IN CODE by the lead engineer
```

The defect: `relocalization_not_unexpectedly_active` was hardcoded `True` for
every scenario except S01, yet it still occupied a slot in the reported check
totals. A check that cannot fail inflated apparent coverage.

The code now (`result_writer.py`, around lines 488-506) counts it for S01 only;
for every other scenario it is reported under `non_discriminating_checks` with a
stated reason and the `NON_DISCRIMINATING_CHECK` classification.

Consequence that must be recorded rather than quietly absorbed:

```text
PREVIOUSLY PUBLISHED:  12/12 for S01-class and 15/15 for S06-class
                       — both included one check that could not fail
CORRECTED:             S01        11 counted checks
                       S06-class  14 counted checks + 1 non-discriminating
```

The old totals must not be presented as if all of their checks were
discriminating. Note also that the check counts recorded **inside** the existing
FINAL `result.json` artefacts predate this code change and still carry the old
shape; those artefacts are immutable evidence and are not rewritten. The
corrected accounting lives in the documentation, here and in
`06_results_and_evidence.md`.

## 10. S06 and S07 navigation ends at `LOCALIZATION_SUSPECT`

```text
STATUS: OPEN AS A READING HAZARD, NOT AS A DEFECT
```

S06 and S07 both reach relocalization `RECOVERED` while their `navigation_state` ends
at `LOCALIZATION_SUSPECT`. Both have `converge_on_match` false, so surrogate covariance
stays high and the AMCL-`REJECTED` branch outranks `RECOVERED` in
`navigation_health_core`'s priority order. S08, with `converge_on_match` true, is the
only run whose navigation reaches `RECOVERED`.

The hazard is a reader concluding that recovery failed in S06 and S07, or that the two
runs contradict S08. Neither is true. Recorded in `02_architecture_and_design.md`
section 14 and `06_results_and_evidence.md` section 3.1 so the two facts always travel
together.

## 11. S06 and S07 reach `RECOVERED` without being chartered to

```text
STATUS: REAL BEHAVIOUR, NOT A DEFECT
```

`_handle_verifying` requires candidate quality plus pose stability and does **not**
require healthy covariance, so both scenarios walk the full sequence to `RECOVERED`
even though neither is chartered to test recovery. Recorded because a future reader
comparing charters to observed states will otherwise treat it as a scenario
mis-specification.

## 12. Structural limits on what S05–S08 can ever prove

```text
STATUS: PERMANENT LIMITATION, NOT A DEFECT
```

The true pose is **authored by the test rig**, and the `SYNTHETIC_AMCL_SURROGATE` reads
synthetic plant ground truth.

```text
GROUND_TRUTH_USED_BY_SUT:       NO
GROUND_TRUTH_USED_BY_SURROGATE: YES
GROUND_TRUTH_USED_FOR_ASSERTION: NO_ONLY_FOR_INPUT_AND_BIAS_CONSTRUCTION
GROUND_TRUTH_LEAKAGE_TO_MATCHER: NO
```

### 12.1 The candidate-pose surrogate boundary — state this wherever S05–S08 candidate poses are discussed

```text
CANDIDATE_POSE_ORIGIN:  an AMCL SURROGATE that READS synthetic ground truth
SUPERVISOR_UNDER_TEST:  no ground-truth leak
```

Both halves are needed. The candidate pose discussed in S05–S08 does not come
from a real localizer: it originates from the synthetic AMCL surrogate, and that
surrogate reads the rig's synthetic ground truth to construct its seed and bias.
The supervisor under test is separately clean — it never reads ground truth.

How `GROUND_TRUTH_LEAK` / `GROUND_TRUTH_LEAKAGE_TO_MATCHER: NO` must be
presented, because the evidence artefact does not measure it:

```text
GROUND_TRUTH_LEAK=NO  IS   a hardcoded string literal emitted by the writer
                           (result_writer.py:331 writes
                           "GROUND_TRUTH_LEAKAGE_TO_MATCHER": "NO")
GROUND_TRUTH_LEAK=NO  IS   corroborated by independent CODE AUDIT of the
                           supervisor and matcher paths
GROUND_TRUTH_LEAK=NO  IS   NOT a measurement, a test result, or a runtime check
```

A hardcoded literal cannot fail, so it carries no evidential weight on its own
and must never be cited as though the run had measured the absence of a leak.
The claim stands on the code audit; the string merely records the audit's
conclusion in the artefact.

### 12.2 What Package A does and does not evidence

```text
PACKAGE_A_DOES_EVIDENCE:
  supervisor logic
  trigger
  candidate handling
  verification flow
  state transitions
  safe handoff

PACKAGE_A_DOES_NOT_EVIDENCE:
  real scan-matcher performance
  real relocalization accuracy
  competition success rate
```

Therefore S05–S08 evidence the **software state chain**, not localization accuracy. No
amount of S05–S08 passing changes this. `mean_distance` is a scan-to-map residual
against synthetic grid geometry (`SYNTHETIC_MATCH_QUALITY_RESULT`), never a positioning
error.

Because no consumer of `map -> odom` exists, the ground-truth leakage path through TF
is **absent rather than mitigated**. See `MAP_ODOM_TF_JUSTIFICATION.md`.

## 13. Carried forward from the handoff, unchanged by Round B

```text
S05-S08 FINAL EVIDENCE:         RUN (was NOT_YET_RUN — corrected, section 4;
                                S06 FINAL canonical = ..._20260829-165043)
Gazebo:                         NOT_RUN (readiness BLOCKED, see the Gazebo report)
real robot:                     NOT_RUN
Jetson target integration:      NOT_DONE
ZED SDK/VIO integration:        NOT_DONE
mmWave radar:                   NOT_DONE
optical flow:                   NOT_DONE
event camera:                   NOT_DONE
BeiDou short-message:           NOT_IMPLEMENTED
XY error < 20 cm:               NOT_PROVEN
Z error < 20 cm:                NOT_PROVEN
feature repeatability >= 95%:   NOT_PROVEN
relocalization success > 95%:   NOT_PROVEN
FULL_COLCON_TEST_STATUS:        666 tests, 0 errors, 335 failures, 10 skipped, NOT PASS
```

Z/elevation remains an `OPEN_TECHNICAL_GAP`: a 2D wheel+IMU EKF and 2D LiDAR/AMCL
cannot prove Z performance, and no candidate option has been selected.

The 335 style failures are pre-existing and are not core DG behavioural failures. They
remain unaddressed and are not a Round B deliverable.

---

## OPEN_ISSUE_INITIALPOSE_QOS_DURABILITY_MISMATCH — `CLOSED_AS_MISDIAGNOSIS`

```text
STATUS:                          CLOSED_AS_MISDIAGNOSIS
INITIALPOSE_QOS_FINAL_VERDICT:   COMPATIBLE
CLOSED_BY:                       target read-back of
                                 src/ylhb_base/scripts/scan_map_relocalization_node.py
```

The original entry had the **direction of the QoS pairing backwards**. It is
corrected here rather than deleted, because the misdiagnosis is itself part of
the process history and a later reader must be able to see what was believed,
what was wrong about it, and how it was settled.

### The superseded record, kept verbatim

`SUPERSEDED` — every line in this block is **wrong** and must not be cited:

> ~~Status: OPEN, does not block Package A.~~
> ~~Observed: during the S06 canonical final run the injector logged~~
> ~~`New subscription discovered on topic '/initialpose', requesting incompatible~~
> ~~QoS. No messages will be sent to it. Last incompatible policy: DURABILITY`.~~
>
> ~~Root cause: `scan_map_relocalization_node.initial_pose_qos_profile()` subscribes~~
> ~~to `/initialpose` with `TRANSIENT_LOCAL` durability, while the synthetic~~
> ~~injector publishes it `VOLATILE`. The two are incompatible, so no message~~
> ~~would flow.~~
>
> ~~Consequence to respect later:~~
> ~~`MUST_RESOLVE_BEFORE_PHASE_A_OR_ANY_SCENARIO_THAT_ACTUALLY_DEPENDS_ON_INITIALPOSE = YES`~~
> ~~S01-S04 DO publish `/initialpose`, so their scan-to-map bootstrap path was~~
> ~~subject to this mismatch. That is consistent with those runs never producing an~~
> ~~accepted candidate, and it should be re-examined if S01-S04 are ever revisited.~~

Three separate errors are in that block: the profile is used on the publisher
and not the subscription; the pairing it describes is compatible rather than
incompatible; and the S01–S04 "never produced an accepted candidate" conclusion
was attributed to a QoS mismatch that does not exist. The quoted warning text
was also inaccurate — see the verbatim line further down.

### The true QoS facts, read back from the target

`src/ylhb_base/scripts/scan_map_relocalization_node.py`:

```text
initial_pose_qos_profile()            -> QoSProfile(depth=10,
                                          RELIABLE, TRANSIENT_LOCAL)
used on the PUBLISHER                 self._initialpose_pub = self.create_publisher(
                                        PoseWithCovarianceStamped,
                                        <initialpose_topic>,
                                        initial_pose_qos_profile())   ~line 275-278
the /initialpose SUBSCRIPTION         create_subscription(
                                        PoseWithCovarianceStamped,
                                        <initialpose_topic>,
                                        self._on_initialpose,
                                        10)                           ~line 293
                                      i.e. the plain default depth-10 profile:
                                      RELIABLE + VOLATILE
```

So the node **publishes** `TRANSIENT_LOCAL` and **subscribes** `VOLATILE`. The
compatibility rule runs one way only:

```text
publisher TRANSIENT_LOCAL  ->  subscriber VOLATILE          COMPATIBLE
publisher VOLATILE         ->  subscriber TRANSIENT_LOCAL   INCOMPATIBLE
```

A publisher may always offer *more* durability than a subscriber requests. The
incompatible pairing is the reverse of the one the superseded entry named, so
the node's own `/initialpose` publish path was never blocked by durability.

### The warning that was actually observed — emitter `UNDETERMINED`

A `requesting/offering incompatible QoS` warning on `/initialpose` **was**
observed. Verbatim, from
`results/synthetic/final/S06_closed_loop_active_scan_recovery_motion_20260829-165043/logs/rosbag.log:21`:

```text
[WARN] [1787993451.489738101] [rosbag2_recorder]: New publisher discovered on
topic '/initialpose', offering incompatible QoS. No messages will be sent to
it. Last incompatible policy: DURABILITY_QOS_POLICY
```

What that line does and does not establish:

| Fact | Status |
|---|---|
| the warning exists on `/initialpose`, `DURABILITY` policy | OBSERVED |
| the emitter is `rosbag2_recorder`, i.e. a **recording-side** subscription, not a DG node | OBSERVED |
| the same warning also appears in the historical S01, S02 and S04 `logs/rosbag.log`, and **not** in the superseded S06 FINAL run `20260829-161050` | OBSERVED |
| **which publisher** offered the incompatible durability | `UNDETERMINED` — the warning does not name it, and no run artefact identifies it |
| any functional message loss on a DG supervisor data path | NOT EVIDENCED |

```text
INITIALPOSE_INCOMPATIBLE_QOS_WARNING_OBSERVED:  YES
EMITTING_SUBSCRIBER:                            rosbag2_recorder
OFFENDING_PUBLISHER_IDENTITY:                   UNDETERMINED
```

The identity is left `UNDETERMINED` deliberately rather than guessed. One
candidate exists in the code — `synthetic_injector_node.py:408` creates a
depth-10 (`VOLATILE`) `/initialpose` publisher unconditionally, i.e. the
publisher object is constructed even when `publish_initialpose` is false and no
message is ever sent, which is enough for discovery — but **this has not been
confirmed against the warning** and is recorded as a hypothesis only.

```text
IF_THIS_WARNING_RECURS: raise it as a SEPARATE issue against the recording
path, with the publisher identified first. It must not be re-attached to the
closed misdiagnosis above.
```

### The S05 symptom was missing TF, not QoS

Recorded separately so the two never merge again:

```text
S05_CONTEXT_ROOT_CAUSE:  missing TF / base_footprint
S05_CONTEXT_QOS_ROLE:    NONE — no QoS incompatibility was involved
```

The real problem observed in the S05 context was an absent TF chain to
`base_footprint`, which is what blocker 4 records (`04_debug_and_failures.md`
section 4: `_scan_to_base_points` always failed with `TF_FAILED`, so no
candidate could exist). S05 additionally runs with the plant disabled, so no TF
buffer exists at all and `tf_base_to_sensor_available` is legitimately empty
(section 2.1 of `06_results_and_evidence.md`). None of that is a durability
problem. No S05 symptom may be attributed to `/initialpose` QoS.

### Why `publish_initialpose: false` still matters

Unchanged and still correct, but for a different reason than the superseded
entry gave: S06/S07/S08 set `publish_initialpose: false` so that every
`match_quality` message is attributable to a real supervisor seed request on
`/dg/relocalization/seed`. That is an **attribution** guarantee, not a QoS
workaround.
