# DG-202611 Round B — 04 Debug and Failures

This is the failure trail. It is deliberately not sanitised.

```text
RULE: failures are never deleted from this record. Wrong ideas that were
proposed and then denied stay here with the reason they were denied.
```

Prefix for every relative path:
`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot`

## 1. Blocker 1 — the trigger path did not exist

The injector's `_make_amcl` pinned the AMCL covariance at `0.05`. The state
machine's `max_covariance` is `0.5`
(`src/ylhb_base/scripts/active_relocalization_core.py:72`).

`0.05 < 0.5`, so `AMCL_COVARIANCE_HIGH` could never fire. Since
`AMCL_COVARIANCE_HIGH` is a `trigger_reasons` member and the GNSS/LiDAR reasons
are not, **no trigger path existed at all**. Every S01–S04 run was structurally
incapable of entering `SUSPECTED` for a localization reason.

This is the root cause behind the misreading of
`RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE`. The warning was never evidence
of a trigger, because a trigger was impossible.

## 2. Blocker 2 — `STOPPING` could never confirm, and hangs forever

The injector's `_make_odom` pinned `twist.linear.x` at `0.05`. The stop condition
requires `abs(linear) <= stop_velocity` with `stop_velocity = 0.03`
(`active_relocalization_core.py:79`).

`0.05 > 0.03`, so the stop was never confirmed.

The compounding detail: **`STOPPING` has no timeout.** It is not that the machine
would fail and report a reason. It would sit in `STOPPING` indefinitely, emitting
zero commands, with no failure to diagnose. Had a scenario ever reached this
state, the symptom would have been a silent hang rather than an error.

## 3. Blocker 3 — `ACTIVE_SCAN` could never reach its target

The injector pinned odom yaw at `0` with no motion model. `ACTIVE_SCAN` rotates
toward `segment_target_yaw` and needs to arrive within `1.5 deg`
(`yaw_tolerance`, `active_relocalization_core.py:82`).

With yaw frozen at `0`, the error never shrank, and the segment ended in
`SEGMENT_TIMEOUT` after `segment_timeout = 8.0 s`. Every segment would time out,
the segments would exhaust, and the machine would walk to `FAILED` and then
`MANUAL_REQUIRED` for a reason that had nothing to do with the algorithm.

Blockers 2 and 3 together are the direct justification for `DECISION_001`: a body
that actually turns when commanded. A recorded pose sequence would not do,
because a recorded sequence encodes the expected answer.

## 4. Blocker 4 — no TF, so no candidate could ever exist

No TF publisher existed anywhere in the integration launch. The real matcher's
`_scan_to_base_points` performs a TF lookup
(`src/ylhb_base/scripts/scan_map_relocalization_node.py:452`) and on failure sets
`self._last_scan_error_reason = "TF_FAILED"`.

Consequence: the matcher never produced a candidate, so `WAITING_CANDIDATE` could
never advance to `VERIFYING`, so `RECOVERED` was unreachable. **S07 was
structurally impossible**, independently of blockers 1–3.

A second, quieter failure sat behind it. The injector's analytic scan was:

```text
range + 0.12*sin(3a) + 0.04*cos(7a)
```

That is a wobbling circle, geometrically inconsistent with the published map. Even
with TF working, no match was possible. Fixing TF alone would have produced a new
symptom (`accepted=false`, low score) rather than a working loop, so both had to
be addressed.

The analytic generator was kept, bit-identical, so the historical S01–S04 waveform
and its recorded evidence remain valid. The ray-cast generator was added
alongside it rather than replacing it.

## 5. Wrong idea, denied — publishing a perfect `map -> odom`

```text
STATUS: PROPOSED, THEN DENIED (DECISION_005)
```

The first proposal for fixing blocker 4 was to publish a **perfect** `map -> odom`
transform from synthetic ground truth. The stated motivation was that RViz would
then display the map frame directly and the TF tree would look complete.

Why it was denied:

- No DG node consumes `map -> odom`. The only TF consumer looks up
  `base_footprint <- laser`. So the transform would have fixed nothing.
- `src/ylhb_base/scripts/multisource_fusion_node.py:196-197` explicitly logs that
  the fusion POC never owns `map -> odom`. Injecting it would contradict a
  deliberate design statement in the code under test.
- A perfect transform derived from ground truth is a ground-truth leakage risk. It
  would place the true pose on a global bus, where any future consumer could
  silently start depending on it.

It was removed from the design. RViz uses `odom` as its fixed frame and the map is
drawn as a visualization-only `MarkerArray` on `/dg_validation_viz/markers`,
outside `/dg/*`. Full ten-question record in `MAP_ODOM_TF_JUSTIFICATION.md`.

Residual state, verified on the remote at the time of writing: a static
**identity** `map -> odom` transform is still emitted by
`src/dg_synthetic_validation/dg_synthetic_validation/synthetic_injector_node.py:386-402`.
An identity transform carries no pose information, so it leaks nothing, but it
does not match `DECISION_005` and is carried as an open issue in
`07_open_issues.md`.

Later in the same round, verified by target read-back: that identity remnant is
**gone**. `_static_transforms` now returns the `base_footprint -> sensor` edge only,
and every S05–S08 precheck artefact records `PERFECT_MAP_ODOM_PUBLISHED: NO`. The
paragraph above is retained because it was the observed state when it was written;
the closure is recorded in `07_open_issues.md` section 1.

## 6. Wrong idea, corrected — folding the AMCL surrogate into the plant

```text
STATUS: IMPLEMENTED, THEN CORRECTED (DECISION_002)
```

The first implementation put the AMCL response model inside the plant
configuration, so one component owned both the robot body and the localization
observation. The remote tree still shows this iteration: `PlantConfig` carries
`amcl_follows_plant` and `covariance_after_match`, and its docstring describes
itself as "Synthetic differential-drive base plus AMCL response model".

Why it was corrected:

- The two components have **different truth boundaries**. The plant is honest
  infrastructure: it integrates commands and reports where the body went. The
  surrogate reads synthetic ground truth, which is a real limitation on the
  evidence.
- Folding them together hides that limitation inside something called a "plant",
  where a reader would not look for it.
- Separating them makes `DIRECT_GROUND_TRUTH_ACCESS: YES` attach to exactly one
  named component that can be audited.

Corrected design: `SYNTHETIC_KINEMATIC_PLANT` publishes `/odom` and TF and never
`/amcl_pose`; `SYNTHETIC_AMCL_SURROGATE` publishes `/amcl_pose` only.

Later in the same round, verified by target read-back: `scenario_schema.py` now
declares `AmclSurrogateConfig` as a separate component, and `PlantConfig` no longer
carries `amcl_follows_plant` or `covariance_after_match`. The pre-`DECISION_002`
iteration described above is no longer on the target. Closure recorded in
`07_open_issues.md` section 1.

## 7. Wrong idea, corrected — the covariance-only tautology

```text
STATUS: DESIGNED, THEN CORRECTED
```

The first surrogate design varied **only** the AMCL covariance. Pose stayed at
the plant's true value; covariance rose to trip `AMCL_COVARIANCE_HIGH` and fell
again after a match.

That design would have made S07 a **tautology**. The chain is:

1. the surrogate publishes `/amcl_pose` at the true pose, with high covariance
2. high covariance trips the trigger, recovery starts
3. `WAITING_CANDIDATE` publishes a seed on `/dg/relocalization/seed`, derived from
   that same true pose
4. the real matcher refines around a seed that is **already correct**
5. the match is accepted, `VERIFYING` sees no jump, `RECOVERED`

Step 4 is the problem. The matcher would not be recovering a lost pose; it would
be confirming a pose it had just been handed. A passing S07 would have proved
nothing about scan-to-map relocalization, only that the matcher does not corrupt a
correct input.

Correction: inject a deliberate constant pose bias of about 0.15 m and 8 deg, so
the seed is genuinely **wrong**. The bias is sized to sit **inside** the matcher's
real coarse search window of +-0.20 m and +-10 deg, so the problem stays solvable
without touching any threshold (`DECISION_008`). The matcher must now recover the
true pose from scan geometry.

The offline verification in `03_implementation_log.md` section 6 confirms the
corrected setup is solvable by the unmodified matcher: `ACCEPTED` with
`score=0.975`, `inlier=1.000`, `mean=0.009`, recovering +10.00 deg and -10.00 deg
from a 0 deg seed.

## 8. GUI capture failure on this host

The Ubuntu103 desktop session is **Wayland (GNOME)**. Automatic capture is not
available. Three methods were tried:

| Method | Result |
|---|---|
| `ffmpeg -f x11grab` | exits `0` but yields a roughly 99% **black** frame; mutter's guard window covers the rootless Xwayland root |
| `xwd` | fails `BadMatch` on every window |
| GNOME Shell D-Bus Screenshot API | returns `AccessDenied` |

```text
GUI_CAPTURE_READY: MANUAL_ONLY
```

The `ffmpeg` case is the dangerous one, because it **exits 0**. An automated
pipeline that only checks the exit code would happily file a black rectangle as a
screenshot.

```text
RULE: a uniform or black frame is treated as FAILURE. It is never filed as a
screenshot, because doing so would be fabricated evidence.
```

A deferred re-probe is retained for the case where RViz turns out to be a mapped
Xwayland client rather than a native Wayland client. It accepts a frame only if it
passes a non-uniform check.

Automatic data plots are unaffected and are still produced. Visible evidence for
S05–S08 therefore depends on a manual capture step; see the
`VISIBLE_EVIDENCE_CHECKPOINT` in `USER_VISIBLE_WORKFLOW.md`.

## 9. Repeated failure class — staging-target divergence, second occurrence

```text
REPEATED_FAILURE_CLASS: STAGING_TARGET_DIVERGENCE
OCCURRENCE_COUNT:       2
```

The root cause was identical both times: implementation was completed in a local
staging directory **after** an earlier target sync, and status was then reported
from local state without re-writing the target.

| # | Reported as done | What the target actually held |
|---|---|---|
| 1 | `map -> odom` removal and the AMCL-surrogate separation | the old code, unchanged |
| 2 | the `InputState` class added to the evaluator | `grep 'InputState\|_inputs'` on the target evaluator returned **nothing** |

A second occurrence of the same class is not a slip, it is a missing gate. The gate
is now mandatory. No file may be called `TARGET_WORKTREE_VERIFIED` unless all of the
following were performed **on the target path**:

1. read-back of the written file
2. a structural `grep` for the symbol, class, or field that was supposed to appear
3. `py_compile` on the target copy
4. where a data path is involved, a runtime sanity check that the value actually
   lands in the artefact

```text
NOT VERIFICATION: a file-transfer command that exited 0.
NOT VERIFICATION: a subagent summary stating the work was done.
```

Both of those describe an intent to change the target. Neither observes the target.

The five-term status vocabulary in `01_goal_and_scope.md` section 3.1 exists for
exactly this failure, and the terms are never used interchangeably:
`LOCAL_STAGING_ONLY`, `WRITTEN_TO_TARGET_WORKTREE`, `TARGET_WORKTREE_VERIFIED`,
`GIT_COMMITTED`, `GIT_PUSHED`.

## 10. The S06 failure that would have been misread

```text
CLASSIFICATION: EVIDENCE_PIPELINE_BUG
NOT:            PLANT_FAILURE
```

This is the consequence of occurrence 2 above, and it is the most instructive
failure of the round.

Because `InputState` never reached the target, all **15 INPUT columns** would have
serialized empty. The S06 criterion `s06_plant_yaw_responded_to_command` requires
`odom_yaw` to move by more than 1 degree. With the column empty the criterion would
have **FAILED for lack of evidence**, and the natural reading of that failure is
"the synthetic plant does not respond to commands" — which was false. The plant was
fine. The recorder was not writing what the plant did.

Two conclusions with opposite root causes were one empty column apart:

| Reading | Points at |
|---|---|
| `INPUT_EVIDENCE_INCOMPLETE` | the recorder / evidence pipeline |
| behavioural failure | the plant, or the arbiter, or the supervisor |

```text
RULE: an assertion that depends on an INPUT column must distinguish
INPUT_EVIDENCE_INCOMPLETE from a genuine behavioural failure.
```

How it was caught: an **independent audit subagent read the target file** instead of
trusting the summary. Nothing else in the chain would have caught it before a run
produced a red S06 and an incorrect diagnosis.

### 10.1 Resolution, verified

Two definitions had been accidentally duplicated in the evaluator: an older,
incomplete `InputState` lacking `tf_base_to_sensor_available`, plus a newer complete
one, and a duplicated `quaternion_yaw`. Python raises nothing for this — the second
definition silently shadows the first — so the file imported cleanly while carrying a
class that could not record the field the S06 and S07 criteria need. The duplicates
were removed.

```text
InputState definitions:     1 (verified by read-back)
quaternion_yaw definitions: 1 (verified by read-back)
```

Runtime round-trip through the real `SampleRecorder`, writing and re-reading
`samples.csv`:

```text
INPUT columns non-empty:  15/15
quaternion_yaw(40 deg quaternion) = 40.00 degrees
```

A structural `grep` alone would not have caught the shadowing, and `py_compile`
alone would not either. The runtime round-trip is the check that closes it.

## 11. Empty-plot false evidence

```text
STATUS: FOUND, FIXED, 14 NEW TESTS
GOVERNING RULE: FILE_EXISTS != VALID_EVIDENCE
```

`plot_results` `save()` called `savefig` and appended the path to `files` **even when
zero data series had been plotted**. A blank chart could therefore enter the evidence
manifest as a figure.

Two live blank paths were confirmed empirically, not theorised:

| Path | Mechanism |
|---|---|
| the `state_timeline` figure | gated only on "did any state column carry text", so with `elapsed_time` blank no `ax.step` call fired, yet the PNG was still written |
| `save()` figures | a populated value column with a blank `elapsed_time` produced `ax.plot` over an all-`None` x-axis, which matplotlib silently accepted |

Fix: data artists are tagged with gid `dg-data-series`, threshold lines with gid
`dg-reference-line`, and a single write choke point refuses `savefig` when the
counted finite-point series is zero.

The tagging is the load-bearing part. `len(ax.lines)` would count `axhline`
threshold lines drawn from hard-coded algorithm defaults, so a panel containing
**only** threshold lines and no measurement would self-certify as populated. A guard
that can be satisfied by its own decoration is not a guard.

This is the same class as the black-screenshot trap in section 8: an artefact
existed, an exit code was zero, and nothing had been measured.

## 12. Forbidden-claim scanner — two defects, the second worse

```text
STATUS: BOTH FOUND, BOTH FIXED
FILE:   src/dg_synthetic_validation/dg_synthetic_validation/evidence_manifest.py
```

**Defect 1 — anchored CJK patterns never matched.** The forbidden-claim patterns
wrapped Chinese phrases in `\b` word-boundary anchors. In Python `re` there is no
word boundary between two CJK characters, so an anchored CJK pattern cannot match
mid-sentence. The scanner passed silently on the very claims it exists to block.

Confirmed missed phrases:

```text
定位精度20厘米
使用Gazebo仿真采集
```

A guard that reports success while matching nothing is worse than no guard, because
it is cited as evidence that the text was checked.

**Defect 2 — the scanner flagged its own mandatory disclaimers.** `NOT REAL ROBOT
DATA` and `非真实机器人数据` were being reported as forbidden claims. Those markers
are **required** on every evidence document.

```text
A guard that fails on the markers it requires is a guard someone switches off.
```

That is the whole severity argument: defect 1 lets a false claim through, defect 2
creates pressure to disable the check entirely, which lets every claim through.

After the fix:

```text
POSITIVE cases caught:      11/11
NEGATIVE cases not flagged: 10/10
```

## 13. Dead RViz displays

```text
STATUS: FOUND, FIXED (displays REMOVED, not re-typed)
```

Three `rviz_default_plugins/Pose` displays were configured and could never render
anything. Two independent defects were stacked, which is why the record names both:

| Display | Frame defect | Type defect |
|---|---|---|
| `/dg/fusion/pose` | `header.frame_id` is `map`, unreachable with no `map -> odom` (`DECISION_005`) | message is `PoseWithCovarianceStamped`; the display consumes `PoseStamped` |
| `/amcl_pose` | same | same |
| `/scan_match_pose` | same | — |

All three would have appeared configured in the display tree and rendered nothing. A
screenshot of that view would have been filed as evidence of a working visualization.

Resolution, verified by read-back of the target `.rviz`: all three were **removed**, not
re-typed, and the file carries an explicit `# REMOVED:` block recording both reasons and
the source lines they were read from. `/amcl_pose` and `/scan_match_pose` are instead
drawn as arrows in `odom` by the marker node, under its declared display-only map-origin
assumption, which the in-view banner states.

```text
/dg/fusion/pose: NO VISUALIZATION PATH AT ALL
```

That absence is recorded rather than quietly filled. Inventing a display for it would
mean choosing a frame for a pose whose frame is unreachable, which is how the dead
displays came to exist in the first place.

```text
RULE: a .rviz file parsing as valid YAML does NOT prove that RViz accepts each
display's field set, nor that the message types match the topics.
```

Verified after the fix by read-back of both the source and the **installed**
`src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz`: 6 displays — `Grid`,
`Raw Scan` (`/scan`), `Filtered Scan` (`/dg/lidar/filtered_scan`), `Odometry` (`/odom`),
`TF`, and `Validation Markers` (`/dg_validation_viz/markers`) — with `Fixed Frame: odom`
and a `rviz_default_plugins/TopDownOrtho` default view at `Scale: 32`, `Y: -2.1`.

The type audit is complete. Field acceptance is not: see `RVIZ_DISPLAY_FIELD_ACCEPTANCE`
in `07_open_issues.md` section 2.3, which stays `NOT_YET_VERIFIED` because a display RViz
silently drops still parses perfectly.

## 14. Capture-helper false positive

```text
STATUS: FOUND, FIXED, VALIDATED AGAINST KNOWN-POSITIVE AND KNOWN-NEGATIVE INPUTS
```

Extends section 8. The first implementation of the capture helper judged frames by
flat-colour and channel-spread only. The mutter guard-window black frame carries a
channel spread of about **11**, which was enough to pass, so the helper reported
`capture_available=true` for a frame with **no content**.

A brightness floor was added (`NEAR_BLACK_CEILING = 32`) and validated against
frames known to be good and frames known to be blank.

```text
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

The pattern repeats across sections 8, 11 and 14: every automatic evidence path in
this project has had at least one way to certify an empty artefact as a real one.

### 14.1 The environment mutates under a cached assumption

The VM was rebooted between sessions, and the reboot is now corroborated **twice**: by
`uptime` (11 minutes at 2026-08-29 11:37, against a prior session on 2026-08-28), and
independently by the Xwayland `XAUTHORITY` filename, whose random suffix
(`/run/user/1000/.mutter-Xwaylandauth.*`) is not the value recorded during the earlier
capture probe. A changed suffix cannot happen without the session being recreated, so it
is evidence of the restart in its own right.

```text
RULE: DISPLAY and XAUTHORITY are resolved at capture time, never cached and never
copied out of a document into a script.
```

This is the failure shape worth naming: a capture script holding a hard-coded
`XAUTHORITY` path would not report a wrong path. It would fail to reach the X server and
produce another empty frame, which sections 8 and 14 have already shown can pass an
insufficient check.

## 15. Manual-capture instruction mismatch

```text
STATUS: FOUND, RESOLVED
```

`MANUAL_CAPTURE_CHECKLIST.md` instructed the user to read the relocalization state
off an RViz screenshot, while the marker node that draws that text had no installed
executable. The instruction asked a person to describe something that was not on
screen — the evidence-instruction equivalent of a blank plot.

Resolution, recorded as the standing division of labour:

| Source | Authoritative for |
|---|---|
| `monitor_scenario` | detailed state text, per-signal values |
| RViz | spatial evidence, plus a key-state summary |

An instruction that cannot be followed honestly produces either an absent shot or a
fabricated caption. There is no third outcome, so the instruction is fixed rather
than the reader trusted to notice.

## 16. Bare-python regression invocation error

```text
CLASSIFICATION: INVOCATION_ERROR
NOT:            CODE_DEFECT
```

An attempt to run the `ylhb_base` regression with bare `/usr/bin/python3` failed at
collection:

```text
ModuleNotFoundError: No module named 'rclpy'
```

Cause: the ROS environment was not sourced. Nothing was wrong with the tests or the
code.

What was deliberately **not** done: the historical `253 passed` figure was not
quoted as reproduced while this error stood. A collection error is not a result, and
a previously recorded number is not reproduced until it is reproduced.

After sourcing `/opt/ros/humble/setup.bash` and the workspace install:

```text
253 passed
PREVIOUS_253_PASS_REPRODUCED: YES
```

## 17. RViz text markers overlapped — resolved this round

```text
DEFECT: text markers severely OVERLAPPING in the default view
STATUS: RESOLVED (visualization only)
MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED
```

When the user opened the GUI, the RViz text markers were severely **overlapping**, which
made the default view unusable as evidence: text drawn on top of other text cannot be
read, and a screenshot of it cannot be captioned honestly. This is the same family as a
blank plot and a black frame — a file that exists and communicates nothing — except that
here the artefact looks full rather than empty, which is worse for a reviewer skimming it.

The fix was visualization-only: separated regions, one marker per line with a unique id
and position, uniform text sizing, and a screenshot-ready default `TopDownOrtho` view.
No algorithm, threshold, or state machine was touched.

Verified, with provenance kept separate. Reported by the coordinator from their own
read-back: `py_compile` PASS on the node, `colcon build --packages-select
dg_synthetic_validation` PASS, the layout audit (24 text markers, one line per marker,
unique id and position, 5 separated regions, 0.70 m minimum pairwise separation against a
0.44 m worst-case line height, about 0.26 m clearance), the display-type audit, and the
`VISUALIZATION_NODE_BOUNDARY` grep. Re-confirmed independently here:

```text
source and installed .rviz:  YAML parse OK, 6 displays, Fixed Frame odom,
                             TopDownOrtho, Scale 32, Y -2.1
node:                        ast.parse OK
boundary:                    1 create_publisher on /dg_validation_viz/markers,
                             0 TF symbols, /cmd_vel only via a graph query
layout constants:            ROW_STEP 0.70, uniform BODY_HEIGHT 0.40,
                             BANNER_HEIGHT 0.60, MAP_KEEPOUT 4.45 against a
                             map spanning x,y in [-4, +4]
regression:                  ros2 pkg executables 8; ylhb_base 0 dirty lines
```

The fix reached the **installed** config, so RViz launched from the install path gets the
fixed layout. That distinction matters here: a source-only fix would have left the very
command in `USER_VISIBLE_WORKFLOW.md` pointing at the old layout, and the failure would
have resurfaced at capture time.

Two limits are carried forward rather than closed, both in `07_open_issues.md` section 2:
`RVIZ_DISPLAY_FIELD_ACCEPTANCE` is `NOT_YET_VERIFIED`, and
`EVIDENCE_TEXT_LEGIBILITY_AT_1280x800` is `USER_JUDGEMENT_REQUIRED`. The second was
deliberately **not** resolved by cropping the view to enlarge the text, because that would
push the text regions out of frame and defeat the layout.

### 17.1 What is NOT a defect and must never be "fixed"

Idle RViz showing no `Raw Scan` / `Filtered Scan` / `Odometry` data, and Monitor showing
`NO_DATA`, is the **correct and expected** idle behaviour when no scenario is running.
Nothing is publishing, so nothing is displayed.

```text
RULE: an empty idle view is never repaired by fabricating a publisher.
```

A synthetic publisher added to make the idle view look alive would put fabricated data on
a real topic, which is the failure mode `DECISION_006` forbids.

## 18. Failure trail summary

| # | Failure or wrong idea | Disposition |
|---|---|---|
| 1 | AMCL covariance pinned at `0.05` | fixed on the synthetic input side |
| 2 | odom `twist.linear.x` pinned at `0.05` | fixed by the plant (`DECISION_001`) |
| 3 | odom yaw pinned at `0` | fixed by the plant (`DECISION_001`) |
| 4 | no TF publisher, plus a map-inconsistent analytic scan | fixed by approved TF edges and the ray-cast generator |
| 5 | perfect `map -> odom` proposed | DENIED and removed (`DECISION_005`); the identity remnant has since been removed from the target |
| 6 | AMCL surrogate folded into the plant | CORRECTED to a separate component (`DECISION_002`), now landed on the target |
| 7 | covariance-only surrogate, a tautology | CORRECTED by injecting a real pose bias |
| 8 | Wayland blocks automatic GUI capture | accepted as `MANUAL_ONLY`; black frames rejected as evidence |
| 9 | staging-target divergence, occurrence 1 (`map -> odom` / surrogate reported done, target unchanged) | verification gate introduced |
| 10 | staging-target divergence, occurrence 2 (`InputState` reported added, absent on target) | same class repeated; gate made mandatory, four-part check required |
| 11 | duplicated `InputState` and `quaternion_yaw`, second silently shadowing the first | duplicates removed; 15/15 INPUT columns verified by runtime round-trip |
| 12 | a false S06 `PLANT_FAILURE` would have been reported for an empty INPUT column | classified `EVIDENCE_PIPELINE_BUG`; assertion rule added |
| 13 | `plot_results` filed blank figures as evidence | gid tagging plus a single write choke point; 14 new tests |
| 14 | forbidden-claim scanner: `\b`-anchored CJK patterns never matched | patterns de-anchored; 11/11 positives caught |
| 15 | forbidden-claim scanner flagged its own mandatory disclaimers | fixed; 10/10 negatives clean |
| 16 | three dead RViz `Pose` displays — all three in the unreachable `map` frame, two also `PoseWithCovarianceStamped` on a `PoseStamped` display | all three REMOVED, not re-typed, verified by read-back; `/dg/fusion/pose` now has no visualization path, recorded as such |
| 17 | capture helper passed a black guard-window frame (channel spread ~11) | brightness floor added and validated |
| 18 | checklist asked the user to read state text that was not on screen | Monitor made authoritative for detailed state text |
| 19 | `ylhb_base` regression run with bare `python3` (`No module named 'rclpy'`) | invocation error; rerun with ROS sourced, `253 passed` |
| 20 | RViz text markers overlapping in the default view | RESOLVED, visualization only; `MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED`, installed config included |

No core threshold, state machine, or algorithm semantic was changed to resolve any
of these.

## 19. The `DiagnosticStatus.level` blocker (F1) — two halves, not one

Recorded here because the historical blocker has repeatedly been summarised as a
single test-rig bug. It has **two** halves, and the second one explains a symptom
that would otherwise look unrelated. The authoritative failure record is
`PACKAGE_A_FAILURE_HISTORY.md`; this section exists so the two halves are not
lost from the debug trail.

```text
HALF 1 — WRITE SIDE (test rig)
The synthetic injector wrote an `int` to DiagnosticStatus.level.

HALF 2 — READ SIDE (src/ylhb_base)
Commit d707193 "fix(ros2): normalize diagnostic status levels" additionally
normalised the READ side in three ylhb_base nodes.
```

Half 2, verified by read-back of the target. `d707193` adds
`src/ylhb_base/scripts/diagnostic_level.py` with a
`normalize_diagnostic_level(level, default=None)` helper, plus
`src/ylhb_base/test/test_diagnostic_level.py` as a unit test, and wires the helper
into exactly three nodes:

```text
gnss_quality_node.py:125         normalize_diagnostic_level(selected.level, default=3)
multisource_fusion_node.py:257   normalize_diagnostic_level(selected.level, default=3)
multisource_fusion_node.py:299   normalize_diagnostic_level(selected.level, default=3)
navigation_health_node.py:162    normalize_diagnostic_level(selected.level, default=3)
```

The helper's own docstring states the incompatibility it absorbs: ROS 2 Humble's
generated Python message exposes the `uint8` field as a one-byte `bytes` object,
while test doubles and older bindings commonly use `int`.

### Why this is the cause of the `gnss_nominal_seen` failure

The read-side half is **precisely** why `gnss_nominal_seen` was false in the
`S01_nominal_20260827-205016` run. Read back from that run's `result.json`:

```text
gnss_nominal_seen = False
errors = ['UNEXPECTED_RELOCALIZATION_STATE', 'GNSS_NOMINAL_SEEN']
```

Mechanism: the GNSS gate in `gnss_quality_node` could not interpret the diagnostic
level, so it never reported `GOOD`, so `gnss_nominal_seen` never became true.

```text
GNSS_SYMPTOM_ROOT_CAUSE:  the F1 read-side half — NOT a separate defect
```

That attribution is recorded explicitly because the symptom presents as a GNSS
problem and invites a separate root cause. No doc in this tree currently
attributes it to a separate cause, and none may start to.
