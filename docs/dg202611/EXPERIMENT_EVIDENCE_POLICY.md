# DG-202611 — Experiment Evidence Policy

```text
SCOPE:  permanent and forward-looking
APPLIES TO: every future test of any kind on this project
```

This is a standing rule, not a Round B record. It applies to every synthetic,
Gazebo, Jetson, real-robot, and competition-metric test, and to any future machine
learning work, from the moment it is written onward.

## 1. The rule

Every experiment retains its evidence. An experiment that reports a conclusion
without retained evidence is not a result; it is an assertion.

Minimum retention for **every** test, regardless of kind:

- logs
- raw data
- rosbag
- CSV
- plots
- key screenshots
- environment photos
- result photos
- anomaly photos

"It passed" is not evidence. A number without the data behind it is not evidence.

## 2. Applies to every test category

| Category | Status today | Policy |
|---|---|---|
| Synthetic software validation | in use | full retention list above |
| Gazebo | `NOT_RUN` | full retention list, when it happens |
| Jetson target | `NOT_DONE` | full retention list, when it happens |
| Real robot | `NOT_RUN` | full retention list, when it happens |
| Competition metric | `NOT_PROVEN` | full retention list, when it happens |
| ML training | none yet | full retention list **plus** section 4 |

No category is exempt on the grounds of being preliminary, exploratory, or "just a
quick check". Quick checks are exactly the runs whose evidence is missing later.

## 3. Photographs, and the rule about vehicles that do not exist

```text
No real-robot photograph may be produced or faked while no real vehicle exists.
```

There is currently no real vehicle. Therefore there are currently no real-robot
photographs, and none may be generated, staged, borrowed, or simulated and captioned
as real. When a vehicle exists, its photographs are taken from it and from nothing
else.

The same rule governs screenshots. A uniform or black frame is a **failure**, never a
screenshot. See `04_debug_and_failures.md` section 8: on this host `ffmpeg -f
x11grab` exits `0` while producing a roughly 99% black frame, so an exit-code check
alone will file fabricated evidence. Frames are accepted only after a non-uniform
check.

## 4. Additional retention for machine learning runs

Any future ML training run retains everything in section 1 **and** all of:

- training configuration
- dataset version
- train/validation split
- epochs
- loss and validation loss
- learning rate
- GPU utilisation
- checkpoints, and which checkpoint is the best one
- training and validation curves
- typical success samples
- typical failure samples
- visualised predictions

```text
Accuracy alone is NEVER an acceptable ML result.
```

A single accuracy figure hides overfitting, a broken split, a mislabelled class, and
a model that learned a shortcut. The failure samples and the visualised predictions
are the parts that make the number interpretable, so they are mandatory rather than
optional.

## 5. Truth labelling

Every retained result carries its scope label, using the established tokens:
`ENGINEERING_FOUNDATION`, `SOFTWARE_POC`, `ROS2_RUNTIME_VALIDATED`,
`SYNTHETIC_VALIDATED`, `NOT_YET_GAZEBO_VALIDATED`,
`NOT_YET_TARGET_HARDWARE_VALIDATED`, `NOT_YET_REAL_ROBOT_VALIDATED`,
`NOT_YET_COMPETITION_METRIC_VALIDATED`, plus `NOT_YET_RUN` and `NOT_YET_VERIFIED`.

A missing observation is recorded as empty or `NO_DATA`. It is never inferred, never
interpolated, and never filled in from an adjacent run.

### 5.1 How to read an empty field in an evidence-manifest caption

The captions in `results/synthetic/final/*/evidence/evidence_manifest.md` are
generated from `samples.csv` columns. Some render with nothing after the label,
for example `/cmd_vel 发布者 。` with no number, or an empty `cmd 来源`.

```text
EMPTY CAPTION FIELD MEANS:          the field was NOT PUBLISHED by any topic
EMPTY CAPTION FIELD DOES NOT MEAN:  the value was lost, dropped, or omitted
```

Read it as `NOT_APPLICABLE`, not as `MISSING`. This is the rule above working as
intended: the labeller writes exactly what the column held, and the column held
nothing because nothing published it. Verified case: the `/cmd_vel 发布者` slot is
filled from `safety_cmd_vel_publishers`, which is empty in every sample of the S06
canonical, S07 and S08 FINAL runs.

The manifests and screenshots are audited artefacts. A blank field is never
back-filled, and a caption is never rewritten to look complete — the reader is
told what the blank means instead, which is what this section does.

## 6. Immutability

Existing evidence is never deleted, overwritten, or edited. This includes evidence
of failure, and it especially includes evidence of failure.

The historical blocker run at
`/home/weiyang/dg202611_ws/results/synthetic/S01_nominal_20260827-205016/` is the
reference case: it preserves the first ROS 2 synthetic-runtime evidence that exposed
the `DiagnosticStatus.level` compatibility blocker. Its rosbag, CSV, timeline, and
logs are immutable.

Plots regenerated later from a preserved CSV are a permitted addition. Modifying the
CSV is not.

## 7. Artefact validity — existence is not evidence

```text
FILE_EXISTS        != VALID_EVIDENCE
FILE_EXISTS        != VALID_SCREENSHOT
CAPTURE_AVAILABLE  != VALID_EVIDENCE
```

Retention is necessary and not sufficient. A retained artefact counts only if it
carries a measurement. Three real defects on this project produced artefacts that
existed and measured nothing:

| Artefact | How it passed | Why it was empty |
|---|---|---|
| plot PNG | `savefig` was called and the path appended to `files` | zero data series had been drawn; a blank chart entered the manifest |
| screenshot PNG | `ffmpeg` exited `0`; a channel-spread check passed | mutter's guard window; the frame was black plus noise, spread about 11 |
| RViz view | the `.rviz` file parsed as valid YAML | display message types did not match the topics, and one display targeted an unreachable frame |

Minimum validity checks, required before an artefact is filed:

- **figures** — at least one tagged data series with at least one finite point.
  Threshold or reference lines do not count, because they are drawn from hard-coded
  defaults and would let a panel with no measurement self-certify
- **screenshots** — both a flat-colour test and a brightness floor. A frame that is
  black plus noise fails
- **RViz** — display types match the topic message types, and every display's frame is
  reachable in the TF tree
- **CSV columns** — an assertion that rests on an `INPUT` column distinguishes
  `INPUT_EVIDENCE_INCOMPLETE` from a behavioural failure. An absent column and a
  stationary robot look identical to a threshold comparison and have opposite causes

```text
An artefact whose validity was never checked is retained as a file and cited as
nothing.
```

## 8. Guards must not fail on their own required markers

The forbidden-claim scanner is part of the evidence chain, so its own defects are
evidence defects. Two were found, and the second is the more dangerous:

| Defect | Effect |
|---|---|
| CJK phrases wrapped in `\b` word-boundary anchors | Python `re` has no boundary between two CJK characters, so the patterns never matched mid-sentence and the scanner silently passed the claims it exists to block |
| the scanner flagged `NOT REAL ROBOT DATA` and `非真实机器人数据` | those markers are **mandatory** on every evidence document |

```text
RULE: a guard that fails on the markers it requires is a guard someone switches off,
and a switched-off guard passes everything.
```

Every guard in the evidence chain is validated in both directions before it is trusted:

```text
POSITIVE cases caught:      required
NEGATIVE cases not flagged: equally required
```

The scanner now stands at 11/11 positives caught and 10/10 negatives clean. A guard
reporting "no findings" is meaningless until its positive set has been demonstrated.

## 9. Run class travels with the number

```text
RUN_CLASS:              PRECHECK | FINAL_EVIDENCE
NOT_FINAL_EVIDENCE:     TRUE for a precheck
NOT_FOR_DOCUMENT_CLAIM: TRUE for a precheck
EVIDENCE_LEVEL:         written in full words
```

A precheck establishes that the chain runs and the criteria evaluate against real
recorded columns. It is not evidence: it is headless, it has no visible evidence, and
it has no verified manifest.

```text
RULE: run class is written INSIDE metadata.json, INSIDE result.json, and at the TOP
of result.md. A directory name is not a label.
```

The reason is mechanical rather than stylistic: a figure, a number, or a table copied
out of a directory leaves the directory name behind, and the copy is what ends up in a
report.

A deterministic figure that appears in two runs from identical synthetic input is
**one** measurement reported twice. It is never presented as two independent
corroborating measurements.

## 10. Verification before any claim of implementation

```text
NOT VERIFICATION: a file-transfer command that exited 0.
NOT VERIFICATION: a subagent summary stating the work was done.
```

Both describe an intent to change a target. Neither observes it. Before anything is
called verified on a target:

1. read the target path back
2. `grep` structurally for the symbol, class, or field that should now exist
3. `py_compile` the target copy
4. where a data path is involved, run a sanity check that the value actually lands in
   the artefact

Step 4 is not redundant. A duplicated class definition compiles cleanly and passes a
`grep`, while silently shadowing the version that carries the field the evidence needs.
Only a runtime round-trip catches it.

Status vocabulary, never used interchangeably:
`LOCAL_STAGING_ONLY`, `WRITTEN_TO_TARGET_WORKTREE`, `TARGET_WORKTREE_VERIFIED`,
`GIT_COMMITTED`, `GIT_PUSHED`.
