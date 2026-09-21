# DG-202611 — `map -> odom` TF Justification

```text
DECISION:  DECISION_005 — a synthetic map -> odom transform is NOT approved
OUTCOME:   NOT IMPLEMENTED in the approved design, because no consumer exists
STATUS:    DENIED
```

Prefix for every relative path:
`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot`

## 1. Why this document exists

Widening the injector boundary to `/tf` and `/tf_static` was necessary (blocker 4:
the real matcher cannot transform scan points into `base_footprint` without TF).
`/tf` is a global bus, so each proposed edge needed an individual justification
rather than a blanket approval.

A **perfect `map -> odom` from synthetic ground truth** was proposed as part of that
change. It was denied. The ten questions below are the record of why.

## 2. The TF investigation

The whole DG chain was searched for TF use. Result:

| Finding | Evidence |
|---|---|
| The **only** TF consumer in the DG chain | `src/ylhb_base/scripts/scan_map_relocalization_node.py:452`, in `_scan_to_base_points` |
| What it looks up | `base_footprint <- scan.header.frame_id`, i.e. `base_footprint <- laser` |
| Every other DG node | uses TF not at all |
| Fusion's explicit position | `src/ylhb_base/scripts/multisource_fusion_node.py:196-197` logs `publish_tf is intentionally ignored; fusion POC never owns map->odom TF` |

Approved edges:

| Edge | Kind | Decision |
|---|---|---|
| `odom -> base_footprint` | dynamic, from the synthetic plant | `DECISION_003` |
| `base_footprint -> laser` | static, from the synthetic plant | `DECISION_004` |

Denied edge:

| Edge | Decision |
|---|---|
| `map -> odom` | `DECISION_005` |

## 3. The ten questions

### Q1 — Which real file needs `map -> odom`?

```text
NO ANSWER EXISTS. No file needs it.
```

The only TF consumer is `scan_map_relocalization_node.py:452`, and it looks up
`base_footprint <- laser`, not `map <- odom`.

### Q2 — Which function reads it?

```text
NO ANSWER EXISTS. No function reads it.
```

`_scan_to_base_points` is the only function in the DG chain performing a TF lookup,
and it does not request this edge.

### Q3 — What computation would consume it?

```text
NO ANSWER EXISTS.
```

No computation in the DG chain consumes `map -> odom`. The scan matcher works from
scan points expressed in `base_footprint` plus the occupancy grid on `/map` plus a
seed pose delivered as a message on `/dg/relocalization/seed`. The map-frame
relationship enters through the **seed message**, not through TF.

### Q4 — Does it enter the algorithm under test?

```text
NO.
```

### Q5 — Does it affect the candidate pose?

```text
NO ANSWER EXISTS, because Q1-Q4 have no answer.
```

The candidate pose comes from `refine_pose_near_seed` operating on scan geometry
and the seed. There is no code path by which this transform could influence it.

### Q6 — Does it affect the score?

```text
NO ANSWER EXISTS, for the same reason.
```

`score`, `inlier_ratio`, and `mean_distance` are computed from scan points against
the distance field. This transform appears nowhere in that computation.

### Q7 — Ground-truth leakage risk?

A **perfect** `map -> odom` derived from synthetic plant truth would place the true
pose of the robot onto a global bus, at full precision, continuously.

The risk is not only that today's code might read it. It is that:

- any future node could start consuming it silently, and a run would still pass
- a reviewer inspecting the TF tree would see a complete, correct tree and could
  reasonably conclude the pose was being estimated rather than supplied
- the leak would be invisible in `samples.csv`, because TF is not an evaluator field

```text
LEAKAGE_RISK_IF_IMPLEMENTED_PERFECTLY: HIGH
```

### Q8 — Could an independent `ground_truth` frame be used instead?

Yes, technically. Publishing plant truth as, for example, `ground_truth ->
base_footprint_truth`, disjoint from the frames the algorithm reads, would let a
visualization compare estimate against truth without placing truth on a path any
algorithm consumes.

```text
STATUS: NOT ADOPTED, because it is not needed.
```

It was not adopted because the visualization requirement was already met more
cheaply. RViz uses `odom` as its fixed frame (verified: `Fixed Frame: odom`,
`src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz:74`) and the map is
drawn as a visualization-only `MarkerArray` on `/dg_validation_viz/markers`
(verified: `visualization_markers_node.py:7,39`), deliberately outside `/dg/*`. A
separate truth frame remains available if a future comparison genuinely needs one.

### Q9 — Could it come from a real algorithm output?

In a real system, yes: `map -> odom` is normally published by AMCL, i.e. by the
localization algorithm itself, as an **output**.

In this validation it cannot, for two reasons:

- real AMCL is not running; what exists is `SYNTHETIC_AMCL_SURROGATE`, which is not
  AMCL and must never be described as such
- the surrogate reads synthetic plant ground truth, so a transform derived from it
  would be ground truth wearing an algorithm's name, which is worse than an openly
  synthetic transform

The one genuine algorithm output available is `/scan_match_pose` from the real
matcher, and it is already consumed as a message where it belongs.

### Q10 — If only synthetic data could provide it, why would it still not leak the answer?

This is the question that settles the matter, and the honest answer is that **it
would not be safe**, which is why the transform is denied rather than mitigated.

A synthetic `map -> odom` has only two possible forms:

1. **Perfect**, tracking plant truth. This is direct ground-truth publication onto a
   global bus. See Q7.
2. **Deliberately biased**, mirroring the surrogate's pose bias. This is the same
   information the surrogate already delivers through `/amcl_pose`, duplicated onto a
   second channel with no consumer, creating two sources of truth that can disagree.

Neither form is defensible, and neither is needed. So the conclusion is not "the
leakage was mitigated":

```text
The leakage path is ABSENT rather than mitigated, because no consumer exists.
The transform is NOT IMPLEMENTED in the approved design.
```

That distinction matters for future rounds. Nothing here guards against leakage. If
a future node ever does consume `map -> odom`, this entire analysis must be redone
from Q1, because the reason for safety is the absence of a consumer, not a safeguard.

## 4. Consequences of the denial

| Area | Consequence |
|---|---|
| RViz | Fixed frame is `odom`, not `map` |
| Map display | A visualization-only `MarkerArray` on `/dg_validation_viz/markers`, outside `/dg/*` |
| TF tree | Deliberately incomplete: two edges, no `map` frame. An incomplete tree here is the correct state, not a defect |
| Approved TF | `odom -> base_footprint` dynamic, `base_footprint -> laser` static |
| Seed delivery | Through `/dg/relocalization/seed` as a message, never through TF |

## 5. Verified state on the target

```text
STATUS: DECISION_005 IS LANDED ON THE TARGET
CODE_SIDE_LANDING_OF_DECISION_005: TARGET_WORKTREE_VERIFIED
```

Earlier in Round B this section recorded an open delta: the target still emitted a
static **identity** `map -> odom`. That remnant is gone. Verified by read-back of
`src/dg_synthetic_validation/dg_synthetic_validation/synthetic_injector_node.py`:
`_static_transforms` now returns the `base_footprint -> sensor` edge only, and its
docstring states

```text
map->odom is deliberately absent.  No algorithm under test reads it
(the only TF consumer in the chain is the Scan-to-Map node, which
looks up base_footprint <- laser), and publishing a perfect one
would risk feeding ground truth to the software under test.
```

`PlantConfig` in `scenario_schema.py` carries the same statement, and records that
`publish_tf` emits only the two formally approved edges.

Corroborated independently in the evidence: every S05–S08 precheck artefact records

```text
PERFECT_MAP_ODOM_PUBLISHED: NO
GROUND_TRUTH_LEAKAGE_TO_MATCHER: NO
```

How the closure was confirmed matters as much as the closure. This same decision was
once reported as landed while the target still held the old code — the first of two
`STAGING_TARGET_DIVERGENCE` occurrences (`04_debug_and_failures.md` section 9). The
statement above rests on a target read-back plus a structural check of the returned
transform list, not on a transfer summary.

## 6. What would invalidate this analysis

```text
TRIGGER FOR REDOING THIS DOCUMENT FROM Q1: any node beginning to consume map -> odom.
```

The reason the design is safe is the **absence of a consumer**, not a safeguard.
Nothing in the current code guards against leakage through this edge; there is simply
nothing to leak into. If a future node performs a `map <- odom` lookup, the leakage
path stops being absent and every question from Q1 onward has to be answered again.

One consequence for reviewers: the TF tree is deliberately incomplete, two edges and no
`map` frame. An incomplete tree here is the correct state. A complete-looking tree would
be the warning sign.
