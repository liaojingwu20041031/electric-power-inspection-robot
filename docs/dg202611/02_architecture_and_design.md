# DG-202611 Round B — 02 Architecture and Design

All line references in this document were read on the remote host during Round B.
Prefix for every relative path:
`/home/weiyang/dg202611_ws/src/electric-power-inspection-robot`

## 1. Active Relocalization state machine — as implemented

Source: `src/ylhb_base/scripts/active_relocalization_core.py:15-24`

There are **exactly ten** states:

```text
NORMAL  SUSPECTED  TRIGGERED  STOPPING  ACTIVE_SCAN
WAITING_CANDIDATE  VERIFYING  RECOVERED  FAILED  MANUAL_REQUIRED
```

`LOST` and `SEARCHING` **do not exist anywhere in the code**. Any document,
diagram, or report using those names is describing something that is not this
system. Only the ten names above may be used.

## 2. Transitions — as implemented

| From | To | Condition |
|---|---|---|
| `NORMAL` | `SUSPECTED` | 2nd consecutive triggerable health sample (`suspect_samples=2`) |
| `SUSPECTED` | `TRIGGERED` | 3rd consecutive triggerable sample (`trigger_samples=2`) |
| `TRIGGERED` | `STOPPING` | next tick, unconditionally |
| `STOPPING` | `ACTIVE_SCAN` | odom fresh **and** `abs(linear) <= 0.03` **and** `abs(angular) <= 0.03` **and** yaw finite |
| `ACTIVE_SCAN` | `WAITING_CANDIDATE` | rotated to `segment_target_yaw` within 1.5 deg, then settled 0.5 s |
| `WAITING_CANDIDATE` | `VERIFYING` | a candidate that is accepted **and** inside threshold |
| `VERIFYING` | `RECOVERED` | 3 consecutive samples with position jump `<=0.5 m` and yaw jump `<=20 deg` |
| `WAITING_CANDIDATE` / `VERIFYING` | `ACTIVE_SCAN` | failure: step to the next segment |
| `ACTIVE_SCAN` | `FAILED` | segments exhausted |
| `FAILED` | `STOPPING` | retry while `attempt_id < max_attempts` (`max_attempts=2`) |
| `FAILED` | `MANUAL_REQUIRED` | attempts exhausted |

`RECOVERED` is **absorbing**. `WAITING_CANDIDATE` is the state that publishes a
seed on `/dg/relocalization/seed`.

Verified configuration defaults (`active_relocalization_core.py:66-91`):

```text
suspect_samples=2   trigger_samples=2   healthy_recovery_samples=3
max_covariance=0.5  min_lidar_quality=0.2
stop_velocity=0.03  angular_speed=0.18  yaw_tolerance=1.5 deg
segment_timeout=8.0 settle_time=0.5     max_total_rotation=60 deg
segment_deltas=(+10, -20, +10) degrees
max_attempts=2      verification_samples=3
max_verify_position_jump=0.5 m           max_verify_yaw_jump=20 deg
```

`command_linear_x` is **always 0.0** (`active_relocalization_core.py:673`).
Recovery motion is pure rotation. Nothing in the recovery path drives forward.

`STOPPING` has **no timeout**. If the stop condition is never satisfied the
machine stays in `STOPPING` indefinitely. This is the mechanism behind blocker 2
in `04_debug_and_failures.md`.

## 3. The trigger/degraded split — the key finding of Round B

Source: `assess_health`, `src/ylhb_base/scripts/active_relocalization_core.py:192-304`

Health reasons are collected into two separate lists. Only one of them can start
a recovery.

| List | Reasons | Can trigger recovery |
|---|---|---|
| `trigger_reasons` | `AMCL_COVARIANCE_HIGH`, `AMCL_COVARIANCE_INVALID`, `AMCL_STALE`, `SCAN_MATCH_QUALITY_LOW`, `SCAN_MATCH_INVALID`, `SCAN_STALE` | YES |
| `degraded_reasons` | `LIDAR_QUALITY_LOW`, `LIDAR_QUALITY_INVALID`, `GNSS_DEGRADED`, `GNSS_REJECTED`, `ODOM_STALE` | NO |

Consequence, stated plainly:

```text
S02-S04-style GNSS/LiDAR degradation can NEVER trigger active
relocalization. This is by construction, not a tuning accident.
```

Therefore the historical S02–S04 warning
`RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE` **must not be read as
closed-loop success**. It records bounded activity observed near a scenario
boundary; it does not record a trigger, a recovery attempt, or a recovery.

This is the single most important design rationale of Round B: a scenario that
only degrades GNSS or LiDAR quality can never exercise the state machine, so
S05–S08 must drive a `trigger_reasons` member — in practice AMCL covariance or
scan-match quality — rather than GNSS/LiDAR degradation.

## 4. Navigation Health mapping

Source: `src/ylhb_base/scripts/navigation_health_core.py:253-262`

| Relocalization state | Navigation `relocalization_state` |
|---|---|
| `TRIGGERED`, `STOPPING`, `ACTIVE_SCAN`, `WAITING_CANDIDATE`, `VERIFYING` | `RECOVERING` |
| `SUSPECTED` | `LOCALIZATION_SUSPECT` |
| `FAILED`, `MANUAL_REQUIRED`, `RECOVERED` | passed through unchanged |
| anything else | `NOMINAL` |

Source: `navigation_health_core.py:199-217`. `RECOVERING` is evaluated **before**
the branch that maps an AMCL `REJECTED` state to `LOCALIZATION_SUSPECT`. That
ordering is deliberate and is why recovery can legally hold control while AMCL
covariance is still high. Without it, the very condition that triggers recovery
would immediately revoke recovery's authority.

## 5. cmd_vel Arbiter

Source: `src/ylhb_base/scripts/cmd_vel_arbiter_core.py:84-109`

- Recovery is forwarded **only** when `navigation_state == RECOVERING`
- `FAILED`, `MANUAL_REQUIRED`, `LOCALIZATION_SUSPECT` force zero output
- A source switch inserts a 0.2 s zero guard before the new source is forwarded
- `NOMINAL`, `DEGRADED`, `RECOVERED` select `NAV`; with no `/cmd_vel_nav`
  publisher this yields zero output with reason `NAV_SOURCE_STALE`

## 6. Injector boundary change

```text
OLD_INJECTOR_BOUNDARY: /gps/fix /gps/rtk_status /scan /odom /amcl_pose /map /initialpose
NEW_INJECTOR_BOUNDARY: the same 7 topics, plus /tf and /tf_static
```

`WHY_CHANGED`: the real scan matcher cannot transform scan points into
`base_footprint` without TF, so S07 was structurally impossible. See blocker 4
in `04_debug_and_failures.md`.

```text
TF_SOURCE:      the synthetic kinematic plant, i.e. a base-driver /
                robot_state_publisher equivalent
TF_SEMANTICS:   odom -> base_footprint dynamic
                base_footprint -> laser static
                no map -> odom
GROUND_TRUTH_ACCESS:            YES, the AMCL surrogate reads synthetic plant truth
ALGORITHM_OUTPUT_FABRICATION:   NO
DIRECT_DG_OUTPUTS_FAKED:        NO
```

This change widens the injector boundary. It is recorded here prominently
because it is the only boundary change in Round B, and because `/tf` is a global
resource: a synthetic TF publisher is robot infrastructure, and it must never be
allowed to become a channel for algorithm results.

## 7. `SYNTHETIC_KINEMATIC_PLANT`

Approved this round as `DECISION_001`. It is the synthetic robot **body** only.

```text
kinematics       planar differential drive, no wheel dynamics
integration      explicit forward Euler
dt               clamped to [0, 0.5] s
update rate      the scenario sensor_hz tick, 10 Hz default
initial state    declared initial pose
velocity input   subscribe-only, from /dg/test_cmd_vel
pose update      yaw += w*dt, then x/y advance along the new yaw
noise            none
latency          none (parameter reserved, default 0)
publishes        /odom and TF only
never publishes  /dg/*, /cmd_vel, /amcl_pose
```

Purpose: without a body that actually turns when commanded, the closed loop
cannot exist. See blockers 2 and 3 in `04_debug_and_failures.md`.

This is not Gazebo physics and not a calibrated robot model.

## 8. `SYNTHETIC_AMCL_SURROGATE`

Approved this round as `DECISION_002`, deliberately kept as a **separately named
component** rather than folded into the plant.

It is **NOT** real AMCL and must never be described as such.

```text
input   synthetic plant ground-truth pose   DIRECT_GROUND_TRUTH_ACCESS: YES
input   the real /scan_match_pose from the node under test
output  /amcl_pose only
error   a deliberate constant pose bias, about 0.15 m and 8 deg
uncert. covariance follows the per-phase schedule/ramp
recover on a real accepted match: bias -> 0 after a configured delay,
        covariance -> a healthy value
noise   none by default, deterministic
```

The bias magnitude is chosen to sit **inside** the matcher's real coarse search
window of +-0.20 m and +-10 deg. That is the whole point: the seed handed to the
real matcher is genuinely **wrong**, so the matcher must recover the true pose
from scan geometry instead of confirming an answer it was already given. See the
tautology failure in `04_debug_and_failures.md` section 7.

Recorded limitation, non-negotiable:

```text
Because the surrogate reads synthetic ground truth, S05-S08 evidences the
SOFTWARE STATE CHAIN, not localization accuracy.
```

## 9. TF topology and the `map -> odom` decision

The **only** TF consumer in the whole DG chain is
`src/ylhb_base/scripts/scan_map_relocalization_node.py:452`, in
`_scan_to_base_points`, which looks up `base_footprint <- scan.header.frame_id`
(i.e. `base_footprint <- laser`). No other DG node uses TF at all.

`src/ylhb_base/scripts/multisource_fusion_node.py:196-197` explicitly logs:

```text
publish_tf is intentionally ignored; fusion POC never owns map->odom TF
```

Approved TF edges:

| Edge | Kind | Source |
|---|---|---|
| `odom -> base_footprint` | dynamic | synthetic kinematic plant (`DECISION_003`) |
| `base_footprint -> laser` | static | synthetic kinematic plant (`DECISION_004`) |

Denied TF edge:

| Edge | Status |
|---|---|
| `map -> odom` | DENIED (`DECISION_005`) |

The full reasoning, as ten explicit questions, is in
`MAP_ODOM_TF_JUSTIFICATION.md`. Summary: questions about which file needs it,
which function reads it, and how it would affect the candidate pose or score have
**no answer, because no consumer exists**. The leakage path is therefore absent
rather than mitigated, and the transform is not part of the approved design.

Consequence for visualization: because there is no `map -> odom`, RViz uses
`odom` as its fixed frame (verified: `Fixed Frame: odom`,
`src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz:74`) and the map
is displayed through a visualization-only `MarkerArray` on
`/dg_validation_viz/markers`, deliberately outside `/dg/*`
(verified: `src/dg_synthetic_validation/dg_synthetic_validation/visualization_markers_node.py:7,39`).

## 10. Decisions `DECISION_001` .. `DECISION_008`

| ID | Decision | Alternatives considered | Consequence |
|---|---|---|---|
| `001` | Synthetic base plant approved | Keep the pinned-constant `/odom` of S01–S04; drive the loop with a scripted pose sequence | Without a body that turns, `STOPPING` and `ACTIVE_SCAN` are unreachable. A scripted pose would have to encode the expected answer, so it was rejected |
| `002` | The synthetic AMCL surrogate must be a separately named component, not folded into the plant | Fold the AMCL response model into `PlantConfig` (the first attempt) | A body and a localization observer have different truth boundaries. Only the surrogate touches ground truth, and keeping it separate makes that limitation auditable instead of hidden inside a "plant" |
| `003` | `odom -> base_footprint` TF approved | No TF; publish only `/odom` | The real matcher needs a TF chain to reach `base_footprint`. This edge is exactly what a real base driver publishes |
| `004` | `base_footprint -> laser` TF approved | Set `scan.header.frame_id` to `base_footprint` to dodge the lookup | Dodging the lookup would bypass the real code path under test. A static edge is what `robot_state_publisher` would provide |
| `005` | A perfect `map -> odom` is NOT approved by default | Publish a perfect ground-truth `map -> odom` so RViz shows the map frame | No DG consumer exists, so it buys nothing and risks ground-truth leakage. RViz uses `odom` as fixed frame and a marker overlay instead |
| `006` | No synthetic `/cmd_vel_nav` | Publish a synthetic nav command so the arbiter forwards `NAV` after recovery | Fabricating a navigation command would fake the one thing the arbiter is being tested on. Zero output with `NAV_SOURCE_STALE` is the honest result |
| `007` | `NAV_SOURCE_STALE` after `RECOVERED` is acceptable | Enable Nav2; fake `/cmd_vel_nav`; treat it as a failure | Nav2 is intentionally disabled this phase. The behaviour is correct arbiter conduct and is recorded as safe control handoff, never as navigation resuming |
| `008` | No core threshold or state-machine modifications | Loosen `stop_velocity`, `max_covariance`, or the match thresholds to make a run pass | Changing the system under test to make the test pass destroys the evidence. All four blockers were fixed on the synthetic input side instead |

Verified consequence of `DECISION_008`: the offline matcher verification in
`03_implementation_log.md` section 6 confirmed that **no** threshold, state
machine, or algorithm semantic change is needed.

## 11. Evidence field-role model

Every recorded column carries a role, so that a reader can tell what the test rig
supplied from what the software under test produced. Verified from
`field_roles.csv` in a precheck run directory:

| Role | Count | Meaning |
|---|---|---|
| `INPUT` | 15 | supplied by the synthetic rig; never evidence of behaviour |
| `SUT_OUTPUT` | 54 | produced by the real unmodified nodes |
| `DERIVED_ASSERTION` | 2 | computed by the evaluator from the columns above |
| `METADATA` | 4 | run identity |

The 15 `INPUT` columns:

```text
odom_x  odom_y  odom_yaw  odom_linear_x  odom_angular_z
amcl_x  amcl_y  amcl_yaw  amcl_covariance
phase_gnss_quality  phase_lidar_mode  phase_amcl_covariance_scheduled
synthetic_plant_mode  synthetic_amcl_surrogate_mode
tf_base_to_sensor_available
```

The role split is what keeps a synthetic input from being quoted as a result. An
`INPUT` column shows what the rig asked for; only a `SUT_OUTPUT` column shows what the
software did about it.

## 12. Assertion design rule — incomplete evidence is not a behavioural failure

```text
RULE: an assertion that depends on an INPUT column MUST distinguish
      INPUT_EVIDENCE_INCOMPLETE from a genuine behavioural failure.
```

This rule exists because the two conclusions point at **opposite** root causes:

| Verdict | Root cause lives in |
|---|---|
| `INPUT_EVIDENCE_INCOMPLETE` | the recorder / evidence pipeline |
| behavioural failure | the plant, arbiter, supervisor, or matcher |

The concrete case: `s06_plant_yaw_responded_to_command` requires `odom_yaw` to move
by more than 1 degree. Had the `INPUT` columns serialized empty, the criterion would
have failed for lack of evidence, and the failure would have been read as "the
synthetic plant does not respond to commands" while the plant was working correctly.
See `04_debug_and_failures.md` section 10.

An absent column and a stationary robot are indistinguishable to a threshold
comparison. They must not be indistinguishable in the verdict.

## 13. Plot evidence-validity contract

```text
FILE_EXISTS != VALID_EVIDENCE
```

A figure is only written when a data series with at least one finite point was
actually drawn. Two mechanisms enforce it:

| Mechanism | Purpose |
|---|---|
| artists tagged with gid `dg-data-series` | counts measurements only |
| threshold lines tagged with gid `dg-reference-line` | excluded from that count |
| a single write choke point | refuses `savefig` when the counted series is zero |

Tagging rather than counting `ax.lines` is deliberate. `axhline` threshold lines are
drawn from hard-coded algorithm defaults, so a panel containing only threshold lines
carries lines but no measurement, and a naive count would let it self-certify as
populated. The same reasoning governs screenshots, where a black frame is a file with
no content: see `04_debug_and_failures.md` sections 8, 11 and 14.

## 14. Why S06 and S07 reach `RECOVERED` while their navigation does not

Both S06 and S07 ran the full sequence

```text
NORMAL -> SUSPECTED -> TRIGGERED -> STOPPING -> ACTIVE_SCAN
       -> WAITING_CANDIDATE -> VERIFYING -> RECOVERED
```

even though neither scenario is chartered to test recovery. This is real behaviour of
the unmodified code, not a defect: `_handle_verifying` requires only candidate
quality plus pose stability, and does **not** require healthy covariance.

The consequence is a deliberate asymmetry between the two state chains:

| Scenario | `converge_on_match` | Relocalization end state | Navigation end state |
|---|---|---|---|
| S06 | false | `RECOVERED` | `LOCALIZATION_SUSPECT` |
| S07 | false | `RECOVERED` | `LOCALIZATION_SUSPECT` |
| S08 | true | `RECOVERED` | `RECOVERED` |

Mechanism: with `converge_on_match` false the surrogate covariance stays high, and in
`navigation_health_core` the AMCL-`REJECTED` branch outranks `RECOVERED` in the
priority order, so navigation reports `LOCALIZATION_SUSPECT`. S08, with
`converge_on_match` true, is the only scenario whose navigation reaches `RECOVERED`.

The surrogate-mode column corroborates the mechanism rather than restating it:

```text
S06 synthetic_amcl_surrogate_mode: BIASED_NO_CONVERGENCE_CONFIGURED
S07 synthetic_amcl_surrogate_mode: BIASED_NO_CONVERGENCE_CONFIGURED
S08 synthetic_amcl_surrogate_mode: BIASED -> CONVERGED
```

The S08 transition is caused by a real accepted `/scan_match_pose` from the real
matcher, not by a timer. That is why S08 is the scenario that can show a control
handoff at all.

## 15. RViz display contract

A `.rviz` file is configuration, not proof of rendering.

```text
RULE: valid YAML does NOT prove that RViz accepts a display's field set,
      nor that the configured message type matches the topic's message type.
```

Two failure shapes have already occurred on this project, both silent:

| Shape | Example |
|---|---|
| message-type mismatch | `PoseWithCovarianceStamped` topics configured as `rviz_default_plugins/Pose`, which consumes `PoseStamped` |
| unreachable frame | a display in the `map` frame, which cannot resolve because `DECISION_005` means no `map -> odom` exists |

Both render nothing while appearing configured, which is the same trap as a blank
plot. Detail in `04_debug_and_failures.md` section 13.

Division of labour for visible evidence, settled this round:

| Source | Authoritative for |
|---|---|
| `monitor_scenario` | detailed state text and per-signal values |
| RViz | spatial evidence, plus a key-state summary |

## 16. Idle behaviour is not a defect

With no scenario running, nothing publishes `/scan`, `/dg/lidar/filtered_scan` or
`/odom`, so RViz shows no data for those displays and the monitor shows `NO_DATA`.

```text
STATUS: CORRECT AND EXPECTED IDLE BEHAVIOUR
```

It must never be "fixed" by fabricating a publisher. A publisher added to make an
idle view look alive would place fabricated data on a real topic, which is precisely
what `DECISION_006` forbids for `/cmd_vel_nav`, for the same reason.
