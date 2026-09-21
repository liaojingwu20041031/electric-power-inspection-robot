# 05 — Parameter Freeze and Verification Plan

```
STATUS   PLAN ONLY
WHEEL_TRACK_PHYSICALLY_VERIFIED    NO
WHEEL_RADIUS_PHYSICALLY_VERIFIED   NO
```

## 1. What "frozen" means here

A value is frozen when its source class can honestly be written as
`PHYSICALLY_MEASURED`, backed by a completed measurement record, rather than
`REPOSITORY_CONFIG`.

Freezing is **not** the same as the value being right. It means: this number came
from a measurement someone can point at, its uncertainty is recorded, and changing
it later requires a new measurement rather than an opinion.

## 2. Confidence criteria — derived, not invented

There is no official tolerance for this project, and inventing one would be worse
than having none. So the criterion is expressed against the tool that took the
reading:

| condition | rating |
|---|---|
| range across trials ≤ 2 × tool resolution | `PHYSICAL_MEASUREMENT_CONFIDENCE = HIGH` |
| range across trials ≤ 5 × tool resolution | `MEDIUM` — usable, record the spread |
| range larger than that | `REMEASUREMENT_REQUIRED` |

Worked example: a 1 mm tape gives HIGH if three track trials span ≤2 mm, and
`REMEASUREMENT_REQUIRED` if they span >5 mm.

Rationale: the spread across repositioned trials is the only empirical estimate of
total error available — it captures technique, wheel settling and reading error
together. A tighter criterion than the tool can resolve would be theatre.

If `REMEASUREMENT_REQUIRED`, check in this order before re-measuring: was the
vehicle repositioned between trials, was the tool perpendicular, was the reading
taken near the contact patch, is the derived tread width consistent. A large spread
almost always means one of those four, not a genuinely varying wheel track.

## 3. Adequacy against the intended use

Confidence is separate from adequacy. A HIGH-confidence measurement can still be
too coarse for the purpose.

From the sensitivity derivations in protocols 01 and 02:

| parameter | fractional error → effect | 1 mm reading error on this value |
|---|---|---|
| `wheel_track` 0.4008 m | same fractional error in yaw rate | 0.25% yaw scale |
| `wheel_radius` 0.0865 m | same fractional error in all distance and speed | **1.2% odometry scale** |

So a tape measurement is adequate for track and **marginal for radius**. This is
the concrete reason Level B exists: `wheel_radius` should ultimately be frozen from
an effective-rolling-radius run, not from a tape against the axle. Freezing it from
Level A is acceptable as an interim state provided the record says so.

Suggested resulting classes:

```
wheel_track   frozen from protocol 01                 -> PHYSICALLY_MEASURED
wheel_radius  frozen from Level A only                -> PHYSICALLY_MEASURED_GEOMETRIC_INTERIM
wheel_radius  frozen from Level B                     -> PHYSICALLY_MEASURED_EFFECTIVE
```

## 4. Static verification after any update

No hardware, no CAN, no `/cmd_vel`. All of this runs on the source tree.

```bash
R=/home/weiyang/dg202611_ws/src/electric-power-inspection-robot
cd $R

# 1. both backend blocks agree, and neither is the C++ default
python3 -c "
import yaml
c=yaml.safe_load(open('src/ylhb_base/config/base_kinematics.yaml',encoding='utf-8'))
z=c['zlac8015d_canopen_controller']['ros__parameters']
b=c['base_controller']['ros__parameters']
print('zlac  track/radius:', z['wheel_track'], z['wheel_radius'])
print('stm32 track       :', b['wheel_track'])
assert z['wheel_track']==b['wheel_track'], 'backends disagree'
assert b['wheel_track']!=0.25, 'stm32 matches the C++ default'
print('OK')
"

# 2. TF stays consistent with the radius, if the radius changed
grep -n 'footprint_to_base_z' src/ylhb_base/urdf/ylhb.urdf.xacro

# 3. the config-chain suite, including the pinned-value tripwires
python3 -m pytest -q src/ylhb_base/test/test_base_kinematics_config_chain.py

# 4. full package regression
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/ylhb_base/test
```

Expected after a value change: step 3 **fails** on the two tests that pin 0.4008
and 0.0865 until their expected values are updated. That failure is the tripwire
working. Update the test, do not delete it.

## 5. Runtime verification, gated

`ros2 param get` reads a live node and sends no motor command, but for either
backend it requires the node running, which means an open serial port or an open
CAN interface. So it stays gated:

Preconditions, all of them:

- drive power physically disconnected, or wheels off the ground
- `ros2 topic info /cmd_vel -v` shows **0 publishers** before starting anything
- no teleop, no Nav2, no mobile bridge running

Then:

```bash
ros2 param get /zlac8015d_canopen_controller wheel_track
ros2 param get /zlac8015d_canopen_controller wheel_radius
ros2 topic info /cmd_vel -v          # re-check publisher count after
```

This confirms the loaded value at runtime rather than in the file. It is the only
check that would have caught the original stm32 defect from the outside, and it is
worth doing once after any parameter-chain change.

## 6. The C++ default question, deferred

```
DEFENSE_IN_DEPTH_DEFAULT_ALIGNMENT = DEFERRED
```

Three C++ defaults are wrong relative to the repository values: `wheel_track` 0.25
in both `base_controller.cpp:30` and `zlac8015d_canopen_controller.cpp:887`, and
`wheel_radius` 0.076 in `zlac8015d_canopen_controller.cpp:886`. None is reached on
the active path.

Options for MASTER after the measurement lands, with the trade-off stated rather
than a recommendation smuggled in:

1. **Keep them as-is.** Zero change. A silently wrong default remains reachable if
   a future launch omits the YAML.
2. **Align them to the frozen value.** One line each. Removes the silent-wrong
   failure, but embeds a physical measurement in source, which is what produced the
   original divergence.
3. **Make the parameters mandatory at startup** — fail fast if unset rather than
   defaulting. Strongest against silent error, and the only option that surfaces a
   missing YAML instead of hiding it. Costs a small code change and makes any
   launch that forgot the file fail loudly, which is the point.

Option 3 addresses the actual failure mode; options 1 and 2 mitigate it. Not
changed in this round either way.

## 7. Governance suggestion, minimal

- `base_kinematics.yaml` stays the single manual source for the physical values,
  with both backend blocks kept identical by the existing test
- C++ defaults are treated as fallbacks only, never as a source of physical truth
- simulation geometry may mirror the frozen values but its physics-solver settings
  (step size, solver iterations, contact parameters) are a separate category and
  must not be folded into the same file
- a new value enters only via a completed measurement record

No architectural change is proposed. The current structure is adequate once the
values are frozen and the tests hold them consistent.
