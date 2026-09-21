# 04 — Minimal Fix Plan

```
REAL_BASE_CONTROLLER_MODIFIED   NO   (this round; plan only)
FIX_RECOMMENDED                 YES
FIX_APPLIED                     NO
```

Nothing in this plan was executed. It requires a separate MASTER-issued FIX task.

## 1. What must be fixed, and what must not be touched

Fix target: the `stm32` / `base_controller` fallback runs on
`wheel_track = 0.25` while the high-confidence value is `0.4008`, and it cannot be
configured because of two independent faults.

Explicitly **not** in scope: the active `zlac` backend. It is correct. Changing
working configuration to satisfy a tidiness goal would risk the one path the
vehicle actually uses.

## 2. Recommended fix — option A (preferred, smallest)

Two coordinated edits. Either alone is insufficient.

**A1. Give `base_controller` its own parameter block in the existing YAML.**

`config/base_kinematics.yaml` currently keys only on
`zlac8015d_canopen_controller`. Add a second top-level key so one file remains the
single source of truth for both backends:

```yaml
base_controller:
  ros__parameters:
    wheel_track: 0.4008
```

Only `wheel_track` is required. `base_controller` declares no radius and needs
none — it emits wheel linear velocity in mm/s and the STM32 firmware owns the
radius conversion.

**A2. Pass the file to the node in `bringup.launch.py`.**

```python
    stm32_base_node = Node(
        package='ylhb_base',
        executable='base_controller',
        name='base_controller',
        output='screen',
        condition=use_stm32,
        parameters=[
            base_kinematics_path,          # <-- added
            {'serial_port': base_port},
            {
                'publish_tf': False,
                ...
            }
        ]
    )
```

Estimated diff: 3 added lines in the YAML, 1 added line in the launch file. No C++
change, no behavioural change to the active path.

### Why both edits

A1 alone does nothing: the file is not in the node's parameter list.
A2 alone does nothing: ROS 2 matches by node name and the file has no
`base_controller` key. A reviewer seeing only one of these would reasonably
believe the bug was fixed.

## 3. Option B — align the C++ default instead

Change `base_controller.cpp:30` from `0.25` to `0.4008`.

One line, and it removes the silent-wrong-value failure even when no YAML is
supplied. Rejected as the primary recommendation because it hardcodes a physical
measurement into source, which is what created the divergence in the first place,
and it leaves the parameter still unconfigurable from the launch.

Worth doing **in addition** to option A as defence in depth: a default that is
merely imprecise is far safer than one that is wrong by 60%.

## 4. Option C — not recommended

Unifying the URDF, `base_kinematics.yaml` and both controllers behind one
generated parameter source. This is the architecturally clean answer and it is
disproportionate here. It touches the working `zlac` path, the URDF used by the
frozen Gazebo lines, and Nav2 configuration, for a fallback-only defect. Recorded
for completeness and explicitly not proposed for this fix.

## 5. Residual gap that no config fix closes

`wheel_track = 0.4008 m` is `REPOSITORY_CONFIG` and appears in exactly **one**
place in the repository. It has no second corroborating source, unlike
`wheel_radius = 0.0865`, which is independently echoed by the URDF's
`footprint_to_base_z`.

So the fix above makes the fallback consistent with the repository's stated value.
It does **not** establish that 0.4008 is the true wheel track. Confirming that
requires a physical measurement of the distance between the two drive wheels'
ground contact points, which is an owner action and is recorded as such.

## 6. Suggested additional safeguard

A startup log line in `base_controller` stating the effective `wheel_track` and its
origin, e.g. whether a parameter override was present. The current failure is
silent; one log line would have surfaced it. This is a suggestion, not part of the
minimal fix.

---

# FIX RESULT — appended after execution

The plan above is left unedited. This section records what was actually done.

```
FIX_APPLIED                            YES
OPTION_CHOSEN                          A  (YAML block + launch parameter)
OPTION_B_CXX_DEFAULT_ALIGNMENT         NOT_APPLIED  (see below)
OPTION_C_ARCHITECTURAL_UNIFICATION     NOT_APPLIED  (out of scope, as planned)
```

## Files changed

| file | change |
|---|---|
| `src/ylhb_base/config/base_kinematics.yaml` | +16 lines: `base_controller:` block carrying `wheel_track: 0.4008` only |
| `src/ylhb_base/launch/bringup.launch.py` | +4 lines: `base_kinematics_path` added first in `stm32_base_node` parameters |
| `src/ylhb_base/test/test_base_kinematics_config_chain.py` | new, 10 static tests |

20 insertions, 0 deletions in the two production files. No C++ was touched, no
formula was touched, and neither 0.4008 nor 0.0865 was altered.

## Both faults closed together

| fault | before | after |
|---|---|---|
| file not passed to the node | `stm32_base_node` parameters had no `base_kinematics_path` | present, listed first |
| YAML key did not match node name | only `zlac8015d_canopen_controller` | `base_controller` block added |

Verified by AST-level inspection of the launch source:

```
stm32_base_node  name=base_controller                  loads_yaml=True  name_matches_key=True
zlac_base_node   name=zlac8015d_canopen_controller     loads_yaml=True  name_matches_key=True
default backend: zlac
```

## Ordering rationale

`base_kinematics_path` is listed **first**, so the inline dicts that follow still
win on any key they set. They deliberately set no `wheel_track`, so the YAML value
survives. This mirrors the zlac node's existing pattern.

## What was deliberately NOT added

`wheel_radius` is absent from the `base_controller` block. That node declares no
such parameter — it transmits wheel linear velocity as mm/s int16 and the STM32
firmware owns the radius conversion. Listing one would be inert and would imply
the node honours it. A test asserts its absence.

Option B (aligning the C++ default at `base_controller.cpp:30` from 0.25 to
0.4008) was recommended in the plan as defence in depth and was **not applied**,
because this round's authorisation covers the config chain only and that edit
touches real-vehicle control source. It remains recommended.

## Test result

```
test_base_kinematics_config_chain.py    10 passed
src/ylhb_base/test (full suite)        263 passed
```

The suite grew from 253 to 263. No test needs ROS, a CAN interface or hardware.

## Hardware guards during the whole round

```
chassis processes running   0
CAN interfaces up           0
/cmd_vel                    absent, 0 publishers
ROS nodes alive             0
```

## Corrections to the record, preserved

Three findings from the audit stand corrected and are NOT deleted from the
earlier sections:

1. The active `zlac` path was **already correct**. An earlier report from this
   line claimed `base_controller` ran with 0.25 as the current default path and
   rated the risk HIGH. That missed `base_backend`'s default of `'zlac'`. The
   defect was latent, fallback-only, MEDIUM.
2. The missing `wheel_radius` in `base_controller` is **not a repository bug**.
   It reflects a different abstraction boundary, not an omission.
3. Both kinematic formulas were verified sound in both backends. Nothing was
   wrong with the mathematics; only one numeric value on one path.

## Still open after this fix

`wheel_track = 0.4008` is now consistent across the configuration chain. It is
still `REPOSITORY_CONFIG`, appearing in exactly one authoritative place with no
corroborating second source, unlike `wheel_radius = 0.0865` which the URDF's
`footprint_to_base_z` independently echoes.

```
WHEEL_TRACK_PHYSICAL_MEASUREMENT_COMPLETE   NO
```

Closing that requires measuring the distance between the two drive wheels' ground
contact points. Owner action. If the measurement differs, update **both** backend
blocks together and re-run this suite — the consistency test will catch a
one-sided edit.
