# 01 — Wheel Track Measurement Protocol

```
STATUS                    PROTOCOL ONLY — no measurement performed by the dev line
REAL_MEASUREMENT_PERFORMED  NO
OWNER_ACTION_REQUIRED     YES
```

## 1. Definition being measured

`wheel_track` is the **lateral distance between the effective rolling centrelines
of the left and right drive wheels**, taken at the ground contact patch.

In the code this is the `L` in `ω_z = r(ω_r − ω_l)/L` and
`v_{l,r} = v ∓ ω_z·L/2`, verified in
`ylhb_base/include/ylhb_base/zlac_drive_mapping.hpp`.

### Do NOT measure any of these

| wrong quantity | why it differs |
|---|---|
| body outer width | chassis is a 553.5 mm eccentric cylinder, unrelated to track |
| wheel outer-face to outer-face | overstates track by one tread width |
| wheel inner-face to inner-face | understates track by one tread width |
| caster spacing | casters are not drive wheels |
| CAD shell diameter | enclosure, not contact geometry |

## 2. The centreline problem, and the way around it

Tread width is **unknown**: the repository contains no wheel width, tyre or tread
data anywhere. Grep across `src/` and `docs/` returns no hit, and the real
`ylhb.urdf.xacro` contains no wheel links at all. The `wheel_width = 0.05` in the
Gazebo model is a `SIMULATION_ASSUMPTION` introduced by the simulation line and
carries no authority for the physical vehicle.

So do not try to eyeball or mark a centreline. Measure two well-defined faces
instead and derive the centreline distance:

```
O = outer face to outer face   (left wheel outer edge -> right wheel outer edge)
I = inner face to inner face   (left wheel inner edge -> right wheel inner edge)

wheel_track = (O + I) / 2
tread_width = (O − I) / 2
```

Both faces are physically crisp, unlike a painted centre mark. The method also
self-checks: the derived `tread_width` must be a plausible wheel width and must be
consistent between trials. If it is not, one of the two readings is wrong.

## 3. Procedure

1. Park on **flat, hard, level ground**. Not carpet, not grass, not a slope.
2. **Disconnect drive power**, or otherwise ensure the drive cannot energise.
   Nothing in this protocol requires the motors.
3. Push the vehicle straight for roughly one metre and let it come to rest, so
   the wheels settle into their natural straight-ahead attitude. A wheel that is
   scrubbed sideways sits at an angle and biases the reading.
4. Confirm both wheels carry normal load and the vehicle sits at its usual ride
   height. Do not lift it.
5. Measure `O`: left wheel outer face to right wheel outer face, taken as low as
   the tool allows, close to the contact patch, perpendicular to the travel axis.
6. Measure `I`: left wheel inner face to right wheel inner face, same height and
   same perpendicularity.
7. Record both raw numbers. Do **not** compute in your head — write `O` and `I`
   into the record template and let the arithmetic happen afterwards.
8. **Reposition the vehicle** — push it a short distance and let the wheels resettle
   — then repeat from step 5.
9. Complete **at least 3 trials**, each after a reposition.
10. Photograph the tool in place for at least one trial of `O` and one of `I`.

Repositioning between trials matters. Three readings taken without moving the
vehicle mostly re-measure the same setup error and will agree with each other
while all being wrong together.

## 4. Perpendicularity and height

The two dominant systematic errors are:

- **Skew**: measuring across a diagonal rather than perpendicular to the travel
  axis always reads *long*. Keep the tool square to the wheel axis.
- **Height**: measuring near the top of a wheel rather than near the contact
  patch reads short on any wheel that is not perfectly cylindrical, and picks up
  camber if the wheels are not exactly vertical.

If a straightedge or square is available, lay it against a wheel face to establish
the perpendicular before measuring.

## 5. How precise does this need to be?

This is derivable rather than a matter of taste. Yaw scale error is linear in
track:

```
ω_reported / ω_true = L_true / L_used
```

So a fractional error `ε` in track produces a fractional error `ε` in yaw rate.

| desired yaw scale accuracy | required track accuracy on ~0.40 m |
|---|---|
| 1% | ±4 mm |
| 0.5% | ±2 mm |
| 0.25% | ±1 mm |

A steel tape resolving 1 mm is therefore adequate for roughly 0.25% yaw scale, and
the limiting factor will be technique (skew, height, wheel settling), not the tool.
Straight-line travel is unaffected by track error — `L` cancels in the mean of the
two wheel speeds — so this measurement matters for rotation and heading only.

## 6. Deliverable

Fill in `03_physical_measurement_record_template.md`. Report raw `O` and `I` per
trial, not a pre-averaged single number. Return the completed record to MASTER.

The freeze decision is made from that record, not from the fact that the
repository already contains 0.4008. If the measurement disagrees with 0.4008, the
measurement wins and `04_kinematics_update_map.md` lists every location to update.
