# 02 — Wheel Radius Measurement Protocol

```
STATUS                      PROTOCOL ONLY — no measurement performed by the dev line
REAL_MEASUREMENT_PERFORMED  NO
LEVEL_B_EXECUTED            NO  (prepared only; requires motion, not authorised)
```

Current repository value: `wheel_radius = 0.0865 m`.

Unlike `wheel_track`, this value already has a second corroborating source: the
URDF's `footprint_to_base_z` is also 0.0865, i.e. `base_link` sits at axle height.
Two independent files agreeing raises confidence but neither is a measurement.

## 1. Two levels, measuring two different things

| level | measures | determines |
|---|---|---|
| A — geometric loaded radius | axle centre height above ground, under load | a good first check |
| B — effective rolling radius | distance travelled per wheel revolution | what odometry scale **actually** depends on |

These are not the same number. Odometry converts wheel revolutions into distance,
so the quantity that governs reported distance is the effective rolling radius. On
a solid wheel the two are close; on anything compliant, loaded radius is smaller
than free radius and rolling radius sits between them.

**Level B is the authoritative one.** Level A is the sanity check you can do today
without moving the vehicle.

## 2. Level A — geometric loaded radius (do this now)

1. Flat, hard, level ground. Drive power disconnected.
2. Vehicle resting at normal ride height, normal payload state, not lifted.
3. Measure the **vertical** distance from the ground to the **axle centre**, for
   the left wheel.
4. Repeat for the right wheel. Record them **separately** — a left/right
   difference indicates uneven tyre state, uneven load or a bent mount, all of
   which matter more than the average.
5. Reposition the vehicle, let the wheels resettle, repeat. **At least 3 trials
   per side.**
6. Photograph the tool in place for at least one trial.

If the axle centre is not directly accessible, measure the overall wheel diameter
at the vertical and halve it, and record that you did so — that reading is the
free diameter and will slightly exceed the loaded radius on a compliant wheel.
Note which method was used in the record; do not silently mix methods between
trials.

## 3. Level B — effective rolling radius (PREPARED, NOT AUTHORISED)

This requires the vehicle to move and is **not** part of this round. It is
documented so it can be executed later without redesigning it.

Principle:

```
r_effective = D / (2π · N)
```

where `D` is a straight-line distance travelled and `N` is the number of wheel
revolutions over that distance.

Two ways to obtain `N`:

**B1 — mechanical, no electronics.** Mark a reference point on the wheel and on
the ground. Push the vehicle in a straight line by hand and count full wheel
revolutions until the mark returns to the ground contact point, for as many
revolutions as the run allows. Measure `D` with a tape. Requires no driver, no
CAN, no power. This is the safest form and can be done by hand.

**B2 — encoder-based.** Drive a measured straight line and read wheel revolutions
from motor feedback. This gives a much better `N` but requires the drive
energised, `/cmd_vel` published and the CAN interface open, so it needs its own
authorisation and safety preconditions.

Requirements common to both:

- straight line only; any curvature makes the two wheels travel different
  distances and invalidates a single-radius fit
- `D` of at least 5 m, ideally 10 m; error in `r` scales as error in `D` divided
  by `D`, so a longer run directly buys precision
- count **whole** revolutions where possible; a partial revolution is the largest
  error source in B1
- measure left and right separately if the two wheels can be counted independently
- at least 3 runs

Precision available from B1: with `D = 10 m` measured to ±5 mm and whole-revolution
counting, `r` lands within roughly 0.05%, which is far better than any tape
measurement of the axle height.

## 4. How precise does this need to be?

Also derivable. Reported distance and linear velocity both scale linearly with
radius:

```
v = r/2 (ω_l + ω_r)
```

So a fractional error `ε` in radius produces a fractional error `ε` in every
reported distance and speed.

| desired distance accuracy | required radius accuracy on 0.0865 m |
|---|---|
| 1% | ±0.87 mm |
| 0.5% | ±0.43 mm |
| 0.1% | ±0.09 mm |

This is the key reason Level B exists: **a 1 mm tape error on an 86.5 mm radius is
already a 1.2% odometry scale error**, and 1 mm is about the best a tape against a
wheel will give. Level A cannot realistically deliver better than ~1%. Level B on
a 10 m run can.

Unlike track, radius error affects straight-line travel directly, so it degrades
every odometry estimate rather than only rotation.

## 5. Left/right asymmetry

Record both sides separately at every level. The code uses a **single**
`wheel_radius` for both wheels
(`zlac_drive_mapping.hpp` applies one `wheel_radius_` to both). If the two wheels
measure meaningfully differently, that is a finding in its own right: a single
scalar cannot represent it, and the vehicle will curve when commanded straight.
Report the asymmetry to MASTER rather than averaging it away.

## 6. Deliverable

Fill in `03_physical_measurement_record_template.md`, Level A now and Level B when
authorised. Report per-side, per-trial raw readings. The freeze decision is made
from the record.
