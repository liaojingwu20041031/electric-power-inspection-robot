# 03 — Mismatch Impact Analysis

```
AUDIT_CLASS  READ_ONLY_STATIC_AUDIT
NOTE         Qualitative and formula-level only. No real-vehicle number is
             fabricated here, and no measurement was taken from hardware.
```

## 1. Scope correction — this is the headline

```
WHEEL_TRACK_MISMATCH_ON_ACTIVE_DEFAULT_PATH   NOT_CONFIRMED
WHEEL_TRACK_MISMATCH_ON_STM32_FALLBACK_PATH   CONFIRMED
```

The active backend (`base_backend` default `zlac`) consumes
`base_kinematics.yaml` correctly and runs on the high-confidence values
`wheel_track = 0.4008 m` and `wheel_radius = 0.0865 m`.

An earlier report from this development line stated that `base_controller` was
running with 0.25 m as "the current default path" and rated the risk `HIGH`. That
was **wrong**: it missed `base_backend`'s default of `'zlac'`. The mismatch is real
but latent, confined to a fallback that requires an explicit
`base_backend:=stm32` to reach.

```
RISK_SEVERITY   MEDIUM   (latent, fallback-only; was reported HIGH in error)
```

## 2. Formula verification against the standard model

Standard differential drive:

```
v    = r/2 (ω_r + ω_l)
ω_z  = r/L (ω_r − ω_l)
ω_r  = (v + ω_z L/2) / r
ω_l  = (v − ω_z L/2) / r
```

`zlac_drive_mapping.hpp`, `twist_to_wheel_rpm`:

```cpp
const double left_mps  = vx - wz * wheel_track_ * 0.5;
const double right_mps = vx + wz * wheel_track_ * 0.5;
const double meters_per_rev = 2.0 * M_PI * wheel_radius_;
return {left_mps / meters_per_rev * 60.0, right_mps / meters_per_rev * 60.0};
```

`left_mps` is the wheel contact-point linear speed, i.e. `r·ω_l`, so
`ω_l = (v − ω_z L/2)/r` — matches. The RPM step is
`(m/s) / (m/rev) · 60 s/min` — dimensionally correct.

`wheel_rpm_to_twist`:

```cpp
const double left_mps  = rpm.left  / 60.0 * meters_per_rev;
const double right_mps = rpm.right / 60.0 * meters_per_rev;
return {(left_mps + right_mps) * 0.5, (right_mps - left_mps) / wheel_track_};
```

`v = r/2(ω_l + ω_r)` and `ω_z = r(ω_r − ω_l)/L` — both match.

| check | result |
|---|---|
| `L` is wheel track | yes, `wheel_track_` |
| `r` is wheel radius | yes, `wheel_radius_` |
| sign convention | `right − left` for positive yaw, i.e. CCW positive — consistent with REP-103 |
| left/right ordering | consistent between forward and inverse |
| units | m/s and RPM handled explicitly, conversion correct |
| forward/inverse consistency | exact inverses of one another |

The `base_controller` fallback uses the same structure in m/s
(`vl = vx − vth·L/2`, `vth = (vr − vl)/L`), also self-consistent.

**No formula defect was found in either backend.** The only issue is the numeric
value of `L` on the fallback path.

## 3. What a wrong `L` would do, if the stm32 path were used

With `L_used = 0.25` and `L_true ≈ 0.4008`, the ratio is
`L_true / L_used ≈ 1.603`.

### cmd_vel → wheel command

For a commanded `ω_z`, the applied wheel speed difference is
`Δv = ω_z · L_used`, but the yaw actually produced by the vehicle is
`ω_actual = Δv / L_true = ω_z · (L_used / L_true) ≈ 0.624 ω_z`.

Direction: **the robot under-rotates**, delivering roughly 62% of the commanded
yaw rate. Straight-line `v` is unaffected, because `v` is the mean of the two
wheel speeds and `L` cancels.

### wheel feedback → odometry

Odometry computes `ω_reported = Δv_measured / L_used`. For a true yaw rate
`ω_true`, the measured difference is `Δv = ω_true · L_true`, so
`ω_reported = ω_true · (L_true / L_used) ≈ 1.603 ω_true`.

Direction: **odometry over-reports yaw** by about 60%.

### Compounding

The two errors act in opposite senses on the two sides of the loop and therefore
do **not** cancel — they compound the deception. A closed-loop controller
commanding a turn would see odometry claiming the turn is happening faster than it
is, while the vehicle physically turns slower than commanded. Heading error grows
with every turn.

### Straight line

Essentially unaffected. `v = (v_l + v_r)/2` has no `L` term, so pure translation
is correct in both command and odometry.

### Rotation / yaw

Worst affected, per the two paragraphs above. Both magnitude errors are large
(≈0.62x and ≈1.60x) and in opposing directions.

### Localization fusion

The wheel observation fed to the EKF would carry a systematically inflated yaw
rate. A fusion filter cannot distinguish a scale error from genuine motion, so it
would bias the heading estimate rather than reject the input. This is the most
damaging consequence, because it corrupts an input the rest of the stack trusts
and it does so consistently rather than noisily.

No numeric claim is made here about the resulting real-vehicle position error.
That would require a measurement on hardware, which this audit did not perform.

## 4. Why this still matters despite being latent

- `base_backend:=stm32` is a documented, supported argument, not dead code
- the launch file labels it a fallback, which is exactly the path someone reaches
  for when the primary hardware misbehaves — that is, under time pressure
- the failure is silent: nothing logs a warning, and the odometry looks plausible
- the two-fault structure (not passed, and wrong node key) means a partial fix
  appears to work while changing nothing
