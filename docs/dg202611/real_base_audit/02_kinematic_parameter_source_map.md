# 02 — Kinematic Parameter Source Map

```
AUDIT_CLASS  READ_ONLY_STATIC_AUDIT
```

Source classes: `REPOSITORY_CONFIG`, `CXX_DEFAULT`, `LAUNCH_OVERRIDE`,
`MEASURED_OR_HIGH_CONFIDENCE`, `UNKNOWN`, `UNUSED_CONFIG`.

## 1. Active backend — `zlac8015d_canopen_controller`

| parameter | effective value | source class | evidence |
|---|---|---|---|
| `wheel_radius` | **0.0865 m** | `REPOSITORY_CONFIG` + `MEASURED_OR_HIGH_CONFIDENCE` | `base_kinematics.yaml:4`; corroborated because the URDF `footprint_to_base_z` is also 0.0865 (base_link at axle height) |
| `wheel_track` | **0.4008 m** | `REPOSITORY_CONFIG` + `MEASURED_OR_HIGH_CONFIDENCE` | `base_kinematics.yaml:6` |
| `max_linear_speed` | 0.35 m/s | `REPOSITORY_CONFIG` | `base_kinematics.yaml:8` |
| `max_angular_speed` | 0.70 rad/s | `REPOSITORY_CONFIG` | `base_kinematics.yaml:10` |
| `low_channel_is_left` | true | `REPOSITORY_CONFIG` | `base_kinematics.yaml:12` |
| `low_channel_direction` | 1.0 | `REPOSITORY_CONFIG` | `base_kinematics.yaml:14` |
| `high_channel_direction` | −1.0 | `REPOSITORY_CONFIG` | `base_kinematics.yaml:16` |
| `target_velocity_unit_per_rpm` | 1.0 | `REPOSITORY_CONFIG` | `zlac8015d.yaml:32` |
| `actual_velocity_unit_per_rpm` | 10.0 | `REPOSITORY_CONFIG` | `zlac8015d.yaml:34` — affects odometry speed and distance |

C++ member defaults that are overridden and therefore **not** effective:

| parameter | C++ default | source class | status |
|---|---|---|---|
| `wheel_radius_` | 0.076 | `CXX_DEFAULT` | overridden by YAML |
| `wheel_track_` | 0.25 | `CXX_DEFAULT` | overridden by YAML |

Both live at `zlac8015d_canopen_controller.cpp:886-887`. They are read at
lines 579 and 587 respectively.

### Alternate parameter names — present but inert

The controller supports a second spelling for three values, each guarded:

```cpp
get_parameter("wheel_radius", wheel_radius_);
if (!has_parameter_override("wheel_radius")) {        // line 580
  double wheel_diameter = 0.0;
  get_parameter("wheel_diameter", wheel_diameter);
  if (wheel_diameter > 0.0) { wheel_radius_ = wheel_diameter * 0.5; }
}
```

The same `has_parameter_override` guard wraps `max_linear_velocity` and
`max_angular_velocity`. Because `base_kinematics.yaml` sets the primary names
explicitly, all three alternates are `UNUSED_CONFIG` on the default path. This is
a correctly built fallback, not a competing source.

## 2. Fallback backend — `base_controller`

| parameter | effective value | source class | evidence |
|---|---|---|---|
| `wheel_track` | **0.25 m** | `CXX_DEFAULT` | `base_controller.cpp:30`, no YAML in its chain |
| `wheel_radius` | **not declared** | n/a | `grep -c wheel_radius base_controller.cpp` = **0** |
| `max_linear_speed` | not declared | n/a | no clamp in this backend |
| `max_angular_speed` | not declared | n/a | no clamp in this backend |

### `wheel_radius` absence is correct here, not a gap

This node speaks a different interface. It converts to **wheel linear velocity in
m/s** and transmits millimetres per second as int16:

```cpp
// base_controller.cpp:169-173
double vl_target = vx - vth * (wheel_track_ / 2.0);
double vr_target = vx + vth * (wheel_track_ / 2.0);
int16_t vl_send = static_cast<int16_t>(vl_target * 1000.0);
```

No radius is needed because the STM32 firmware owns the conversion from wheel
linear velocity to motor revolutions. The abstraction boundary simply sits
elsewhere than in the zlac backend, which must produce RPM and therefore needs a
radius.

```
WHEEL_RADIUS_CONFIGURABLE   YES for zlac, NOT_APPLICABLE for stm32
CONFIGURABILITY_GAP         NO  (for wheel_radius, on the grounds above)
```

Correction of record: an earlier report from this line described the missing
`wheel_radius` in `base_controller` as a defect. On the evidence above that
framing was wrong. The genuine defect in this backend is `wheel_track` only.

## 3. Same parameters on both directions?

Active backend: yes. Line 480 constructs a single `DifferentialDriveKinematics`;
line 704 uses it for command conversion and line 803 for odometry.

Fallback backend: yes. `wheel_track_` is used at line 169-170 for command and at
line 221 for odometry yaw.

```
CMD_CONVERSION_AND_ODOMETRY_USE_SAME_PARAMETERS   YES  (both backends)
```

## 4. Multiple sources and single source of truth

```
MULTIPLE_PARAMETER_SOURCES_PRESENT   YES
SINGLE_SOURCE_OF_TRUTH_PRESENT       PARTIAL
```

Sources in play: `base_kinematics.yaml`, `zlac8015d.yaml`, C++ member defaults,
alternate parameter names, and the URDF's own copies of radius and track.

No conflict exists on the active path. The partial rating is because
`base_kinematics.yaml` is the single source of truth for the zlac backend only.
`base_controller` has no link to it, and the URDF carries its own independent
copies of the same physical quantities.
