# 04 — Kinematics Update Map

```
STATUS          MAP ONLY — no numeric value was changed in this round
VALUES_CHANGED  NONE
```

If the physical measurement produces a value different from the current
repository value, these are every location that must be checked, and which of them
must actually change.

Locations were established by grep across the repository, not from memory.

## wheel_track — authoritative value currently 0.4008 m

| PARAMETER | AUTHORITATIVE_VALUE | FILE | KEY | CONSUMER | UPDATE_REQUIRED | VERIFICATION |
|---|---|---|---|---|---|---|
| `wheel_track` | 0.4008 | `src/ylhb_base/config/base_kinematics.yaml` | `zlac8015d_canopen_controller.ros__parameters.wheel_track` | active zlac backend, command + odometry | **YES** | `test_base_kinematics_config_chain.py::test_zlac_kinematics_values_unchanged` will fail until its expected value is updated too |
| `wheel_track` | 0.4008 | `src/ylhb_base/config/base_kinematics.yaml` | `base_controller.ros__parameters.wheel_track` | stm32 fallback, command + odometry | **YES — must change in the same edit** | `test_shared_wheel_track_is_identical_across_backends` fails on a one-sided edit |
| `wheel_track` | 0.25 | `src/ylhb_base/src/base_controller.cpp:30` | `declare_parameter<double>("wheel_track", 0.25)` | fallback default when no YAML supplied | **DEFERRED** — see section 3 | `test_stm32_wheel_track_is_not_the_cxx_default` guards against the YAML matching it |
| `wheel_track` | 0.25 | `src/ylhb_base/src/zlac8015d_canopen_controller.cpp:887` | `double wheel_track_ = 0.25` | overridden by YAML, inert on the default path | **DEFERRED** | as above |
| `wheel_separation` | 0.4008 | Gazebo P1 URDF (`dg202611_gazebo_p0_ws`) | diff drive plugin `wheel_separation` | simulation only | **YES, if simulation must track reality** | Gazebo P1 smoke test; separate workspace, separate decision |

Note the two C++ defaults are the *same wrong number* in two files. Neither is
reached on the active path today.

## wheel_radius — authoritative value currently 0.0865 m

| PARAMETER | AUTHORITATIVE_VALUE | FILE | KEY | CONSUMER | UPDATE_REQUIRED | VERIFICATION |
|---|---|---|---|---|---|---|
| `wheel_radius` | 0.0865 | `src/ylhb_base/config/base_kinematics.yaml` | `zlac8015d_canopen_controller.ros__parameters.wheel_radius` | active backend, RPM conversion + odometry | **YES** | `test_zlac_kinematics_values_unchanged` |
| `wheel_radius` | 0.076 | `src/ylhb_base/src/zlac8015d_canopen_controller.cpp:886` | `double wheel_radius_ = 0.076` | overridden by YAML | **DEFERRED** | none currently |
| — | n/a | `src/ylhb_base/src/base_controller.cpp` | no `wheel_radius` parameter exists | stm32 sends mm/s; firmware owns radius | **NO** | `test_stm32_block_does_not_declare_wheel_radius` asserts it stays absent |
| `footprint_to_base_z` | 0.0865 | `src/ylhb_base/urdf/ylhb.urdf.xacro` | `footprint_to_base_joint` origin z | TF: base_footprint → base_link | **YES** | no automated test; visual/manual check |
| `wheel_radius` | 0.0865 | Gazebo P1 URDF | `wheel_radius` property | simulation geometry and `wheel_diameter` | **YES, if simulation must track reality** | Gazebo P1 smoke test |

`footprint_to_base_z` is the corroborating source that gives `wheel_radius` more
confidence than `wheel_track` has. It is a *consequence* of the radius — base_link
sits at axle height — so if the radius changes, this must change with it or the TF
tree silently disagrees with the kinematics.

## Related values that are NOT the same quantity

Do not update these when the radius or track changes; they are independent.

| value | file | why it is unrelated |
|---|---|---|
| `body_radius` 0.27675 | URDF, `nav2_params.yaml` footprint | chassis envelope, MEASURED separately |
| `caster_radius` 0.030 | Gazebo P1 URDF | simulation caster, SIMULATION_ASSUMPTION |
| `actual_velocity_unit_per_rpm` 10.0 | `zlac8015d.yaml:34` | driver feedback unit scale — affects odometry, but is a protocol constant not a geometry value |
| `target_velocity_unit_per_rpm` 1.0 | `zlac8015d.yaml:32` | as above, command direction |

The two `*_unit_per_rpm` values deserve a flag: they multiply into odometry exactly
like radius does. A wrong feedback scale is indistinguishable from a wrong radius
in the odometry output. If a Level B rolling-radius measurement disagrees with a
Level A geometric measurement by a suspicious factor, check
`actual_velocity_unit_per_rpm` before concluding the wheel is the wrong size.

## Update procedure, when a measurement lands

1. Update **both** YAML blocks in the same edit. The consistency test exists to
   catch a one-sided change.
2. Update the expected values inside
   `test_base_kinematics_config_chain.py` — two tests deliberately pin 0.4008 and
   0.0865, so they will fail until updated. That failure is the intended
   tripwire, not a nuisance.
3. If the radius changed, update `footprint_to_base_z` in `ylhb.urdf.xacro`.
4. Decide separately whether the Gazebo workspace follows. It is a different
   workspace and its evidence is frozen; changing it invalidates the P0/P1
   comparison baseline unless deliberately re-run.
5. Re-run the full `src/ylhb_base/test` suite.
6. Record the new value's source class as `PHYSICALLY_MEASURED`, with the
   measurement record referenced.
