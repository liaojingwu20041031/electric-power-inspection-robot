# 01 — URDF Geometry Fact Map

```
AUDIT_CLASS    READ_ONLY
URDF_MODIFIED  NO
CAD_MODIFIED   NO
SOURCE         src/ylhb_base/urdf/ylhb.urdf.xacro  (85 lines)
```

## 1. Links and joints actually present

| link | geometry | source class |
|---|---|---|
| `base_footprint` | none (frame only) | URDF_EXPLICIT |
| `base_link` | cylinder r=0.27675, l=0.3255, origin `-0.09025 0 0.07625` | URDF_EXPLICIT + COMMENT_MEASUREMENT |
| `imu_link` | box 0.02 cubed (visual placeholder) | URDF_EXPLICIT |
| `laser_link` | cylinder r=0.035, l=0.03 (visual placeholder) | URDF_EXPLICIT |
| `gps_link` | none (frame only) | URDF_EXPLICIT |

| joint | parent to child | origin xyz | source class |
|---|---|---|---|
| `footprint_to_base_joint` | base_footprint to base_link | `0 0 0.0865` | URDF_EXPLICIT |
| `base_to_imu_joint` | base_footprint to imu_link | `0.1095 0 0.110` | COMMENT_MEASUREMENT |
| `base_to_laser_joint` | base_link to laser_link | `-0.090 0 0.1805` | URDF_EXPLICIT |
| `base_to_gps_joint` | base_footprint to gps_link | `0 0 0` | **PROVISIONAL** |

## 2. What the URDF does NOT contain

- **No wheel links or joints at all.** `grep -c wheel ylhb.urdf.xacro` returns **0**.
  No wheel link, no axle, no transmission. The URDF therefore makes no geometric
  statement whatsoever about wheel track or wheel radius.
- **No camera / ZED link.** The stereo camera has no frame in the real URDF.
- **No mass, no centre of mass, no inertia.** No `<inertial>` element anywhere.
- **No collision geometry** on imu_link, laser_link or gps_link — visuals only.
- **No rotations.** Every `rpy` in the file is `0 0 0`; no sensor orientation is
  declared.

## 3. The body envelope is the one measured-labelled geometry, and it corroborates

```xml
<!-- 2. base_link, described in the file as an eccentric cylinder envelope,
        labelled as measured -->
<cylinder radius="0.27675" length="0.3255"/>
<origin xyz="-0.09025 0 0.07625"/>
```

This has genuine independent corroboration. `src/ylhb_base/config/nav2_params.yaml`
carries a 16-gon costmap footprint whose extent is x −0.367 … +0.1865 and
y ±0.27675 — the same eccentric circle, radius 0.27675 centred 0.09025 m behind
base_footprint, written in a different file for a different consumer (Nav2
costmaps, not TF).

```
BODY_ENVELOPE_SOURCE_COUNT  2   (URDF visual/collision, nav2 footprint polygon)
BODY_ENVELOPE_CORROBORATED  YES
```

Derived vertical extent: base_link z span is `0.07625 ± 0.16275` = −0.0865 … +0.2390.
The bottom therefore sits exactly at ground level when base_link is 0.0865 m above
it. **The body envelope and the 0.0865 axle height are geometrically coupled inside
the URDF**, so they are not independent of one another. That coupling is the crux of
the wheel-radius question in doc 03.

## 4. Parentage inconsistency worth recording

`laser_link` hangs off `base_link`, while `imu_link` and `gps_link` hang off
`base_footprint`. The two frames differ by exactly 0.0865 m in z, so any consumer
that mixes them without care silently inherits that offset. The file is
self-consistent; this is a trap for anyone reading one origin in isolation, not a
defect.

## 5. Simulation URDF is a separate artefact and must not be conflated

`/home/weiyang/dg202611_gazebo_p0_ws/src/dg_gazebo_sim/urdf/dg202611_p0.urdf.xacro`
does contain wheel links, a `wheel_radius` property of 0.0865 and a `wheel_width`
of 0.05. Those were authored for the Gazebo P0/P1 lines by copying the config
value and inventing what the repository lacked.

They are **not** an independent source for the real vehicle. `wheel_width = 0.05`
in particular is a `SIMULATION_ASSUMPTION` with no repository basis, and citing the
simulation URDF as corroboration of the real geometry would be circular.
