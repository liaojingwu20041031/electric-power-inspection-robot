# Gazebo Simulation Readiness Report — DG-202611

**Document type:** read-only audit of the CURRENT repository state.
**Contains no simulation results, no performance claims, no accuracy or reliability
claims, and no statement about real-robot or competition performance.**
Nothing in this document should be read as evidence that any simulation was run.
No simulator was installed, started, or configured while producing it. No code,
config, URDF, or launch file was modified.

| Field | Value |
| --- | --- |
| Repository | `/home/weiyang/dg202611_ws/src/electric-power-inspection-robot` |
| Branch | `dg202611-synthetic-validation` |
| HEAD | `57ec8ecff90d4e383e8c5197105bc8d182474245` (2026-08-28) |
| Host / OS | `ubuntu103` / Ubuntu 22.04.5 LTS |
| ROS distro | Humble (`/opt/ros/humble`) |
| Audit scope | robot description, kinematics config, sensor mounting, Gazebo assets |
| Audit type | READ-ONLY inventory. No build, no test, no scenario, no install. |

---

## 1. Executive verdict

**GAZEBO_SIMULATION_READINESS = BLOCKED**

Blocked on *missing physical data*, not on missing effort or missing tooling.

Three independent reasons, any one of which is sufficient:

1. **No Gazebo asset of any kind exists in the repository.** No `.sdf`, no
   `.world`, no model directory, no plugin, no spawn call, no `ros_gz` /
   `gz_ros2_control` / `gazebo_ros2_control` dependency. See section 2.
2. **The robot description contains no wheels and no movable joints.** The URDF
   has 5 links and 4 joints, and *all four joints are `type="fixed"`*. There is
   no `left_wheel_link`, no `right_wheel_link`, no continuous joint, no
   transmission, and no `ros2_control` block anywhere in the repository. Nothing
   in the current description can be actuated by a simulator.
3. **There is zero mass, zero centre-of-mass, and zero inertia data.** A
   repository-wide search for `<inertial>`, `<mass>`, `<inertia>`, and `ixx`
   across every URDF and xacro returned **no matches at all**. Gazebo treats a
   link with no inertial block as having no dynamics; a physics simulation built
   on the current description would be meaningless.

What *is* in good shape: the planar frame skeleton, the body collision envelope,
and the Nav2 / SLAM / EKF configuration are coherent and mutually consistent, and
are directly reusable (section 7). The gap is exclusively in **rigid-body physical
properties and wheel geometry**, which cannot be recovered from this repository
and must be measured by hand (section 6).

A geometry-only visual shell could be produced from what exists, but it would be
dominated by `SIMULATION_ASSUMPTION` values and would not support any claim about
motion, odometry, traction, tipping, or controller behaviour. This report does not
recommend building one before the section 6 measurements are taken.

---

## 2. GAZEBO_ASSETS_FOUND

**None. No Gazebo, Ignition, or `gz` simulation asset exists in this repository.**

Stated plainly so it cannot be misread: there is nothing to reuse, extend, or fix.
A future Gazebo phase starts from zero on the simulation side.

Searches performed and their results:

| Search | Result |
| --- | --- |
| `*.sdf` | none |
| `*.world` | none |
| directories named `world*`, `gazebo*`, `plugin*`, `mesh*`, `*gz*` | none |
| `spawn_entity`, `gzserver`, `gzclient`, `gz sim`, `create -world` | none |
| `libgazebo*`, `GazeboRosPlugin`, `<gazebo>` tags in any URDF/xacro | none |
| `ros2_control`, `hardware_interface`, `gz_ros2_control` | **none anywhere in `src/`** |
| `.rviz` configs | 2 found (RViz only, not simulation) |

The only directory that could be mistaken for simulation content is
`src/ylhb_perception/models/`, which holds a single `README.md` placeholder for
TensorRT detector weights. It is machine-learning model storage, not Gazebo models.

### Every `gazebo` string that does exist is a disclaimer or an unused flag

This matters, because a careless grep makes the repository look Gazebo-aware when
it is the opposite — the codebase actively asserts Gazebo was *not* used:

- `src/dg_synthetic_validation/dg_synthetic_validation/result_writer.py:24` —
  `"simulator_class": "NOT_GAZEBO_DATA"`
- `src/dg_synthetic_validation/dg_synthetic_validation/result_writer.py:275` and
  `scenario_runner.py:258` — `"gazebo": False`
- `src/dg_synthetic_validation/dg_synthetic_validation/scenario_runner.py:121` —
  `"hardware": {"real_robot_connected": False, "gazebo_started": False}`
- `src/dg_synthetic_validation/dg_synthetic_validation/evidence_manifest.py:123` —
  a regex `\b(gazebo|ignition|isaac\s+sim)\b` that **rejects** simulator claims in
  evidence text
- `docs/dg202611/06_results_and_evidence.md:167`, `07_open_issues.md:157`,
  `08_handoff_summary.md:90` — `Gazebo validation: NOT_RUN`
- `docs/dg202611/CLAUDE_CODE_HANDOFF.md:375` — Gazebo is scheduled as "Phase E",
  i.e. explicitly future work

- Three launch files carry a `use_sim_time` argument whose *description text*
  mentions Gazebo, but the flag is only a standard Nav2/ROS clock switch and no
  simulator is ever launched: `navigation.launch.py:41`,
  `navigation_keepout.launch.py:59`, `mapping.launch.py:17`.

### Simulator tooling on the VM (observed, not installed)

No simulator binary is present: `gazebo`, `gz`, and `ign` are all absent from
`PATH`. The only Gazebo-named packages installed are transitive maths/build
libraries pulled in by other ROS packages, **not a simulator**:

`libignition-cmake2-dev`, `libignition-math6`, `libignition-math6-dev`,
`ros-humble-ignition-cmake2-vendor`, `ros-humble-ignition-math6-vendor`.

Installing a simulator was out of scope for this audit and was not done.

---

## 3. Existing robot description inventory, file by file

### 3.1 `src/ylhb_base/urdf/ylhb.urdf.xacro` — the only robot description

85 lines. Despite the `.xacro` extension it uses **no xacro features at all**: no
properties, no macros, no includes, no arguments. It is plain URDF. It is loaded at
`src/ylhb_base/launch/bringup.launch.py:398` via
`ParameterValue(Command(['xacro ', urdf_file]), value_type=str)` into
`robot_state_publisher`.

**Links (5):**

| Link | Visual | Collision | Inertial | Notes |
| --- | --- | --- | --- | --- |
| `base_footprint` (L5) | none | none | **none** | empty root link, ground projection |
| `base_link` (L8–24) | eccentric cylinder r=0.27675, l=0.3255 | identical cylinder | **none** | comment L7 calls it 实测 (measured) envelope |
| `imu_link` (L34–44) | 0.02 m box | none | **none** | comment L35 says visual is RViz-only |
| `laser_link` (L47–57) | cylinder r=0.035, l=0.03 | none | **none** | comment L48 says RViz-only |
| `gps_link` (L59) | none | none | **none** | bare link, no geometry |

**Joints (4) — every one is `type="fixed"`:**

| Joint | Parent → Child | origin xyz | rpy | Line |
| --- | --- | --- | --- | --- |
| `footprint_to_base_joint` | `base_footprint` → `base_link` | `0 0 0.0865` | `0 0 0` | L27–31 |
| `base_to_imu_joint` | `base_footprint` → `imu_link` | `0.1095 0 0.110` | `0 0 0` | L64–69 |
| `base_to_laser_joint` | **`base_link`** → `laser_link` | `-0.090 0 0.1805` | `0 0 0` | L71–76 |
| `base_to_gps_joint` | `base_footprint` → `gps_link` | `0 0 0` | `0 0 0` | L78–83 |

Three observations that matter for a future Gazebo model:

- **No wheels exist.** There is no wheel link, no wheel joint, no `<axis>`, no
  `<limit>`, no `<transmission>`. `wheel_track` therefore has no geometric
  representation in the description at all — it lives only in a YAML file.
- **Mixed parent frames.** `laser_link` hangs off `base_link` while `imu_link` and
  `gps_link` hang off `base_footprint`. Geometrically valid, but it means the laser
  origin must be read together with the 0.0865 base offset to get its true height
  above ground (0.0865 + 0.1805 = **0.2670 m**). This is an easy off-by-one-link
  mistake to make when porting to SDF.
- **Only `base_link` has collision.** A Gazebo contact model would have the body
  envelope but nothing for wheels or casters.

### 3.2 `src/zed-ros2-wrapper/zed_wrapper/urdf/zed_descr.urdf.xacro` and `zed_macro.urdf.xacro`

Vendored upstream Stereolabs description. **Not referenced by `ylhb.urdf.xacro` and
not launched by any `ylhb_*` package.** It exposes `camera_name`, `camera_model`,
`custom_baseline`, and `gnss_x/y/z` args (all defaulting to `0.0`) but nothing in
this repository ever sets them. It contains no mass or inertia either. Treat as
third-party code that is present but unused by the robot's TF tree.

### 3.3 `CAD/Retail-Cart-3D-Model/` — CAD exists but is not simulation-consumable

A real SolidWorks CAD set is present, which is the most valuable asset for closing
the physical-parameter gap. However **no dimension, mass, or inertia value has been
extracted from it into the repository**, and the formats are native:

- `sldasm/` — 3 assemblies: `小车底盘.SLDASM` (chassis), `万向轮.SLDASM` (caster
  wheel), `机械爪.SLDASM` (gripper)
- `stp/` — ~50 `.SLDPRT` part files (native SolidWorks, *not* STEP despite the
  directory name), including `电机轮.SLDPRT` (motor wheel), `电机.SLDPRT` (motor),
  `万向轮.SLDPRT` / `万向轮底座` / `万向轮支架` (caster + base + bracket),
  `人工智能小车底盘.SLDPRT` (chassis), `上底盘`, `顶板`, `伸缩杆` / `伸缩柱`
  (telescoping mast), `激光雷达固定架` (lidar mount), `双目垫板` (stereo camera
  shim plate), and five arm segments `第二节`–`第五节机械臂`.
- `stl/` — despite the directory name, contains **`.3MF` files only, no `.stl`**.
  A repository-wide `*.stl` / `*.dae` search returned **no mesh files**.
- `readme.txt` — one line, 3D-printing quantities only
  (`支架垫片4个，固定盖2个，底盘连接处支架2个`). No dimensions, no masses.

**Consequence:** there are no meshes a simulator can load, and no exported mass
properties. The CAD *can* yield real numbers (SolidWorks reports mass and inertia
tensors directly once material density is assigned), but that extraction has not
been done. This is the single highest-leverage action available — see section 6.

**Also important:** the CAD shows a **telescoping mast, a 5-segment manipulator
arm, a gripper, and caster wheels**. None of these appear in `ylhb.urdf.xacro`.
The current description models a static cylindrical body only. Any Gazebo model
built solely from the URDF would omit the robot's entire upper structure and its
caster contact points, which changes both the centre of mass height and the
ground-contact behaviour.

### 3.4 No dimension documentation

`src/电力巡检机器人使用与调试手册.md` (88 KB debug manual) was searched for
尺寸 / 重量 / 质量 / kg / 轮距 / 轮径 / 直径 / 长宽高 / 载重. It contains **no
physical dimension or mass table**. Its only geometry-related entry is
`line 301`, which points maintainers at the config files:

> `| CAN 接口、轮径、轮距、速度限幅 | src/ylhb_base/config/zlac8015d.yaml、base_kinematics.yaml | 修改后先台架低速验收 ... |`

That is a pointer, not a measurement record.

---

## 4. Parameter table

Classification legend, applied strictly:
`REAL_MEASURED` = a measurement record exists ·
`REPO_CONFIG_HIGH_CONFIDENCE` = consistent across working config/code ·
`CAD_DERIVED` = extracted from CAD ·
`PROVISIONAL` = flagged tentative in-repo, or inconsistent between files ·
`SIMULATION_ASSUMPTION` = would have to be invented for simulation ·
`MISSING_NEEDS_MEASUREMENT` = absent; must be measured.

**No value in this table was inferred, rounded, or filled in from typical hardware.
Every row is either quoted from a file or marked missing.**

### 4.1 Drivetrain kinematics

| Parameter | Value | Classification | Evidence (file:line) |
| --- | --- | --- | --- |
| `wheel_radius` | **0.0865 m** | REPO_CONFIG_HIGH_CONFIDENCE | `src/ylhb_base/config/base_kinematics.yaml:4` |
| ” corroboration 1 | 0.0865 as `base_footprint`→`base_link` height | REPO_CONFIG_HIGH_CONFIDENCE | `src/ylhb_base/urdf/ylhb.urdf.xacro:30` |
| ” corroboration 2 | `assert base_offset == (0.0, 0.0, 0.0865)` | REPO_CONFIG_HIGH_CONFIDENCE | `src/ylhb_base/test/test_robot_geometry.py:100` |
| `wheel_radius` C++ fallback default | **0.076 m** (diverges) | PROVISIONAL | `src/ylhb_base/src/zlac8015d_canopen_controller.cpp:539` and `:886` |
| `wheel_track` | **0.4008 m** | REPO_CONFIG_HIGH_CONFIDENCE (single source, **not corroborated**) | `src/ylhb_base/config/base_kinematics.yaml:6` |
| `wheel_track` C++ fallback default | **0.25 m** (diverges) | PROVISIONAL | `src/ylhb_base/src/zlac8015d_canopen_controller.cpp:541` and `:887` |
| `wheel_track` STM32 fallback default | **0.25 m** (diverges) | PROVISIONAL | `src/ylhb_base/src/base_controller.cpp:30` |
| Wheel width | — | MISSING_NEEDS_MEASUREMENT | no wheel link exists |
| Wheel collision shape | — | MISSING_NEEDS_MEASUREMENT | only `base_link` has `<collision>` |
| Gearbox / encoder ratio | not stated; only `actual_velocity_unit_per_rpm: 10.0` | PROVISIONAL | `src/ylhb_base/config/zlac8015d.yaml:34` |
| Caster wheel geometry / position | — | MISSING_NEEDS_MEASUREMENT | present in CAD (`万向轮.SLDASM`), absent from URDF |

### 4.2 Body envelope — the best-corroborated geometry in the repository

| Parameter | Value | Classification | Evidence (file:line) |
| --- | --- | --- | --- |
| `base_link` cylinder radius | 0.27675 m | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:12`, `:21` |
| `base_link` cylinder length | 0.3255 m | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:12`, `:21` |
| `base_link` visual/collision origin | `-0.09025 0 0.07625` | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:10`, `:19` |
| visual == collision (locked by test) | equal | REPO_CONFIG_HIGH_CONFIDENCE | `test/test_robot_geometry.py:82–88` |
| Envelope centre above ground | 0.16275 m (locked) | REPO_CONFIG_HIGH_CONFIDENCE | `test/test_robot_geometry.py:101` |
| Nav2 footprint (16-gon of same cylinder) | x ∈ [-0.367, 0.1865], y ∈ [-0.27675, 0.27675] | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:248`, `:295`; locked by `test_robot_geometry.py:104–118` |
| `footprint_padding` | 0.01 m | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:250`, `:297` |

**Geometric self-consistency check (performed during this audit):**
0.0865 + 0.07625 = 0.16275 = 0.3255 / 2. The body cylinder's bottom face therefore
lands **exactly at z = 0 (ground)**. The wheel radius and the body envelope are
mutually consistent to five decimal places, and that relationship is pinned by a
test. This is genuine internal corroboration of 0.0865 — it is not merely repeated,
it is dimensionally coherent with an independently stated envelope.

### 4.3 Rigid-body dynamics — entirely absent

| Parameter | Value | Classification | Evidence |
| --- | --- | --- | --- |
| Total robot mass | — | MISSING_NEEDS_MEASUREMENT | repo-wide `<mass>` search: **0 matches** |
| `base_link` mass | — | MISSING_NEEDS_MEASUREMENT | no `<inertial>` block in any URDF |
| `base_link` centre of mass | — | MISSING_NEEDS_MEASUREMENT | idem |
| `base_link` inertia tensor | — | MISSING_NEEDS_MEASUREMENT | repo-wide `ixx` search: **0 matches** |
| Wheel mass / inertia | — | MISSING_NEEDS_MEASUREMENT | wheel links do not exist |
| Sensor link masses | — | MISSING_NEEDS_MEASUREMENT | `imu_link`, `laser_link`, `gps_link` have no inertial |
| Ground/tyre friction (`mu1`, `mu2`) | — | MISSING_NEEDS_MEASUREMENT / SIMULATION_ASSUMPTION | no `<gazebo>` or surface block anywhere |
| Joint effort / velocity limits | — | MISSING_NEEDS_MEASUREMENT | no non-fixed joint exists |
| Motor torque / stall curve | — | MISSING_NEEDS_MEASUREMENT | driver config exposes RPM units only, no torque |

The eccentric envelope (origin x = -0.09025, i.e. the body is offset 90 mm behind
the wheel centreline) strongly implies a non-central centre of mass, which makes
guessing a CoM particularly dangerous here. No CoM value is offered.

### 4.4 Sensor mounting and extrinsics

| Parameter | Value | Classification | Evidence (file:line) |
| --- | --- | --- | --- |
| IMU (HiPNUC N300WP PRO) offset | `xyz 0.1095 0 0.110`, parent `base_footprint` | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:64–69` |
| ” stated provenance | comment: 轮间中心前方 109.5 mm、高 110 mm，X 轴朝车头 | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:67` |
| IMU orientation (`rpy`) | `0 0 0` — assumed perfect alignment | PROVISIONAL | `urdf/ylhb.urdf.xacro:68` |
| IMU frame id | `imu_link` | REPO_CONFIG_HIGH_CONFIDENCE | `launch/bringup.launch.py:313` |
| IMU topic / baud | `/imu/data` @ 115200 | REPO_CONFIG_HIGH_CONFIDENCE | `launch/bringup.launch.py:310`, `:318` |
| RPLidar offset | `xyz -0.090 0 0.1805`, parent **`base_link`** | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:71–76` |
| RPLidar height above ground (derived) | 0.2670 m (= 0.0865 + 0.1805) | REPO_CONFIG_HIGH_CONFIDENCE (arithmetic) | derived from `:30` + `:75` |
| RPLidar orientation (`rpy`) | `0 0 0` | REPO_CONFIG_HIGH_CONFIDENCE | `urdf/ylhb.urdf.xacro:75` |
| RPLidar model | A2M8 | REPO_CONFIG_HIGH_CONFIDENCE | `launch/bringup.launch.py:326`; `config/slam_toolbox_params.yaml:29–30` |
| RPLidar min range | 0.20 m (datasheet-sourced per comment) | REPO_CONFIG_HIGH_CONFIDENCE | `config/slam_toolbox_params.yaml:29` |
| RPLidar max range | 12.0 m (datasheet-sourced per comment) | REPO_CONFIG_HIGH_CONFIDENCE | `config/slam_toolbox_params.yaml:30` |
| RPLidar samples/scan, update rate, FOV | — | MISSING_NEEDS_MEASUREMENT | not in repo; needed for a Gazebo ray sensor |
| **GNSS/RTK antenna offset** | **`xyz 0 0 0`** — explicit placeholder | **PROVISIONAL / MISSING_NEEDS_MEASUREMENT** | `urdf/ylhb.urdf.xacro:78–83` |
| ” in-repo flag | comment: *"Provisional antenna origin. Replace XYZ with the measured antenna offset."* | PROVISIONAL | `urdf/ylhb.urdf.xacro:81` |
| ” pinned by test name | `test_gps_link_is_fixed_to_base_with_provisional_zero_offset` | PROVISIONAL | `test/test_robot_geometry.py:69–79` |
| RTK receiver / frame | WTRTK980, frame `gps_link`, `/dev/rtk_4g` @115200, default **disabled** | REPO_CONFIG_HIGH_CONFIDENCE | `launch/bringup.launch.py:332–347` |
| **ZED 2i extrinsics rel. robot** | **do not exist** | **MISSING_NEEDS_MEASUREMENT** | see 4.5 |

### 4.5 ZED 2i — special scrutiny

**There is no transform anywhere in this repository between the ZED 2i and the
robot.** Specifically:

- The ZED does **not** appear in `ylhb.urdf.xacro`. There is no `zed_link`, no
  `camera_link`, no mounting joint.
- No `static_transform_publisher` exists in `bringup.launch.py` (or any
  `ylhb_*` launch file) — the whole repository has **zero** static transform
  publisher nodes.
- The vendored `zed_wrapper` URDF (which *would* provide `camera_link`) is never
  included, and `zed_camera.launch.py`'s `publish_urdf` path is not used by any
  `ylhb_*` launch file.
- The ZED is driven instead by a **custom SDK node**, `zed_spatial_mapping_node`
  (`src/ylhb_3d_mapping/launch/zed_spatial_mapping.launch.py:37–41`), which
  publishes its point cloud into a **standalone, unparented frame**
  `zed_3d_map` (`src/ylhb_3d_mapping/config/zed_spatial_mapping.yaml:14`) using
  `coordinate_system: RIGHT_HANDED_Z_UP`.

So the 3-D mapping output is currently floating in its own frame, not registered to
`base_footprint`. For a Gazebo phase this means the camera pose must be measured or
CAD-derived from scratch. The CAD part `双目垫板.SLDPRT` ("stereo camera shim
plate") is the natural source for a CAD-derived mounting pose, but no number has
been extracted.

What *does* exist for the ZED (SDK settings, usable as a starting point for a
simulated camera but **not** intrinsics or extrinsics):

| Parameter | Value | Classification | Evidence |
| --- | --- | --- | --- |
| Resolution / FPS | HD720 @ 15 | REPO_CONFIG_HIGH_CONFIDENCE | `ylhb_3d_mapping/config/zed_spatial_mapping.yaml:16–17` |
| Depth mode | `NEURAL` | REPO_CONFIG_HIGH_CONFIDENCE | idem `:20` |
| Depth min / max | 0.25 m / 4.0 m | REPO_CONFIG_HIGH_CONFIDENCE | idem `:21–22` |
| Coordinate units / system | METER / RIGHT_HANDED_Z_UP | REPO_CONFIG_HIGH_CONFIDENCE | idem `:18–19` |
| Camera intrinsics, baseline | — | MISSING_NEEDS_MEASUREMENT | `custom_baseline` defaults to `0.0`, never set |
| Pose relative to `base_link` | — | MISSING_NEEDS_MEASUREMENT | no transform exists |

### 4.6 Velocity and acceleration limits (control limits, not physical properties)

| Parameter | Value | Classification | Evidence |
| --- | --- | --- | --- |
| Chassis `max_linear_speed` | 0.35 m/s | REPO_CONFIG_HIGH_CONFIDENCE | `config/base_kinematics.yaml:8` |
| Chassis `max_angular_speed` | 0.70 rad/s | REPO_CONFIG_HIGH_CONFIDENCE | `config/base_kinematics.yaml:10` |
| Nav2 `max_vel_x` | 0.12 m/s | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:168` |
| Nav2 `max_vel_theta` | 0.65 rad/s | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:171` |
| Nav2 `acc_lim_x` / `acc_lim_theta` | 0.30 / 0.80 | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:178`, `:180` |
| Velocity smoother max | `[0.12, 0.0, 0.65]` | REPO_CONFIG_HIGH_CONFIDENCE | `config/nav2_params.yaml:401` |
| C++ speed defaults | 0.6 / 1.2 (diverge from YAML) | PROVISIONAL | `zlac8015d_canopen_controller.cpp:542–543` |

Note the chassis ceiling (0.35 m/s) is roughly 3× the Nav2 commanded ceiling
(0.12 m/s). Both are deliberate limits, not measured capability, and neither is
evidence of achieved speed.

---

## 5. Parameter classification summary

### 5.1 KNOWN_PARAMETERS (`REAL_MEASURED`)

**Empty.** No parameter in this repository is backed by a measurement record.

Two values carry a 实测 ("measured") annotation in a code comment —
`base_kinematics.yaml:3` and `urdf/ylhb.urdf.xacro:7` — but a comment is not a
measurement record. `base_kinematics.yaml:3` in particular reads
"实测轮直径后填「直径 / 2」" ("after measuring the wheel diameter, enter
diameter / 2"), which is an **instruction to the maintainer**, not a statement that
the measurement was performed. Neither value is promoted to `REAL_MEASURED` here.

### 5.2 HIGH_CONFIDENCE_PARAMETERS (`REPO_CONFIG_HIGH_CONFIDENCE`)

Safe to carry into a Gazebo model as-is:

1. `wheel_radius` = 0.0865 m — three consistent sources plus a dimensional
   cross-check (section 4.2)
2. Body envelope: cylinder r = 0.27675 m, l = 0.3255 m, origin `-0.09025 0 0.07625`
3. Nav2 16-gon footprint, consistent with the envelope and test-locked
4. IMU mount `0.1095 0 0.110` off `base_footprint`
5. RPLidar mount `-0.090 0 0.1805` off `base_link` (0.2670 m above ground)
6. RPLidar A2M8 range window 0.20–12.0 m
7. Frame naming and TF topology: `map → odom → base_footprint → {base_link, imu_link, gps_link}`, `base_link → laser_link`
8. Velocity/acceleration limits (as limits, section 4.6)
9. `wheel_track` = 0.4008 m — **included here only because nothing contradicts it;
   see 5.3, it is single-source and uncorroborated**

### 5.3 CONFLICTING_PARAMETERS

**One genuine numeric divergence class, and it is a real trap for a Gazebo launch file.**

| Parameter | Value A (YAML, wins at runtime) | Value B (C++ compiled default) | Status |
| --- | --- | --- | --- |
| `wheel_radius` | **0.0865 m** — `config/base_kinematics.yaml:4` | **0.076 m** — `zlac8015d_canopen_controller.cpp:539`, `:886` | CONFLICTING |
| `wheel_track` | **0.4008 m** — `config/base_kinematics.yaml:6` | **0.25 m** — `zlac8015d_canopen_controller.cpp:541`, `:887`; `base_controller.cpp:30` | CONFLICTING |
| `max_linear_speed` | 0.35 — `base_kinematics.yaml:8` | 0.6 — `zlac8015d_canopen_controller.cpp:542` | CONFLICTING |
| `max_angular_speed` | 0.70 — `base_kinematics.yaml:10` | 1.2 — `zlac8015d_canopen_controller.cpp:543` | CONFLICTING |

**Which one is actually in force, verified:** `base_kinematics.yaml` is passed to
the chassis node at `src/ylhb_base/launch/bringup.launch.py:365`, so on the normal
`bringup` path the YAML values (0.0865 / 0.4008) override the C++ defaults. The
divergence is therefore latent rather than active — **today**.

Why it still matters for the Gazebo phase: `load_parameters()` at
`zlac8015d_canopen_controller.cpp:579–587` gates its `wheel_diameter` fallback on
`has_parameter_override("wheel_radius")`. Any new simulation launch file that
starts this node (or reuses its kinematics) **without** explicitly loading
`base_kinematics.yaml` will silently run on 0.076 m / 0.25 m — a 12% radius error
and a 38% track error. That is exactly the class of failure that produces a
plausible-looking but worthless simulation. Recommendation: a future sim launch
file must load `base_kinematics.yaml` explicitly, and the divergent C++ defaults
should be treated as a known hazard.

### 5.4 PROVISIONAL_PARAMETERS

1. **GNSS/RTK antenna offset — `xyz 0 0 0`.** The repository explicitly flags this
   as provisional in a comment (`urdf/ylhb.urdf.xacro:81`) and pins the zero in a
   test whose *name* declares it provisional
   (`test_robot_geometry.py:69`). This is an honest placeholder, not a value.
   **It must never be treated as a real antenna position.** Using it in simulation
   would place the antenna at the wheel-centre ground point, silently removing all
   GNSS lever-arm effects. Status confirmed: **PROVISIONAL, effectively missing.**
2. **IMU orientation `rpy = 0 0 0`.** A stated position with an assumed
   orientation. Mounting yaw/pitch error is unvalidated.
3. **RPLidar orientation `rpy = 0 0 0`.** Comment at `urdf/ylhb.urdf.xacro:74`
   asserts the laser X axis is aligned with the forward direction; the test at
   `test_robot_geometry.py:61–66` only checks the string is `"0 0 0"`, which
   verifies the file's content, not the physical mounting.
4. **`wheel_track` = 0.4008 m — single-source.** Not contradicted by any file, but
   not corroborated by any either (see 5.5 note).
5. **All four C++ default divergences** listed in 5.3.
6. **`actual_velocity_unit_per_rpm: 10.0`** — an empirical scale factor standing in
   for an unstated gearbox/encoder ratio (`config/zlac8015d.yaml:34`).

### 5.5 MISSING_CRITICAL_PARAMETERS — count: **16**

Each of these blocks a physics-faithful Gazebo model:

1. Total robot mass
2. `base_link` mass
3. `base_link` centre of mass (non-trivial: envelope is eccentric by 90 mm)
4. `base_link` inertia tensor (ixx…izz)
5. Drive-wheel links themselves (absent from URDF — structural, not just numeric)
6. Drive-wheel joint origins and rotation axis (no geometric encoding of `wheel_track`)
7. Drive-wheel mass
8. Drive-wheel inertia tensor
9. Drive-wheel width
10. Drive-wheel collision geometry / contact shape
11. Caster wheel geometry, position, mass, inertia (in CAD, absent from URDF)
12. GNSS antenna offset (provisional zero — see 5.4)
13. ZED 2i pose relative to the robot (no transform exists at all)
14. Tyre–ground friction coefficients
15. Joint effort and velocity limits / motor torque characteristics
16. Telescoping mast and 5-segment manipulator links (in CAD, absent from URDF —
    they change CoM height materially)

**Independent verification note on the two prompted values.** `wheel_radius`
0.0865 m is corroborated three ways and is dimensionally coherent with the body
envelope. `wheel_track` 0.4008 m is **not corroborated**: it occurs exactly once in
the entire repository. The C++ gtest that reads `base_kinematics.yaml`
(`test/test_zlac_drive_mapping.cpp:75–85`) asserts **only** the three channel-polarity
strings `low_channel_is_left: true`, `low_channel_direction: 1.0`,
`high_channel_direction: -1.0` — it does **not** assert `wheel_track` or
`wheel_radius`. And because the URDF has no wheel links, there is no geometric
cross-check available. Treat 0.4008 m as unverified until physically measured.

---

## 6. MEASUREMENTS THE OWNER MUST TAKE BY HAND

Split by whether a physical robot is actually required. **Do not let anyone fill
these in from a datasheet or a similar robot.** Record each with date, method,
instrument, and who took it, so the next audit can classify them `REAL_MEASURED`.

### Group A — Derivable from CAD or code, no robot needed

Highest leverage first. All of these can be done at a desk.

| # | Item | Source | How |
| --- | --- | --- | --- |
| A1 | `base_link` mass, CoM, full inertia tensor | `CAD/.../sldasm/小车底盘.SLDASM` | Assign real materials, then SolidWorks *Mass Properties* → export tensor at CoM. Highest-value single action in this list. |
| A2 | Drive-wheel mass, inertia, radius, width | `CAD/.../stp/电机轮.SLDPRT` | Same tool. Also gives an **independent CAD check on 0.0865 m**. |
| A3 | Caster wheel geometry, mass, inertia | `万向轮.SLDASM`, `万向轮底座`, `万向轮支架` | Same tool; needed for contact points. |
| A4 | Wheel-mount spacing → **independent `wheel_track` check** | `小车底盘.SLDASM` | Measure motor-mount centre distance in CAD. This is the missing corroboration for 0.4008 m. |
| A5 | Mast + arm segment masses and inertias | `伸缩杆`, `伸缩柱`, `第二…第五节机械臂`, `机械爪.SLDASM` | Same tool; needed for CoM height. |
| A6 | ZED 2i mounting pose | `双目垫板.SLDPRT` + assembly | Read the plate's position and orientation in the assembly. |
| A7 | GNSS antenna mounting pose | assembly (if the antenna is modelled) | If absent from CAD, escalate to B4. |
| A8 | Exportable meshes | all CAD | Export STL/DAE; the repo currently has **no mesh files at all**. |
| A9 | Gearbox / encoder ratio | ZLAC8015D vendor docs in `官方通信协议/` | Replaces the empirical `actual_velocity_unit_per_rpm: 10.0`. |

### Group B — Genuinely requires the physical robot

| # | Item | Method | Why CAD cannot answer it |
| --- | --- | --- | --- |
| B1 | **Total robot mass, as-built** | Platform scale, fully assembled with battery and all payload | CAD omits cabling, fasteners, adhesives, and the actual battery; as-built mass routinely diverges 10–20% from CAD |
| B2 | **Wheel radius under load** | Measure loaded rolling radius: mark tyre, roll ≥10 revolutions on the real floor, divide distance by 2π·N | Verifies 0.0865 m against tyre deflection, which CAD cannot model |
| B3 | **Wheel track, contact-patch centre to contact-patch centre** | Tape measure between tyre contact patches, robot loaded, repeated both sides | Confirms or corrects the uncorroborated 0.4008 m; CAD gives nominal, not loaded, spacing |
| B4 | **GNSS antenna phase-centre offset** | Measure from wheel-centre ground point to the antenna's phase centre (not its housing centre) | Currently a placeholder zero; phase centre is not a CAD feature |
| B5 | **ZED 2i extrinsics, verified** | Measure the left-lens optical centre pose relative to `base_footprint`; cross-check against A6 | As-mounted differs from as-designed; also resolves the missing `zed_3d_map` → `base_footprint` link |
| B6 | **IMU mounting orientation** | Level the robot, log stationary gravity vector and yaw drift, compare to `rpy = 0 0 0` | Validates the assumed alignment at `urdf/ylhb.urdf.xacro:68` |
| B7 | **Effective CoM height** | Tilt-table or suspension test | The eccentric envelope makes this both important and non-obvious |
| B8 | **Tyre–floor friction** | Pull-force or incline-slip test on the actual operating surface | Surface-dependent; no datasheet applies |
| B9 | **Motor torque / stall behaviour** | Bench test, or read from ZLAC8015D docs and confirm on the bench | Driver config exposes RPM units only, never torque |
| B10 | **RPLidar A2M8 samples/scan and update rate** | Inspect a live `/scan` message header and array length | Needed for a Gazebo ray sensor; not recorded in the repo |

**Priority order if time is short:** A1 → A2 → A4 → B1 → B3 → B2. Those six close
the wheel-geometry corroboration gap and give the single largest chunk of the
dynamics data. B4 and B5 are required before any GNSS or ZED simulation work.

---

## 7. Reusable assets for a Gazebo MVP

Genuinely reusable today, without any measurement:

| Asset | Path | Why it is reusable |
| --- | --- | --- |
| Frame skeleton + body envelope | `src/ylhb_base/urdf/ylhb.urdf.xacro` | Correct, test-locked TF topology and collision envelope. Sound base to extend with wheels and inertials — extend, do not replace. |
| Geometry regression test | `src/ylhb_base/test/test_robot_geometry.py` | Already pins URDF↔Nav2 footprint consistency. The natural place to add guards so a sim URDF cannot silently drift from the real one. |
| Pure kinematics header | `src/ylhb_base/include/ylhb_base/zlac_drive_mapping.hpp` | Dependency-free twist↔wheel-RPM maths (`:51–62`). Lets a Gazebo diff-drive plugin be validated against the real robot's exact kinematic convention. |
| Chassis kinematics config | `src/ylhb_base/config/base_kinematics.yaml` | The authoritative `wheel_radius` / `wheel_track`. **A sim launch file must load this explicitly** (see 5.3). |
| Full Nav2 stack config | `src/ylhb_base/config/nav2_params.yaml`, `nav2_params_keepout.yaml` (17 KB each) | Complete, commented, footprint already matches the URDF. |
| Nav2 behaviour tree | `src/ylhb_base/config/nav2_no_recovery.xml` | Reusable as-is. |
| SLAM config | `src/ylhb_base/config/slam_toolbox_params.yaml` | Includes datasheet-sourced A2M8 range window, useful for a simulated lidar. |
| EKF config | `src/ylhb_base/config/ekf.yaml` | Already carries `use_sim_time` at `:7` (currently `false`) — a one-line switch, no restructuring. |
| `robot_state_publisher` pattern | `src/ylhb_base/launch/bringup.launch.py:392–400` | Correct `xacro`-via-`Command` idiom to copy. |
| Existing `use_sim_time` plumbing | `navigation.launch.py:41`, `navigation_keepout.launch.py:59`, `mapping.launch.py:17` | Sim-clock argument already threaded through the nav stack. |
| Maps and routes as world references | `maps/my_map.pgm` + `.yaml`, `maps/keepout/`, `maps/route_patrol_001.json`, `route_patrol_002.json` | A Gazebo world can be built to match the existing occupancy map so recorded routes stay valid. |
| RViz configs | `src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz`, `src/rplidar_ros-ros2/rviz/rplidar_ros.rviz` | Visualisation starting points. |
| Synthetic-validation harness | `src/dg_synthetic_validation/` | Existing scenario/evidence tooling whose declared role (`scenario_schema.py:11`) is the one Gazebo would later fill. Its evidence-labelling discipline should be carried forward, not discarded. |

**Not reusable:** the vendored `zed-ros2-wrapper` URDF (unused, section 3.2), and
the CAD files in their current native formats (no meshes exported, section 3.3).

---

## 8. Risks and unknowns

**R1 — Fabricated physical parameters (highest risk).** With 16 missing critical
parameters, the temptation to fill in "reasonable" masses and inertias is the main
threat to this phase. A diff-drive robot with invented inertia and invented friction
will drive convincingly in Gazebo while being quantitatively meaningless, and any
tuning done against it transfers incorrectly to the real robot. Every value
introduced for simulation must be labelled `SIMULATION_ASSUMPTION` in-file, and
never promoted without a measurement record.

**R2 — The silent C++ default divergence.** A sim launch file that omits
`base_kinematics.yaml` gets 0.076 m / 0.25 m instead of 0.0865 m / 0.4008 m, with no
warning (section 5.3). This risk is structural, not hypothetical.

**R3 — `wheel_track` has no second witness.** 0.4008 m occurs once, repo-wide, with
no geometric or test cross-check. Every odometry rotation and every in-place turn in
simulation scales directly with it. Measurement A4 or B3 should precede any use.

**R4 — GNSS placeholder promotion.** The zero offset is well-flagged today. The risk
is that a future edit copies `0 0 0` into an SDF where the "provisional" comment does
not travel with it, converting an honest placeholder into a false value.

**R5 — The URDF models less than half the robot.** Mast, 5-segment arm, gripper, and
casters are all in CAD and none are in the description. A sim built from the URDF
alone would have a materially wrong CoM height and wrong ground contact, while
looking superficially correct.

**R6 — ZED is unregistered to the TF tree.** `zed_3d_map` is parentless
(section 4.5). This is a live gap in the real system too, not only a simulation gap.

**R7 — Orientation assumptions are unvalidated.** Three `rpy = 0 0 0` mountings are
assumed rather than verified; existing tests check file contents, not physical
alignment.

**R8 — No simulator installed, no target version chosen.** ROS 2 Humble is present;
no Gazebo binary is. The Humble-era choice between Gazebo Classic 11 and Gazebo
Fortress (`ros_gz`) has not been made and determines the plugin and `ros2_control`
approach. No `ros2_control` layer exists to build on either. Selecting and installing
a simulator was out of scope here.

**R9 — Unknown: whether 0.0865 m was ever physically measured.** The evidence is
strongly self-consistent but the provenance is a comment that reads as an
instruction. Classified `REPO_CONFIG_HIGH_CONFIDENCE`, deliberately not
`REAL_MEASURED`.

**R10 — Working tree is not clean.** At audit time, 6 modified files and a number of
untracked files were present under `src/dg_synthetic_validation/` and
`docs/dg202611/`. Nothing was committed, reset, or checked out by this audit, but the
parameter evidence above reflects the working tree as read, not a tagged commit.

---

### Audit boundary statement

Read-only. No `.py`, `.yaml`, `.urdf`, `.xacro`, `.launch`, or `setup.py` file was
modified. No Gazebo was installed or started. No world, plugin, URDF, or mesh was
created. No `colcon build`, `colcon test`, or scenario was run. No git commit, push,
reset, or checkout was performed. `results/` was not touched. The only file written
is this report. Every number above is quoted from the repository with a
`file:line` reference, or is explicitly marked missing.










