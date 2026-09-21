# 07 — Real Robot Follow-up Requirements

```
PURPOSE:      Checklist for deploying three-model system on real hardware
AUDIENCE:     Integration team, hardware team, testing team
STATUS:       PLANNING (simulation validation complete, real robot pending)
```

## 1. Overview

This document defines requirements and actions needed before deploying the three-model
system on the real DG-202611 robot. It assumes simulation validation (S05-S08) has
passed and core software is functionally correct.

## 2. Geometry Parameters (Accept Simulation Config)

### 2.1 Wheel Track and Radius

**Decision:** Accept current simulation values as WORKING_CONFIG.

```
wheel_track:   0.4008 m   (repository-configured)
wheel_radius:  0.0865 m   (repository-configured, moderate corroboration)
```

**Rationale:**
- CAD unlock not available (no SolidWorks access)
- Physical measurement deferred (not blocking software validation)
- Values functional in simulation (vehicle runs, kinematics sound)

**Status:** ✅ ACCEPTED

**Future action (optional):**
- If CAD becomes available: verify via STEP export
- If vehicle behavior anomalous: measure wheel track and rolling radius
- Before competition: as-built verification recommended

---

### 2.2 GNSS Antenna Lever Arm

**Current status:** PROVISIONAL (URDF marks `0 0 0` as placeholder)

**Required action:** 🔴 **MANDATORY before real robot experiments**

**How to measure:**
1. Mark vehicle's base_footprint position (wheelbase midpoint projected to ground)
2. Locate GNSS antenna phase centre (check antenna datasheet for offset from housing top)
3. Measure 3D vector from base_footprint to antenna phase centre
4. Express in vehicle frame (x forward, y left, z up)
5. Precision: ±0.01 m or better

**Where to record:**
```
src/ylhb_base/urdf/ylhb.urdf.xacro
<joint name="base_to_gps_joint" ...>
  <origin xyz="X Y Z" rpy="0 0 0"/>  <!-- replace 0 0 0 -->
</joint>
```

**Impact if skipped:**
- Fusion accuracy degraded (especially with IMU integration)
- GNSS corrections misaligned with vehicle motion
- Localization error grows with lever arm magnitude

---

### 2.3 Sensor Orientations

**Current status:** All sensors marked `rpy="0 0 0"` (aligned with vehicle frame)

**Required action:** 🟡 **VERIFY before real experiments**

**How to verify:**
1. Check if IMU, LiDAR, camera physically rotated relative to vehicle
2. If aligned (axes parallel to vehicle frame): keep `rpy="0 0 0"`
3. If rotated: measure with spirit level or inclinometer
4. Express as roll-pitch-yaw (radians) in vehicle frame
5. Update URDF `rpy` attribute

**Sensors to check:**
- IMU (comment says X-axis forward, verify this is true)
- LiDAR (may be pitched or yawed for FOV adjustment)
- Camera (ZED often mounted with pitch for better view)

**Impact if wrong:**
- IMU angular velocity misaligned → poor odometry
- LiDAR scan misaligned → scan matching errors
- Camera depth misaligned → visual odometry drift

---

## 3. Sensor Integration

### 3.1 GNSS/RTK

**Hardware:** Already integrated (based on existing code)

**Required actions:**
1. 🔴 **Set ENU reference from first RTK fix**
   - Wait for first `quality=4` (RTK fixed) message
   - Record lat/lon/alt as `GeodeticReference`
   - Use for all subsequent ENU conversions
   - Store in parameter file or config

2. 🟡 **Verify RTK baseline**
   - Check base station coordinates
   - Verify differential age < 2 s
   - Monitor satellite count and HDOP

3. 🟡 **Characterize quality vs. environment**
   - Open sky: expect quality 4, 12+ sats, HDOP < 1.5
   - Urban canyon: quality may drop to 1-2
   - Indoors: no fix
   - Tune `min_gnss_quality` threshold based on real data

---

### 3.2 LiDAR (RPLiDAR A2/A3)

**Hardware:** Already integrated (rplidar_ros package present)

**Required actions:**
1. 🔴 **Verify scan rate and timing**
   - Check published rate (5-10 Hz typical)
   - Measure end-to-end latency (laser to /scan publish)
   - Ensure within `signal_timeout` (1.0 s default)

2. 🟡 **Tune persistence window**
   - Current default: 3-5 scans for stability
   - Adjust based on real update rate and robot velocity

3. 🟡 **Test with real dynamic objects**
   - Walk through scan FOV
   - Verify dynamic filtering removes person
   - Verify static background retained

4. 🟡 **Test outlier rejection on real materials**
   - Glass (specular reflection)
   - Metal (multi-path)
   - Wet surfaces (abnormal returns)
   - Adjust outlier thresholds if needed

---

### 3.3 IMU (N300WP PRO)

**Hardware:** Already integrated (hipnuc_imu package present)

**Required actions:**
1. 🟡 **Verify axes alignment**
   - URDF comment says "X轴朝车头" (X forward)
   - Physical check: rotate robot, verify IMU angular velocity sign correct
   - Update `rpy` if misaligned

2. 🟡 **Characterize noise**
   - Collect stationary data (robot not moving)
   - Compute gyro bias and random walk
   - Tune filter parameters if needed

3. 🟢 **Integrate with robot_localization EKF** (if not already)
   - Fuse IMU + wheel odom
   - Publish to `/odometry/filtered`
   - MODEL-1 consumes fused odom

---

### 3.4 Visual (ZED Camera)

**Hardware:** ZED wrapper present, full integration pending

**Required actions:**
1. 🟡 **Complete ZED configuration**
   - Set resolution, frame rate, depth mode
   - Verify `/zed/odom` and `/zed/pose` published
   - Check TF tree (zed_camera_link → base_link)

2. 🟡 **Integrate into MODEL-1**
   - Add visual odom to `multisource_fusion_core.py` source list
   - Compute quality metric (feature count, tracking confidence from ZED API)
   - Tune quality thresholds

3. 🟡 **Validate in real environment**
   - Test in low-light (ZED may fail)
   - Test with dynamic objects (people walking)
   - Test with feature-poor walls

**Blocking:** ZED hardware must be available and mounted on robot

---

### 3.5 UWB (Optional, if required)

**Hardware:** Not yet integrated

**Required actions:**
1. 🟢 **Connect UWB hardware** (e.g., DecaWave DW1000)
   - Interface via serial or Ethernet
   - Publish range measurements as ROS2 message

2. 🟢 **Survey anchor positions**
   - Use total station or RTK GNSS
   - Record 3D coordinates in map frame
   - Store in config file

3. 🟢 **Implement NLOS detection**
   - Residual-based (range error vs. position estimate)
   - Or channel impulse response analysis
   - Mark NLOS ranges as low quality or rejected

4. 🟢 **Integrate into MODEL-1**
   - Add UWB to source list
   - Compute quality metric
   - Fuse with GNSS, LiDAR, odom

**Blocking:** UWB hardware availability, anchor survey

---

## 4. System Integration

### 4.1 Parameter Tuning

**Health thresholds (MODEL-1):**
```
max_covariance:              0.5     (AMCL pose uncertainty, m²)
min_lidar_quality:           0.2     (geometry score, 0…1)
min_scan_match_score:        0.45    (matcher confidence, 0…1)
min_scan_match_inlier_ratio: 0.45    (fraction matched, 0…1)
max_scan_match_mean_distance: 0.30 m (point-to-map error)
signal_timeout:              1.0 s   (staleness threshold)
```

**Action:** 🟡 Tune on real data
- Collect sensor data in nominal conditions
- Compute statistics (AMCL cov typical range, match quality distribution)
- Adjust thresholds so trigger happens at genuine failures, not transient noise

**Trigger logic (MODEL-3):**
```
suspect_samples:  2   (consecutive poor-health samples before SUSPECTED)
trigger_samples:  2   (additional samples before TRIGGERED)
```

**Action:** 🟡 Tune for real robot dynamics
- Too sensitive: false triggers on transient noise
- Too insensitive: delayed recovery, long degraded periods
- Iterate based on real failure modes

---

### 4.2 Motion Control

**Stopping criterion:**
```
stop_velocity: 0.03 m/s   (threshold for "vehicle stopped")
```

**Action:** 🔴 Validate on real robot
- Command stop, measure actual velocity from /odom
- Adjust threshold if needed (encoder resolution, noise)

**Active scan motion:**
```
angular_speed:   0.18 rad/s  (~10°/s)
yaw_tolerance:   0.0262 rad  (1.5°)
settle_time:     0.5 s
```

**Action:** 🟡 Tune for real robot
- Test rotation: smooth, no oscillation, arrives within tolerance
- Adjust angular_speed if too fast (overshoot) or too slow (timeout)
- Adjust settle_time if scan still moving after nominal stop

**Safety constraints:**
```
max_total_rotation: 1.047 rad  (60°)
max_attempt_duration: 30 s
```

**Action:** 🔴 Validate safety
- Ensure rotation stays within safe workspace
- Add collision avoidance if operating near obstacles
- Test timeout behavior (does robot stop safely?)

---

### 4.3 Command Arbitration

**Current:** MODEL-3 publishes to `/dg/relocalization/cmd_vel`

**Required:** 🔴 Verify arbitration logic
- Who listens to `/dg/relocalization/cmd_vel`?
- Is there a cmd_vel arbiter that prioritizes recovery vs. navigation?
- Ensure real `/cmd_vel` (to motors) comes from arbiter, not directly from MODEL-3

**Action:**
1. Check `cmd_vel_arbiter` node (or equivalent)
2. Confirm priority: emergency stop > recovery > navigation > idle
3. Test: trigger recovery, verify navigation commands paused

---

## 5. Testing Protocol

### 5.1 Bench Tests (No Motion)

**Sensor health monitoring:**
1. Launch system, all sensors publishing
2. Verify MODEL-1 reports all sources GOOD
3. Unplug GNSS → verify GNSS marked STALE, overall DEGRADED (not FAILED)
4. Restore GNSS → verify recovery to NOMINAL

**LiDAR quality:**
1. Point LiDAR at wall (full FOV) → verify geometry_score high
2. Cover half of LiDAR → verify geometry_score drops
3. Point at empty space (all max range) → verify low score, no crash

---

### 5.2 Static Tests (Robot Stationary)

**Localization health:**
1. Place robot in known position, launch localization
2. Verify AMCL converges, covariance < 0.5
3. Verify MODEL-1 NOMINAL

**Trigger test:**
1. Manually increase AMCL covariance (publish fake amcl_pose with high cov)
2. Verify MODEL-3 SUSPECTED → TRIGGERED
3. Verify state transitions logged

---

### 5.3 Motion Tests (Controlled Environment)

**Odometry check:**
1. Drive straight 1 meter, measure actual distance
2. Compare with /odom reported distance
3. If error > 5%, check wheel_radius calibration

**Rotation check:**
1. Rotate 90°, measure actual angle (compass or manual)
2. Compare with /odom reported yaw
3. If error > 5°, check wheel_track calibration

**Active scan:**
1. Trigger recovery (cover LiDAR or inject poor AMCL)
2. Verify MODEL-3 executes scan segments
3. Verify rotation smooth, arrives at target yaw
4. Verify candidate generated and verified
5. Verify return to NOMINAL

---

### 5.4 Stress Tests (Real Environment)

**GNSS outage:**
1. Drive to GPS-denied area (indoors, urban canyon)
2. Verify GNSS marked REJECTED
3. Verify fusion continues with LiDAR + odom
4. Verify no crash or divergence

**LiDAR degradation:**
1. Drive in long corridor (low angular diversity)
2. Verify geometry_score drops
3. Verify MODEL-1 marks LiDAR DEGRADED
4. Verify system continues, does not trigger unless AMCL cov also rises

**Dynamic objects:**
1. Person walks through LiDAR FOV
2. Verify MODEL-2 filters dynamic points
3. Verify static features retained
4. Verify localization stable

**Recovery:**
1. Manually kidnap robot (move to different location)
2. Verify AMCL divergence → MODEL-3 trigger
3. Verify active scan → candidate → verification → RECOVERED
4. Verify pose corrected

---

## 6. Safety Checklist

Before enabling autonomous motion:

- ✅ Emergency stop button functional
- ✅ Collision avoidance enabled (if applicable)
- ✅ Workspace boundaries defined
- ✅ Human observer present
- ✅ Test area clear of obstacles
- ✅ Battery charged
- ✅ All sensors publishing
- ✅ Localization converged (AMCL covariance low)
- ✅ MODEL-1 reports NOMINAL
- ✅ Command arbitration validated

---

## 7. Failure Mode Characterization

After initial tests, document real robot failure modes:

**GNSS:**
- Where does RTK lose fix? (indoors, under trees, near buildings)
- Typical HDOP and satellite count in each area
- Differential age behavior

**LiDAR:**
- Where does geometry degrade? (corridors, open areas, feature-poor walls)
- Typical geometry_score range
- Dynamic object detection accuracy

**Odometry:**
- Wheel slip scenarios (carpet vs. tile, uphill vs. downhill)
- Typical drift rate (m per 10 m traveled)

**Recovery:**
- How often does passive localization fail?
- How long does active recovery take?
- Success rate of candidate verification

**Use this data to:**
- Refine thresholds
- Improve failure detection
- Optimize recovery strategy

---

## 8. Competition Readiness

Before official competition:

1. 🔴 **Geometry verification complete**
   - Wheel track and radius verified (CAD or physical)
   - GNSS lever arm measured
   - Sensor orientations confirmed

2. 🔴 **End-to-end test on competition-like course**
   - RTK baseline setup
   - Map created and verified
   - Full autonomy test (no manual intervention)
   - Multiple runs, success rate > 90%

3. 🔴 **Repeatability evaluation on real data**
   - LiDAR feature repeatability measured
   - Localization accuracy measured (vs. surveyed ground truth)
   - Performance metrics documented

4. 🟡 **Failure mode mitigation**
   - Known failure scenarios tested
   - Recovery strategy validated
   - Timeout and safe-fail paths confirmed

5. 🟡 **Documentation complete**
   - System architecture
   - Tuning parameters
   - Known limitations
   - Emergency procedures

---

## 9. Handoff to Competition Team

Provide competition team with:

**System description:**
- Architecture diagram (three models, interfaces)
- Parameter files with tuned values
- Known surrogates and limitations

**Operation guide:**
- Startup sequence
- Health monitoring dashboard
- Emergency stop procedure
- Recovery from common failures

**Test results:**
- Validation summary (S05-S08 synthetic + real robot tests)
- Performance metrics (accuracy, success rate, recovery time)
- Known failure modes and mitigations

**Configuration files:**
- URDF (with measured geometry)
- Launch files
- Parameter files (tuned thresholds)
- Map and localization initialization

---

## 10. Summary Checklist

**Before first real robot test:**
- 🔴 GNSS lever arm measured
- 🔴 Sensor timing validated
- 🔴 Stopping criterion tuned
- 🔴 Command arbitration verified
- 🔴 Safety checklist complete
- 🟡 Quality thresholds initially tuned
- 🟡 Active scan motion tested

**Before competition:**
- 🔴 End-to-end autonomy test passed
- 🔴 Repeatability evaluated on real data
- 🔴 All geometry verified
- 🟡 Visual odometry integrated (if available)
- 🟡 UWB integrated (if required)
- 🟡 Failure modes characterized
- 🟢 BDS short message (if time permits)

**Known deferrals:**
- Wheel track/radius physical measurement (optional if CAD unavailable)
- Learning-based upgrades (MODEL-2 features, MODEL-3 actions)
- Advanced matcher (if AMCL/scan_tools sufficient)

---

**Legend:**
- 🔴 MANDATORY (blocking real robot test or competition)
- 🟡 IMPORTANT (strongly recommended, may defer if low risk)
- 🟢 OPTIONAL (nice-to-have, time permitting)
