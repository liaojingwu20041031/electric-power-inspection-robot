# 06 — Known Surrogates and Gaps

```
PURPOSE:           Document temporary components and known limitations
AUDIENCE:          Development team, reviewers, real robot integration team
UPDATE_FREQUENCY:  Each sprint, before real hardware deployment
```

## 1. Overview

This document catalogs all surrogate components, placeholder implementations, and
known gaps in the three-model system. Each item is marked with its current status
and upgrade path.

## 2. Surrogate Component Registry

### 2.1 MODEL-1 Surrogates

#### Visual Odometry Integration

**Status:** `SURROGATE`

**What exists:**
- ZED ROS2 wrapper present (`src/zed-ros2-wrapper/`)
- Topic interface defined (`/zed/odom`, `/zed/pose`)
- Quality scoring framework placeholder

**What is missing:**
- Full integration into `multisource_fusion_core.py`
- Quality metric computation (feature count, tracking confidence)
- Health assessment thresholds tuned for ZED
- Validation with real ZED camera

**Upgrade path:**
1. Complete ZED wrapper configuration
2. Integrate visual odom into fusion state machine
3. Tune quality thresholds with real camera data
4. Add visual odom to health aggregation
5. Test in real dynamic environment

**Blocking issues:**
- No real ZED hardware in current validation
- Quality scoring needs real feature tracking data

---

#### UWB Ranging

**Status:** `SURROGATE`

**What exists:**
- Interface defined (range measurements to known anchors)
- Placeholder for NLOS detection
- Fusion framework ready to accept UWB observations

**What is missing:**
- Real UWB hardware integration
- Anchor survey (known positions)
- NLOS detection algorithm
- Quality metric computation
- Validation with real UWB data

**Upgrade path:**
1. Connect real UWB hardware (e.g., DecaWave DW1000)
2. Survey anchor positions (total station or RTK)
3. Implement NLOS detection (residual-based or channel impulse response)
4. Integrate into `multisource_fusion_core.py`
5. Tune quality thresholds with real data

**Blocking issues:**
- No real UWB hardware available
- Anchor positions not surveyed

---

#### BDS Short Message

**Status:** `NOT_INTEGRATED`

**What exists:**
- Vendor-neutral payload validation, queue, acknowledgement, timeout/retry,
  inbound record, and audit contract
- No vendor frame codec or ROS 2 transport node

**What is needed:**
- BDS terminal hardware
- Vendor protocol adapter and ROS 2 sending/receiving node
- Integration into communication stack
- Use case definition (emergency backup, status reporting)

**Upgrade path:**
- To be designed when BDS terminal available

---

### 2.2 MODEL-2 Surrogates

#### Feature Extraction Method

**Status:** `SURROGATE_NOW` (engineering baseline)

**What exists:**
- Geometric feature extraction (range discontinuity, local curvature)
- Temporal persistence filtering
- Robust outlier rejection (statistical)

**What is NOT:**
- Learning-based feature extraction
- Semantic segmentation
- Deep neural network for dynamic object classification

**Why engineering-based now:**
- 2D LiDAR data insufficient for complex learning models
- No large training corpus of labeled dynamic objects
- Geometric methods provide interpretable, tunable baseline

**Upgrade path (FINAL_ALGORITHM_LATER):**
1. Collect real LiDAR data corpus (diverse environments, dynamic objects)
2. Label data (static/dynamic, outlier/normal)
3. Train learning model (if justified by data quality and compute budget)
4. Validate on held-out test set
5. Deploy if performance exceeds geometric baseline

**Current confidence:**
- Engineering baseline: HIGH (unit tested, validated in S05-S08)
- Learning upgrade necessity: UNCLEAR (may not be needed if geometric suffices)

---

#### Repeatability Evaluation

**Status:** `SIMULATION_DIAGNOSTIC_ONLY`

**What exists:**
- Repeatability metrics computed (R_min, R_avg, R_pool)
- Following frozen protocol
- Implemented in evaluator

**What it is NOT:**
- Competition performance metric
- Real robot accuracy prediction
- Comparison with other teams

**Limitations:**
- Computed on synthetic or Gazebo data (not real LiDAR)
- Analytic scan waveform (not real geometry)
- No real dynamic objects
- No real material properties

**Upgrade path:**
1. Run on real LiDAR data (RPLiDAR A2/A3)
2. Real dynamic environment (people walking)
3. Real surveyed map
4. Compute repeatability on multiple test runs
5. Compare with competition baseline

---

### 2.3 MODEL-3 Surrogates

#### Scan-to-Map Matcher

**Status:** `SURROGATE_COMPONENT`

**What exists:**
- Uses AMCL or scan_tools matcher
- MODEL-3 does not own scan matching algorithm
- Interface: candidate pose + quality metrics

**What is NOT:**
- Dedicated matcher optimized for active relocalization
- Learning-based place recognition
- Multi-hypothesis tracker

**Why surrogate:**
- Existing matchers (AMCL, scan_tools) functional for baseline
- MODEL-3 focuses on supervision and verification, not matching itself

**Upgrade path:**
1. Evaluate if dedicated matcher needed (e.g., faster, more robust)
2. If yes: implement or integrate advanced matcher (e.g., HDL graph SLAM, LIO-SAM)
3. If no: keep existing matcher, tune thresholds

**Current confidence:**
- AMCL/scan_tools as surrogate: MEDIUM (functional but not optimized)
- Upgrade necessity: UNCLEAR (depends on real robot failure modes)

---

#### Action Policy

**Status:** `SURROGATE_NOW` (rotation-only)

**What exists:**
- Fixed scan segments: +10°, -20°, +10°
- Rotation only, no translation
- Parameters tunable (angular_speed, segment_deltas)

**What is NOT:**
- Learned action policy (RL-based)
- Search-based planning (RRT, A*)
- Adaptive segment selection based on environment

**Why rotation-only now:**
- Simple, safe, interpretable
- Sufficient for baseline validation
- Real robot constraints (collision avoidance, workspace limits) not yet known

**Upgrade path (FINAL_ALGORITHM_LATER):**
1. Characterize real robot failure modes (where does passive localization fail?)
2. Design action space (rotation, translation, combinations)
3. If learning: collect training data (failure scenarios, recovery actions, success)
4. Train RL policy or search planner
5. Validate in real environment
6. Deploy if performance exceeds fixed baseline

**Current confidence:**
- Rotation-only baseline: HIGH (working in S06-S08)
- Upgrade necessity: MEDIUM (may need more complex actions for some environments)

---

## 3. Known Gaps

### 3.1 Real Hardware Integration

**GNSS antenna lever arm:**
- **Status:** PROVISIONAL (URDF marks 0 0 0 as placeholder)
- **Impact:** Fusion accuracy degraded, especially with IMU integration
- **Action:** Physical measurement required before real robot experiments

**Sensor timing:**
- **Status:** NOT_VALIDATED
- **Impact:** Fusion assumes reasonable latency (<100 ms), not verified on real hardware
- **Action:** Measure end-to-end latency (sensor → ROS2 topic publish)

**Quality threshold tuning:**
- **Status:** SYNTHETIC_VALUES (max_covariance 0.5, min_lidar_quality 0.2, etc.)
- **Impact:** Thresholds may be too loose or too tight for real sensor noise
- **Action:** Tune on real robot data, iterate based on failure modes

---

### 3.2 Test Coverage

**Concurrent degradation scenario:**
- **Status:** MISSING
- **Impact:** Not validated that GNSS + LiDAR simultaneous failure triggers correctly
- **Action:** Create M1_concurrent scenario, run and validate

**Negative tests:**
- **Status:** MISSING (bad candidate, timeout, verification jump, GT leak)
- **Impact:** Failure paths not explicitly validated end-to-end
- **Action:** Create M3 negative test scenarios

**MODEL-2 dedicated scenarios:**
- **Status:** MISSING (static, dynamic, outlier, zero-feature)
- **Impact:** Feature extraction logic tested in unit tests, not integration
- **Action:** Create M2 scenarios with Gazebo dynamic actors

---

### 3.3 Real Robot Unknowns

**Wheel slip:**
- **Status:** NOT_MODELED
- **Impact:** Odometry drift in simulation vs. real may differ significantly
- **Action:** Characterize slip on real surfaces (tile, carpet, outdoor)

**Vibration and mechanical noise:**
- **Status:** NOT_MODELED
- **Impact:** IMU and encoder noise in simulation cleaner than real
- **Action:** Collect real sensor data, tune filters if needed

**Computation budget:**
- **Status:** NOT_VALIDATED
- **Impact:** Real-time performance on Jetson not confirmed
- **Action:** Profile on target hardware, optimize if needed

**Battery and power:**
- **Status:** NOT_MODELED
- **Impact:** Sensor degradation with low battery not considered
- **Action:** Define low-battery behavior (graceful degradation vs. emergency stop)

---

## 4. Surrogate Marking Convention

Throughout the codebase and documentation, surrogates are marked with:

```python
# SURROGATE_NOW: engineering baseline, upgrade path defined
# FINAL_ALGORITHM_LATER: learning-based or advanced version planned

# SURROGATE_COMPONENT: external dependency, not owned by this model
```

**Examples:**

```python
# MODEL-2: geometric feature extraction
# SURROGATE_NOW: uses curvature and persistence, not learned
# FINAL_ALGORITHM_LATER: may upgrade to CNN if data available

# MODEL-3: scan-to-map matcher
# SURROGATE_COMPONENT: uses AMCL, not owned by MODEL-3
```

## 5. Acceptance Criteria for Surrogate Removal

A surrogate can be removed (marked as final) when:

1. **Functional requirement met:** component performs its role reliably
2. **Validated on real hardware:** tested on target platform with real sensors
3. **Performance acceptable:** meets accuracy/latency/robustness targets
4. **No blocking upgrade:** upgrade path exists but not urgent

A surrogate must be upgraded when:

1. **Real hardware available:** blocking issue resolved (e.g., UWB hardware arrives)
2. **Failure mode discovered:** current surrogate insufficient for real scenarios
3. **Performance gap:** surrogate significantly underperforms vs. requirements

## 6. Summary Table

| component | model | status | blocking issue | upgrade urgency |
|---|---|---|---|---|
| Visual odometry | MODEL-1 | SURROGATE | no real ZED | MEDIUM |
| UWB ranging | MODEL-1 | SURROGATE | no real UWB hardware | LOW (not in baseline) |
| BDS short message | MODEL-1 | COMMON_CONTRACT_ONLY | no identified BDS terminal or vendor protocol | HIGH (teacher requirement) |
| Feature extraction | MODEL-2 | SURROGATE_NOW | none (baseline working) | LOW (may not need upgrade) |
| Repeatability eval | MODEL-2 | SIMULATION_ONLY | need real data | HIGH (before competition) |
| Scan-to-map matcher | MODEL-3 | SURROGATE_COMPONENT | none (AMCL functional) | MEDIUM (depends on failure modes) |
| Action policy | MODEL-3 | SURROGATE_NOW | none (rotation works) | MEDIUM (may need translation) |
| GNSS lever arm | All | PROVISIONAL | need measurement | HIGH (before real robot) |
| Sensor timing | All | NOT_VALIDATED | need real hardware | HIGH (before real robot) |
| Concurrent degradation test | All | MISSING | need scenario | MEDIUM |
| Negative tests | MODEL-3 | MISSING | need scenarios | MEDIUM |
| MODEL-2 integration tests | MODEL-2 | MISSING | need scenarios | LOW |

## 7. Upgrade Priority

**Before real robot deployment:**
1. 🔴 GNSS antenna lever arm measurement
2. 🔴 Sensor timing validation
3. 🟡 Quality threshold tuning
4. 🟡 Visual odometry integration (if ZED available)
5. 🟢 Concurrent degradation scenario
6. 🟢 Negative test scenarios

**Before competition:**
1. 🔴 Repeatability evaluation on real data
2. 🔴 Real robot end-to-end test
3. 🟡 UWB integration (if required by rules)
4. 🟡 Action policy upgrade (if rotation-only insufficient)
5. 🟢 Feature extraction upgrade (if geometric baseline fails)

**Future work:**
- BDS short message
- Learning-based components (if justified)

---

**Legend:**
- 🔴 HIGH urgency (blocking or critical)
- 🟡 MEDIUM urgency (important but not blocking)
- 🟢 LOW urgency (nice-to-have or contingent)
