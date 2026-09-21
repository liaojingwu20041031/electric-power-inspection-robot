# 05 — Simulation Validation Summary

```
VALIDATION_TYPE:      SYNTHETIC + GAZEBO
SCENARIOS_RUN:        S05, S06, S07, S08
RESULTS_LOCATION:     ~/dg202611_ws/results/synthetic/final/
EVIDENCE_STATUS:      COMPLETE
COMPETITION_METRIC:   NO (simulation diagnostic only)
```

## 1. Overview

This document summarizes synthetic and Gazebo validation of the three core models.
All results are marked:

```
SYNTHETIC
NOT_REAL_ROBOT
NOT_COMPETITION_PERFORMANCE_EVIDENCE
```

## 2. Validation Environment

### 2.1 Platform

- **OS:** Ubuntu 22.04.5 LTS
- **ROS:** ROS 2 Humble
- **Simulator:** Gazebo (for P0/P1 baseline), synthetic injector (for DG scenarios)
- **Physics step:** 0.001 s (default, known robustness issue in P1, not blocking this sprint)

### 2.2 Configuration

**Kinematic parameters (WORKING_SIMULATION_CONFIG):**
```
wheel_track:   0.4008 m   (repository-configured, not physically verified)
wheel_radius:  0.0865 m   (repository-configured, moderate corroboration)
```

**Sensor surrogate:**
- GNSS: synthetic fix injection (quality, satellites, HDOP)
- LiDAR: analytic scan waveform or Gazebo ray-trace
- IMU: synthetic or Gazebo IMU plugin
- Odom: synthetic twist schedule or Gazebo differential drive
- Camera: not used in current scenarios
- UWB: not used in current scenarios

## 3. Scenario Results

### 3.1 S05: AMCL Covariance Ramp Localization Trigger

**Purpose:** Test MODEL-1 health monitoring and MODEL-3 trigger logic.

**Input conditions:**
- P1 (0-6 s): NOMINAL (RTK fixed, full LiDAR, amcl_cov=0.05)
- P2 (6-12 s): LIDAR_GEOMETRY_DEGRADED (narrow FOV ±10°, amcl_cov=0.05)
- P3 (12-18 s): AMCL_COVARIANCE_RAMP (full LiDAR, amcl_cov 0.20→0.90)
- P4 (18-24 s): AMCL_COVARIANCE_HIGH_HOLD (amcl_cov=0.90)

**Expected behavior:**
- P1: NORMAL
- P2: LiDAR DEGRADED, but NORMAL (degraded-only reason does not trigger)
- P3: AMCL covariance crosses 0.5 → SUSPECTED → TRIGGERED (~14.9 s)
- P4: STOPPING held (plant disabled, no motion)

**Result:**
```
PASS
```

**Evidence:**
- `results/synthetic/final/S05_amcl_covariance_ramp_localization_trigger_20260829-141521/`
- `result.json`: relocalization_state reached SUSPECTED then TRIGGERED in P3
- `timeline.csv`: state transitions at expected times
- `samples.csv`: amcl_covariance values, lidar_state, navigation_state
- `rosbag/rosbag_0.db3`: full message log

**Key findings:**
- ✅ Trigger logic working (suspect_samples=2, trigger_samples=2)
- ✅ LiDAR degradation alone did not trigger (correct)
- ✅ AMCL covariance threshold (0.5) correctly enforced
- ✅ No false triggers during P1/P2

### 3.2 S06: Closed-Loop Active Scan Recovery Motion

**Purpose:** Test MODEL-3 active motion execution and candidate acceptance.

**Input conditions:**
- Trigger from MODEL-1
- Plant enabled (motion possible)
- Scan segments: +10°, -20°, +10° (cumulative 0° return)

**Expected behavior:**
- TRIGGERED → STOPPING → wait for stop
- STOPPING → ACTIVE_SCAN → execute segments
- ACTIVE_SCAN → WAITING_CANDIDATE → await match
- WAITING_CANDIDATE → VERIFYING → multi-frame check
- VERIFYING → RECOVERED → handoff to MODEL-1

**Result:**
```
PASS
```

**Evidence:**
- `results/synthetic/final/S06_closed_loop_active_scan_recovery_motion_20260829-161050/`
- `result.json`: all states reached, motion executed
- Commanded `/dg/relocalization/cmd_vel` published
- Segments completed within timeout
- Candidate accepted, verification passed

**Key findings:**
- ✅ Motion control working (rotation only)
- ✅ Yaw tracking within tolerance (1.5°)
- ✅ Settle time enforced (0.5 s)
- ✅ No timeout (segments completed in ~10-15 s)
- ✅ Safe handoff after verification

### 3.3 S07: Real Candidate from Real Seed Request

**Purpose:** Test candidate source integration and quality evaluation.

**Input conditions:**
- Scan-to-map matcher produces candidate
- Quality metrics: score, inlier_ratio, mean_distance

**Expected behavior:**
- Candidate quality checked against thresholds
- Accept if all pass, reject if any fail
- Accepted candidate enters VERIFYING

**Result:**
```
PASS
```

**Evidence:**
- `results/synthetic/final/S07_real_candidate_from_real_seed_request_20260829-171148/`
- Candidate quality metrics logged
- Acceptance decision recorded

**Key findings:**
- ✅ Quality thresholds enforced (score≥0.45, inlier≥0.45, distance≤0.30)
- ✅ Candidate accepted and verified
- ❌ No explicit rejection test (low-quality candidate not injected)

### 3.4 S08: Multiframe Verification and Control Handoff

**Purpose:** Test verification stability checks and progressive re-entry.

**Input conditions:**
- Candidate accepted
- Verification requires 3 consecutive stable samples
- Stability: position_jump<0.5m, yaw_jump<20°

**Expected behavior:**
- VERIFYING: collect samples, check stability
- If stable → RECOVERED
- If jump → reject, retry
- MODEL-1 progressive re-entry, not immediate NOMINAL

**Result:**
```
PASS
```

**Evidence:**
- `results/synthetic/final/S08_multiframe_verification_and_control_handoff_20260829-171722/`
- Verification samples logged
- Stability checks passed
- Handoff to NOMINAL confirmed

**Key findings:**
- ✅ Multiframe verification working
- ✅ Stability thresholds enforced
- ✅ Progressive re-entry prevents false recovery
- ❌ No explicit jump injection test (instability not tested)

## 4. Test Coverage Matrix

| model | scenario | unit tests | integration | negative tests | pass |
|---|---|---|---|---|---|
| MODEL-1 health | S05 | ✅ | ✅ S05-S08 | ❌ | ✅ |
| MODEL-1 GNSS outage | — | ✅ | ❌ | — | — |
| MODEL-1 LiDAR degradation | S05 P2 | ✅ | ✅ | — | ✅ |
| MODEL-1 concurrent degrad | — | ✅ | ❌ | — | — |
| MODEL-1 recovery | S08 | ✅ | ✅ | — | ✅ |
| MODEL-2 static baseline | — | ✅ | ❌ | — | — |
| MODEL-2 dynamic object | — | ✅ | ❌ | — | — |
| MODEL-2 outlier | — | ✅ | ❌ | — | — |
| MODEL-2 zero-feature | — | ✅ | ❌ | — | — |
| MODEL-3 trigger | S05 | ✅ | ✅ | — | ✅ |
| MODEL-3 active scan | S06 | ✅ | ✅ | — | ✅ |
| MODEL-3 candidate accept | S07 | ✅ | ✅ | ❌ | ✅ |
| MODEL-3 verification | S08 | ✅ | ✅ | ❌ | ✅ |
| MODEL-3 timeout | — | ✅ | ❌ | — | — |
| MODEL-3 GT leak | — | ✅ | ❌ | — | — |
| Integration trigger→recovery | S05-S08 | — | ✅ | — | ✅ |

**Summary:**
- ✅ Core happy paths validated (S05-S08)
- ❌ Missing dedicated MODEL-1 and MODEL-2 integration scenarios
- ❌ Missing negative tests (bad candidate, timeout, GT leak, jump)

## 5. Unit Test Summary

**Run command:**
```bash
cd ~/dg202611_ws
colcon test --packages-select ylhb_base
colcon test-result --verbose
```

**Results:**
- Total tests: 263 (including base kinematics, bringup, diagnostics)
- MODEL-1: 18 tests pass
- MODEL-2: 12 tests pass
- MODEL-3: 24 tests pass
- Other: 209 tests pass (baseline infra)

**Key test files:**
```
test_multisource_fusion_core.py              ✅
test_navigation_health_core.py               ✅
test_lidar_robust_features.py                ✅
test_active_relocalization_core.py           ✅
test_navigation_integration_core.py          ✅
```

## 6. Known Limitations

### 6.1 Synthetic Injector Limitations

**Not real sensor data:**
- GNSS: synthetic fix, not real satellite signal
- LiDAR: analytic waveform or simplified Gazebo ray-trace
- IMU: synthetic or Gazebo plugin, not real noise characteristics
- Odom: scheduled twist, not real wheel encoder noise

**Timing:**
- All scenarios run at 10 Hz
- Sub-100 ms ordering not validated
- Real-time jitter not modeled

**Environment:**
- No real dynamic objects (people, vehicles)
- No real material properties (glass, metal, wet surfaces)
- No real GPS multipath or NLOS

### 6.2 Gazebo Limitations (P0/P1 baseline)

**Physics:**
- Default step size 0.001 s has known robustness issue
- Simplified friction and contact model
- No tire deformation

**Sensors:**
- LiDAR: no intensity, no multi-echo
- Camera: simplified lens model
- No UWB in Gazebo models

### 6.3 Competition Metric Disclaimer

```
REPEATABILITY_SCORES_HERE != COMPETITION_METRIC
LOCALIZATION_ACCURACY_HERE != COMPETITION_METRIC
```

All metrics computed in synthetic/Gazebo are for:
- Software validation (does the code work?)
- Scenario pass/fail (did state transitions happen as expected?)

NOT for:
- Competition performance claims
- Real robot accuracy prediction
- Comparison with other teams

Real competition metrics require:
- Real hardware
- Real surveyed test course
- Real dynamic environment
- Official evaluation protocol

## 7. Validation Evidence Structure

Each scenario result directory contains:

```
scenario.yaml              — input configuration
metadata.json              — run metadata (duration, seed, rates)
result.json                — pass/fail verdict, scenario-specific checks
result.md                  — human-readable summary
samples.csv                — time series (states, health, scores)
timeline.csv               — state transitions
field_roles.csv            — scenario phase definitions
logs/                      — node logs
plots/                     — time series plots
rosbag/rosbag_0.db3        — ROS2 bag (all messages)
```

**Reproducible:**
- Scenario seed fixed
- Input schedule deterministic
- Same scenario.yaml → same result (modulo floating-point)

## 8. Missing Scenarios (To Be Created)

### 8.1 MODEL-1 Dedicated Scenarios

❌ **M1_GNSS_outage:**
- GNSS quality 4 → 0, all other sources nominal
- Verify GNSS marked REJECTED, overall_state DEGRADED (not FAILED)
- Verify fusion continues with LiDAR + odom

❌ **M1_LiDAR_only:**
- LiDAR geometry degraded (narrow FOV), GNSS nominal
- Verify LiDAR marked DEGRADED, does not alone trigger MODEL-3

❌ **M1_concurrent:**
- GNSS + LiDAR both degrade simultaneously
- Verify combined effect triggers MODEL-3

❌ **M1_recovery:**
- Degraded → nominal transition
- Verify progressive source re-entry

### 8.2 MODEL-2 Dedicated Scenarios

❌ **M2_static_baseline:**
- Stationary robot, static environment
- Verify all features stable, repeatability high

❌ **M2_dynamic_object:**
- Inject moving obstacle in Gazebo
- Verify dynamic points rejected, static background retained

❌ **M2_outlier:**
- Inject abnormal ranges, multi-path
- Verify outlier rejection, feature count reasonable

❌ **M2_zero_feature:**
- Empty environment or max-range scan
- Verify graceful handling, no crash

### 8.3 MODEL-3 Negative Tests

❌ **M3_bad_candidate:**
- Inject candidate with score<0.45
- Verify rejection, retry

❌ **M3_timeout:**
- No candidate within max_attempt_duration
- Verify FAILED state, no infinite loop

❌ **M3_verification_jump:**
- Candidate position jumps during verification
- Verify rejection, no false recovery

❌ **M3_GT_leak:**
- Inject candidate with good quality but high GT error
- Verify acceptance (quality passes, GT not checked)
- Inject candidate with poor quality but low GT error
- Verify rejection (quality fails, GT ignored)

## 9. Summary

**Validation status:**
- ✅ Core integration validated (S05-S08 PASS)
- ✅ Happy path working end-to-end
- ✅ Unit tests comprehensive (263 total, MODEL core ~54)
- ✅ Evidence structure complete and reproducible
- ❌ Missing dedicated MODEL-1 and MODEL-2 scenarios
- ❌ Missing negative test scenarios
- ❌ All results marked SYNTHETIC/NOT_COMPETITION_METRIC

**Confidence level:**
- Software correctness: HIGH (unit tests + S05-S08 integration)
- Real robot readiness: MEDIUM (surrogates and sim limitations)
- Competition performance prediction: NONE (synthetic data)

**Next steps:**
- Create missing scenarios (M1, M2, M3 negative tests)
- Run full suite and update this summary
- Document surrogates and gaps (doc 06)
- Document real robot requirements (doc 07)
