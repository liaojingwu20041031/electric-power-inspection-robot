# 11 — Three Model Integration Closure

```
E2E_SCENARIO_CREATED:  1
SCENARIO_RUN:          0 (created, not yet executed)
DOCUMENTATION_STATUS:  COMPLETE
MODEL2_DECISION_PATH:  DOCUMENTED (validation pending)
```

## 1. Overview

End-to-end scenario created to validate the complete three-model layered decision chain:
MODEL-1 detects degradation → MODEL-2 provides evidence → MODEL-1 determines
insufficient confidence → MODEL-3 active recovery → verification → safe handoff →
MODEL-1 progressive re-entry.

## 2. E2E_01: Three Model Layered Decision

**File:** `E2E_01_three_model_layered.yaml`

**Duration:** 36 seconds

**Phases:**
1. **NOMINAL_BASELINE** (0-6s): All sources healthy
2. **GNSS_LIDAR_CONCURRENT_DEGRADATION** (6-12s): Both degrade, AMCL cov starts rising
3. **AMCL_COVARIANCE_CROSSES_THRESHOLD** (12-18s): AMCL cov crosses 0.5 → trigger
4. **ACTIVE_RECOVERY_PHASE** (18-30s): Sources restored, MODEL-3 executes recovery
5. **PROGRESSIVE_REENTRY** (30-36s): MODEL-1 re-enables sources progressively

### 2.1 Expected Decision Flow

**Phase 1 (0-6s): NOMINAL**
```
MODEL-1: all sources GOOD, fusion NOMINAL
MODEL-2: geometry_score ~1.0 (full FOV, static environment)
MODEL-3: NORMAL
```

**Phase 2 (6-12s): CONCURRENT DEGRADATION**
```
MODEL-2: geometry_score drops to ~0.30 (narrow FOV ±15°, valid_fraction 0.6)
  ↓
MODEL-1: receives geometry_score via /dg/lidar/quality
MODEL-1: marks LiDAR DEGRADED, reasons: ["LIDAR_QUALITY_LOW"]
MODEL-1: marks GNSS DEGRADED (quality=1, hdop=5.0), reasons: ["GNSS_DEGRADED"]
MODEL-1: fusion continues with degraded sources + odom
MODEL-1: AMCL cov starts rising (lack of good corrections)
  ↓
MODEL-3: receives health, not yet triggerable (AMCL cov < 0.5)
```

**Phase 3 (12-18s): THRESHOLD CROSSED**
```
MODEL-1: AMCL cov crosses 0.5 (ramps to 0.70)
MODEL-1: trigger_reasons: ["GNSS_DEGRADED", "LIDAR_QUALITY_LOW", "AMCL_COVARIANCE_HIGH"]
MODEL-1: overall_state → LOCALIZATION_SUSPECT
  ↓
MODEL-3: receives triggerable health
MODEL-3: NORMAL → SUSPECTED (2 samples) → TRIGGERED (2 more samples)
MODEL-3: publishes relocalization_state TRIGGERED
  ↓
MODEL-1: receives TRIGGERED signal
MODEL-1: overall_state → RECOVERING
```

**Phase 4 (18-30s): ACTIVE RECOVERY**
```
MODEL-3: TRIGGERED → STOPPING → ACTIVE_SCAN
MODEL-3: executes scan segments (+10°, -20°, +10°)
  ↓
MODEL-2: extracts stable features from each segment
MODEL-2: publishes stable_features to /dg/lidar/stable_features
  ↓
Scan-to-map matcher: consumes /dg/lidar/filtered_scan (not raw /scan)
Matcher: produces candidate with quality metrics
  ↓
MODEL-3: evaluates candidate (score, inlier_ratio, mean_distance)
MODEL-3: candidate passes thresholds → WAITING_CANDIDATE → VERIFYING
MODEL-3: multiframe verification (3 samples, position/yaw stable)
MODEL-3: verification passes → RECOVERED
MODEL-3: publishes relocalization_state RECOVERED
  ↓
MODEL-1: receives RECOVERED signal
MODEL-1: begins staged re-entry and keeps navigation health at RECOVERING
```

**Phase 5 (30-36s): PROGRESSIVE RE-ENTRY**
```
MODEL-1: checks post-recovery health
MODEL-1: sources now nominal (GNSS quality=4, LiDAR full FOV)
MODEL-1: requires AMCL GOOD plus fresh odometry and scan for a continuous interval
MODEL-1: a gap resets the interval instead of granting navigation authority
MODEL-1: after the interval → RECOVERED once → NOMINAL
```

### 2.2 MODEL-2 Decision Path Validation

**Critical requirement:**
```
MODEL-2 geometry_score must ACTUALLY influence MODEL-1 decision, not just logged.
```

**Evidence to verify:**
- `/dg/lidar/quality` published with geometry_score time series
- `/dg/navigation/status` shows lidar_health transitions based on geometry_score
- `timeline.csv` shows LiDAR marked DEGRADED when geometry_score drops in P2
- MODEL-1 health aggregation code reads geometry_score (code review confirms)

**If MODEL-2 decision path is DIAGNOSTIC_ONLY:**
- geometry_score published but ignored by MODEL-1
- LiDAR health transitions NOT driven by geometry_score
- MODEL-2 does not participate in decision, only logs data

**Expected result (if properly integrated):**
```
MODEL2_DECISION_PATH: ACTIVE
```

Geometry_score directly influences MODEL-1 lidar_health, which contributes to
overall_state and trigger decision.

**If integration incomplete:**
```
MODEL2_DECISION_PATH: DIAGNOSTIC_ONLY
THREE_MODEL_INTEGRATION: PARTIAL
```

Then additional work needed to wire MODEL-2 output into MODEL-1 decision logic.

### 2.3 Layered Decision Verification

**What distinguishes layered decision from test path:**

❌ **Test path (not layered):**
- MODEL-1 triggers based on AMCL cov only (ignores MODEL-2 geometry_score)
- MODEL-3 uses raw scan (ignores MODEL-2 stable features)
- Models run in parallel but decisions independent

✅ **Layered decision (true integration):**
- MODEL-2 geometry_score → MODEL-1 lidar_health → MODEL-1 trigger decision
- MODEL-2 filtered_scan → matcher input → MODEL-3 candidate quality
- Each model's output feeds next layer's decision

**Validation checklist:**
```
✓ MODEL-2 geometry_score visible in MODEL-1 logs
✓ MODEL-1 lidar_health transitions match geometry_score changes
✓ Matcher uses /dg/lidar/filtered_scan (not raw /scan)
✓ MODEL-3 candidate quality reflects stable feature input
✓ All three models' decisions visible in timeline
```

## 3. Execution Plan

**Scenario file:** ✅ CREATED (`E2E_01_three_model_layered.yaml`)

**Execution readiness:** ✅ READY (synthetic, no Gazebo required)

**To execute:**
```bash
cd ~/dg202611_ws/src/electric-power-inspection-robot
ros2 launch dg_synthetic_validation synthetic_validation.launch.py \
  scenario:=scenarios/E2E_01_three_model_layered.yaml \
  output_dir:=~/dg202611_ws/results/synthetic/integration_closure/
```

**Expected runtime:** ~40 seconds (36s scenario + 4s setup/teardown)

**Evidence to collect:**
```
result.json              — pass/fail verdict
timeline.csv             — state transitions (all 3 models)
samples.csv              — time series (geometry_score, health, states)
logs/model1_node.log     — MODEL-1 decisions
logs/model2_node.log     — MODEL-2 feature extraction
logs/model3_node.log     — MODEL-3 state machine
rosbag/rosbag_0.db3      — all messages
plots/                   — time series visualization
```

**Key plots to generate:**
```
- geometry_score vs. time (MODEL-2 output)
- lidar_health vs. time (MODEL-1 response to MODEL-2)
- amcl_covariance vs. time (trigger condition)
- relocalization_state vs. time (MODEL-3 state machine)
- overall_state vs. time (MODEL-1 aggregation)
```

## 4. Post-Run Analysis

**After execution, verify:**

### 4.1 MODEL-2 → MODEL-1 Link
```bash
# Extract geometry_score time series:
ros2 bag play rosbag_0.db3 --topics /dg/lidar/quality

# Extract lidar_health transitions:
grep "lidar_health" logs/model1_node.log

# Verify correlation:
# P2 (6-12s): geometry_score drops → lidar_health DEGRADED
# P5 (30-36s): geometry_score restored → lidar_health GOOD
```

**Expected:**
- Geometry_score drops in P2 → LiDAR marked DEGRADED within 1 second
- Proves MODEL-1 reads and acts on MODEL-2 output

### 4.2 MODEL-2 → MODEL-3 Link (via Matcher)
```bash
# Verify matcher subscribed to the filtered scan:
ros2 topic info /dg/lidar/filtered_scan

# Verify candidate quality reflects stable input:
grep "candidate" logs/model3_node.log | grep "score"

# Compare:
# P4 (18-30s): matcher uses stable features → candidate quality reasonable
```

**Expected:**
- Matcher subscribes to `/dg/lidar/filtered_scan` (not raw `/scan`)
- Candidate quality metrics reflect stable feature input

### 4.3 Three-Way Decision Timeline
```bash
# Generate combined timeline plot:
python plot_three_model_timeline.py samples.csv

# Expected phases:
# P1: all NORMAL
# P2: MODEL-2 degrades → MODEL-1 marks DEGRADED → MODEL-3 still NORMAL
# P3: MODEL-1 crosses threshold → MODEL-3 TRIGGERED
# P4: MODEL-3 active recovery → MODEL-1 RECOVERING
# P5: MODEL-3 RECOVERED → MODEL-1 progressive re-entry → all NORMAL
```

## 5. Decision Path Determination

**If geometry_score → lidar_health correlation confirmed:**
```
MODEL2_DECISION_PATH: ACTIVE
THREE_MODEL_INTEGRATION: PASS
```

**If geometry_score published but lidar_health independent:**
```
MODEL2_DECISION_PATH: DIAGNOSTIC_ONLY
THREE_MODEL_INTEGRATION: PARTIAL
REMEDIATION_REQUIRED: Wire MODEL-2 output into MODEL-1 health logic
```

**If filtered_scan is not consumed by matcher:**
```
MODEL2_DECISION_PATH: DIAGNOSTIC_ONLY
REMEDIATION_REQUIRED: Configure matcher to subscribe /dg/lidar/filtered_scan
```

## 6. Comparison with S05-S08

**S05-S08 (existing):**
- Validated MODEL-1 trigger and MODEL-3 recovery individually
- MODEL-2 implicit (geometry_score existed but may not drive decisions)
- Primarily MODEL-3-focused (active relocalization)

**E2E_01 (new):**
- Explicitly validates MODEL-2 → MODEL-1 → MODEL-3 decision chain
- Concurrent degradation (not single-source)
- Progressive re-entry explicitly tested
- All three models' contributions visible

**Coverage complement:**
```
S05: MODEL-1 trigger (AMCL cov ramp)
S06: MODEL-3 motion
S07: MODEL-3 candidate
S08: MODEL-3 verification

E2E_01: MODEL-1 + MODEL-2 + MODEL-3 layered decision
```

## 7. Summary

**Three-model integration closure status:**
- ✅ E2E scenario created
- ✅ Layered decision flow documented
- ✅ Validation checklist defined
- ✅ Post-run analysis plan specified
- ❌ Not yet executed
- ⚠️ MODEL2_DECISION_PATH determination pending execution

**Expected outcome (if properly integrated):**
```
THREE_MODEL_INTEGRATION: PASS
MODEL2_DECISION_PATH: ACTIVE
```

**Acceptable closure:**
- Scenario created and execution-ready
- Validation criteria clearly defined
- If execution shows PARTIAL integration, remediation path documented

**Next step:** Execute E2E_01, analyze results, update MODEL2_DECISION_PATH status.
