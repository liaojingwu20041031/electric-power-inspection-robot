# 09 — MODEL-2 Scenario Closure

```
SCENARIOS_CREATED:     4
SCENARIOS_RUN:         0 (created, not yet executed)
DOCUMENTATION_STATUS:  COMPLETE
EXECUTION_READY:       PARTIAL (3 synthetic, 1 requires Gazebo)
```

## 1. Overview

Four scenarios created to validate MODEL-2 stable feature extraction, dynamic
filtering, outlier rejection, and repeatability evaluation under various conditions.

## 2. Scenarios

### 2.1 M2_01: Static Baseline

**File:** `M2_01_static_baseline.yaml`

**Phases:**
- P1 (0-12s): STATIC_ENVIRONMENT (robot stationary, no dynamics)

**Expected behavior:**
- All scan points classified as static
- High temporal persistence (points appear in consecutive scans)
- No dynamic filtering triggered
- Repeatability high (R_avg > 0.90 expected)
- stable_feature_count ≈ raw_feature_count (minimal filtering)

**Validation points:**
```
✓ retention_static > 0.95 (most points classified static)
✓ retention_stable > 0.90 (static points also stable)
✓ R_avg > 0.90 (high repeatability in static environment)
✓ valid_pair_count > 0 (no zero-feature edge case)
✓ No false dynamic detections
```

**Guard:**
```
DO_NOT_DELETE_ALL_FEATURES_FOR_HIGH_REPEATABILITY
```
If stable_feature_count approaches 0 but R_avg is 1.0, this is a bug (empty scan
artificially inflates repeatability). Minimum feature count threshold must prevent this.

**Evidence to collect:**
- `R_min`, `R_avg`, `R_pool`
- `retention_static`, `retention_stable`
- `valid_pair_count`, `zero_feature_pair_count`
- Feature count time series

**Execution:** ✅ READY (synthetic, no Gazebo required)

---

### 2.2 M2_02: Dynamic Object Filtering

**File:** `M2_02_dynamic_object.yaml`

**Phases:**
- P1 (0-6s): STATIC_PHASE (no dynamics)
- P2 (6-12s): DYNAMIC_OBJECT_PRESENT (valid_fraction 0.85, simulating occlusion)
- P3 (12-18s): DYNAMIC_CLEARED (valid_fraction 1.0)

**Expected behavior:**
- P1: baseline static features
- P2: dynamic points detected and filtered
- P2: stable background features retained
- P2: repeatability drops slightly (dynamic occludes some static features)
- P3: full feature set restored

**Validation points:**
```
✓ Dynamic points rejected in P2
✓ Static background retained despite dynamic object
✓ Feature count drops in P2 but does not go to zero
✓ Repeatability recovers in P3
✓ No crash or instability
```

**Limitation:**
```
NOTE: Current analytic scan generator does not model moving objects.
valid_fraction reduction is a surrogate (removes random points, not coherent moving blob).
Full validation requires Gazebo with dynamic actor (person walking through FOV).
```

**Execution:** ⚠️ PARTIAL (synthetic surrogate, Gazebo actor recommended)

**Gazebo setup (if available):**
- Add actor to world file (person walking across LiDAR FOV)
- Record /scan during motion
- Verify dynamic filtering removes moving points

---

### 2.3 M2_03: Outlier / Abnormal Reflection

**File:** `M2_03_outlier_injection.yaml`

**Phases:**
- P1 (0-4s): CLEAN_BASELINE (valid_fraction 1.0)
- P2 (4-8s): OUTLIER_INJECTION (valid_fraction 0.80, simulating abnormal returns)
- P3 (8-12s): OUTLIER_CLEARED (valid_fraction 1.0)

**Expected behavior:**
- P1: clean baseline
- P2: outliers detected (isolated points, range discontinuities)
- P2: outliers rejected by robust filter
- P2: feature count drops slightly but core features retained
- P3: full feature set restored

**Validation points:**
```
✓ Outlier rejection active in P2
✓ Core features not over-filtered
✓ Repeatability remains reasonable (> 0.70)
✓ No crash on abnormal data
```

**Real outlier types (need real LiDAR to fully test):**
- Specular reflections (glass, metal)
- Multi-path returns
- Far outliers (beyond expected range)
- Beam occlusion artifacts

**Execution:** ✅ READY (synthetic surrogate, full test needs real LiDAR)

---

### 2.4 M2_04: Zero Feature Edge Case

**File:** `M2_04_zero_feature.yaml`

**Phases:**
- P1 (0-4s): NORMAL_FEATURES (valid_fraction 1.0)
- P2 (4-8s): EMPTY_ENVIRONMENT (valid_fraction 0.0, all max range)
- P3 (8-12s): FEATURES_RESTORED (valid_fraction 1.0)

**Expected behavior:**
- P1: normal feature extraction
- P2: zero or near-zero features extracted
- P2: repeatability undefined or explicitly marked ZERO_FEATURE
- P2: MODEL-2 does NOT crash
- P2: geometry_score → 0.0 (unusable)
- P3: features restored, normal operation

**Validation points:**
```
✓ Graceful handling of zero-feature case (no crash, no assert)
✓ zero_feature_pair_count increments in P2
✓ Repeatability reported as undefined or 0.0 (not artificially 1.0)
✓ geometry_score correctly reflects unusable state
✓ Recovery when features return
```

**Critical guard:**
```
IF feature_count == 0:
    R = undefined (or 0.0 by convention)
    NOT R = 1.0 (this would falsely imply high repeatability)
```

**Evidence to collect:**
- `zero_feature_pair_count` in P2
- `R_avg` handling (undefined or 0.0, not 1.0)
- `geometry_score` time series (must drop to ~0.0 in P2)

**Execution:** ✅ READY (synthetic)

---

## 3. Repeatability Evaluation Status

**Metrics implemented:**
```
R_min:              minimum repeatability across all pairs
R_avg:              average repeatability
R_pool:             pooled repeatability
valid_pair_count:   pairs with non-zero features
zero_feature_pair_count: pairs where either scan had zero features
retention_static:   fraction classified static
retention_stable:   fraction static that are also stable
```

**Marked as:**
```
SIMULATION_DIAGNOSTIC_ONLY
NOT_COMPETITION_METRIC
```

**Why:**
- Computed on synthetic or Gazebo data (not real LiDAR noise)
- Analytic scan waveform (not real geometry)
- No real dynamic objects or material properties
- Cannot predict real robot competition performance

**Upgrade path:**
- Run on real LiDAR data (RPLiDAR A2/A3)
- Real dynamic environment (people walking)
- Real surveyed map
- Compare with competition baseline

---

## 4. Implementation Status

**Scenario files:** ✅ CREATED (4 YAML files)

**Execution readiness:**
- ✅ M2_01, M2_03, M2_04: synthetic, ready to run
- ⚠️ M2_02: synthetic surrogate created, Gazebo actor recommended for full test

**Not yet executed:** Scenarios created but not run.

**To execute:**
```bash
# Synthetic-ready scenarios:
for scenario in M2_01 M2_03 M2_04; do
  ros2 launch dg_synthetic_validation synthetic_validation.launch.py \
    scenario:=scenarios/${scenario}_*.yaml \
    output_dir:=~/dg202611_ws/results/synthetic/model2_closure/
done

# M2_02 (Gazebo actor, if available):
# 1. Launch Gazebo with actor in world
# 2. Record /scan during actor motion
# 3. Run feature extraction and repeatability eval offline
```

---

## 5. Expected Results Summary

| scenario | feature count | repeatability | special handling |
|---|---|---|---|
| M2_01 static | ~100-200 | R_avg > 0.90 | none |
| M2_02 dynamic | ~80-170 (filtered) | R_avg > 0.70 | dynamic filtering active |
| M2_03 outlier | ~80-180 (cleaned) | R_avg > 0.70 | outlier rejection |
| M2_04 zero | 0 in P2 | undefined/0.0 | zero-feature guard |

---

## 6. Known Limitations

**Engineering baseline (not learning-based):**
```
SURROGATE_NOW: geometric feature extraction
FINAL_ALGORITHM_LATER: learning-based (if justified)
```

Current implementation uses:
- Range discontinuity
- Local curvature
- Temporal persistence
- Statistical outlier rejection

NOT:
- Deep neural network
- Semantic segmentation
- Learned dynamic object classifier

**Justification:**
- 2D LiDAR data insufficient for complex learning
- Geometric methods provide interpretable baseline
- Can upgrade if real robot failure modes demand it

---

## 7. Integration with MODEL-1 and MODEL-3

**MODEL-2 → MODEL-1:**
```
geometry_score (0.0 … 1.0) published to /dg/lidar/quality
MODEL-1 reads geometry_score for health assessment
If geometry_score < min_lidar_quality (0.2) → LiDAR marked DEGRADED
```

**MODEL-2 → MODEL-3:**
```
stable_features published to /dg/lidar/stable_features
Scan-to-map matcher consumes stable features (not raw scan)
Matcher produces candidate with quality metrics
MODEL-3 evaluates candidate
```

**Validated in E2E scenario (E2E_01):**
- MODEL-2 geometry_score drops → MODEL-1 marks LiDAR DEGRADED
- Combined with GNSS degradation → MODEL-3 trigger
- MODEL-3 active scan uses MODEL-2 stable features
- Candidate verified → recovery

---

## 8. Summary

**MODEL-2 scenario closure status:**
- ✅ 4 scenarios created
- ✅ All validation points documented
- ✅ 3/4 execution-ready (synthetic)
- ⚠️ 1/4 requires Gazebo actor (M2_02)
- ❌ Not yet executed
- ✅ Fills gaps identified in doc 05

**Confidence level:**
- Geometric baseline: HIGH (unit tests + documented scenarios)
- Dynamic filtering: MEDIUM (synthetic surrogate, Gazebo actor preferred)
- Zero-feature handling: HIGH (explicit edge case)
- Learning upgrade necessity: UNCLEAR (may not be needed)
