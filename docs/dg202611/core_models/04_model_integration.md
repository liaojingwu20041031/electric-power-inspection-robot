# 04 — Three-Model Integration

```
INTEGRATION_STATUS:   VALIDATED (S05-S08)
INTERFACE_LOCATION:   ROS2 topics, state machines
VALIDATION:           SYNTHETIC
REAL_HARDWARE:        ROBOT_DEPLOYMENT_PENDING
```

## 1. Overview

The three core models form an integrated loop for robust localization under degraded
sensor conditions. Each model has a specific role, and their interfaces are designed
to minimize coupling while maintaining coherent system behavior.

## 2. Integration Architecture

### 2.1 Information Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                          MODEL-1                                │
│         Multi-Source Fusion + Health Monitoring                 │
│                                                                 │
│  Inputs:  /odom, accepted GNSS, /amcl_pose, scan match, UWB    │
│  Outputs: /dg/fusion/* and /dg/navigation/status               │
│           overall_state, source_health[], reasons[]             │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     │ health degradation
                     ↓
┌─────────────────────────────────────────────────────────────────┐
│                          MODEL-2                                │
│         LiDAR Stable Feature Extraction                         │
│                                                                 │
│  Inputs:  /scan                                                 │
│  Outputs: /dg/lidar/filtered_scan                               │
│           /dg/lidar/stable_features                             │
│           /dg/lidar/quality (geometry_score)                    │
│                                                                 │
│  ← feeds geometry_score to MODEL-1 health assessment            │
│  ← provides filtered scan to the scan-to-map matcher            │
└─────────────────────────────────────────────────────────────────┘
                     │
                     │ filtered scan and quality
                     ↓
┌─────────────────────────────────────────────────────────────────┐
│                          MODEL-3                                │
│         Active Relocalization Supervisor                        │
│                                                                 │
│  Inputs:  MODEL-1 health (amcl_covariance, source_health)      │
│           MODEL-2 filtered scan (via scan matcher)              │
│  Outputs: /dg/relocalization/status                             │
│           /cmd_vel_recovery (isolated recovery commands)        │
│                                                                 │
│  Trigger: sustained poor health from MODEL-1                    │
│  Action:  scan segments to gather new evidence                  │
│  Result:  candidate → verification → RECOVERED                  │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     │ recovery confirmed
                     ↓
┌─────────────────────────────────────────────────────────────────┐
│                          MODEL-1                                │
│            Progressive Source Re-Entry                          │
│                                                                 │
│  ← receives RECOVERED signal from MODEL-3                       │
│  → checks post-recovery health                                  │
│  → holds RECOVERING for a continuous healthy interval           │
│  → emits RECOVERED once, then returns to NOMINAL                │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Key Interfaces

#### MODEL-1 → MODEL-3

**Topic:** `/dg/navigation/status`

**Message fields:**
```python
overall_state: str           # NOMINAL, DEGRADED, LOCALIZATION_SUSPECT, RECOVERING
reasons: List[str]           # degradation/trigger reasons
source_health: Dict[str, SourceHealth]
timestamp: float
```

**Trigger condition:**
```python
if amcl_covariance > max_covariance (0.5):
    trigger_reasons.append("AMCL_COVARIANCE_HIGH")
    triggerable = True
```

#### MODEL-2 → MODEL-1

**Topic:** `/dg/lidar/quality`

**Message fields:**
```python
geometry_score: float        # 0.0 (unusable) … 1.0 (ideal)
point_count: int
angular_span: float
stable_ratio: float
timestamp: float
```

**Used by MODEL-1:**
```python
if geometry_score < min_lidar_quality (0.2):
    lidar_health = DEGRADED
    reasons.append("LIDAR_QUALITY_LOW")
```

#### MODEL-2 → MODEL-3 (via scan matcher)

**Indirect:** MODEL-2 publishes a filtered `LaserScan`; the scan-to-map matcher
consumes that scan, produces candidates, and MODEL-3 evaluates the candidates.
`/dg/lidar/stable_features` is a `PoseArray` used for feature evidence and
repeatability analysis; it is not the matcher's scan input.

**Flow:**
```
MODEL-2: /dg/lidar/filtered_scan (LaserScan)
  ↓
Scan Matcher: /scan_match_pose + /dg/relocalization/match_quality
  ↓
MODEL-3: candidate evaluation and verification
```

#### MODEL-3 → MODEL-1

**Topic:** `/dg/relocalization/status`

**Message fields:**
```python
state: str                   # NORMAL, TRIGGERED, ACTIVE_SCAN, VERIFYING, RECOVERED
trigger_reason: List[str]
amcl_health: str             # GOOD, BAD
recovery_success: bool
timestamp: float
```

**Used by MODEL-1:**
```python
if relocalization_state == "RECOVERED":
    # staged re-entry, not immediate navigation authority
    if amcl_good and odom_fresh and scan_fresh:
        hold_recovering_for(post_recovery_stable_sec)
        emit_once("RECOVERED")
        overall_state = "NOMINAL"
```

## 3. Integration Scenarios

### 3.1 S05: Trigger Chain

**Scenario:** AMCL covariance ramps from 0.05 to 0.90 over 6 seconds.

**Model interactions:**
1. **MODEL-1** detects `amcl_covariance > 0.5` for 2 consecutive samples
2. **MODEL-1** state: NORMAL → LOCALIZATION_SUSPECT
3. **MODEL-1** publishes trigger reasons: `["AMCL_COVARIANCE_HIGH"]`
4. **MODEL-3** receives health, `triggerable=True`
5. **MODEL-3** accumulates suspect samples → SUSPECTED
6. **MODEL-3** accumulates trigger samples → TRIGGERED
7. **MODEL-3** transitions to STOPPING

**Validation:**
- ✅ MODEL-1 correctly marks AMCL health as BAD
- ✅ MODEL-3 receives trigger and transitions to TRIGGERED
- ✅ Timeline: TRIGGERED observed ~14.9 s (within P3 ramp phase)

### 3.2 S06: Active Recovery Loop

**Scenario:** Trigger → active scan → candidate → verification → recovery.

**Model interactions:**
1. **MODEL-3** TRIGGERED → STOPPING → ACTIVE_SCAN
2. **MODEL-3** executes scan segments (rotation)
3. **MODEL-2** provides stable features per segment
4. Scan matcher produces candidate using MODEL-2 features
5. **MODEL-3** receives candidate, checks quality
6. Candidate accepted → VERIFYING
7. Multi-frame verification (3 samples, stable pose)
8. Verification passes → RECOVERED
9. **MODEL-1** receives RECOVERED signal
10. **MODEL-1** checks post-recovery health
11. **MODEL-1** returns to NOMINAL (progressive re-entry)

**Validation:**
- ✅ Closed-loop motion executed
- ✅ Candidate accepted and verified
- ✅ Safe handoff to NOMINAL

### 3.3 S07: Candidate Quality Gate

**Scenario:** Test candidate evaluation with real scan matcher output.

**Model interactions:**
1. **MODEL-3** ACTIVE_SCAN complete
2. Scan matcher produces candidate with quality metrics
3. **MODEL-3** evaluates:
   ```
   score >= 0.45 ✓
   inlier_ratio >= 0.45 ✓
   mean_distance <= 0.30 m ✓
   ```
4. Candidate accepted → VERIFYING
5. Verification confirms stable pose → RECOVERED

**Validation:**
- ✅ Quality thresholds enforced
- ✅ Low-quality candidates would be rejected (not tested in S07, but logic present)

### 3.4 S08: Multiframe Verification and Handoff

**Scenario:** Test verification stability and progressive re-entry.

**Model interactions:**
1. Candidate received
2. **MODEL-3** VERIFYING: collect 3 consecutive samples
3. Each sample checked for:
   ```
   position_jump < 0.5 m
   yaw_jump < 20°
   quality still passing thresholds
   ```
4. All samples stable → RECOVERED
5. **MODEL-1** confirms post-recovery health before NOMINAL

**Validation:**
- ✅ Multiframe verification working
- ✅ No false recovery (jump would reject)
- ✅ Progressive re-entry prevents immediate full trust

## 4. Concurrent Degradation Handling

### 4.1 Scenario: GNSS Outage + LiDAR Degradation

**Not yet explicitly tested, but logic present:**

1. **MODEL-1** receives poor GNSS quality
2. GNSS marked REJECTED, reasons: `["GNSS_REJECTED"]`
3. **MODEL-2** reports low geometry_score (narrow FOV or low structure)
4. **MODEL-1** marks LiDAR DEGRADED, reasons: `["LIDAR_QUALITY_LOW"]`
5. **MODEL-1** overall_state: NOMINAL → DEGRADED
6. If AMCL covariance also rises (due to lack of corrections):
   ```
   reasons: ["GNSS_REJECTED", "LIDAR_QUALITY_LOW", "AMCL_COVARIANCE_HIGH"]
   triggerable = True
   ```
7. **MODEL-3** triggers active recovery

**Key principle:**
```
Individual degraded sources do not alone trigger MODEL-3.
Only when combined effect crosses health threshold (e.g., AMCL covariance)
does system escalate to active recovery.
```

### 4.2 Missing Explicit Test

❌ **Concurrent GNSS + LiDAR degradation scenario**
- Inject both simultaneously
- Verify neither alone triggers, but combined does
- Verify recovery restores both sources progressively

## 5. GT Leak Prevention

### 5.1 Design Principle

```
GT_ONLY_FOR_EVALUATION
GT_NEVER_IN_ONLINE_DECISION
```

Ground truth used only in:
- Post-run evaluation (compare estimated pose to GT)
- Plotting and visualization
- Pass/fail verdict generation

GT never used in:
- Candidate acceptance (only matcher quality metrics)
- Health assessment (only sensor data)
- State transitions (only internal state machine logic)

### 5.2 Verification

**Unit tests:**
- `test_active_relocalization_core.py` verifies no GT parameter in candidate evaluation
- `test_multisource_fusion_core.py` verifies no GT in fusion

**Scenario validation:**
- S06-S08 results show candidate acceptance based on score/inlier/distance, not GT error
- No GT topic subscribed by MODEL-1, MODEL-2, or MODEL-3 nodes

**Missing explicit test:**
❌ **GT leak negative test**
- Inject candidate with poor quality but low GT error
- Verify rejection (quality fails, GT ignored)
- Inject candidate with good quality but high GT error
- Verify acceptance (quality passes, GT not checked online)

## 6. Known Gaps and Surrogates

### 6.1 Surrogates in Integration

| component | model | status |
|---|---|---|
| Scan-to-map matcher | MODEL-3 | SURROGATE (uses AMCL or scan_tools) |
| Visual odometry | MODEL-1 | SURROGATE (ZED interface, quality scoring placeholder) |
| UWB ranging | MODEL-1 | SURROGATE (no real hardware) |
| Action policy | MODEL-3 | SURROGATE (rotation-only, not learned) |
| Dynamic filtering | MODEL-2 | Engineering baseline (not learned) |

### 6.2 Integration Gaps

❌ **UWB + GNSS + LiDAR fusion**
- UWB interface present but not tested in integration
- Need scenario with all three absolute sources

❌ **Visual odometry in health**
- ZED wrapper present, not integrated into MODEL-1 health aggregation

❌ **BDS short message**
- Not integrated, future work

## 7. Real Robot Integration Checklist

Before deploying three-model integration on real hardware:

### 7.1 MODEL-1
- ✅ Accept simulation config (wheel_track, wheel_radius)
- ❌ Set GNSS ENU reference from first RTK fix
- ❌ Tune quality thresholds for real sensor noise
- ❌ Integrate real UWB hardware
- ❌ Complete ZED integration

### 7.2 MODEL-2
- ❌ Tune persistence window for real LiDAR rate
- ❌ Test with real dynamic objects
- ❌ Validate outlier rejection on real materials
- ❌ Benchmark repeatability on surveyed course

### 7.3 MODEL-3
- ❌ Tune thresholds for real motion dynamics
- ❌ Validate safety constraints (collision avoidance)
- ❌ Test timeout recovery in real environment
- ❌ Verify arbitration with navigation stack

### 7.4 Integration
- ❌ End-to-end test on real vehicle (trigger → active scan → recovery)
- ❌ Concurrent degradation test (GNSS + LiDAR real failures)
- ❌ Long-duration stability test (multiple recovery cycles)
- ❌ Competition scenario dry run

## 8. Summary

**Integration status:**
- ✅ MODEL-1 ↔ MODEL-3 interface validated (S05-S08)
- ✅ MODEL-2 → MODEL-1 quality input working
- ✅ MODEL-2 → MODEL-3 feature input working (via matcher)
- ✅ Trigger → active recovery → verification → handoff loop complete
- ✅ Progressive source re-entry prevents false recovery
- ❌ Missing concurrent degradation explicit test
- ❌ Missing GT leak negative test
- ❌ Surrogates clearly marked

**Next steps:**
- Create concurrent degradation scenario
- Create GT leak negative test
- Document real robot integration requirements
- Generate validation summary
