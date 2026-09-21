# 03 — MODEL-3 Implementation: Embodied Active Relocalization and Safe Recovery

```
IMPLEMENTATION_STATUS:  COMPLETE
CODE_LOCATION:          src/ylhb_base/scripts/active_relocalization_core.py
UNIT_TESTS:             test_active_relocalization_core.py
VALIDATION:             SYNTHETIC (S05-S08 scenarios)
REAL_HARDWARE:          NOT_YET_INTEGRATED
```

## 1. Overview

MODEL-3 implements embodied active relocalization: when passive localization health
degrades, the system executes deliberate motion (scan segments, viewpoint changes)
to gather new evidence, evaluate candidates, verify recovery, and safely hand control
back to nominal operation.

## 2. Core Component

### 2.1 active_relocalization_core.py (688 lines)

**Purpose:** Supervisor state machine for active recovery from localization failure.

**Key classes:**
- `LocalizationHealth` — health inputs from MODEL-1
- `CandidateQuality` — match quality from scan-to-map
- `ActiveRelocalizationConfig` — thresholds and parameters
- State machine with 11 states

**States:**
```
NORMAL              — passive localization healthy
SUSPECTED           — degradation detected, not yet trigger
TRIGGERED           — confirmed degradation, escalate
STOPPING            — commanding stop before active scan
ACTIVE_SCAN         — executing scan segments
WAITING_CANDIDATE   — scan complete, awaiting match result
VERIFYING           — candidate received, multi-frame verification
RECOVERED           — verification passed, safe handoff
FAILED              — recovery exhausted, timeout
MANUAL_REQUIRED     — cannot auto-recover, human intervention
```

## 3. Trigger Mechanism

### 3.1 Health-Based Trigger

Receives health from MODEL-1:
- `amcl_covariance` — pose uncertainty
- `lidar_quality` — geometry score from MODEL-2
- `gnss_quality` — RTK status
- `scan_match_score`, `scan_match_inlier_ratio`, `scan_match_mean_distance`

**Degradation reasons:**
```
AMCL_COVARIANCE_HIGH
GNSS_REJECTED
GNSS_DEGRADED
LIDAR_QUALITY_LOW
SCAN_MATCH_QUALITY_LOW
SIGNAL_STALE (any source timeout)
```

**Trigger logic:**
1. Assess health every cycle (10 Hz typical)
2. If any triggerable reason present → `triggerable=True`
3. Accumulate `triggerable` samples
4. `suspect_samples` (default 2) → SUSPECTED
5. `trigger_samples` (default 2) additional → TRIGGERED

**Key principle:**
```
DEGRADED_ONLY_REASON != TRIGGER
```

LiDAR geometry degradation or GNSS quality drop marks degraded, but does not
alone trigger active recovery. Only sustained poor AMCL covariance or match
quality crosses trigger threshold.

### 3.2 Streak Logic

Prevents premature trigger on transient noise:
- Requires N consecutive poor-health samples
- Single good sample resets streak
- Hysteresis: recovery requires M consecutive good samples

## 4. Active Motion Strategy

### 4.1 Scan Segments (First Version)

**Default action:** small-angle rotations to gather new viewpoints

**Parameters:**
- `segment_deltas`: tuple of yaw increments (default: +10°, -20°, +10°)
- `angular_speed`: rotation speed (default: 0.18 rad/s)
- `yaw_tolerance`: arrival criterion (default: 1.5°)
- `segment_timeout`: max time per segment (default: 8 s)
- `settle_time`: pause after arrival (default: 0.5 s)

**Execution:**
1. STOPPING → wait for velocity < `stop_velocity` (0.03 m/s)
2. ACTIVE_SCAN → publish /cmd_vel with angular velocity
3. Monitor /odom yaw, stop when within tolerance
4. Settle, then next segment
5. After all segments → WAITING_CANDIDATE

**Safety constraints:**
- `max_total_rotation`: cumulative yaw limit (default: 60°)
- `max_attempt_duration`: timeout per attempt (default: 30 s)
- No translation motion (only rotation)
- Commanded to `/dg/relocalization/cmd_vel`, not real `/cmd_vel` (arbitration)

### 4.2 Alternative Actions (Not Yet Implemented)

First version uses rotation only. Future extensions:
- Small forward/backward translation for parallax
- Spiral search pattern
- RL-based action selection

**Current status:**
```
ROTATION_ONLY
FINAL_ALGORITHM_LATER (for complex action policies)
```

## 5. Candidate Evaluation

### 5.1 Candidate Source

Candidates provided by existing scan-to-map matcher (SURROGATE):
- Uses AMCL, scan_tools, or equivalent matcher
- MODEL-3 does not implement scan matching itself
- Receives candidate pose + quality metrics

**Quality dimensions:**
- `score`: matcher confidence (0…1)
- `inlier_ratio`: fraction of scan points matched (0…1)
- `mean_distance`: average point-to-map error (m)
- `used_points`: number of points in match

### 5.2 Acceptance Criteria

Candidate must pass all thresholds:
```python
score >= min_scan_match_score (0.45)
inlier_ratio >= min_scan_match_inlier_ratio (0.45)
mean_distance <= max_scan_match_mean_distance (0.30 m)
```

**Rejection reasons:**
- SCORE_TOO_LOW
- INLIER_RATIO_TOO_LOW
- MEAN_DISTANCE_TOO_HIGH
- USED_POINTS_TOO_LOW (if threshold set)

**Ground truth guard:**
```
GT_NEVER_PARTICIPATES_IN_ONLINE_DECISION
```

GT used only for post-run evaluation, never for candidate acceptance.

## 6. Multi-Frame Verification

### 6.1 Verification Phase

Candidate acceptance does not immediately restore NORMAL. Must verify:

**VERIFYING state:**
- Receive N consecutive candidate updates (default: `verification_samples=3`)
- Each must pass quality thresholds
- Position/yaw must remain stable (no jumps)

**Stability checks:**
- `max_verify_position_jump`: 0.5 m between samples
- `max_verify_yaw_jump`: 20° between samples

**Failure modes:**
- Quality drops below threshold during verification → reject, retry
- Position/yaw jump → reject (probably wrong match)
- Timeout → FAILED

### 6.2 Safe Handoff

Once verification passes:
- State → RECOVERED
- Publish recovery success message
- MODEL-1 receives notification
- MODEL-1 progressively re-enables sources (not immediate full trust)
- System returns to NORMAL only after MODEL-1 confirms stable

**Progressive re-entry:**
```
RECOVERED
  ↓ (MODEL-1 checks post-recovery health)
sources marked GOOD again
  ↓
NORMAL
```

## 7. Failure Paths

### 7.1 Timeout

- `max_attempt_duration` exceeded → attempt fails
- Increment attempt counter
- If `attempts < max_attempts` (default: 2) → retry with new scan
- If exhausted → FAILED

### 7.2 Repeated Rejection

- All segments scanned, no candidate accepted
- Or candidate rejected during verification
- Retry logic same as timeout

### 7.3 Safe Fail

If cannot recover:
- State → FAILED or MANUAL_REQUIRED
- Publish diagnostic (which segments tried, why candidates rejected)
- Stop commanding motion
- Wait for human intervention or external reset

**No silent continue:**
```
FALSE_RECOVERY_PREVENTED
```

System does not silently return to NORMAL with unverified pose.

## 8. Synthetic Validation

### 8.1 S05: Trigger Confirmation

- Tests NORMAL → SUSPECTED → TRIGGERED transition
- STOPPING held (no motion, plant disabled)
- **Status:** PASS

### 8.2 S06: Closed-Loop Recovery Motion

- Full active scan + candidate + verification + handoff
- **Status:** PASS

### 8.3 S07: Real Candidate from Real Seed

- Tests candidate source integration
- **Status:** PASS

### 8.4 S08: Multiframe Verification and Handoff

- Tests verification stability checks
- Safe return to NORMAL
- **Status:** PASS

### 8.5 Missing Negative Test Scenarios

❌ **Rejected bad candidate:**
- Inject low-quality candidate (score < 0.45)
- Verify rejection, retry

❌ **Timeout:**
- No candidate arrives within max_attempt_duration
- Verify FAILED state, no motion continues

❌ **Verification instability:**
- Candidate jumps during verification
- Verify rejection, no false recovery

❌ **GT leak check:**
- Verify GT never used in online decision
- Only in post-run evaluation

## 9. Unit Tests

```
src/ylhb_base/test/test_active_relocalization_core.py
```

**Coverage:**
- State transitions
- Health assessment
- Trigger logic (suspect/trigger samples)
- Segment execution logic
- Candidate acceptance/rejection
- Verification stability
- Timeout handling

**Run:**
```bash
colcon test --packages-select ylhb_base --pytest-args "-k active_reloc"
```

## 10. Known Surrogates

### 10.1 Scan-to-Map Matcher

```
SURROGATE_COMPONENT: scan-to-map matcher
```

Current implementation uses existing AMCL or scan_tools matcher. MODEL-3 does not
own scan matching algorithm. Final system may use dedicated matcher, but interface
remains same (candidate pose + quality metrics).

### 10.2 Action Policy

```
SURROGATE_NOW: rotation-only scan segments
FINAL_ALGORITHM_LATER: RL-based or search-based action selection
```

First version uses fixed rotation pattern. Future upgrade may learn optimal
actions based on environment and failure mode.

## 11. Integration with MODEL-1 and MODEL-2

### 11.1 Input from MODEL-1

- Localization health (amcl_covariance, source quality)
- Trigger reasons
- Recovery confirmation request

### 11.2 Input from MODEL-2

- LiDAR geometry score (via MODEL-1 health aggregation)
- Stable features for scan-to-map matching

### 11.3 Output to MODEL-1

- Recovery state (TRIGGERED, ACTIVE_SCAN, VERIFYING, RECOVERED)
- Verification result
- Safe handoff signal

## 12. Real Robot Follow-up

Before real hardware:

1. ❌ **Tune thresholds** for real sensor noise
2. ❌ **Test with real motion** (not synthetic plant)
3. ❌ **Validate safety constraints** (collision avoidance, workspace limits)
4. ❌ **Integrate real scan matcher** or confirm AMCL suffices
5. ❌ **Test timeout recovery** in real environment
6. ❌ **Verify arbitration** with navigation stack

## 13. Summary

**MODEL-3 status:**
- ✅ Core state machine implemented
- ✅ Trigger logic validated (S05)
- ✅ Active scan motion working (S06)
- ✅ Candidate evaluation and verification validated (S07-S08)
- ✅ Safe handoff confirmed
- ❌ Missing negative test scenarios (bad candidate, timeout, GT leak)
- ❌ Rotation-only action marked as SURROGATE_NOW
- ❌ Scan matcher marked as SURROGATE_COMPONENT

**Next steps:**
- Create negative test scenarios
- Document failure paths explicitly
- Verify GT never leaks into online decisions
