#!/usr/bin/env python3
"""Synthetic validation tests for UWB MODEL-1 integration.

Tests:
- UWB_01_STABLE: Normal UWB operation
- UWB_02_RANGE_JUMP: Range jump rejection
- UWB_03_STALE: Stale data rejection
- UWB_04_RECOVERY_HYSTERESIS: Recovery with hysteresis
- UWB_05_LOGICAL6_PHYSICAL3_DEDUP: Proper deduplication
"""

import sys
import math
sys.path.insert(0, 'C:/Users/peng/dg202611_stage')
sys.path.insert(0, 'C:/Users/peng/dg202611_stage/relay_repo/src/ylhb_base/scripts')

from uwb_data_model import UwbRangeObservation, SYNTHETIC_ANCHOR_COORDINATES
from uwb_2d_estimator import estimate_2d_position
from uwb_quality_gate import UwbQualityGate, UwbQualityConfig
from multisource_fusion_core import (
    MultisourceFusionCore,
    FusionConfig,
    FusionInput,
    Pose2D,
    UwbPositionMeasurement,
    UWB_AIDED,
)
from navigation_health_core import NavigationHealthAggregator, NavigationHealthInput, GOOD, DEGRADED, REJECTED


def create_stable_observations(timestamp, physical_ranges):
    """Create 6 logical observations from 3 physical anchors."""
    observations = []
    logical_ids = ["A0", "A1", "A2", "A3", "A4", "A5"]
    physical_ids = [0, 1, 2, 0, 1, 2]  # Mirror mapping

    for i, (logical_id, physical_id) in enumerate(zip(logical_ids, physical_ids)):
        observations.append(UwbRangeObservation(
            timestamp=timestamp,
            logical_anchor_id=logical_id,
            physical_source_id=physical_id,
            correlation_group=physical_id,
            range_m=physical_ranges[physical_id],
            valid=True,
        ))
    return observations


def test_uwb_01_stable():
    """UWB_01_STABLE: Normal operation with stable data."""
    print("\n" + "="*70)
    print("TEST: UWB_01_STABLE - Normal UWB Operation")
    print("="*70)

    config = UwbQualityConfig()
    gate = UwbQualityGate(config, SYNTHETIC_ANCHOR_COORDINATES)
    fusion_config = FusionConfig()
    fusion = MultisourceFusionCore(fusion_config)

    # Initialize with odometry
    odom = Pose2D(0.0, 0.0, 0.0, 0.0)
    fusion.process(FusionInput(now=0.0, local_odom=odom))

    # Stable UWB observations
    test_ranges = [5.0, 7.0, 6.0]
    observations = create_stable_observations(1.0, test_ranges)

    position_est, decision = gate.update(observations, 1.0)

    assert position_est is not None, "Position estimation failed"
    assert decision == "ACCEPT", f"Expected ACCEPT, got {decision}"
    assert gate.state.state == "GOOD", f"Expected GOOD state, got {gate.state.state}"
    assert position_est.unique_anchor_count == 3, f"Expected 3 unique anchors, got {position_est.unique_anchor_count}"

    # Feed to MODEL-1
    uwb_meas = UwbPositionMeasurement(
        x=position_est.x,
        y=position_est.y,
        timestamp=position_est.timestamp,
        state="GOOD",
        fresh=True,
        accepted=True,
        residual_rms=position_est.residual_rms,
        geometry_score=position_est.geometry_metric,
        unique_anchor_count=position_est.unique_anchor_count,
        confidence=position_est.confidence,
    )

    output = fusion.process(FusionInput(now=1.0, local_odom=odom, uwb=uwb_meas))

    assert output.fusion_mode == UWB_AIDED, f"Expected UWB_AIDED, got {output.fusion_mode}"
    assert output.accepted_source == "UWB", f"Expected UWB source, got {output.accepted_source}"

    print(f"✅ PASS: UWB_01_STABLE")
    print(f"   Position: ({position_est.x:.2f}, {position_est.y:.2f})")
    print(f"   Decision: {decision}, State: {gate.state.state}")
    print(f"   Fusion mode: {output.fusion_mode}")
    print(f"   Unique anchors: {position_est.unique_anchor_count}")

    return True


def test_uwb_02_range_jump():
    """UWB_02_RANGE_JUMP: Range jump causes rejection."""
    print("\n" + "="*70)
    print("TEST: UWB_02_RANGE_JUMP - Range Jump Rejection")
    print("="*70)

    config = UwbQualityConfig()
    gate = UwbQualityGate(config, SYNTHETIC_ANCHOR_COORDINATES)
    fusion = MultisourceFusionCore(FusionConfig())

    # Initialize
    odom = Pose2D(0.0, 0.0, 0.0, 0.0)
    fusion.process(FusionInput(now=0.0, local_odom=odom))

    # First: stable
    observations = create_stable_observations(1.0, [5.0, 7.0, 6.0])
    position_est, decision = gate.update(observations, 1.0)
    assert decision == "ACCEPT", "Initial observation should be accepted"

    # Feed to fusion
    uwb_meas = UwbPositionMeasurement(
        x=position_est.x, y=position_est.y, timestamp=1.0,
        state="GOOD", fresh=True, accepted=True,
        residual_rms=position_est.residual_rms,
        geometry_score=position_est.geometry_metric,
        unique_anchor_count=3,
        confidence=position_est.confidence,
    )
    output1 = fusion.process(FusionInput(now=1.0, local_odom=odom, uwb=uwb_meas))
    assert output1.fusion_mode == UWB_AIDED, "Should be UWB_AIDED initially"

    # Second: large jump in range
    observations_jump = create_stable_observations(1.1, [10.0, 7.0, 6.0])  # 5.0m jump on A0
    position_est2, decision2 = gate.update(observations_jump, 1.1)

    assert decision2 == "REJECT", f"Expected REJECT on jump, got {decision2}"
    assert gate.state.state == "REJECTED", f"Expected REJECTED state, got {gate.state.state}"
    assert "RANGE_JUMP" in gate.state.rejection_reason, f"Expected RANGE_JUMP reason, got {gate.state.rejection_reason}"

    # UWB rejected should not update fusion
    uwb_meas_bad = UwbPositionMeasurement(
        x=0.0, y=0.0, timestamp=1.1,
        state="REJECTED", fresh=True, accepted=False,
        residual_rms=None, geometry_score=None,
        unique_anchor_count=0, confidence=0.0,
    )
    output2 = fusion.process(FusionInput(now=1.1, local_odom=odom, uwb=uwb_meas_bad))
    assert output2.accepted_source != "UWB", "Bad UWB should not be accepted"

    print(f"✅ PASS: UWB_02_RANGE_JUMP")
    print(f"   Initial decision: ACCEPT")
    print(f"   After jump decision: {decision2}, State: {gate.state.state}")
    print(f"   Rejection reason: {gate.state.rejection_reason}")
    print(f"   Fusion did not accept bad UWB: {output2.accepted_source}")

    return True


def test_uwb_03_stale():
    """UWB_03_STALE: Stale data is rejected."""
    print("\n" + "="*70)
    print("TEST: UWB_03_STALE - Stale Data Rejection")
    print("="*70)

    config = UwbQualityConfig(freshness_timeout=0.5)
    gate = UwbQualityGate(config, SYNTHETIC_ANCHOR_COORDINATES)

    observations = create_stable_observations(1.0, [5.0, 7.0, 6.0])

    # Update with old timestamp (stale)
    position_est, decision = gate.update(observations, 2.0)  # 1.0 second old

    assert decision == "REJECT", f"Expected REJECT for stale, got {decision}"
    assert gate.state.state == "REJECTED", f"Expected REJECTED state"
    assert "STALE" in gate.state.rejection_reason, f"Expected STALE in reason"

    print(f"✅ PASS: UWB_03_STALE")
    print(f"   Decision: {decision}, State: {gate.state.state}")
    print(f"   Rejection reason: {gate.state.rejection_reason}")

    return True


def test_uwb_04_recovery_hysteresis():
    """UWB_04_RECOVERY_HYSTERESIS: Recovery requires N consecutive good frames."""
    print("\n" + "="*70)
    print("TEST: UWB_04_RECOVERY_HYSTERESIS - Recovery Hysteresis")
    print("="*70)

    config = UwbQualityConfig(recovery_good_frames=3)
    gate = UwbQualityGate(config, SYNTHETIC_ANCHOR_COORDINATES)

    # 1. Normal observation -> GOOD
    observations = create_stable_observations(1.0, [5.0, 7.0, 6.0])
    position_est, decision = gate.update(observations, 1.0)
    assert decision == "ACCEPT" and gate.state.state == "GOOD", "Should start GOOD"
    print(f"   Step 1: {decision}, State: {gate.state.state}")

    # 2. Range jump -> REJECTED
    observations_bad = create_stable_observations(1.1, [10.0, 7.0, 6.0])
    position_est, decision = gate.update(observations_bad, 1.1)
    assert decision == "REJECT" and gate.state.state == "REJECTED", "Should become REJECTED"
    print(f"   Step 2 (jump): {decision}, State: {gate.state.state}")

    # 3. Good data returns -> RECOVERING (frame 1)
    observations_good = create_stable_observations(1.2, [5.0, 7.0, 6.0])
    position_est, decision = gate.update(observations_good, 1.2)
    assert decision == "HOLD_FOR_RECOVERY" and gate.state.state == "RECOVERING", "Should enter RECOVERING"
    assert gate.state.consecutive_good_frames == 1, f"Expected 1 good frame, got {gate.state.consecutive_good_frames}"
    print(f"   Step 3 (recovery 1/3): {decision}, State: {gate.state.state}, Good frames: {gate.state.consecutive_good_frames}")

    # 4. Second good frame -> RECOVERING (frame 2)
    position_est, decision = gate.update(observations_good, 1.3)
    assert decision == "HOLD_FOR_RECOVERY" and gate.state.state == "RECOVERING", "Should stay RECOVERING"
    assert gate.state.consecutive_good_frames == 2, f"Expected 2 good frames, got {gate.state.consecutive_good_frames}"
    print(f"   Step 4 (recovery 2/3): {decision}, State: {gate.state.state}, Good frames: {gate.state.consecutive_good_frames}")

    # 5. Third good frame -> GOOD (recovered)
    position_est, decision = gate.update(observations_good, 1.4)
    assert decision == "ACCEPT" and gate.state.state == "GOOD", "Should recover to GOOD"
    assert gate.state.consecutive_good_frames >= 3, f"Expected >=3 good frames, got {gate.state.consecutive_good_frames}"
    print(f"   Step 5 (recovered 3/3): {decision}, State: {gate.state.state}, Good frames: {gate.state.consecutive_good_frames}")

    print(f"✅ PASS: UWB_04_RECOVERY_HYSTERESIS")
    print(f"   Recovery required {config.recovery_good_frames} consecutive good frames")

    return True


def test_uwb_05_logical6_physical3_dedup():
    """UWB_05_LOGICAL6_PHYSICAL3_DEDUP: Proper deduplication of mirror channels."""
    print("\n" + "="*70)
    print("TEST: UWB_05_LOGICAL6_PHYSICAL3_DEDUP - Deduplication")
    print("="*70)

    # Create 6 logical observations (A0-A5) from 3 physical anchors
    observations = create_stable_observations(1.0, [5.0, 7.0, 6.0])

    # Verify we have 6 logical observations
    logical_count = len(observations)
    assert logical_count == 6, f"Expected 6 logical observations, got {logical_count}"

    # Estimate position
    position_est = estimate_2d_position(observations, SYNTHETIC_ANCHOR_COORDINATES, min_anchors=3)

    assert position_est is not None, "Position estimation failed"
    assert position_est.unique_anchor_count == 3, f"Expected 3 unique anchors, got {position_est.unique_anchor_count}"

    # Verify deduplication: unique physical anchors
    unique_physical = set(obs.physical_source_id for obs in observations if obs.valid)
    assert len(unique_physical) == 3, f"Expected 3 unique physical IDs, got {len(unique_physical)}"

    # Compare with baseline (3 observations only)
    observations_baseline = observations[:3]  # A0, A1, A2 only
    position_baseline = estimate_2d_position(observations_baseline, SYNTHETIC_ANCHOR_COORDINATES, min_anchors=3)

    # Results should be identical (mirrors don't add new information)
    assert position_baseline is not None, "Baseline estimation failed"
    position_diff = math.sqrt((position_est.x - position_baseline.x)**2 + (position_est.y - position_baseline.y)**2)
    assert position_diff < 0.01, f"Positions should match, diff={position_diff:.4f}m"

    # Confidence should not be inflated by mirrors
    # (Both should have same anchor count contributing to confidence)
    assert position_est.unique_anchor_count == position_baseline.unique_anchor_count, "Anchor counts should match"

    print(f"✅ PASS: UWB_05_LOGICAL6_PHYSICAL3_DEDUP")
    print(f"   Logical anchor count: {logical_count}")
    print(f"   Unique physical anchor count: {position_est.unique_anchor_count}")
    print(f"   Position (6 logical): ({position_est.x:.2f}, {position_est.y:.2f})")
    print(f"   Position (3 baseline): ({position_baseline.x:.2f}, {position_baseline.y:.2f})")
    print(f"   Position difference: {position_diff:.4f}m")
    print(f"   Geometry metric: {position_est.geometry_metric:.3f}")
    print(f"   ✓ Mirror channels properly deduplicated")

    return True


def run_all_tests():
    """Run all UWB integration tests."""
    print("\n" + "="*70)
    print("UWB MODEL-1 INTEGRATION - SYNTHETIC VALIDATION")
    print("="*70)

    tests = [
        ("UWB_01_STABLE", test_uwb_01_stable),
        ("UWB_02_RANGE_JUMP", test_uwb_02_range_jump),
        ("UWB_03_STALE", test_uwb_03_stale),
        ("UWB_04_RECOVERY_HYSTERESIS", test_uwb_04_recovery_hysteresis),
        ("UWB_05_LOGICAL6_PHYSICAL3_DEDUP", test_uwb_05_logical6_physical3_dedup),
    ]

    results = {}
    for name, test_func in tests:
        try:
            passed = test_func()
            results[name] = "PASS" if passed else "FAIL"
        except AssertionError as e:
            print(f"❌ FAIL: {name}")
            print(f"   Error: {e}")
            results[name] = "FAIL"
        except Exception as e:
            print(f"❌ ERROR: {name}")
            print(f"   Exception: {e}")
            results[name] = "ERROR"

    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    for name, result in results.items():
        symbol = "✅" if result == "PASS" else "❌"
        print(f"{symbol} {name}: {result}")

    passed = sum(1 for r in results.values() if r == "PASS")
    total = len(results)
    print(f"\nTotal: {passed}/{total} PASS")

    return all(r == "PASS" for r in results.values())


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
