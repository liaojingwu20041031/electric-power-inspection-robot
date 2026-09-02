#!/usr/bin/env python3
"""Test: UWB single-source failure should NOT trigger MODEL-3.

Verifies that UWB rejection does not cause whole-system failure when
other localization sources (GNSS, LiDAR, odom) remain healthy.
"""

import sys
sys.path.insert(0, 'C:/Users/peng/dg202611_stage')
sys.path.insert(0, 'C:/Users/peng/dg202611_stage/relay_repo/src/ylhb_base/scripts')

from multisource_fusion_core import (
    MultisourceFusionCore,
    FusionConfig,
    FusionInput,
    Pose2D,
    UwbPositionMeasurement,
    GnssPosition,
    UWB_AIDED,
    GNSS_AIDED,
    DEAD_RECKONING,
)
from navigation_health_core import (
    NavigationHealthAggregator,
    NavigationHealthInput,
    GOOD,
    DEGRADED,
    REJECTED,
    STALE,
    NOMINAL,
    LOCALIZATION_SUSPECT,
)


def test_uwb_single_source_failure_does_not_trigger_model3():
    """Verify UWB rejection alone does not trigger MODEL-3."""
    print("\n" + "="*70)
    print("TEST: UWB Single Source Failure - MODEL-3 NOT Triggered")
    print("="*70)

    fusion = MultisourceFusionCore(FusionConfig())
    health = NavigationHealthAggregator()

    # Scenario: GNSS healthy, UWB rejected, odom healthy
    # Expected: System continues operating, no MODEL-3 trigger

    # Step 1: Initialize with odometry
    odom = Pose2D(0.0, 0.0, 0.0, 0.0)
    fusion.process(FusionInput(now=0.0, local_odom=odom))

    # Step 2: UWB provides initial anchor (healthy)
    uwb_good = UwbPositionMeasurement(
        x=10.0, y=5.0, timestamp=1.0,
        state="GOOD", fresh=True, accepted=True,
        residual_rms=0.2, geometry_score=0.5,
        unique_anchor_count=3, confidence=0.8,
    )
    output1 = fusion.process(FusionInput(now=1.0, local_odom=odom, uwb=uwb_good))
    assert output1.fusion_mode == UWB_AIDED, "Should be UWB_AIDED initially"
    print(f"   Step 1: UWB healthy, mode={output1.fusion_mode}")

    # Step 3: UWB fails (rejected), but GNSS is available and healthy
    gnss_good = GnssPosition(
        latitude=30.0, longitude=120.0, altitude=100.0, timestamp=2.0,
        state="GOOD", fresh=True, accepted=True,
        hdop=1.0, satellites=12, differential_age=0.5,
    )
    uwb_bad = UwbPositionMeasurement(
        x=0.0, y=0.0, timestamp=2.0,
        state="REJECTED", fresh=True, accepted=False,
        residual_rms=None, geometry_score=None,
        unique_anchor_count=0, confidence=0.0,
    )

    # Need to set GNSS reference and alignment for GNSS to work
    from multisource_fusion_core import GeodeticReference, Alignment2D
    fusion.config = FusionConfig(
        gnss_reference=GeodeticReference(30.0, 120.0, 100.0),
        alignment=Alignment2D(0.0, 0.0, 0.0),
    )
    fusion._alignment = fusion.config.alignment

    output2 = fusion.process(FusionInput(now=2.0, local_odom=odom, gnss=gnss_good, uwb=uwb_bad))

    # Verify: UWB rejected, but system continues with GNSS
    assert output2.accepted_source != "UWB", "UWB should not be accepted when rejected"
    assert output2.fusion_mode != "FAILED", "System should not fail"
    assert output2.fusion_mode != "LOCALIZATION_SUSPECT", "Should not be suspect with healthy GNSS"
    print(f"   Step 2: UWB rejected, GNSS healthy, mode={output2.fusion_mode}")

    # Check navigation health
    health_input = NavigationHealthInput(
        now=2.0,
        gnss_state="GOOD",
        gnss_fresh=True,
        lidar_state="GOOD",
        lidar_fresh=True,
        uwb_state="REJECTED",
        uwb_fresh=True,
        odom_fresh=True,
    )
    health_output = health.evaluate(health_input)

    # Verify: Overall health degraded but NOT localization suspect
    assert health_output.overall_state != "FAILED", "Should not be FAILED"
    assert health_output.overall_state != "LOCALIZATION_SUSPECT", "Should not be LOCALIZATION_SUSPECT with healthy GNSS+LiDAR"
    assert health_output.uwb_state == REJECTED, f"UWB should be REJECTED, got {health_output.uwb_state}"
    print(f"   Step 3: Navigation health overall_state={health_output.overall_state}")
    print(f"   Step 4: GNSS={health_output.gnss_state}, LiDAR={health_output.lidar_state}, UWB={health_output.uwb_state}")

    # Verify UWB in reasons but system continues
    assert any("UWB" in reason for reason in health_output.reasons), "UWB rejection should be in reasons"
    print(f"   Step 5: Reasons: {health_output.reasons}")

    print(f"\n✅ PASS: UWB single-source failure does NOT trigger MODEL-3")
    print(f"   - UWB REJECTED")
    print(f"   - Other sources (GNSS, LiDAR, odom) healthy")
    print(f"   - System continues operating: {health_output.overall_state}")
    print(f"   - No MODEL-3 triggered")

    return True


def test_uwb_not_required_signal():
    """Verify UWB is not a hard-required signal by default."""
    print("\n" + "="*70)
    print("TEST: UWB Not Required Signal")
    print("="*70)

    health = NavigationHealthAggregator()

    # UWB stale, but not in required_signals
    health_input = NavigationHealthInput(
        now=1.0,
        gnss_state="GOOD",
        gnss_fresh=True,
        lidar_state="GOOD",
        lidar_fresh=True,
        uwb_state="STALE",
        uwb_fresh=False,
        odom_fresh=True,
        required_signals=(),  # Empty - UWB not required
    )
    health_output = health.evaluate(health_input)

    # Should be DEGRADED (due to UWB_STALE reason) but not LOCALIZATION_SUSPECT
    assert health_output.overall_state != "LOCALIZATION_SUSPECT", "Should not be LOCALIZATION_SUSPECT when UWB not required"
    assert health_output.uwb_state == STALE, f"UWB should be STALE, got {health_output.uwb_state}"

    print(f"   Overall state: {health_output.overall_state}")
    print(f"   UWB not required: system continues normally")
    print(f"✅ PASS: UWB is not a hard-required signal")

    return True


def test_gnss_lidar_healthy_uwb_failed():
    """Multiple healthy sources mask UWB failure."""
    print("\n" + "="*70)
    print("TEST: GNSS + LiDAR Healthy, UWB Failed")
    print("="*70)

    health = NavigationHealthAggregator()

    health_input = NavigationHealthInput(
        now=1.0,
        gnss_state="GOOD",
        gnss_fresh=True,
        lidar_state="GOOD",
        lidar_fresh=True,
        amcl_state="GOOD",
        amcl_fresh=True,
        amcl_covariance=0.1,
        uwb_state="REJECTED",
        uwb_fresh=True,
        odom_fresh=True,
        scan_fresh=True,
    )
    health_output = health.evaluate(health_input)

    # Should be DEGRADED (UWB rejection noted) but system functional
    assert health_output.overall_state in {NOMINAL, DEGRADED}, f"Expected NOMINAL or DEGRADED, got {health_output.overall_state}"
    assert "UWB_REJECTED" in health_output.reasons, "UWB_REJECTED should be in reasons"

    print(f"   Overall state: {health_output.overall_state}")
    print(f"   GNSS: {health_output.gnss_state}")
    print(f"   LiDAR: {health_output.lidar_state}")
    print(f"   UWB: {health_output.uwb_state}")
    print(f"   Reasons: {health_output.reasons}")
    print(f"✅ PASS: System healthy despite UWB failure")

    return True


if __name__ == "__main__":
    print("\n" + "="*70)
    print("UWB SINGLE-SOURCE FAILURE - MODEL-3 INTEGRATION TEST")
    print("="*70)

    tests = [
        test_uwb_single_source_failure_does_not_trigger_model3,
        test_uwb_not_required_signal,
        test_gnss_lidar_healthy_uwb_failed,
    ]

    results = {}
    for test_func in tests:
        try:
            passed = test_func()
            results[test_func.__name__] = "PASS" if passed else "FAIL"
        except AssertionError as e:
            print(f"❌ FAIL: {test_func.__name__}")
            print(f"   Error: {e}")
            results[test_func.__name__] = "FAIL"
        except Exception as e:
            print(f"❌ ERROR: {test_func.__name__}")
            print(f"   Exception: {e}")
            results[test_func.__name__] = "ERROR"

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

    all_pass = all(r == "PASS" for r in results.values())
    if all_pass:
        print("\n" + "="*70)
        print("✅ CONFIRMED: UWB single-source failure does NOT trigger MODEL-3")
        print("="*70)

    sys.exit(0 if all_pass else 1)
