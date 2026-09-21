#!/usr/bin/env python3
"""System-level scenarios for DG202611 pre-hardware closure."""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "src" / "ylhb_base" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from multisource_fusion_core import (
    MultisourceFusionCore, FusionInput, FusionConfig,
    Pose2D, GnssPosition, MapPoseMeasurement, UwbPositionMeasurement,
    GeodeticReference, GOOD, DEGRADED_QUALITY
)
from navigation_health_core import NavigationHealthAggregator, NavigationHealthInput

REF = GeodeticReference(30.0, 120.0, 10.0)

def core():
    return MultisourceFusionCore(FusionConfig(gnss_reference=REF))

def odom(x, t=0.0):
    return Pose2D(x, 0.0, 0.0, t)

def gnss(t=0.0, state=GOOD, hdop=1.0):
    return GnssPosition(30.0, 120.0, 10.0, t, state, True, True, hdop, 12.0, 0.2)

def lidar(x, t=0.0, state=GOOD, match=0.85):
    return MapPoseMeasurement(odom(x, t), t, state, True, True, "amcl", None, None, None, match, None, None)

def uwb_ok(t):
    return UwbPositionMeasurement(x=10.0, y=20.0, timestamp=t, state="GOOD", fresh=True, accepted=True,
                                   residual_rms=0.1, geometry_score=0.5, unique_anchor_count=3, confidence=0.8)

def uwb_bad(t):
    return UwbPositionMeasurement(x=10.0, y=20.0, timestamp=t, state="REJECTED", fresh=True, accepted=False,
                                   residual_rms=5.0, geometry_score=0.0, unique_anchor_count=3, confidence=0.0)

def health_input(t, gnss="GOOD", lidar="GOOD", uwb="GOOD"):
    return NavigationHealthInput(t, gnss_state=gnss, gnss_fresh=True, lidar_state=lidar, lidar_fresh=True,
                                  amcl_state=lidar, amcl_fresh=True, odom_fresh=True, scan_fresh=True,
                                  imu_fresh=True, uwb_state=uwb, uwb_fresh=True)

def test_sys_01():
    print("\n=== SYS_01: NOMINAL ===")
    fusion = core()
    for i in range(10):
        t = float(i)
        r = fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t), amcl_pose=lidar(0.0, t)))
        assert r is not None
    print("✓ PASS")
    return True

def test_sys_02():
    print("\n=== SYS_02: GNSS_DEGRADE ===")
    fusion = core()
    for i in range(5):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t)))
    for i in range(5, 10):
        t = float(i)
        r = fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t, DEGRADED_QUALITY, 8.0), amcl_pose=lidar(0.0, t)))
        assert r is not None
    print("✓ PASS")
    return True

def test_sys_03():
    print("\n=== SYS_03: LIDAR_DEGRADE ===")
    fusion = core()
    health = NavigationHealthAggregator()
    for i in range(5):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t), amcl_pose=lidar(0.0, t)))
    cnt = 0
    for i in range(5, 10):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t), amcl_pose=lidar(0.0, t, DEGRADED_QUALITY, 0.3)))
        h = health.evaluate(health_input(t, lidar="DEGRADED"))
        if h.overall_state == "DEGRADED":
            cnt += 1
    assert cnt > 0
    print(f"✓ PASS ({cnt}/5 degraded)")
    return True

def test_sys_04():
    print("\n=== SYS_04: LOCALIZATION_FAILURE_RECOVERY ===")
    fusion = core()
    health = NavigationHealthAggregator()
    for i in range(5):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t)))
        h = health.evaluate(NavigationHealthInput(t, gnss_state="REJECTED", gnss_fresh=False,
                                                    lidar_state="REJECTED", lidar_fresh=False,
                                                    amcl_state="REJECTED", amcl_fresh=False,
                                                    uwb_state="REJECTED", uwb_fresh=False))
    assert h.overall_state in ["LOCALIZATION_SUSPECT", "DEGRADED", "WAITING"]
    print("✓ PASS")
    return True

def test_sys_05():
    print("\n=== SYS_05: UWB_STABLE ===")
    fusion = core()
    for i in range(10):
        t = float(i)
        r = fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t), uwb=uwb_ok(t)))
        assert r is not None
    print("✓ PASS")
    return True

def test_sys_06():
    print("\n=== SYS_06: UWB_FAILURE_OTHER_SOURCES_HEALTHY ===")
    fusion = core()
    health = NavigationHealthAggregator()
    for i in range(10):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t), amcl_pose=lidar(0.0, t), uwb=uwb_bad(t)))
        h = health.evaluate(health_input(t, uwb="REJECTED"))
        assert h.overall_state in ["NOMINAL", "DEGRADED", "WAITING"]
    print("✓ PASS - UWB single failure does NOT trigger MODEL-3")
    return True

def test_sys_07():
    print("\n=== SYS_07: MULTI_SOURCE_DEGRADATION ===")
    fusion = core()
    health = NavigationHealthAggregator()
    for i in range(10):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t, DEGRADED_QUALITY, 8.0),
                                    amcl_pose=lidar(0.0, t, DEGRADED_QUALITY, 0.3)))
        h = health.evaluate(health_input(t, gnss="DEGRADED", lidar="DEGRADED", uwb="REJECTED"))
        if i > 3:
            assert h.overall_state in ["DEGRADED", "LOCALIZATION_SUSPECT", "WAITING"]
    print("✓ PASS")
    return True

def test_sys_08():
    print("\n=== SYS_08: RECOVERY_REENTRY ===")
    fusion = core()
    for i in range(5):
        t = float(i)
        fusion.process(FusionInput(t, local_odom=odom(0.0, t)))
    for i in range(5, 10):
        t = float(i)
        r = fusion.process(FusionInput(t, local_odom=odom(0.0, t), gnss=gnss(t), amcl_pose=lidar(0.0, t)))
        assert r is not None
    print("✓ PASS")
    return True

def main():
    print("===DG202611 SYSTEM SCENARIOS===")
    tests = [test_sys_01, test_sys_02, test_sys_03, test_sys_04,
             test_sys_05, test_sys_06, test_sys_07, test_sys_08]
    passed = sum(1 for t in tests if t())
    failed = len(tests) - passed
    print(f"\n===RESULTS===\nPASSED: {passed}/8\nFAILED: {failed}/8")
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
