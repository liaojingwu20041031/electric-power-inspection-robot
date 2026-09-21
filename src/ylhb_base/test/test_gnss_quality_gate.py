import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gnss_quality_gate import (  # noqa: E402
    ACCEPT,
    ACCEPT_DEGRADED,
    DEGRADED,
    GnssGateConfig,
    GnssObservation,
    GnssQualityGate,
    GOOD,
    HOLD_FOR_RECOVERY,
    REJECT,
    REJECTED,
    RECOVERING,
    parse_status_values,
)


def observation(
    timestamp: float = 0.0,
    latitude: float | None = 30.0,
    longitude: float | None = 120.0,
    quality: int | None = 4,
    satellites: int | None = 12,
    hdop: float | None = 0.8,
    differential_age: float | None = 1.0,
    fix_available: bool = True,
    stale: bool = False,
    status_age: float | None = 0.1,
    status_reasons: tuple[str, ...] = (),
) -> GnssObservation:
    return GnssObservation(
        timestamp, latitude, longitude, 10.0, fix_available, quality,
        satellites, hdop, differential_age, stale, status_age, status_reasons,
    )


class GnssQualityGateTests(unittest.TestCase):
    def test_healthy_observation_is_good(self) -> None:
        result = GnssQualityGate().evaluate(observation())
        self.assertEqual(result.current_state, GOOD)
        self.assertEqual(result.decision, ACCEPT)
        self.assertTrue(result.accepted)

    def test_low_satellites_and_high_hdop_are_degraded_at_configured_boundary(self) -> None:
        gate = GnssQualityGate()
        result = gate.evaluate(observation(satellites=7))
        self.assertEqual(result.current_state, DEGRADED)
        self.assertEqual(result.decision, ACCEPT_DEGRADED)
        self.assertIn("LOW_SATELLITES", result.reasons)

        result = gate.evaluate(observation(timestamp=1.0, satellites=12, hdop=2.0))
        self.assertEqual(result.current_state, DEGRADED)
        self.assertIn("HIGH_HDOP", result.reasons)

    def test_very_low_quality_or_no_fix_is_rejected(self) -> None:
        gate = GnssQualityGate()
        result = gate.evaluate(observation(quality=0, fix_available=False))
        self.assertEqual(result.current_state, REJECTED)
        self.assertEqual(result.decision, REJECT)
        self.assertIn("NO_FIX", result.reasons)

    def test_stale_and_old_status_are_rejected(self) -> None:
        gate = GnssQualityGate()
        self.assertEqual(gate.evaluate(observation(stale=True)).current_state, REJECTED)
        result = GnssQualityGate().evaluate(observation(status_age=3.1))
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("STALE_STATUS", result.reasons)
        result = GnssQualityGate().evaluate(observation(timestamp=0.0), now=4.0)
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("STALE", result.reasons)

    def test_normal_continuous_motion_is_not_a_jump(self) -> None:
        gate = GnssQualityGate()
        gate.evaluate(observation(timestamp=0.0))
        result = gate.evaluate(observation(timestamp=1.0, latitude=30.000005))
        self.assertEqual(result.current_state, GOOD)
        self.assertNotIn("POSITION_JUMP", result.reasons)
        self.assertLess(result.implied_speed, 3.0)

    def test_single_large_jump_is_rejected(self) -> None:
        gate = GnssQualityGate()
        gate.evaluate(observation(timestamp=0.0))
        result = gate.evaluate(observation(timestamp=1.0, latitude=30.001))
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("POSITION_JUMP", result.reasons)
        self.assertIn("IMPLIED_SPEED_HIGH", result.reasons)

    def test_rejected_state_recovers_only_after_n_healthy_samples(self) -> None:
        config = GnssGateConfig(recovery_good_samples=3)
        gate = GnssQualityGate(config)
        gate.evaluate(observation(timestamp=0.0))
        self.assertEqual(gate.evaluate(observation(timestamp=1.0, latitude=30.001)).current_state, REJECTED)
        first = gate.evaluate(observation(timestamp=2.0))
        second = gate.evaluate(observation(timestamp=3.0))
        third = gate.evaluate(observation(timestamp=4.0))
        self.assertEqual(first.current_state, RECOVERING)
        self.assertEqual(first.decision, HOLD_FOR_RECOVERY)
        self.assertIn("RECOVERY_PENDING", first.reasons)
        self.assertEqual(second.current_state, RECOVERING)
        self.assertEqual(third.current_state, GOOD)
        self.assertEqual(third.decision, ACCEPT)

    def test_recovery_is_reset_by_another_bad_observation(self) -> None:
        gate = GnssQualityGate(GnssGateConfig(recovery_good_samples=2))
        gate.evaluate(observation(timestamp=0.0))
        gate.evaluate(observation(timestamp=1.0, latitude=30.001))
        self.assertEqual(gate.evaluate(observation(timestamp=2.0)).current_state, RECOVERING)
        bad = gate.evaluate(observation(timestamp=3.0, fix_available=False, quality=0))
        self.assertEqual(bad.current_state, REJECTED)
        self.assertEqual(bad.recovery_count, 0)

    def test_differential_age_and_invalid_coordinates_have_reasons(self) -> None:
        result = GnssQualityGate().evaluate(observation(differential_age=3.1))
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("DIFFERENTIAL_AGE_HIGH", result.reasons)
        result = GnssQualityGate().evaluate(observation(latitude=91.0))
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("INVALID_COORDINATE", result.reasons)

    def test_non_positive_dt_is_explicitly_rejected(self) -> None:
        gate = GnssQualityGate()
        gate.evaluate(observation(timestamp=1.0))
        result = gate.evaluate(observation(timestamp=1.0))
        self.assertEqual(result.current_state, REJECTED)
        self.assertIn("DT_NON_POSITIVE", result.reasons)

    def test_status_parser_handles_missing_empty_and_invalid_fields(self) -> None:
        parsed = parse_status_values({"quality": "4", "quality_text": "RTK fixed", "satellites": "12", "hdop": "", "differential_age": ""})
        self.assertEqual(parsed.quality, 4)
        self.assertIsNone(parsed.hdop)
        self.assertIsNone(parsed.differential_age)
        self.assertIn("HDOP_MISSING", parsed.reasons)
        parsed = parse_status_values({"quality": "oops", "satellites": "many", "hdop": "nan", "quality_text": "bad"})
        self.assertIn("QUALITY_INVALID", parsed.reasons)
        self.assertIn("SATELLITES_INVALID", parsed.reasons)
        self.assertIn("HDOP_INVALID", parsed.reasons)

    def test_status_parser_recognises_stale_diagnostic_level(self) -> None:
        parsed = parse_status_values({"quality": "4", "quality_text": "RTK fixed", "satellites": "12", "hdop": "0.8"}, 3)
        self.assertTrue(parsed.stale)
        self.assertIn("STALE", parsed.reasons)

    def test_thresholds_are_not_invented_by_the_test(self) -> None:
        config = GnssGateConfig(min_satellites_good=8, min_satellites_degraded=4, max_hdop_good=1.5, max_hdop_degraded=3.0)
        gate = GnssQualityGate(config)
        result = gate.evaluate(observation(satellites=8, hdop=1.5, differential_age=3.0))
        self.assertEqual(result.current_state, GOOD)
        self.assertTrue(math.isfinite(result.recovery_count))


if __name__ == "__main__":
    unittest.main()
