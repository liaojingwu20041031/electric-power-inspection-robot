import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_arbiter_core import ArbiterConfig, ArbiterInput as _ArbiterInput, CmdVelArbiter, TwistCommand  # noqa: E402
from navigation_health_core import NavigationHealthAggregator, NavigationHealthInput  # noqa: E402


def health(**changes: object) -> NavigationHealthInput:
    values = dict(
        now=0.0,
        gnss_state="GOOD",
        gnss_fresh=True,
        lidar_state="GOOD",
        lidar_fresh=True,
        amcl_covariance=0.1,
        amcl_fresh=True,
        odom_fresh=True,
        scan_fresh=True,
        imu_fresh=True,
        scan_match_state="GOOD",
        scan_match_fresh=True,
        relocalization_state="NORMAL",
    )
    values.update(changes)
    return NavigationHealthInput(**values)


NAV_COMMAND = TwistCommand(0.3, 0.0)
RECOVERY_COMMAND = TwistCommand(0.0, 0.15)


def ArbiterInput(*args: object, **kwargs: object) -> _ArbiterInput:
    """Keep legacy test fixtures explicit about a fresh health status."""
    if "navigation_status_received_at" not in kwargs:
        kwargs["navigation_status_received_at"] = args[0] if args else kwargs.get("now")
    return _ArbiterInput(*args, **kwargs)


class NavigationIntegrationCoreTests(unittest.TestCase):
    def test_all_healthy_is_nominal(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health()).overall_state, "NOMINAL")

    def test_gnss_degraded_is_degraded_but_not_localization_lost(self) -> None:
        output = NavigationHealthAggregator().evaluate(health(gnss_state="DEGRADED"))
        self.assertEqual(output.overall_state, "DEGRADED")
        self.assertEqual(output.amcl_state, "GOOD")

    def test_gnss_rejected_alone_is_not_amcl_lost(self) -> None:
        output = NavigationHealthAggregator().evaluate(health(gnss_state="REJECTED"))
        self.assertEqual(output.overall_state, "DEGRADED")

    def test_lidar_mild_degradation_is_degraded(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health(lidar_state="DEGRADED")).overall_state, "DEGRADED")

    def test_amcl_abnormality_is_localization_suspect(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health(amcl_covariance=1.0)).overall_state, "LOCALIZATION_SUSPECT")

    def test_active_trigger_is_recovering(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health(relocalization_state="TRIGGERED")).overall_state, "RECOVERING")

    def test_recovering_blocks_navigation_and_allows_recovery(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(ArbiterInput(0.0, NAV_COMMAND, 0.0, RECOVERY_COMMAND, 0.0, "RECOVERING"))
        output = arbiter.process(ArbiterInput(0.1, NAV_COMMAND, 0.1, RECOVERY_COMMAND, 0.1, "RECOVERING"))
        self.assertEqual(output.command, RECOVERY_COMMAND)

    def test_recovering_with_stale_recovery_source_is_zero(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(1.0, NAV_COMMAND, 1.0, RECOVERY_COMMAND, 0.0, "RECOVERING")
        )
        self.assertEqual(output.command, TwistCommand())

    def test_recovered_restores_navigation_after_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        arbiter.process(ArbiterInput(0.0, NAV_COMMAND, 0.0))
        arbiter.process(ArbiterInput(0.3, NAV_COMMAND, 0.3, RECOVERY_COMMAND, 0.3, "RECOVERING"))
        arbiter.process(ArbiterInput(0.6, NAV_COMMAND, 0.6, RECOVERY_COMMAND, 0.6, "RECOVERED"))
        output = arbiter.process(ArbiterInput(0.9, NAV_COMMAND, 0.9, RECOVERY_COMMAND, 0.9, "RECOVERED"))
        self.assertEqual(output.command, NAV_COMMAND)

    def test_failed_state_is_zero(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, RECOVERY_COMMAND, 0.0, "FAILED")
        )
        self.assertEqual(output.command, TwistCommand())

    def test_manual_required_state_is_zero(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, RECOVERY_COMMAND, 0.0, "MANUAL_REQUIRED")
        )
        self.assertEqual(output.command, TwistCommand())

    def test_stale_navigation_command_is_zero(self) -> None:
        output = CmdVelArbiter().process(ArbiterInput(1.0, NAV_COMMAND, 0.0, navigation_state="NOMINAL"))
        self.assertEqual(output.command, TwistCommand())

    def test_stale_recovery_command_is_zero(self) -> None:
        output = CmdVelArbiter().process(ArbiterInput(1.0, NAV_COMMAND, 1.0, RECOVERY_COMMAND, 0.0, "RECOVERING"))
        self.assertEqual(output.command, TwistCommand())

    def test_source_switch_has_zero_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        arbiter.process(ArbiterInput(0.0, NAV_COMMAND, 0.0))
        first = arbiter.process(ArbiterInput(0.3, NAV_COMMAND, 0.3, RECOVERY_COMMAND, 0.3, "RECOVERING"))
        self.assertTrue(first.guard_active)
        self.assertEqual(first.command, TwistCommand())

    def test_manual_takeover_blocks_recovery_command(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, RECOVERY_COMMAND, 0.0, "RECOVERING", True)
        )
        self.assertEqual(output.command, TwistCommand())

    def test_optional_gnss_missing_still_allows_nominal(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health(gnss_state=None, gnss_fresh=None)).overall_state, "NOMINAL")

    def test_optional_lidar_missing_still_allows_nominal(self) -> None:
        self.assertEqual(NavigationHealthAggregator().evaluate(health(lidar_state=None, lidar_fresh=None)).overall_state, "NOMINAL")

    def test_stale_side_channel_is_not_old_good(self) -> None:
        output = NavigationHealthAggregator().evaluate(health(gnss_state="GOOD", gnss_fresh=False))
        self.assertEqual(output.gnss_state, "STALE")

    def test_repeated_health_ticks_do_not_repeat_transition(self) -> None:
        aggregator = NavigationHealthAggregator()
        aggregator.evaluate(health(now=0.0))
        self.assertFalse(aggregator.evaluate(health(now=0.1)).transition)

    def test_nominal_suspect_recovering_candidate_recovered_sequence(self) -> None:
        aggregator = NavigationHealthAggregator(post_recovery_stable_sec=1.0)
        self.assertEqual(aggregator.evaluate(health(relocalization_state="NORMAL")).overall_state, "NOMINAL")
        self.assertEqual(aggregator.evaluate(health(relocalization_state="SUSPECTED")).overall_state, "LOCALIZATION_SUSPECT")
        self.assertEqual(aggregator.evaluate(health(relocalization_state="ACTIVE_SCAN")).overall_state, "RECOVERING")
        self.assertEqual(aggregator.evaluate(health(relocalization_state="VERIFYING")).overall_state, "RECOVERING")
        self.assertEqual(aggregator.evaluate(health(now=1.0, relocalization_state="RECOVERED")).overall_state, "RECOVERING")
        self.assertEqual(aggregator.evaluate(health(now=2.0, relocalization_state="RECOVERED")).overall_state, "RECOVERED")
        self.assertEqual(aggregator.evaluate(health(now=2.1, relocalization_state="RECOVERED")).overall_state, "NOMINAL")

    def test_progressive_reentry_holds_recovery_then_releases_navigation(self) -> None:
        aggregator = NavigationHealthAggregator(post_recovery_stable_sec=1.0)
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))

        for now in (0.0, 0.5):
            state = aggregator.evaluate(
                health(now=now, relocalization_state="RECOVERED")
            ).overall_state
            output = arbiter.process(
                ArbiterInput(now, NAV_COMMAND, now, TwistCommand(), now, state)
            )
            self.assertNotEqual(output.command, NAV_COMMAND)

        recovered = aggregator.evaluate(
            health(now=1.0, relocalization_state="RECOVERED")
        ).overall_state
        switched = arbiter.process(
            ArbiterInput(1.0, NAV_COMMAND, 1.0, TwistCommand(), 1.0, recovered)
        )
        self.assertEqual(recovered, "RECOVERED")
        self.assertTrue(switched.guard_active)
        self.assertEqual(switched.command, TwistCommand())

        nominal = aggregator.evaluate(
            health(now=1.3, relocalization_state="RECOVERED")
        ).overall_state
        forwarded = arbiter.process(
            ArbiterInput(1.3, NAV_COMMAND, 1.3, TwistCommand(), 1.3, nominal)
        )
        self.assertEqual(nominal, "NOMINAL")
        self.assertEqual(forwarded.command, NAV_COMMAND)

    def test_localization_suspect_is_zero_at_integration_boundary(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, navigation_state="LOCALIZATION_SUSPECT")
        )
        self.assertEqual(output.command, TwistCommand())

    def test_unknown_health_state_is_zero_at_integration_boundary(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, navigation_state="UNKNOWN")
        )
        self.assertEqual(output.command, TwistCommand())

    def test_stale_health_status_cannot_forward_a_new_navigation_command(self) -> None:
        output = CmdVelArbiter().process(
            ArbiterInput(
                now=2.0,
                navigation_command=NAV_COMMAND,
                navigation_received_at=2.0,
                navigation_state="NOMINAL",
                navigation_status_received_at=0.0,
            )
        )
        self.assertEqual(output.command, TwistCommand())

    def test_stale_relocalization_becomes_suspect_before_arbiter(self) -> None:
        output = NavigationHealthAggregator().evaluate(
            health(relocalization_state="RECOVERED", relocalization_fresh=False)
        )
        self.assertEqual(output.overall_state, "LOCALIZATION_SUSPECT")
        arbiter_output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            ArbiterInput(0.0, NAV_COMMAND, 0.0, navigation_state=output.overall_state)
        )
        self.assertEqual(arbiter_output.command, TwistCommand())


if __name__ == "__main__":
    unittest.main()
