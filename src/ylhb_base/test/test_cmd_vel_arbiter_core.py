import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_arbiter_core import (  # noqa: E402
    NAV,
    MANUAL,
    NONE,
    RECOVERY,
    ArbiterConfig,
    ArbiterInput,
    CmdVelArbiter,
    TwistCommand,
)


def nav_cmd() -> TwistCommand:
    return TwistCommand(0.4, 0.1)


def recovery_cmd() -> TwistCommand:
    return TwistCommand(0.0, 0.2)


def manual_cmd() -> TwistCommand:
    return TwistCommand(0.12, -0.1)


def event(
    now: float,
    state: str = "NOMINAL",
    nav: TwistCommand | None = None,
    nav_at: float | None = None,
    recovery: TwistCommand | None = None,
    recovery_at: float | None = None,
    status_at: float | None = None,
    manual: bool = False,
) -> ArbiterInput:
    return ArbiterInput(
        now=now,
        navigation_command=nav if nav is not None else nav_cmd(),
        navigation_received_at=now if nav_at is None and nav is not None else nav_at,
        recovery_command=recovery,
        recovery_received_at=recovery_at,
        navigation_state=state,
        manual_takeover=manual,
        navigation_status_received_at=status_at if status_at is not None else now,
    )


class CmdVelArbiterTests(unittest.TestCase):
    def test_nominal_forwards_navigation_after_initial_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        first = arbiter.process(event(0.0, nav=nav_cmd()))
        second = arbiter.process(event(0.3, nav=nav_cmd()))
        self.assertEqual(first.source, NAV)
        self.assertTrue(first.guard_active)
        self.assertEqual(second.command, nav_cmd())

    def test_recovering_forwards_only_recovery(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(event(0.0, nav=nav_cmd()))
        arbiter.process(event(1.0, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=1.0))
        output = arbiter.process(event(1.1, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=1.1))
        self.assertEqual(output.source, RECOVERY)
        self.assertEqual(output.command, recovery_cmd())

    def test_recovering_does_not_forward_navigation(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            event(0.0, "RECOVERING", nav=nav_cmd())
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "RECOVERY_SOURCE_STALE")

    def test_failed_and_manual_states_are_zero(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        for state in ("FAILED", "MANUAL_REQUIRED"):
            output = arbiter.process(event(0.0, state, nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.0))
            self.assertEqual(output.command, TwistCommand(), state)
        manual = arbiter.process(event(0.1, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.1, manual=True))
        self.assertEqual(manual.command, TwistCommand())
        self.assertEqual(manual.reason, "MANUAL_TAKEOVER")

    def test_stale_navigation_source_is_zero(self) -> None:
        output = CmdVelArbiter().process(event(1.0, nav=nav_cmd(), nav_at=0.0))
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.source, NONE)

    def test_stale_recovery_source_is_zero(self) -> None:
        output = CmdVelArbiter().process(
            event(1.0, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.0)
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "RECOVERY_SOURCE_STALE")

    def test_source_switch_inserts_zero_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.5))
        arbiter.process(event(0.0, nav=nav_cmd()))
        arbiter.process(event(0.6, nav=nav_cmd()))
        switched = arbiter.process(event(0.7, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.7))
        self.assertTrue(switched.guard_active)
        self.assertEqual(switched.command, TwistCommand())
        resumed = arbiter.process(event(1.3, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=1.3))
        self.assertEqual(resumed.source, RECOVERY)
        self.assertEqual(resumed.command, recovery_cmd())

    def test_recovered_switches_back_to_navigation_with_guard(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.2))
        arbiter.process(event(0.0, nav=nav_cmd()))
        arbiter.process(event(0.3, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.3))
        output = arbiter.process(event(0.6, "RECOVERED", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=0.6))
        self.assertTrue(output.guard_active)
        self.assertEqual(output.command, TwistCommand())

    def test_nonfinite_command_is_never_forwarded(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            event(0.0, nav=TwistCommand(math.nan, 0.0))
        )
        self.assertEqual(output.command, TwistCommand())

    def test_manual_takeover_does_not_change_to_recovery_source(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(event(0.0, nav=nav_cmd()))
        output = arbiter.process(event(1.0, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=1.0, manual=True))
        self.assertEqual(output.source, NONE)

    def test_authorized_manual_command_has_priority_over_navigation(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(
            ArbiterInput(
                now=1.0,
                navigation_command=nav_cmd(),
                navigation_received_at=1.0,
                navigation_status_received_at=1.0,
                manual_command=manual_cmd(),
                manual_received_at=1.0,
                manual_authorized=True,
            )
        )
        output = arbiter.process(
            ArbiterInput(
                now=1.1,
                navigation_command=nav_cmd(),
                navigation_received_at=1.1,
                navigation_status_received_at=1.1,
                manual_command=manual_cmd(),
                manual_received_at=1.1,
                manual_authorized=True,
            )
        )
        self.assertEqual(output.source, MANUAL)
        self.assertEqual(output.command, manual_cmd())

    def test_authorized_manual_command_expires_to_zero(self) -> None:
        output = CmdVelArbiter().process(
            ArbiterInput(
                now=1.0,
                navigation_command=nav_cmd(),
                navigation_received_at=1.0,
                navigation_status_received_at=1.0,
                manual_command=manual_cmd(),
                manual_received_at=0.0,
                manual_authorized=True,
            )
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "MANUAL_SOURCE_STALE")

    def test_missing_navigation_health_status_is_zero(self) -> None:
        output = CmdVelArbiter().process(
            ArbiterInput(now=0.0, navigation_command=nav_cmd(), navigation_received_at=0.0)
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.source, NONE)
        self.assertEqual(output.reason, "NAVIGATION_STATUS_STALE")

    def test_stale_navigation_health_status_is_zero(self) -> None:
        output = CmdVelArbiter().process(event(2.0, nav=nav_cmd(), status_at=0.0))
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.reason, "NAVIGATION_STATUS_STALE")

    def test_localization_suspect_is_zero(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            event(0.0, "LOCALIZATION_SUSPECT", nav=nav_cmd())
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.source, NONE)

    def test_recovered_allows_navigation(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0))
        arbiter.process(event(0.0, "RECOVERED", nav=nav_cmd()))
        output = arbiter.process(event(0.1, "RECOVERED", nav=nav_cmd()))
        self.assertEqual(output.source, NAV)
        self.assertEqual(output.command, nav_cmd())

    def test_unknown_navigation_state_is_fail_safe_zero(self) -> None:
        output = CmdVelArbiter(ArbiterConfig(switch_guard_duration=0.0)).process(
            event(0.0, "NEW_STATE", nav=nav_cmd())
        )
        self.assertEqual(output.command, TwistCommand())
        self.assertEqual(output.source, NONE)
        self.assertEqual(output.reason, "UNKNOWN_NAVIGATION_STATE")

    def test_old_nominal_status_cannot_keep_forwarding_navigation(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(status_timeout=0.5, switch_guard_duration=0.0))
        arbiter.process(event(0.0, nav=nav_cmd(), status_at=0.0))
        first = arbiter.process(event(0.1, nav=nav_cmd(), status_at=0.1))
        stale = arbiter.process(event(1.0, nav=nav_cmd(), status_at=0.0))
        self.assertEqual(first.command, nav_cmd())
        self.assertEqual(stale.command, TwistCommand())

    def test_old_recovering_status_cannot_keep_forwarding_recovery(self) -> None:
        arbiter = CmdVelArbiter(ArbiterConfig(status_timeout=0.5, switch_guard_duration=0.0))
        stale = arbiter.process(
            event(1.0, "RECOVERING", nav=nav_cmd(), recovery=recovery_cmd(), recovery_at=1.0, status_at=0.0)
        )
        self.assertEqual(stale.command, TwistCommand())
        self.assertEqual(stale.reason, "NAVIGATION_STATUS_STALE")


if __name__ == "__main__":
    unittest.main()
