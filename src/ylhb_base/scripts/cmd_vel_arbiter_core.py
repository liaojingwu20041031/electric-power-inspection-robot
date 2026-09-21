#!/usr/bin/env python3
"""ROS-independent command ownership and stale-source safety core."""

from __future__ import annotations

import math
from dataclasses import dataclass


NAV = "NAV"
RECOVERY = "RECOVERY"
MANUAL = "MANUAL"
NONE = "NONE"


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class ArbiterConfig:
    source_timeout: float = 0.5
    switch_guard_duration: float = 0.2
    status_timeout: float = 1.0


@dataclass(frozen=True)
class ArbiterInput:
    now: float
    navigation_command: TwistCommand | None = None
    navigation_received_at: float | None = None
    recovery_command: TwistCommand | None = None
    recovery_received_at: float | None = None
    navigation_state: str = "NOMINAL"
    manual_takeover: bool = False
    navigation_status_received_at: float | None = None
    # Manual commands are accepted only while an explicit authorization signal
    # is fresh.  The legacy manual_takeover flag remains a hard stop latch.
    manual_command: TwistCommand | None = None
    manual_received_at: float | None = None
    manual_authorized: bool = False


@dataclass(frozen=True)
class ArbiterOutput:
    command: TwistCommand
    source: str
    reason: str
    guard_active: bool
    timestamp: float


def _fresh(received_at: float | None, now: float, timeout: float) -> bool:
    return received_at is not None and now - received_at <= timeout


def _finite_command(command: TwistCommand | None) -> bool:
    return command is not None and math.isfinite(command.linear_x) and math.isfinite(command.angular_z)


class CmdVelArbiter:
    """Choose exactly one command source and insert a zero guard on switches."""

    def __init__(self, config: ArbiterConfig | None = None) -> None:
        self.config = config or ArbiterConfig()
        if (
            self.config.source_timeout < 0.0
            or self.config.switch_guard_duration < 0.0
            or self.config.status_timeout < 0.0
        ):
            raise ValueError("arbiter durations must be non-negative")
        self._source = NONE
        self._guard_until = 0.0

    def process(self, event: ArbiterInput) -> ArbiterOutput:
        if event.manual_takeover:
            self._source = NONE
            return self._zero(event, "MANUAL_TAKEOVER")
        manual_fresh = _fresh(event.manual_received_at, event.now, self.config.source_timeout)
        manual_ok = event.manual_authorized and manual_fresh and _finite_command(event.manual_command)
        if event.manual_authorized and not manual_ok:
            self._source = NONE
            return self._zero(event, "MANUAL_SOURCE_STALE")
        if manual_ok:
            if self._source != MANUAL:
                self._source = MANUAL
                self._guard_until = event.now + self.config.switch_guard_duration
                return self._zero(event, "SOURCE_SWITCH_TO_MANUAL", guard=True)
            if event.now < self._guard_until:
                return self._zero(event, "SOURCE_SWITCH_GUARD", guard=True)
            return ArbiterOutput(event.manual_command or TwistCommand(), MANUAL, "FORWARDED", False, event.now)
        if not _fresh(event.navigation_status_received_at, event.now, self.config.status_timeout):
            self._source = NONE
            return self._zero(event, "NAVIGATION_STATUS_STALE")
        state = str(event.navigation_state or "DEGRADED").upper()
        nav_fresh = _fresh(event.navigation_received_at, event.now, self.config.source_timeout)
        recovery_fresh = _fresh(event.recovery_received_at, event.now, self.config.source_timeout)
        nav_ok = nav_fresh and _finite_command(event.navigation_command)
        recovery_ok = recovery_fresh and _finite_command(event.recovery_command)

        if state in {"FAILED", "MANUAL_REQUIRED", "LOCALIZATION_SUSPECT"}:
            self._source = NONE
            return self._zero(event, f"STATE_{state}")
        if state == "RECOVERING":
            if not recovery_ok:
                self._source = NONE
                return self._zero(event, "RECOVERY_SOURCE_STALE")
            selected = RECOVERY
            command = event.recovery_command
        elif state in {"NOMINAL", "DEGRADED", "RECOVERED"}:
            if not nav_ok:
                self._source = NONE
                return self._zero(event, "NAV_SOURCE_STALE")
            selected = NAV
            command = event.navigation_command
        else:
            self._source = NONE
            return self._zero(event, "UNKNOWN_NAVIGATION_STATE")

        if selected != self._source:
            self._source = selected
            self._guard_until = event.now + self.config.switch_guard_duration
            return self._zero(event, f"SOURCE_SWITCH_TO_{selected}", guard=True)
        if event.now < self._guard_until:
            return self._zero(event, "SOURCE_SWITCH_GUARD", guard=True)
        return ArbiterOutput(command or TwistCommand(), selected, "FORWARDED", False, event.now)

    def _zero(self, event: ArbiterInput, reason: str, guard: bool = False) -> ArbiterOutput:
        return ArbiterOutput(TwistCommand(), NONE if reason.endswith("STALE") or reason.startswith("STATE_") or reason == "MANUAL_TAKEOVER" else self._source, reason, guard, event.now)
