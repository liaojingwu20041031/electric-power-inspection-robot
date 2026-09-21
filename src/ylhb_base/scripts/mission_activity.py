#!/usr/bin/env python3
"""Mission lifecycle gate shared by DG navigation supervision nodes.

The gate is deliberately ROS-independent so lifecycle semantics can be tested
without a running robot.  Recovery supervision is active only during a live
patrol whose core navigation components are running.  Missing or stale system
status always falls back to a motion-free standby state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


STANDBY = "STANDBY"
WARMUP = "WARMUP"
ACTIVE = "ACTIVE"

ACTIVE_PATROL_STATES = frozenset({"running", "returning_home"})
WARMUP_PATROL_STATES = frozenset({"starting", "command_sent"})
TERMINAL_PATROL_STATES = frozenset({"succeeded", "failed", "canceled", "cancelled"})
REQUIRED_COMPONENTS = ("bringup", "navigation", "patrol_executor")


@dataclass(frozen=True)
class MissionActivity:
    phase: str
    mission_active: bool
    status_fresh: bool
    patrol_state: str
    standby_reason: str = ""
    terminal_state: str = ""
    missing_components: tuple[str, ...] = ()


class MissionActivityGate:
    """Convert supervisor status snapshots into a safe mission phase."""

    def __init__(self, status_timeout_sec: float = 3.0, activation_grace_sec: float = 2.0) -> None:
        if status_timeout_sec <= 0.0:
            raise ValueError("status_timeout_sec must be positive")
        if activation_grace_sec < 0.0:
            raise ValueError("activation_grace_sec must be non-negative")
        self.status_timeout_sec = float(status_timeout_sec)
        self.activation_grace_sec = float(activation_grace_sec)
        self._ready_since: float | None = None
        self._last_terminal_state = ""

    @property
    def last_terminal_state(self) -> str:
        return self._last_terminal_state

    def reset(self) -> None:
        self._ready_since = None
        self._last_terminal_state = ""

    def evaluate(
        self,
        payload: Mapping[str, Any] | None,
        received_at: float | None,
        now: float,
    ) -> MissionActivity:
        if payload is None or received_at is None:
            self._ready_since = None
            return MissionActivity(STANDBY, False, False, "unknown", "NO_SYSTEM_STATUS")

        age = max(0.0, float(now) - float(received_at))
        if age > self.status_timeout_sec:
            self._ready_since = None
            return MissionActivity(
                STANDBY,
                False,
                False,
                self._patrol_state(payload),
                "SYSTEM_STATUS_STALE",
                self._last_terminal_state,
            )

        patrol_state = self._patrol_state(payload)
        if patrol_state in TERMINAL_PATROL_STATES:
            self._last_terminal_state = "canceled" if patrol_state == "cancelled" else patrol_state
            self._ready_since = None
            return MissionActivity(
                STANDBY,
                False,
                True,
                patrol_state,
                f"MISSION_{self._last_terminal_state.upper()}",
                self._last_terminal_state,
            )

        if patrol_state in {"paused", "pausing"}:
            self._ready_since = None
            return MissionActivity(
                STANDBY, False, True, patrol_state, "MISSION_PAUSED", self._last_terminal_state
            )

        if patrol_state in {"canceling", "stopping"}:
            self._ready_since = None
            return MissionActivity(
                STANDBY, False, True, patrol_state, "MISSION_STOPPING", self._last_terminal_state
            )

        missing = tuple(
            name for name in REQUIRED_COMPONENTS if str(payload.get(name) or "").lower() != "running"
        )
        if patrol_state in WARMUP_PATROL_STATES:
            self._ready_since = None
            return MissionActivity(
                WARMUP,
                False,
                True,
                patrol_state,
                "MISSION_STARTING",
                self._last_terminal_state,
                missing,
            )

        if patrol_state in ACTIVE_PATROL_STATES:
            if missing:
                self._ready_since = None
                return MissionActivity(
                    WARMUP,
                    False,
                    True,
                    patrol_state,
                    "COMPONENTS_NOT_READY",
                    self._last_terminal_state,
                    missing,
                )
            if self._ready_since is None:
                self._ready_since = float(now)
            if float(now) - self._ready_since < self.activation_grace_sec:
                return MissionActivity(
                    WARMUP,
                    False,
                    True,
                    patrol_state,
                    "ACTIVATION_GRACE",
                    self._last_terminal_state,
                )
            return MissionActivity(
                ACTIVE, True, True, patrol_state, "", self._last_terminal_state
            )

        self._ready_since = None
        reason = "MISSION_IDLE" if patrol_state in {"", "idle", "ready", "stopped"} else "MISSION_INACTIVE"
        return MissionActivity(
            STANDBY, False, True, patrol_state or "idle", reason, self._last_terminal_state
        )

    @staticmethod
    def _patrol_state(payload: Mapping[str, Any]) -> str:
        value = payload.get("patrol_mode_state")
        if value is None or str(value).strip() == "":
            value = payload.get("inspection_state") or "idle"
        return str(value).strip().lower()


def ungated_activity() -> MissionActivity:
    """Return the compatibility behavior used when mission gating is disabled."""

    return MissionActivity(ACTIVE, True, True, "ungated")
