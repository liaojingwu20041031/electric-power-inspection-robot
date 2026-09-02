#!/usr/bin/env python3
"""ROS-independent health aggregation for the DG-202611 integration POC."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


NOMINAL = "NOMINAL"
DEGRADED = "DEGRADED"
LOCALIZATION_SUSPECT = "LOCALIZATION_SUSPECT"
RECOVERING = "RECOVERING"
RECOVERED = "RECOVERED"
FAILED = "FAILED"
MANUAL_REQUIRED = "MANUAL_REQUIRED"

GOOD = "GOOD"
REJECTED = "REJECTED"
STALE = "STALE"
UNKNOWN = "UNKNOWN"

SIGNAL_STATE_KEYS = {
    "gnss": ("current_state", "decision"),
    "lidar": ("state", "temporal_status"),
    "scan_match": ("accepted", "reason"),
    "relocalization": ("state",),
    "uwb": ("state",),
}


@dataclass(frozen=True)
class SignalObservation:
    state: str
    fresh: bool | None
    received_at: float | None
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class NavigationHealthInput:
    now: float
    gnss_state: str | None = None
    gnss_fresh: bool | None = None
    lidar_state: str | None = None
    lidar_fresh: bool | None = None
    amcl_state: str | None = None
    amcl_covariance: float | None = None
    amcl_fresh: bool | None = None
    odom_fresh: bool | None = None
    scan_fresh: bool | None = None
    scan_match_state: str | None = None
    scan_match_fresh: bool | None = None
    relocalization_state: str | None = None
    relocalization_fresh: bool | None = None
    uwb_state: str | None = None
    uwb_fresh: bool | None = None
    required_signals: tuple[str, ...] = ()


@dataclass(frozen=True)
class NavigationHealthOutput:
    overall_state: str
    gnss_state: str
    lidar_state: str
    amcl_state: str
    scan_match_state: str
    relocalization_state: str
    uwb_state: str
    reasons: tuple[str, ...]
    timestamp: float
    transition: bool
    previous_state: str | None
    transition_message: str
    transition_level: str


def _finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalize_signal_state(value: str | None, fresh: bool | None = None) -> str:
    """Normalize publisher-specific words without changing freshness meaning."""
    if fresh is False:
        return STALE
    raw = str(value or "").strip().upper()
    if raw in {"GOOD", "OK", "NOMINAL", "ACCEPT", "ACCEPTED", "FIXED", "STABLE"}:
        return GOOD
    if raw in {"DEGRADED", "WARN", "WARNING", "RECOVERING"}:
        return DEGRADED
    if raw in {"REJECTED", "BAD", "ERROR", "INVALID", "FAILED", "FAIL"}:
        return REJECTED
    if raw == "STALE":
        return STALE
    return UNKNOWN


def parse_signal_diagnostic(
    signal: str,
    values: Mapping[str, str],
    level: int,
    now: float,
    received_at: float | None,
    timeout: float,
    min_lidar_quality: float = 0.2,
) -> SignalObservation:
    """Map the fields emitted by existing POCs into one signal contract."""
    fresh = None if received_at is None else now - received_at <= timeout
    upper_values = {str(key): str(value).strip() for key, value in values.items()}
    reasons: list[str] = []
    if signal == "gnss":
        raw = upper_values.get("current_state") or upper_values.get("decision")
        state = normalize_signal_state(raw, fresh)
    elif signal == "lidar":
        if fresh is False:
            state = STALE
        elif level >= 2:
            state = REJECTED
            reasons.append("LIDAR_DIAGNOSTIC_ERROR")
        else:
            valid_ratio = upper_values.get("valid_ratio")
            geometry_score = upper_values.get("geometry_score")
            if valid_ratio is None or not _finite(valid_ratio) or float(valid_ratio) <= 0.0:
                state = REJECTED
                reasons.append("LIDAR_NO_VALID_POINTS")
            elif geometry_score is None or not _finite(geometry_score):
                state = REJECTED
                reasons.append("LIDAR_GEOMETRY_SCORE_MISSING")
            elif float(geometry_score) < min_lidar_quality:
                state = DEGRADED
                reasons.append("LIDAR_GEOMETRY_DEGRADED")
            else:
                state = GOOD
    elif signal == "scan_match":
        accepted = upper_values.get("accepted", "").lower()
        if accepted in {"false", "0", "no"}:
            state = normalize_signal_state("REJECTED", fresh)
        elif accepted in {"true", "1", "yes"}:
            state = normalize_signal_state("GOOD", fresh)
        else:
            state = normalize_signal_state(upper_values.get("reason"), fresh)
    elif signal == "relocalization":
        state = normalize_signal_state(upper_values.get("state"), fresh)
    else:
        raise ValueError(f"unsupported diagnostic signal: {signal}")
    if level >= 2 and state == GOOD:
        state = REJECTED
    elif level == 1 and state == GOOD:
        state = DEGRADED
    if fresh is False:
        reasons.append(f"{signal.upper()}_STALE")
    return SignalObservation(state, fresh, received_at, tuple(reasons))


class NavigationHealthAggregator:
    """Aggregate observations; it never commands motion or triggers recovery."""

    def __init__(
        self,
        max_amcl_covariance: float = 0.5,
        freshness_timeout: float = 1.0,
        min_lidar_quality: float = 0.2,
    ) -> None:
        self.max_amcl_covariance = max_amcl_covariance
        self.freshness_timeout = freshness_timeout
        if min_lidar_quality < 0.0:
            raise ValueError("min_lidar_quality must be non-negative")
        self.min_lidar_quality = min_lidar_quality
        self._last_overall_state: str | None = None

    def evaluate(self, event: NavigationHealthInput) -> NavigationHealthOutput:
        gnss = normalize_signal_state(event.gnss_state, event.gnss_fresh)
        lidar = normalize_signal_state(event.lidar_state, event.lidar_fresh)
        scan_match = normalize_signal_state(event.scan_match_state, event.scan_match_fresh)
        uwb = normalize_signal_state(event.uwb_state, event.uwb_fresh)
        amcl = self._amcl_state(event)
        relocalization = (
            STALE
            if event.relocalization_fresh is False
            else self._relocalization_state(event.relocalization_state)
        )
        reasons: list[str] = []
        required = set(event.required_signals)
        for name, state in (("GNSS", gnss), ("LIDAR", lidar), ("SCAN_MATCH", scan_match), ("UWB", uwb)):
            if state in {DEGRADED, REJECTED, STALE}:
                reasons.append(f"{name}_{state}")
                if state == STALE and name.lower() in required:
                    reasons.append(f"{name}_REQUIRED_STALE")
        if amcl in {REJECTED, STALE}:
            reasons.append(f"AMCL_{amcl}")
        if event.odom_fresh is False:
            reasons.append("ODOM_STALE")
        if event.scan_fresh is False:
            reasons.append("SCAN_STALE")
        if event.amcl_fresh is False and "AMCL_STALE" not in reasons:
            reasons.append("AMCL_STALE")
        if relocalization == STALE:
            reasons.append("RELOCALIZATION_STALE")

        if relocalization == MANUAL_REQUIRED:
            overall = MANUAL_REQUIRED
        elif relocalization == FAILED:
            overall = FAILED
        elif relocalization == RECOVERING:
            overall = RECOVERING
        elif (
            relocalization == LOCALIZATION_SUSPECT
            or relocalization == STALE
            or amcl in {REJECTED, STALE}
            or any(state == STALE and name.lower() in required for name, state in (("GNSS", gnss), ("LIDAR", lidar), ("SCAN_MATCH", scan_match)))
        ):
            overall = LOCALIZATION_SUSPECT
        elif relocalization == RECOVERED:
            overall = RECOVERED
        elif reasons:
            overall = DEGRADED
        else:
            overall = NOMINAL
        reasons = list(dict.fromkeys(reasons))
        previous = self._last_overall_state
        transition = overall != previous
        self._last_overall_state = overall
        if transition:
            transition_message = f"navigation health {previous or 'UNKNOWN'} -> {overall}"
            transition_level = "ERROR" if overall in {FAILED, MANUAL_REQUIRED} else "WARN" if overall in {DEGRADED, LOCALIZATION_SUSPECT, RECOVERING} else "INFO"
        else:
            transition_message = ""
            transition_level = ""
        return NavigationHealthOutput(
            overall_state=overall,
            gnss_state=gnss,
            lidar_state=lidar,
            amcl_state=amcl,
            scan_match_state=scan_match,
            relocalization_state=relocalization,
            uwb_state=uwb,
            reasons=tuple(reasons),
            timestamp=event.now,
            transition=transition,
            previous_state=previous,
            transition_message=transition_message,
            transition_level=transition_level,
        )

    def _amcl_state(self, event: NavigationHealthInput) -> str:
        if event.amcl_fresh is False:
            return STALE
        state = normalize_signal_state(event.amcl_state, event.amcl_fresh)
        if event.amcl_covariance is not None:
            if not _finite(event.amcl_covariance) or float(event.amcl_covariance) > self.max_amcl_covariance:
                return REJECTED
            return GOOD
        return state

    @staticmethod
    def _relocalization_state(value: str | None) -> str:
        raw = str(value or "").strip().upper()
        if raw in {"TRIGGERED", "STOPPING", "ACTIVE_SCAN", "WAITING_CANDIDATE", "VERIFYING"}:
            return RECOVERING
        if raw == "SUSPECTED":
            return LOCALIZATION_SUSPECT
        if raw in {FAILED, MANUAL_REQUIRED, RECOVERED}:
            return raw
        return NOMINAL
