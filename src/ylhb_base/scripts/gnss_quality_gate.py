#!/usr/bin/env python3
"""ROS-independent GNSS quality gate POC.

The core only uses fields already exposed by the current WTRTK980 NMEA
driver.  It does not infer pseudorange, carrier phase, RTCM, or satellite-
level observations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping


GOOD = "GOOD"
DEGRADED = "DEGRADED"
REJECTED = "REJECTED"
RECOVERING = "RECOVERING"
INITIAL = "INITIAL"

ACCEPT = "ACCEPT"
ACCEPT_DEGRADED = "ACCEPT_DEGRADED"
REJECT = "REJECT"
HOLD_FOR_RECOVERY = "HOLD_FOR_RECOVERY"

STALE_DIAGNOSTIC_LEVEL = 3


@dataclass(frozen=True)
class GnssGateConfig:
    """Conservative POC thresholds; none are claimed to be competition-optimal."""

    min_satellites_good: int = 8
    min_satellites_degraded: int = 4
    max_hdop_good: float = 1.5
    max_hdop_degraded: float = 3.0
    max_differential_age: float = 3.0
    stale_timeout: float = 3.0
    status_timeout: float = 3.0
    max_jump_distance: float = 5.0
    max_implied_speed: float = 3.0
    recovery_good_samples: int = 3

    def __post_init__(self) -> None:
        if self.min_satellites_degraded < 0 or self.min_satellites_good < self.min_satellites_degraded:
            raise ValueError("satellite thresholds are inconsistent")
        if self.recovery_good_samples < 1:
            raise ValueError("recovery_good_samples must be at least one")
        for value in (
            self.max_hdop_good,
            self.max_hdop_degraded,
            self.max_differential_age,
            self.stale_timeout,
            self.status_timeout,
            self.max_jump_distance,
            self.max_implied_speed,
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("GNSS thresholds must be finite and non-negative")
        if self.max_hdop_good > self.max_hdop_degraded:
            raise ValueError("HDOP thresholds are inconsistent")


@dataclass(frozen=True)
class GnssObservation:
    """Only data fields supported by the current fix/status topics."""

    timestamp: float
    latitude: float | None
    longitude: float | None
    altitude: float | None
    fix_available: bool
    quality: int | None
    satellites: int | None
    hdop: float | None
    differential_age: float | None
    stale: bool = False
    status_age: float | None = None
    status_reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class ParsedStatus:
    """Typed result of parsing DiagnosticArray KeyValue fields."""

    quality: int | None
    quality_text: str | None
    satellites: int | None
    hdop: float | None
    differential_age: float | None
    base_station_id: str | None
    stale: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class GateResult:
    previous_state: str
    current_state: str
    decision: str
    reasons: tuple[str, ...]
    accepted: bool
    distance_from_previous: float | None
    dt: float | None
    implied_speed: float | None
    recovery_count: int


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _parse_int(values: Mapping[str, str], key: str, reasons: list[str]) -> int | None:
    text = values.get(key, "").strip()
    if not text:
        reasons.append(f"{key.upper()}_MISSING")
        return None
    try:
        return int(text)
    except ValueError:
        reasons.append(f"{key.upper()}_INVALID")
        return None


def _parse_float(values: Mapping[str, str], key: str, reasons: list[str], empty_is_missing: bool = True) -> float | None:
    text = values.get(key, "").strip()
    if not text:
        if empty_is_missing:
            reasons.append(f"{key.upper()}_MISSING")
        return None
    try:
        value = float(text)
    except ValueError:
        reasons.append(f"{key.upper()}_INVALID")
        return None
    if not math.isfinite(value):
        reasons.append(f"{key.upper()}_INVALID")
        return None
    return value


def parse_status_values(
    values: Mapping[str, str],
    diagnostic_level: int | None = None,
) -> ParsedStatus:
    """Parse current WTRTK980 DiagnosticStatus fields without throwing.

    An empty ``differential_age`` stays ``None``; it is never converted to
    zero.  A missing optional base station ID is retained as ``None``.
    """

    reasons: list[str] = []
    quality = _parse_int(values, "quality", reasons)
    satellites = _parse_int(values, "satellites", reasons)
    hdop = _parse_float(values, "hdop", reasons)
    differential_age = _parse_float(values, "differential_age", reasons, empty_is_missing=False)
    quality_text = values.get("quality_text", "").strip() or None
    if quality_text is None:
        reasons.append("QUALITY_TEXT_MISSING")
    base_station_id = values.get("base_station_id", "").strip() or None
    stale = diagnostic_level == STALE_DIAGNOSTIC_LEVEL
    if stale:
        reasons.append("STALE")
    return ParsedStatus(
        quality=quality,
        quality_text=quality_text,
        satellites=satellites,
        hdop=hdop,
        differential_age=differential_age,
        base_station_id=base_station_id,
        stale=stale,
        reasons=tuple(dict.fromkeys(reasons)),
    )


def haversine_distance_m(latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float) -> float:
    """Return short/long distance between WGS84 coordinates using haversine."""

    earth_radius = 6371008.8
    lat_a = math.radians(latitude_a)
    lat_b = math.radians(latitude_b)
    delta_lat = math.radians(latitude_b - latitude_a)
    delta_lon = math.radians(longitude_b - longitude_a)
    value = math.sin(delta_lat / 2.0) ** 2 + math.cos(lat_a) * math.cos(lat_b) * math.sin(delta_lon / 2.0) ** 2
    return 2.0 * earth_radius * math.asin(math.sqrt(max(0.0, min(1.0, value))))


def _coordinate_reasons(observation: GnssObservation) -> list[str]:
    reasons: list[str] = []
    if not all(_finite(value) for value in (observation.latitude, observation.longitude)):
        reasons.append("INVALID_COORDINATE")
        return reasons
    if not -90.0 <= float(observation.latitude) <= 90.0 or not -180.0 <= float(observation.longitude) <= 180.0:
        reasons.append("INVALID_COORDINATE")
    if observation.altitude is not None and not _finite(observation.altitude):
        reasons.append("INVALID_ALTITUDE")
    return reasons


def _motion_reasons(
    observation: GnssObservation,
    previous: GnssObservation | None,
    config: GnssGateConfig,
) -> tuple[list[str], float | None, float | None, float | None]:
    if previous is None or "INVALID_COORDINATE" in _coordinate_reasons(previous):
        return [], None, None, None
    dt = float(observation.timestamp) - float(previous.timestamp)
    if not _finite(dt) or dt <= 0.0:
        return ["DT_NON_POSITIVE"], None, dt, None
    distance = haversine_distance_m(
        float(previous.latitude),
        float(previous.longitude),
        float(observation.latitude),
        float(observation.longitude),
    )
    speed = distance / dt
    reasons: list[str] = []
    if distance > config.max_jump_distance:
        reasons.append("POSITION_JUMP")
    if speed > config.max_implied_speed:
        reasons.append("IMPLIED_SPEED_HIGH")
    return reasons, distance, dt, speed


def _quality_reasons(observation: GnssObservation, config: GnssGateConfig) -> tuple[list[str], list[str]]:
    """Return hard rejection reasons and quality-degradation reasons."""

    hard: list[str] = []
    degraded: list[str] = []
    if not observation.fix_available or observation.quality == 0:
        hard.append("NO_FIX")
    if observation.quality is None:
        hard.append("QUALITY_MISSING")
    if observation.satellites is None:
        hard.append("SATELLITES_MISSING")
    elif observation.satellites < config.min_satellites_degraded:
        hard.append("LOW_SATELLITES")
    elif observation.satellites < config.min_satellites_good:
        degraded.append("LOW_SATELLITES")
    if observation.hdop is None:
        hard.append("HDOP_MISSING")
    elif observation.hdop > config.max_hdop_degraded:
        hard.append("HIGH_HDOP")
    elif observation.hdop > config.max_hdop_good:
        degraded.append("HIGH_HDOP")
    if observation.differential_age is not None and observation.differential_age > config.max_differential_age:
        hard.append("DIFFERENTIAL_AGE_HIGH")
    elif observation.quality in (4, 5) and observation.differential_age is None:
        degraded.append("DIFFERENTIAL_AGE_MISSING")
    return hard, degraded


def _evaluate_observation(
    observation: GnssObservation,
    previous: GnssObservation | None,
    config: GnssGateConfig,
    now: float | None = None,
) -> tuple[bool, tuple[str, ...], float | None, float | None, float | None]:
    reasons: list[str] = list(observation.status_reasons)
    hard: list[str] = []
    degraded: list[str] = []
    if not _finite(observation.timestamp):
        hard.append("INVALID_TIMESTAMP")
    elif now is not None and _finite(now) and float(now) - float(observation.timestamp) > config.stale_timeout:
        hard.append("STALE")
    elif now is not None and not _finite(now):
        hard.append("INVALID_TIMESTAMP")
    elif observation.status_age is not None and (
        not _finite(observation.status_age) or observation.status_age > config.status_timeout
    ):
        hard.append("STALE_STATUS")
    if observation.stale:
        hard.append("STALE")
    if not observation.fix_available:
        hard.append("NO_FIX")
    coordinate_reasons = _coordinate_reasons(observation)
    hard.extend(coordinate_reasons)
    quality_hard, quality_degraded = _quality_reasons(observation, config)
    hard.extend(quality_hard)
    degraded.extend(quality_degraded)
    motion, distance, dt, speed = ([], None, None, None)
    if not hard and observation.latitude is not None and observation.longitude is not None:
        motion, distance, dt, speed = _motion_reasons(observation, previous, config)
        hard.extend(motion)
    if "STALE" in observation.status_reasons and "STALE" not in hard:
        hard.append("STALE")
    for reason in reasons:
        if reason.endswith("_INVALID") or reason in {"QUALITY_MISSING", "SATELLITES_MISSING", "HDOP_MISSING", "STALE_STATUS"}:
            hard.append(reason)
        elif reason in {"DIFFERENTIAL_AGE_MISSING", "QUALITY_TEXT_MISSING"}:
            degraded.append(reason)
    ordered = []
    for reason in [*hard, *degraded, *reasons]:
        if reason not in ordered:
            ordered.append(reason)
    return not hard and not degraded, tuple(ordered), distance, dt, speed


class GnssQualityGate:
    """Deterministic stateful gate with recovery hysteresis."""

    def __init__(self, config: GnssGateConfig | None = None) -> None:
        self.config = config or GnssGateConfig()
        self.state = INITIAL
        self.recovery_count = 0
        self.previous_observation: GnssObservation | None = None

    def evaluate(self, observation: GnssObservation, now: float | None = None) -> GateResult:
        previous_state = self.state
        healthy, reasons, distance, dt, speed = _evaluate_observation(
            observation,
            self.previous_observation,
            self.config,
            now,
        )
        hard = any(
            reason in {
                "INVALID_TIMESTAMP",
                "STALE",
                "STALE_STATUS",
                "NO_FIX",
                "INVALID_COORDINATE",
                "INVALID_ALTITUDE",
                "POSITION_JUMP",
                "IMPLIED_SPEED_HIGH",
                "DT_NON_POSITIVE",
                "QUALITY_MISSING",
                "QUALITY_INVALID",
                "SATELLITES_MISSING",
                "SATELLITES_INVALID",
                "HDOP_MISSING",
                "HDOP_INVALID",
                "DIFFERENTIAL_AGE_INVALID",
                "DIFFERENTIAL_AGE_HIGH",
            }
            for reason in reasons
        )
        if "LOW_SATELLITES" in reasons:
            hard = hard or (
                self.config.min_satellites_degraded > 0
                and (observation.satellites is None or observation.satellites < self.config.min_satellites_degraded)
            )
        if "HIGH_HDOP" in reasons:
            hard = hard or (
                observation.hdop is None
                or observation.hdop > self.config.max_hdop_degraded
            )
        degraded = not healthy and not hard
        if previous_state == INITIAL:
            if hard:
                current_state, decision = REJECTED, REJECT
            elif degraded:
                current_state, decision = DEGRADED, ACCEPT_DEGRADED
            else:
                current_state, decision = GOOD, ACCEPT
            self.recovery_count = 0
        elif previous_state == REJECTED:
            if hard:
                current_state, decision = REJECTED, REJECT
                self.recovery_count = 0
            elif degraded:
                current_state, decision = RECOVERING, HOLD_FOR_RECOVERY
                self.recovery_count = 0
            else:
                self.recovery_count = 1
                if self.recovery_count >= self.config.recovery_good_samples:
                    current_state, decision = GOOD, ACCEPT
                else:
                    current_state, decision = RECOVERING, HOLD_FOR_RECOVERY
        elif previous_state == RECOVERING:
            if hard:
                current_state, decision = REJECTED, REJECT
                self.recovery_count = 0
            elif degraded:
                current_state, decision = RECOVERING, HOLD_FOR_RECOVERY
                self.recovery_count = 0
            else:
                self.recovery_count += 1
                if self.recovery_count >= self.config.recovery_good_samples:
                    current_state, decision = GOOD, ACCEPT
                else:
                    current_state, decision = RECOVERING, HOLD_FOR_RECOVERY
        elif previous_state == GOOD:
            if hard:
                current_state, decision = REJECTED, REJECT
                self.recovery_count = 0
            elif degraded:
                current_state, decision = DEGRADED, ACCEPT_DEGRADED
            else:
                current_state, decision = GOOD, ACCEPT
        else:  # DEGRADED
            if hard:
                current_state, decision = REJECTED, REJECT
                self.recovery_count = 0
            elif degraded:
                current_state, decision = DEGRADED, ACCEPT_DEGRADED
            else:
                current_state, decision = GOOD, ACCEPT
        result_reasons = list(reasons)
        if current_state == RECOVERING and "RECOVERY_PENDING" not in result_reasons:
            result_reasons.append("RECOVERY_PENDING")
        if current_state == REJECTED and not result_reasons:
            result_reasons.append("REJECTED")
        self.state = current_state
        # Do not let a rejected jump become the next reference and cause a
        # permanent rejection cascade.  The last non-rejected fix remains the
        # trusted comparison point during recovery.
        if current_state != REJECTED and not hard and _coordinate_reasons(observation) == []:
            self.previous_observation = observation
        return GateResult(
            previous_state=previous_state,
            current_state=current_state,
            decision=decision,
            reasons=tuple(result_reasons),
            accepted=decision in {ACCEPT, ACCEPT_DEGRADED},
            distance_from_previous=distance,
            dt=dt,
            implied_speed=speed,
            recovery_count=self.recovery_count,
        )
