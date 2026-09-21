#!/usr/bin/env python3
"""ROS-independent active relocalization supervisor POC.

This is an event-driven safety/state-machine core.  It does not implement
scan matching; candidates are supplied by the existing Scan-to-Map component.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


NORMAL = "NORMAL"
SUSPECTED = "SUSPECTED"
TRIGGERED = "TRIGGERED"
STOPPING = "STOPPING"
ACTIVE_SCAN = "ACTIVE_SCAN"
WAITING_CANDIDATE = "WAITING_CANDIDATE"
VERIFYING = "VERIFYING"
RECOVERED = "RECOVERED"
FAILED = "FAILED"
MANUAL_REQUIRED = "MANUAL_REQUIRED"


@dataclass(frozen=True)
class LocalizationHealth:
    """Optional health inputs from AMCL and the two side-channel POCs."""

    timestamp: float = 0.0
    amcl_covariance: float | None = None
    lidar_quality: float | None = None
    gnss_quality: str | None = None
    scan_match_score: float | None = None
    scan_match_inlier_ratio: float | None = None
    scan_match_mean_distance: float | None = None
    scan_fresh: bool | None = None
    odom_fresh: bool | None = None
    imu_fresh: bool | None = None
    amcl_fresh: bool | None = None
    lidar_fresh: bool | None = None
    gnss_fresh: bool | None = None
    scan_match_fresh: bool | None = None
    odom_linear_velocity: float | None = None
    odom_angular_velocity: float | None = None
    pose_x: float | None = None
    pose_y: float | None = None
    pose_yaw: float | None = None


def recovery_motion_gate(
    recovery_motion_armed: bool, health: LocalizationHealth | None
) -> tuple[bool, str]:
    """Return whether a non-zero recovery command is safe to publish.

    The active-relocalization state machine may continue observing and producing
    a proposed command while the robot is unarmed.  The ROS node must enforce
    this final actuator gate so an observation-only launch cannot move the base.
    """
    if not recovery_motion_armed:
        return False, "RECOVERY_MOTION_NOT_ARMED"
    if health is None:
        return False, "HEALTH_MISSING"
    for name in ("scan_fresh", "odom_fresh", "imu_fresh"):
        if getattr(health, name) is not True:
            return False, f"{name.upper()}_NOT_FRESH"
    return True, ""


@dataclass(frozen=True)
class CandidateQuality:
    timestamp: float
    score: float
    inlier_ratio: float
    mean_distance: float
    x: float
    y: float
    yaw: float
    used_points: int = 0
    accepted: bool = True
    reason: str = ""
    received_time: float | None = None


@dataclass(frozen=True)
class ActiveRelocalizationConfig:
    suspect_samples: int = 2
    trigger_samples: int = 2
    healthy_recovery_samples: int = 3
    suspect_duration: float = 0.0
    trigger_duration: float = 0.0
    healthy_recovery_duration: float = 0.0
    max_covariance: float = 0.5
    min_lidar_quality: float = 0.2
    min_scan_match_score: float = 0.45
    min_scan_match_inlier_ratio: float = 0.45
    max_scan_match_mean_distance: float = 0.30
    signal_timeout: float = 1.0
    stop_velocity: float = 0.03
    angular_speed: float = 0.18
    yaw_tolerance: float = math.radians(1.5)
    segment_timeout: float = 8.0
    settle_time: float = 0.5
    segment_deltas: tuple[float, ...] = (math.radians(10.0), math.radians(-20.0), math.radians(10.0))
    max_total_rotation: float = math.radians(60.0)
    max_attempt_duration: float = 30.0
    max_attempts: int = 2
    verification_samples: int = 3
    max_verify_position_jump: float = 0.5
    max_verify_yaw_jump: float = math.radians(20.0)
    required_signals: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.suspect_samples < 1 or self.trigger_samples < 1 or self.healthy_recovery_samples < 1:
            raise ValueError("sample thresholds must be positive")
        if self.max_attempts < 1 or self.verification_samples < 1:
            raise ValueError("attempt and verification thresholds must be positive")
        if not self.segment_deltas:
            raise ValueError("at least one active sensing segment is required")
        for value in (
            self.max_covariance,
            self.min_lidar_quality,
            self.min_scan_match_score,
            self.min_scan_match_inlier_ratio,
            self.max_scan_match_mean_distance,
            self.signal_timeout,
            self.stop_velocity,
            self.angular_speed,
            self.yaw_tolerance,
            self.segment_timeout,
            self.settle_time,
            self.max_total_rotation,
            self.max_attempt_duration,
            self.max_verify_position_jump,
            self.max_verify_yaw_jump,
            self.suspect_duration,
            self.trigger_duration,
            self.healthy_recovery_duration,
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("relocalization thresholds must be finite and non-negative")


@dataclass(frozen=True)
class SupervisorInput:
    now: float
    health: LocalizationHealth | None = None
    current_yaw: float | None = None
    scan_updated: bool = False
    candidate: CandidateQuality | None = None
    manual_takeover: bool = False
    shutdown: bool = False


@dataclass(frozen=True)
class SupervisorOutput:
    state: str
    attempt_id: int
    trigger_reason: str
    elapsed: float
    active_segment: int
    verification_count: int
    command_linear_x: float
    command_angular_z: float
    candidate_score: float | None
    candidate_inlier_ratio: float | None
    candidate_mean_distance: float | None
    lidar_health: str
    gnss_health: str
    amcl_health: str
    failure_reason: str
    manual_takeover: bool
    health_reasons: tuple[str, ...]
    request_candidate: bool
    candidate_request_time: float | None
    seed_source: str
    seed_pose: tuple[float, float, float] | None


@dataclass(frozen=True)
class _HealthAssessment:
    triggerable: bool
    degraded: bool
    reasons: tuple[str, ...]
    lidar_health: str
    gnss_health: str
    amcl_health: str


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _angle_distance(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def _distance_2d(x1: float | None, y1: float | None, x2: float | None, y2: float | None) -> float | None:
    if not all(_finite(value) for value in (x1, y1, x2, y2)):
        return None
    return math.hypot(float(x2) - float(x1), float(y2) - float(y1))


def _required(config: ActiveRelocalizationConfig, *names: str) -> bool:
    return any(name in config.required_signals for name in names)


def assess_health(health: LocalizationHealth | None, config: ActiveRelocalizationConfig) -> _HealthAssessment:
    if health is None:
        return _HealthAssessment(False, True, ("HEALTH_INPUT_MISSING",), "UNKNOWN", "UNKNOWN", "UNKNOWN")
    reasons: list[str] = []
    trigger_reasons: list[str] = []
    degraded_reasons: list[str] = []
    if health.amcl_covariance is None:
        if "amcl_covariance" in config.required_signals:
            reasons.append("AMCL_COVARIANCE_MISSING")
            trigger_reasons.append("AMCL_COVARIANCE_MISSING")
        amcl_health = "UNKNOWN"
    elif not _finite(health.amcl_covariance):
        reasons.append("AMCL_COVARIANCE_INVALID")
        trigger_reasons.append("AMCL_COVARIANCE_INVALID")
        amcl_health = "BAD"
    elif health.amcl_covariance > config.max_covariance:
        reasons.append("AMCL_COVARIANCE_HIGH")
        trigger_reasons.append("AMCL_COVARIANCE_HIGH")
        amcl_health = "BAD"
    else:
        amcl_health = "GOOD"
    if health.amcl_fresh is False:
        reasons.append("AMCL_STALE")
        trigger_reasons.append("AMCL_STALE")
        amcl_health = "BAD"
    elif health.amcl_fresh is None and _required(config, "amcl", "amcl_fresh"):
        reasons.append("AMCL_FRESHNESS_MISSING")
        trigger_reasons.append("AMCL_FRESHNESS_MISSING")
        amcl_health = "BAD"

    if health.scan_match_score is None or health.scan_match_inlier_ratio is None or health.scan_match_mean_distance is None:
        if "scan_match_score" in config.required_signals:
            reasons.append("SCAN_MATCH_MISSING")
            trigger_reasons.append("SCAN_MATCH_MISSING")
        scan_match_health = "UNKNOWN"
    elif not all(_finite(value) for value in (health.scan_match_score, health.scan_match_inlier_ratio, health.scan_match_mean_distance)):
        reasons.append("SCAN_MATCH_INVALID")
        trigger_reasons.append("SCAN_MATCH_INVALID")
        scan_match_health = "BAD"
    elif (
        health.scan_match_score < config.min_scan_match_score
        or health.scan_match_inlier_ratio < config.min_scan_match_inlier_ratio
        or health.scan_match_mean_distance > config.max_scan_match_mean_distance
    ):
        reasons.append("SCAN_MATCH_QUALITY_LOW")
        trigger_reasons.append("SCAN_MATCH_QUALITY_LOW")
        scan_match_health = "BAD"
    else:
        scan_match_health = "GOOD"
    if health.scan_match_fresh is False:
        reasons.append("SCAN_MATCH_STALE")
        scan_match_health = "STALE"
        if _required(config, "scan_match", "scan_match_fresh"):
            trigger_reasons.append("SCAN_MATCH_STALE")
    elif health.scan_match_fresh is None and _required(config, "scan_match", "scan_match_fresh"):
        reasons.append("SCAN_MATCH_FRESHNESS_MISSING")
        trigger_reasons.append("SCAN_MATCH_FRESHNESS_MISSING")
    if health.scan_fresh is False:
        reasons.append("SCAN_STALE")
        trigger_reasons.append("SCAN_STALE")
    if health.odom_fresh is False:
        reasons.append("ODOM_STALE")
        degraded_reasons.append("ODOM_STALE")

    if health.lidar_quality is None:
        if "lidar_quality" in config.required_signals:
            reasons.append("LIDAR_QUALITY_MISSING")
            degraded_reasons.append("LIDAR_QUALITY_MISSING")
        lidar_health = "UNKNOWN"
    elif not _finite(health.lidar_quality):
        reasons.append("LIDAR_QUALITY_INVALID")
        degraded_reasons.append("LIDAR_QUALITY_INVALID")
        lidar_health = "BAD"
    elif health.lidar_quality < config.min_lidar_quality:
        reasons.append("LIDAR_QUALITY_LOW")
        degraded_reasons.append("LIDAR_QUALITY_LOW")
        lidar_health = "DEGRADED"
    else:
        lidar_health = "GOOD"
    if health.lidar_fresh is False:
        reasons.append("LIDAR_STALE")
        lidar_health = "STALE"
        if _required(config, "lidar", "lidar_fresh"):
            trigger_reasons.append("LIDAR_STALE")
    elif health.lidar_fresh is None and _required(config, "lidar", "lidar_fresh"):
        reasons.append("LIDAR_FRESHNESS_MISSING")
        trigger_reasons.append("LIDAR_FRESHNESS_MISSING")

    gnss_health = str(health.gnss_quality or "UNKNOWN").upper()
    if gnss_health == "REJECTED":
        reasons.append("GNSS_REJECTED")
        degraded_reasons.append("GNSS_REJECTED")
    elif gnss_health == "DEGRADED":
        reasons.append("GNSS_DEGRADED")
        degraded_reasons.append("GNSS_DEGRADED")
    if health.gnss_fresh is False:
        reasons.append("GNSS_STALE")
        gnss_health = "STALE"
        if _required(config, "gnss", "gnss_fresh"):
            trigger_reasons.append("GNSS_STALE")
    elif health.gnss_fresh is None and _required(config, "gnss", "gnss_fresh"):
        reasons.append("GNSS_FRESHNESS_MISSING")
        trigger_reasons.append("GNSS_FRESHNESS_MISSING")

    reasons = list(dict.fromkeys(reasons))
    return _HealthAssessment(
        bool(trigger_reasons),
        bool(trigger_reasons or degraded_reasons),
        tuple(reasons),
        lidar_health,
        gnss_health,
        amcl_health,
    )


class ActiveRelocalizationController:
    """Deterministic controller for trigger, safe motion, candidate validation and retry."""

    def __init__(self, config: ActiveRelocalizationConfig | None = None) -> None:
        self.config = config or ActiveRelocalizationConfig()
        self.state = NORMAL
        self.attempt_id = 0
        self.trigger_reason = ""
        self.failure_reason = ""
        self.bad_streak = 0
        self.trigger_streak = 0
        self.healthy_streak = 0
        self.bad_started: float | None = None
        self.suspected_started: float | None = None
        self.healthy_started: float | None = None
        self.attempt_started: float | None = None
        self.segment_index = -1
        self.segment_started: float | None = None
        self.segment_start_yaw: float | None = None
        self.segment_target_yaw: float | None = None
        self.settle_until: float | None = None
        self.waiting_since: float | None = None
        self.last_scan_time: float | None = None
        self.total_rotation = 0.0
        self.verification_count = 0
        self.last_verify_pose: tuple[float, float, float] | None = None
        self.candidate: CandidateQuality | None = None
        self.last_trusted_pose: tuple[float, float, float] | None = None
        self.seed_source = "NONE"
        self.seed_pose: tuple[float, float, float] | None = None
        self.candidate_request_time: float | None = None
        self.candidate_requested_for_segment = False
        self._manual_latched = False
        self._shutdown_latched = False

    def process(self, event: SupervisorInput) -> SupervisorOutput:
        if event.scan_updated:
            self.last_scan_time = event.now
        if event.candidate is not None and self._candidate_is_fresh(event.candidate):
            self.candidate = event.candidate
        if event.manual_takeover:
            self._manual_latched = True
            self.state = MANUAL_REQUIRED
            self.failure_reason = "MANUAL_TAKEOVER"
        if event.shutdown:
            self._shutdown_latched = True
            self.state = FAILED
            self.failure_reason = "SHUTDOWN"
        if self._manual_latched:
            return self._output(event, None, ("MANUAL_TAKEOVER",))
        if self._shutdown_latched:
            return self._output(event, None, ("SHUTDOWN",))

        assessment = assess_health(event.health, self.config)
        if self.state == NORMAL and not assessment.degraded and event.health is not None:
            trusted_pose = self._pose_from_health(event.health)
            if trusted_pose is not None:
                self.last_trusted_pose = trusted_pose
        if self.state in {NORMAL, SUSPECTED}:
            self._update_trigger_state(assessment, event.now)
            if self.state in {NORMAL, SUSPECTED}:
                return self._output(event, assessment, assessment.reasons)
            if self.state == TRIGGERED:
                return self._output(event, assessment, ("TRIGGERED",))
        if self.state == TRIGGERED:
            self.state = STOPPING
            self.failure_reason = ""
            return self._output(event, assessment, assessment.reasons)
        if self.state == STOPPING:
            return self._handle_stopping(event, assessment)
        if self.state == ACTIVE_SCAN:
            return self._handle_active_scan(event, assessment)
        if self.state == WAITING_CANDIDATE:
            return self._handle_waiting_candidate(event, assessment)
        if self.state == VERIFYING:
            return self._handle_verifying(event, assessment)
        if self.state == FAILED:
            if self.attempt_id < self.config.max_attempts:
                self.state = STOPPING
                self.failure_reason = ""
                return self._output(event, assessment, ("RETRY_PENDING",))
            self.state = MANUAL_REQUIRED
            self.failure_reason = self.failure_reason or "MAX_ATTEMPTS_EXCEEDED"
            return self._output(event, assessment, (self.failure_reason,))
        return self._output(event, assessment, assessment.reasons)

    def _update_trigger_state(self, assessment: _HealthAssessment, now: float) -> None:
        if assessment.triggerable:
            if self.bad_started is None:
                self.bad_started = now
            self.bad_streak += 1
            self.healthy_streak = 0
            self.healthy_started = None
            if (
                self.state == NORMAL
                and self.bad_streak >= self.config.suspect_samples
                and now - self.bad_started >= self.config.suspect_duration
            ):
                self.state = SUSPECTED
                self.trigger_streak = 1
                self.suspected_started = now
                self.trigger_reason = ";".join(assessment.reasons) or "LOCALIZATION_HEALTH_LOW"
            elif self.state == SUSPECTED:
                self.trigger_streak += 1
                if (
                    self.trigger_streak >= self.config.trigger_samples
                    and self.suspected_started is not None
                    and now - self.suspected_started >= self.config.trigger_duration
                ):
                    self.state = TRIGGERED
        elif assessment.degraded:
            self.healthy_streak = 0
            self.healthy_started = None
        else:
            self.bad_streak = 0
            self.trigger_streak = 0
            self.bad_started = None
            self.suspected_started = None
            if self.state == SUSPECTED:
                if self.healthy_started is None:
                    self.healthy_started = now
                self.healthy_streak += 1
                if (
                    self.healthy_streak >= self.config.healthy_recovery_samples
                    and now - self.healthy_started >= self.config.healthy_recovery_duration
                ):
                    self.state = NORMAL
                    self.healthy_streak = 0
                    self.healthy_started = None

    def _handle_stopping(self, event: SupervisorInput, assessment: _HealthAssessment) -> SupervisorOutput:
        if event.health is None or event.health.odom_fresh is not True:
            self._fail("ODOM_STALE_OR_MISSING")
            return self._output(event, assessment, ("ODOM_STALE_OR_MISSING",))
        linear = event.health.odom_linear_velocity
        angular = event.health.odom_angular_velocity
        if not _finite(linear) or not _finite(angular):
            self._fail("ODOM_VELOCITY_MISSING")
            return self._output(event, assessment, ("ODOM_VELOCITY_MISSING",))
        if abs(float(linear)) > self.config.stop_velocity or abs(float(angular)) > self.config.stop_velocity:
            return self._output(event, assessment, ("WAITING_FOR_STOP",))
        if not _finite(event.current_yaw):
            self._fail("YAW_MISSING")
            return self._output(event, assessment, ("YAW_MISSING",))
        self._begin_attempt(event.now, float(event.current_yaw), event.health)
        return self._output(event, assessment, ("STOP_CONFIRMED",))

    def _begin_attempt(self, now: float, yaw: float, health: LocalizationHealth | None) -> None:
        self.attempt_id += 1
        self.attempt_started = now
        self.segment_index = 0
        self.segment_started = now
        self.segment_start_yaw = yaw
        self.segment_target_yaw = yaw + self.config.segment_deltas[0]
        self.settle_until = None
        self.waiting_since = None
        self.total_rotation = 0.0
        self.verification_count = 0
        self.last_verify_pose = None
        self.candidate = None
        self.candidate_request_time = None
        self.candidate_requested_for_segment = False
        current_pose = None if health is None else self._pose_from_health(health)
        if self.last_trusted_pose is not None:
            self.seed_source = "LAST_TRUSTED"
            self.seed_pose = self.last_trusted_pose
        elif current_pose is not None:
            self.seed_source = "CURRENT_AMCL"
            self.seed_pose = current_pose
        else:
            self.seed_source = "NONE"
            self.seed_pose = None
        self.state = ACTIVE_SCAN

    def _handle_active_scan(self, event: SupervisorInput, assessment: _HealthAssessment) -> SupervisorOutput:
        if self._attempt_timed_out(event.now):
            self._fail("ATTEMPT_TIMEOUT")
            return self._output(event, assessment, ("ATTEMPT_TIMEOUT",))
        if event.health is None or event.health.odom_fresh is not True:
            self._fail("ODOM_STALE_DURING_ACTIVE_SCAN")
            return self._output(event, assessment, ("ODOM_STALE_DURING_ACTIVE_SCAN",))
        if event.health.scan_fresh is False:
            self._fail("SCAN_STALE_DURING_ACTIVE_SCAN")
            return self._output(event, assessment, ("SCAN_STALE_DURING_ACTIVE_SCAN",))
        if self.candidate is not None:
            if self.candidate.accepted and self._candidate_is_good(self.candidate):
                self.state = VERIFYING
                self.verification_count = 0
                self.last_verify_pose = None
                return self._output(event, assessment, ("CANDIDATE_RECEIVED",))
            return self._candidate_failed(event, assessment, self.candidate.reason or "CANDIDATE_QUALITY_LOW")
        if not _finite(event.current_yaw):
            self._fail("YAW_MISSING_DURING_ACTIVE_SCAN")
            return self._output(event, assessment, ("YAW_MISSING_DURING_ACTIVE_SCAN",))
        if self.settle_until is not None:
            if event.now < self.settle_until:
                return self._output(event, assessment, ("SETTLING",))
            self.settle_until = None
            self.waiting_since = event.now
            self.candidate_requested_for_segment = False
            self.state = WAITING_CANDIDATE
            if event.scan_updated:
                return self._request_candidate(event, assessment)
            return self._output(event, assessment, ("SETTLE_COMPLETE",))
        if self.segment_started is None or self.segment_target_yaw is None:
            self._fail("SEGMENT_NOT_INITIALIZED")
            return self._output(event, assessment, ("SEGMENT_NOT_INITIALIZED",))
        if event.now - self.segment_started > self.config.segment_timeout:
            self._fail("SEGMENT_TIMEOUT")
            return self._output(event, assessment, ("SEGMENT_TIMEOUT",))
        error = _angle_distance(float(event.current_yaw), self.segment_target_yaw)
        if abs(error) <= self.config.yaw_tolerance:
            self.total_rotation += abs(self.config.segment_deltas[self.segment_index])
            self.settle_until = event.now + self.config.settle_time
            return self._output(event, assessment, ("SEGMENT_REACHED",))
        if self.total_rotation + abs(self.config.segment_deltas[self.segment_index]) > self.config.max_total_rotation:
            self._fail("MAX_TOTAL_ROTATION")
            return self._output(event, assessment, ("MAX_TOTAL_ROTATION",))
        angular = math.copysign(self.config.angular_speed, error)
        return self._output(event, assessment, ("ACTIVE_ROTATION",), angular)

    def _handle_waiting_candidate(self, event: SupervisorInput, assessment: _HealthAssessment) -> SupervisorOutput:
        if self._attempt_timed_out(event.now):
            self._fail("ATTEMPT_TIMEOUT")
            return self._output(event, assessment, ("ATTEMPT_TIMEOUT",))
        if event.health is None or event.health.odom_fresh is not True:
            self._fail("ODOM_STALE_WHILE_WAITING")
            return self._output(event, assessment, ("ODOM_STALE_WHILE_WAITING",))
        if event.health.scan_fresh is False:
            self._fail("SCAN_STALE_WHILE_WAITING")
            return self._output(event, assessment, ("SCAN_STALE_WHILE_WAITING",))
        if self.candidate is not None:
            if self.candidate.accepted and self._candidate_is_good(self.candidate):
                self.state = VERIFYING
                self.verification_count = 0
                self.last_verify_pose = None
                return self._output(event, assessment, ("CANDIDATE_RECEIVED",))
            return self._candidate_failed(event, assessment, self.candidate.reason or "CANDIDATE_QUALITY_LOW")
        if self.waiting_since is not None and event.now - self.waiting_since > self.config.segment_timeout:
            self._fail("WAITING_FOR_CANDIDATE_TIMEOUT")
            return self._output(event, assessment, ("WAITING_FOR_CANDIDATE_TIMEOUT",))
        if event.scan_updated and not self.candidate_requested_for_segment:
            return self._request_candidate(event, assessment)
        return self._output(event, assessment, ("WAITING_FOR_CANDIDATE",))

    def _handle_verifying(self, event: SupervisorInput, assessment: _HealthAssessment) -> SupervisorOutput:
        if self._attempt_timed_out(event.now):
            self._fail("VERIFY_ATTEMPT_TIMEOUT")
            return self._output(event, assessment, ("VERIFY_ATTEMPT_TIMEOUT",))
        if self.candidate is None or not self.candidate.accepted or not self._candidate_is_good(self.candidate):
            reason = "CANDIDATE_QUALITY_LOW" if self.candidate is None else self.candidate.reason or "CANDIDATE_QUALITY_LOW"
            return self._verification_failed(event, assessment, reason)
        if event.health is None or event.health.odom_fresh is not True or event.health.scan_fresh is False or event.health.amcl_fresh is False:
            return self._verification_failed(event, assessment, "VERIFICATION_SIGNAL_STALE")
        pose = event.health
        if not all(_finite(value) for value in (pose.pose_x, pose.pose_y, pose.pose_yaw)):
            return self._verification_failed(event, assessment, "VERIFICATION_POSE_MISSING")
        current_pose = (float(pose.pose_x), float(pose.pose_y), float(pose.pose_yaw))
        if self.last_verify_pose is not None:
            position_jump = _distance_2d(self.last_verify_pose[0], self.last_verify_pose[1], current_pose[0], current_pose[1])
            yaw_jump = abs(_angle_distance(self.last_verify_pose[2], current_pose[2]))
            if position_jump is None or position_jump > self.config.max_verify_position_jump or yaw_jump > self.config.max_verify_yaw_jump:
                return self._verification_failed(event, assessment, "VERIFY_POSE_JUMP")
        self.last_verify_pose = current_pose
        self.verification_count += 1
        if self.verification_count >= self.config.verification_samples:
            self.state = RECOVERED
            self.failure_reason = ""
            return self._output(event, assessment, ("RECOVERED",))
        return self._output(event, assessment, ("VERIFICATION_PENDING",))

    def _verification_failed(self, event: SupervisorInput, assessment: _HealthAssessment, reason: str) -> SupervisorOutput:
        self.failure_reason = reason
        self.verification_count = 0
        self.last_verify_pose = None
        self.candidate = None
        if self.segment_index + 1 < len(self.config.segment_deltas):
            self.segment_index += 1
            if _finite(event.current_yaw):
                self.segment_started = event.now
                self.segment_start_yaw = float(event.current_yaw)
                self.segment_target_yaw = self.segment_start_yaw + self.config.segment_deltas[self.segment_index]
                self.candidate_requested_for_segment = False
                self.state = ACTIVE_SCAN
                return self._output(event, assessment, (reason, "RETURN_TO_ACTIVE_SCAN"))
        self._fail(reason)
        return self._output(event, assessment, (reason,))

    def _candidate_is_good(self, candidate: CandidateQuality) -> bool:
        return (
            all(_finite(value) for value in (candidate.timestamp, candidate.score, candidate.inlier_ratio, candidate.mean_distance, candidate.x, candidate.y, candidate.yaw))
            and candidate.score >= self.config.min_scan_match_score
            and candidate.inlier_ratio >= self.config.min_scan_match_inlier_ratio
            and candidate.mean_distance <= self.config.max_scan_match_mean_distance
        )

    def _candidate_is_fresh(self, candidate: CandidateQuality) -> bool:
        if self.candidate_request_time is None:
            return False
        received_time = candidate.received_time if candidate.received_time is not None else candidate.timestamp
        return _finite(received_time) and float(received_time) >= self.candidate_request_time

    @staticmethod
    def _pose_from_health(health: LocalizationHealth) -> tuple[float, float, float] | None:
        if not all(_finite(value) for value in (health.pose_x, health.pose_y, health.pose_yaw)):
            return None
        return float(health.pose_x), float(health.pose_y), float(health.pose_yaw)

    def _request_candidate(self, event: SupervisorInput, assessment: _HealthAssessment) -> SupervisorOutput:
        self.candidate_request_time = event.now
        self.candidate_requested_for_segment = True
        self.candidate = None
        return self._output(event, assessment, ("REQUEST_CANDIDATE",), request_candidate=True)

    def _candidate_failed(self, event: SupervisorInput, assessment: _HealthAssessment, reason: str) -> SupervisorOutput:
        self.failure_reason = reason
        self.candidate = None
        if self.segment_index + 1 < len(self.config.segment_deltas) and _finite(event.current_yaw):
            next_segment = self.segment_index + 1
            if self.total_rotation + abs(self.config.segment_deltas[next_segment]) > self.config.max_total_rotation:
                self._fail("MAX_TOTAL_ROTATION")
                return self._output(event, assessment, ("MAX_TOTAL_ROTATION",))
            self.segment_index = next_segment
            self.segment_started = event.now
            self.segment_start_yaw = float(event.current_yaw)
            self.segment_target_yaw = self.segment_start_yaw + self.config.segment_deltas[self.segment_index]
            self.candidate_request_time = None
            self.candidate_requested_for_segment = False
            self.state = ACTIVE_SCAN
            return self._output(event, assessment, (reason, "RETURN_TO_ACTIVE_SCAN"))
        self._fail(reason)
        return self._output(event, assessment, (reason,))

    def _attempt_timed_out(self, now: float) -> bool:
        return self.attempt_started is not None and now - self.attempt_started > self.config.max_attempt_duration

    def _fail(self, reason: str) -> None:
        self.state = FAILED
        self.failure_reason = reason
        self.settle_until = None
        self.waiting_since = None
        self.segment_started = None
        self.segment_target_yaw = None

    def _output(
        self,
        event: SupervisorInput,
        assessment: _HealthAssessment | None,
        reasons: tuple[str, ...],
        angular: float = 0.0,
        request_candidate: bool = False,
    ) -> SupervisorOutput:
        elapsed = 0.0 if self.attempt_started is None else max(0.0, event.now - self.attempt_started)
        candidate = self.candidate
        assessment = assessment or _HealthAssessment(False, False, (), "UNKNOWN", "UNKNOWN", "UNKNOWN")
        if self.state in {NORMAL, SUSPECTED, TRIGGERED, STOPPING, WAITING_CANDIDATE, VERIFYING, RECOVERED, FAILED, MANUAL_REQUIRED}:
            angular = 0.0
        if self.state in {FAILED, MANUAL_REQUIRED, RECOVERED}:
            angular = 0.0
        return SupervisorOutput(
            state=self.state,
            attempt_id=self.attempt_id,
            trigger_reason=self.trigger_reason,
            elapsed=elapsed,
            active_segment=max(0, self.segment_index + 1),
            verification_count=self.verification_count,
            command_linear_x=0.0,
            command_angular_z=angular,
            candidate_score=None if candidate is None else candidate.score,
            candidate_inlier_ratio=None if candidate is None else candidate.inlier_ratio,
            candidate_mean_distance=None if candidate is None else candidate.mean_distance,
            lidar_health=assessment.lidar_health,
            gnss_health=assessment.gnss_health,
            amcl_health=assessment.amcl_health,
            failure_reason=self.failure_reason,
            manual_takeover=self._manual_latched,
            health_reasons=tuple(dict.fromkeys((*assessment.reasons, *reasons))),
            request_candidate=request_candidate,
            candidate_request_time=self.candidate_request_time,
            seed_source=self.seed_source,
            seed_pose=self.seed_pose,
        )
