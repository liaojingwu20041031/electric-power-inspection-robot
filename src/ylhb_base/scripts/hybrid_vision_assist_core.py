#!/usr/bin/env python3
"""Deterministic safety gate between HVS observations and MODEL-2."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class HybridVisionAssistConfig:
    input_timeout_sec: float = 0.5
    max_linear_speed_for_uncompensated_mask: float = 0.02
    max_yaw_rate_for_uncompensated_mask: float = 0.03

    def __post_init__(self) -> None:
        values = (
            self.input_timeout_sec,
            self.max_linear_speed_for_uncompensated_mask,
            self.max_yaw_rate_for_uncompensated_mask,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("hybrid vision thresholds must be finite")
        if self.input_timeout_sec <= 0.0 or any(value < 0.0 for value in values[1:]):
            raise ValueError("hybrid vision timeout must be positive and speed gates non-negative")


@dataclass(frozen=True)
class HybridVisionObservation:
    now_sec: float
    camera_quality_received_sec: float | None
    features_received_sec: float | None
    mask_received_sec: float | None
    camera_state: str
    event_feature_count: int
    motion_mask_active_fraction: float
    camera_intrinsics_valid: bool
    lidar_extrinsics_valid: bool
    motion_compensation_ready: bool
    robot_linear_speed: float
    robot_yaw_rate: float


@dataclass(frozen=True)
class HybridVisionDecision:
    state: str
    reason: str
    camera_fresh: bool
    features_fresh: bool
    mask_fresh: bool
    model2_consumes_event_assist: bool
    may_modify_lidar: bool


def _fresh(now_sec: float, received_sec: float | None, timeout_sec: float) -> bool:
    return received_sec is not None and 0.0 <= now_sec - received_sec <= timeout_sec


def evaluate_hybrid_vision_assist(
    observation: HybridVisionObservation,
    config: HybridVisionAssistConfig | None = None,
) -> HybridVisionDecision:
    """Return the only legal event-to-MODEL-2 mode for the current evidence.

    `STATIC_IDLE` is a valid camera state. A single event-camera fault only disables
    this side channel and never requests MODEL-3 recovery.
    """

    config = config or HybridVisionAssistConfig()
    if observation.event_feature_count < 0:
        raise ValueError("event_feature_count must be non-negative")
    numeric = (
        observation.now_sec,
        observation.motion_mask_active_fraction,
        observation.robot_linear_speed,
        observation.robot_yaw_rate,
    )
    if any(not math.isfinite(value) for value in numeric):
        raise ValueError("hybrid vision observations must be finite")
    if not 0.0 <= observation.motion_mask_active_fraction <= 1.0:
        raise ValueError("motion_mask_active_fraction must be within [0,1]")

    camera_fresh = _fresh(
        observation.now_sec, observation.camera_quality_received_sec, config.input_timeout_sec
    )
    features_fresh = _fresh(
        observation.now_sec, observation.features_received_sec, config.input_timeout_sec
    )
    mask_fresh = _fresh(
        observation.now_sec, observation.mask_received_sec, config.input_timeout_sec
    )
    base = dict(camera_fresh=camera_fresh, features_fresh=features_fresh, mask_fresh=mask_fresh)

    if not camera_fresh:
        return HybridVisionDecision("UNAVAILABLE", "CAMERA_QUALITY_STALE", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if observation.camera_state == "FAULT":
        return HybridVisionDecision("UNAVAILABLE", "CAMERA_FAULT", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if observation.camera_state == "DEGRADED":
        return HybridVisionDecision("DIAGNOSTIC_ONLY", "CAMERA_DEGRADED", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if observation.camera_state not in {"GOOD", "STATIC_IDLE"}:
        return HybridVisionDecision("DIAGNOSTIC_ONLY", "CAMERA_STATE_UNKNOWN", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if not observation.camera_intrinsics_valid or not observation.lidar_extrinsics_valid:
        return HybridVisionDecision("DIAGNOSTIC_ONLY", "CALIBRATION_REQUIRED", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if not features_fresh or not mask_fresh:
        return HybridVisionDecision("DIAGNOSTIC_ONLY", "VISION_INPUT_STALE", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    moving = (
        abs(observation.robot_linear_speed) > config.max_linear_speed_for_uncompensated_mask
        or abs(observation.robot_yaw_rate) > config.max_yaw_rate_for_uncompensated_mask
    )
    if moving and not observation.motion_compensation_ready:
        return HybridVisionDecision("DIAGNOSTIC_ONLY", "SELF_MOTION_COMPENSATION_REQUIRED", **base,
                                    model2_consumes_event_assist=False, may_modify_lidar=False)
    if observation.camera_state == "STATIC_IDLE" or observation.motion_mask_active_fraction == 0.0:
        return HybridVisionDecision("READY_IDLE", "NO_DYNAMIC_EVENT_REGION", **base,
                                    model2_consumes_event_assist=True, may_modify_lidar=False)
    return HybridVisionDecision("SOFT_ASSIST_READY", "CALIBRATED_DYNAMIC_REGION", **base,
                                model2_consumes_event_assist=True, may_modify_lidar=True)
