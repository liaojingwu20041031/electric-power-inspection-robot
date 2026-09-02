#!/usr/bin/env python3
"""UWB quality gate with state machine for MODEL-1 integration.

State machine:
- INITIAL → GOOD (first valid observation)
- GOOD → DEGRADED (quality issues but not critical)
- GOOD/DEGRADED → REJECTED (critical failures)
- REJECTED → RECOVERING (quality returns but hysteresis required)
- RECOVERING → GOOD (after N consecutive good frames)
"""

from __future__ import annotations
import math
import sys
sys.path.insert(0, 'C:/Users/peng/dg202611_stage')

from dataclasses import dataclass, replace
from typing import Optional, List
from uwb_data_model import (
    UwbRangeObservation,
    UwbAnchorConfig,
    Uwb2DPositionEstimate,
    UwbQualityState,
)
from uwb_2d_estimator import estimate_2d_position


@dataclass(frozen=True)
class UwbQualityConfig:
    """Configuration for UWB quality gate thresholds."""
    # Freshness
    freshness_timeout: float = 0.5  # seconds

    # Range validity
    min_range_m: float = 0.3
    max_range_m: float = 50.0
    max_range_jump_m: float = 2.0
    max_range_rate_mps: float = 5.0  # max radial velocity

    # Geometry
    min_unique_physical_anchors: int = 3
    min_geometry_score: float = 0.1

    # Position quality
    max_residual_rms_m: float = 1.0
    degraded_residual_rms_m: float = 0.5
    max_position_jump_m: float = 3.0
    max_position_speed_mps: float = 3.0

    # Innovation (vs MODEL-1 prediction)
    max_innovation_m: float = 5.0

    # Recovery hysteresis
    recovery_good_frames: int = 3

    # Default uncertainty
    default_position_sigma: float = 0.5


class UwbQualityGate:
    """Quality gate for UWB observations with recovery hysteresis."""

    def __init__(self, config: UwbQualityConfig, anchors: List[UwbAnchorConfig]):
        self.config = config
        self.anchors = anchors
        self.state = UwbQualityState()
        self.state.state = UwbQualityState.INITIAL

        # History for jump detection
        self.last_ranges: dict[int, float] = {}  # physical_source_id -> range_m
        self.last_range_time: dict[int, float] = {}
        self.last_position: Optional[tuple[float, float]] = None
        self.last_position_time: Optional[float] = None

    def update(
        self,
        observations: List[UwbRangeObservation],
        now: float,
        model1_prediction: Optional[tuple[float, float]] = None,
    ) -> tuple[Optional[Uwb2DPositionEstimate], str]:
        """
        Process UWB observations and return position estimate + decision.

        Returns:
            (position_estimate, decision)
            decision: "ACCEPT", "ACCEPT_DEGRADED", "REJECT", "HOLD_FOR_RECOVERY"
        """
        reasons = []

        # Check freshness
        if not observations:
            self.state.rejection_reason = "NO_OBSERVATIONS"
            self._transition_to_rejected()
            return None, "REJECT"

        latest_time = max(obs.timestamp for obs in observations if obs.valid)
        age = now - latest_time

        if age > self.config.freshness_timeout:
            self.state.rejection_reason = f"STALE_{age:.2f}s"
            self._transition_to_rejected()
            return None, "REJECT"

        self.state.last_update_time = now

        # Deduplicate by physical source
        unique_obs = {}
        for obs in observations:
            if obs.valid and obs.physical_source_id not in unique_obs:
                unique_obs[obs.physical_source_id] = obs

        unique_count = len(unique_obs)
        self.state.unique_physical_count = unique_count

        if unique_count < self.config.min_unique_physical_anchors:
            reasons.append(f"INSUFFICIENT_ANCHORS_{unique_count}")
            self.state.rejection_reason = reasons[0]
            self._transition_to_rejected()
            return None, "REJECT"

        # Check range validity and jumps
        for phys_id, obs in unique_obs.items():
            # Range limits
            if not (self.config.min_range_m <= obs.range_m <= self.config.max_range_m):
                reasons.append(f"RANGE_OUT_OF_BOUNDS_A{phys_id}_{obs.range_m:.2f}m")
                self.state.rejection_reason = reasons[-1]
                self._transition_to_rejected()
                return None, "REJECT"

            # Range jump
            if phys_id in self.last_ranges:
                delta_range = abs(obs.range_m - self.last_ranges[phys_id])
                if delta_range > self.config.max_range_jump_m:
                    reasons.append(f"RANGE_JUMP_A{phys_id}_{delta_range:.2f}m")
                    self.state.rejection_reason = reasons[-1]
                    self._transition_to_rejected()
                    return None, "REJECT"

                # Range rate
                if phys_id in self.last_range_time:
                    dt = obs.timestamp - self.last_range_time[phys_id]
                    if dt > 1e-6:
                        range_rate = abs(delta_range / dt)
                        if range_rate > self.config.max_range_rate_mps:
                            reasons.append(f"RANGE_RATE_A{phys_id}_{range_rate:.1f}mps")
                            self.state.rejection_reason = reasons[-1]
                            self._transition_to_rejected()
                            return None, "REJECT"

            # Update history
            self.last_ranges[phys_id] = obs.range_m
            self.last_range_time[phys_id] = obs.timestamp

        # Estimate position
        position_est = estimate_2d_position(observations, self.anchors, self.config.min_unique_physical_anchors)

        if position_est is None:
            reasons.append("POSITION_ESTIMATION_FAILED")
            self.state.rejection_reason = reasons[-1]
            self._transition_to_rejected()
            return None, "REJECT"

        # Check geometry
        if position_est.geometry_metric < self.config.min_geometry_score:
            reasons.append(f"GEOMETRY_POOR_{position_est.geometry_metric:.3f}")
            self.state.rejection_reason = reasons[-1]
            self._transition_to_rejected()
            return None, "REJECT"

        self.state.geometry_metric = position_est.geometry_metric
        self.state.residual_rms = position_est.residual_rms

        # Check residual
        if position_est.residual_rms > self.config.max_residual_rms_m:
            reasons.append(f"RESIDUAL_HIGH_{position_est.residual_rms:.3f}m")
            self.state.rejection_reason = reasons[-1]
            self._transition_to_rejected()
            return None, "REJECT"

        # Check position jump
        if self.last_position is not None and self.last_position_time is not None:
            dx = position_est.x - self.last_position[0]
            dy = position_est.y - self.last_position[1]
            position_jump = math.sqrt(dx*dx + dy*dy)
            self.state.position_jump = position_jump

            if position_jump > self.config.max_position_jump_m:
                reasons.append(f"POSITION_JUMP_{position_jump:.2f}m")
                self.state.rejection_reason = reasons[-1]
                self._transition_to_rejected()
                return None, "REJECT"

            # Check implied speed
            dt = position_est.timestamp - self.last_position_time
            if dt > 1e-6:
                speed = position_jump / dt
                if speed > self.config.max_position_speed_mps:
                    reasons.append(f"IMPLIED_SPEED_{speed:.1f}mps")
                    self.state.rejection_reason = reasons[-1]
                    self._transition_to_rejected()
                    return None, "REJECT"

        # Check innovation (vs MODEL-1 prediction)
        if model1_prediction is not None:
            dx = position_est.x - model1_prediction[0]
            dy = position_est.y - model1_prediction[1]
            innovation = math.sqrt(dx*dx + dy*dy)
            self.state.innovation = innovation

            if innovation > self.config.max_innovation_m:
                reasons.append(f"INNOVATION_HIGH_{innovation:.2f}m")
                # Don't reject, but degrade
                if self.state.state == UwbQualityState.GOOD:
                    self.state.state = UwbQualityState.DEGRADED
                    self.state.rejection_reason = reasons[-1]

        # Update position history
        self.last_position = (position_est.x, position_est.y)
        self.last_position_time = position_est.timestamp

        # State machine logic
        decision = self._evaluate_state_transition(position_est, reasons)

        return position_est, decision

    def _evaluate_state_transition(
        self,
        position_est: Uwb2DPositionEstimate,
        reasons: List[str],
    ) -> str:
        """Evaluate state transitions and return decision."""
        current_state = self.state.state

        # Determine if current observation is "good"
        is_good_observation = (
            position_est.residual_rms <= self.config.degraded_residual_rms_m
            and position_est.geometry_metric >= self.config.min_geometry_score
            and not reasons
        )

        is_degraded_observation = (
            position_est.residual_rms <= self.config.max_residual_rms_m
            and position_est.geometry_metric >= self.config.min_geometry_score
            and not reasons
        )

        if current_state == UwbQualityState.INITIAL:
            if is_good_observation:
                self.state.state = UwbQualityState.GOOD
                self.state.consecutive_good_frames = 1
                return "ACCEPT"
            elif is_degraded_observation:
                self.state.state = UwbQualityState.DEGRADED
                self.state.consecutive_good_frames = 0
                return "ACCEPT_DEGRADED"
            else:
                return "REJECT"

        elif current_state == UwbQualityState.GOOD:
            if is_good_observation:
                self.state.consecutive_good_frames += 1
                return "ACCEPT"
            elif is_degraded_observation:
                self.state.state = UwbQualityState.DEGRADED
                self.state.consecutive_good_frames = 0
                return "ACCEPT_DEGRADED"
            else:
                # Already handled by earlier checks
                return "REJECT"

        elif current_state == UwbQualityState.DEGRADED:
            if is_good_observation:
                self.state.state = UwbQualityState.GOOD
                self.state.consecutive_good_frames = 1
                return "ACCEPT"
            elif is_degraded_observation:
                return "ACCEPT_DEGRADED"
            else:
                return "REJECT"

        elif current_state == UwbQualityState.REJECTED:
            if is_good_observation:
                self.state.state = UwbQualityState.RECOVERING
                self.state.consecutive_good_frames = 1
                self.state.rejection_reason = ""
                return "HOLD_FOR_RECOVERY"
            else:
                return "REJECT"

        elif current_state == UwbQualityState.RECOVERING:
            if is_good_observation:
                self.state.consecutive_good_frames += 1
                if self.state.consecutive_good_frames >= self.config.recovery_good_frames:
                    self.state.state = UwbQualityState.GOOD
                    self.state.rejection_reason = ""
                    return "ACCEPT"
                else:
                    return "HOLD_FOR_RECOVERY"
            else:
                # Reset recovery attempt
                self.state.state = UwbQualityState.REJECTED
                self.state.consecutive_good_frames = 0
                return "REJECT"

        return "REJECT"

    def _transition_to_rejected(self):
        """Transition to REJECTED state."""
        self.state.state = UwbQualityState.REJECTED
        self.state.consecutive_good_frames = 0

    def get_state_summary(self) -> dict:
        """Get current state as dictionary for logging/diagnostics."""
        return {
            "state": self.state.state,
            "unique_physical_count": self.state.unique_physical_count,
            "geometry_metric": self.state.geometry_metric,
            "residual_rms": self.state.residual_rms,
            "position_jump": self.state.position_jump,
            "innovation": self.state.innovation,
            "consecutive_good_frames": self.state.consecutive_good_frames,
            "rejection_reason": self.state.rejection_reason,
        }


if __name__ == "__main__":
    from uwb_data_model import SYNTHETIC_ANCHOR_COORDINATES

    print("Test: UWB Quality Gate")

    config = UwbQualityConfig()
    gate = UwbQualityGate(config, SYNTHETIC_ANCHOR_COORDINATES)

    # Test 1: Normal observations
    test_obs = [
        UwbRangeObservation(1.0, "A0", 0, 0, 5.0, True),
        UwbRangeObservation(1.0, "A1", 1, 1, 7.0, True),
        UwbRangeObservation(1.0, "A2", 2, 2, 6.0, True),
    ]

    pos, decision = gate.update(test_obs, 1.0)
    print(f"\nTest 1 - Normal: decision={decision}, state={gate.state.state}")
    if pos:
        print(f"  Position: ({pos.x:.2f}, {pos.y:.2f}), residual={pos.residual_rms:.3f}m")
    print(f"  Summary: {gate.get_state_summary()}")

    # Test 2: Range jump
    test_obs_jump = [
        UwbRangeObservation(1.1, "A0", 0, 0, 10.0, True),  # Big jump from 5.0
        UwbRangeObservation(1.1, "A1", 1, 1, 7.0, True),
        UwbRangeObservation(1.1, "A2", 2, 2, 6.0, True),
    ]

    pos, decision = gate.update(test_obs_jump, 1.1)
    print(f"\nTest 2 - Range Jump: decision={decision}, state={gate.state.state}")
    print(f"  Rejection reason: {gate.state.rejection_reason}")

    # Test 3: Recovery (good observations after rejection)
    for i in range(config.recovery_good_frames + 1):
        pos, decision = gate.update(test_obs, 1.2 + i * 0.1)
        print(f"\nTest 3.{i+1} - Recovery: decision={decision}, state={gate.state.state}, good_frames={gate.state.consecutive_good_frames}")

    print("\n✅ UWB Quality Gate tests complete")
