#!/usr/bin/env python3
"""ROS2 supervisor for the local active-relocalization POC.

The node is deliberately not included in the default bringup.  It observes
the existing AMCL, LiDAR/GNSS quality and Scan-to-Map side channels, and only
publishes a bounded rotation command after the state machine has confirmed a
safe stop.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402
from active_relocalization_core import (  # noqa: E402
    ActiveRelocalizationConfig,
    ActiveRelocalizationController,
    CandidateQuality,
    LocalizationHealth,
    SupervisorInput,
    recovery_motion_gate,
)
from pose_freshness import motion_aware_pose_fresh  # noqa: E402
from mission_activity import (  # noqa: E402
    ACTIVE,
    MissionActivity,
    MissionActivityGate,
    ungated_activity,
)


def _stamp_to_seconds(stamp: Any) -> float | None:
    try:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _value(values: dict[str, str], key: str) -> str | None:
    result = values.get(key)
    return None if result is None or result == "" else result


def _float_value(values: dict[str, str], key: str) -> float | None:
    raw = _value(values, key)
    if raw is None:
        return None
    try:
        result = float(raw)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _int_value(values: dict[str, str], key: str) -> int:
    raw = _value(values, key)
    if raw is None:
        return 0
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        return 0


def _bool_value(values: dict[str, str], key: str, default: bool = False) -> bool:
    raw = _value(values, key)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "accepted", "ok"}


def parse_required_signals(value: Any) -> tuple[str, ...]:
    """Normalize a ROS string-array parameter for the ROS-free core."""
    if isinstance(value, str):
        value = (value,)
    return tuple(str(item).strip() for item in (value or ()) if str(item).strip())


def parse_match_quality_values(
    values: dict[str, str],
    received_time: float,
    default_reason: str = "",
) -> CandidateQuality:
    """Convert a match diagnostic into a candidate, preserving failures.

    Failure diagnostics intentionally use finite sentinel values because ROS
    diagnostic fields are strings but the core uses numeric quality fields.
    An accepted diagnostic with a non-finite/missing quality is converted into
    an explicit rejected candidate instead of being silently discarded.
    """
    accepted = _bool_value(values, "accepted", default=True)
    reason = _value(values, "reason") or default_reason
    timestamp = _float_value(values, "timestamp")
    score = _float_value(values, "score")
    inlier_ratio = _float_value(values, "inlier_ratio")
    mean_distance = _float_value(values, "mean_distance")
    x = _float_value(values, "candidate_x")
    y = _float_value(values, "candidate_y")
    yaw = _float_value(values, "candidate_yaw")

    if accepted and None in (score, inlier_ratio, mean_distance, x, y, yaw):
        accepted = False
        reason = reason or "NONFINITE_MATCH_QUALITY"

    return CandidateQuality(
        timestamp=timestamp if timestamp is not None else received_time,
        score=score if score is not None else 0.0,
        inlier_ratio=inlier_ratio if inlier_ratio is not None else 0.0,
        mean_distance=mean_distance if mean_distance is not None else 1_000_000.0,
        x=x if x is not None else 0.0,
        y=y if y is not None else 0.0,
        yaw=yaw if yaw is not None else 0.0,
        used_points=_int_value(values, "used_points"),
        accepted=accepted,
        reason=reason or ("ACCEPTED" if accepted else "MATCH_REJECTED"),
        received_time=received_time,
    )


def _quaternion_yaw(orientation: Any) -> float | None:
    try:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        result = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    except (AttributeError, TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the DG-202611 active relocalization supervisor.")
    parser.add_argument("--node-name", default="active_relocalization_node", help="ROS node name")
    return parser


def create_node(node_name: str) -> Any:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String

    class ActiveRelocalizationNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self._declare_parameters()
            self._config = self._read_config()
            self._controller = ActiveRelocalizationController(self._config)
            self._mission_gate_enabled = bool(self.get_parameter("mission_gate_enabled").value)
            self._mission_gate = MissionActivityGate(
                status_timeout_sec=float(self.get_parameter("mission_status_timeout_sec").value),
                activation_grace_sec=float(self.get_parameter("mission_activation_grace_sec").value),
            )
            self._mission_status: dict[str, Any] | None = None
            self._mission_status_received_at: float | None = None
            self._mission_phase: str | None = None
            self._last_failure_reason = ""
            self._last_tick = self._now()
            self._scan_received_at: float | None = None
            self._scan_sequence = 0
            self._consumed_scan_sequence = 0
            self._odom_received_at: float | None = None
            self._imu_received_at: float | None = None
            self._odom_pose: tuple[float, float, float] | None = None
            self._odom_yaw: float | None = None
            self._odom_linear = 0.0
            self._odom_angular = 0.0
            self._amcl_received_at: float | None = None
            self._amcl_reference_odom_pose: tuple[float, float, float] | None = None
            self._amcl_covariance: float | None = None
            self._amcl_pose: tuple[float, float, float] | None = None
            self._lidar_quality: float | None = None
            self._lidar_health_received_at: float | None = None
            self._gnss_quality: str | None = None
            self._gnss_health_received_at: float | None = None
            self._candidate: CandidateQuality | None = None
            self._candidate_received_at: float | None = None
            self._candidate_sequence = 0
            self._consumed_candidate_sequence = 0
            self._scan_match_score: float | None = None
            self._scan_match_inlier_ratio: float | None = None
            self._scan_match_mean_distance: float | None = None
            self._match_quality_received_at: float | None = None
            self._manual_takeover = False
            self._last_motion_gate_reason = ""

            self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
            self._seed_pub = self.create_publisher(
                PoseWithCovarianceStamped, str(self.get_parameter("seed_topic").value), 10
            )
            self._status_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("status_topic").value), 10
            )
            self.create_subscription(
                LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, 10
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
            )
            self.create_subscription(
                Imu, str(self.get_parameter("imu_topic").value), self._on_imu, 10
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("amcl_pose_topic").value),
                self._on_amcl_pose,
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("lidar_quality_topic").value),
                self._on_lidar_quality,
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("gnss_quality_topic").value),
                self._on_gnss_quality,
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("match_quality_topic").value),
                self._on_match_quality,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("manual_topic").value),
                self._on_manual_takeover,
                10,
            )
            if self._mission_gate_enabled:
                self.create_subscription(
                    String,
                    str(self.get_parameter("system_status_topic").value),
                    self._on_system_status,
                    10,
                )
            timer_period = float(self.get_parameter("timer_period").value)
            self._timer = self.create_timer(timer_period, self._tick)
            self.get_logger().warn(
                "active relocalization POC is a manual side-channel node; default bringup is unchanged"
            )

        def _declare_parameters(self) -> None:
            defaults = {
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "imu_topic": "/imu/data",
                "amcl_pose_topic": "/amcl_pose",
                "lidar_quality_topic": "/dg/lidar/quality",
                "gnss_quality_topic": "/dg/gnss/quality",
                "match_quality_topic": "/dg/relocalization/match_quality",
                # Declare this as a ROS string array; an empty Python list is
                # inferred as BYTE_ARRAY by rclpy on Humble.
                "required_signals": [""],
                "seed_topic": "/dg/relocalization/seed",
                "seed_frame_id": "map",
                "manual_topic": "/dg/relocalization/manual_takeover",
                "cmd_vel_topic": "/cmd_vel_recovery",
                "status_topic": "/dg/relocalization/status",
                "system_status_topic": "/inspection_ai/system_status",
                "mission_gate_enabled": False,
                "mission_status_timeout_sec": 3.0,
                "mission_activation_grace_sec": 2.0,
                "recovery_motion_armed": False,
                "timer_period": 0.1,
                "suspect_samples": 2,
                "trigger_samples": 2,
                "healthy_recovery_samples": 3,
                "suspect_duration": 0.0,
                "trigger_duration": 0.0,
                "healthy_recovery_duration": 0.0,
                "max_covariance": 0.5,
                "min_lidar_quality": 0.2,
                "min_scan_match_score": 0.45,
                "min_scan_match_inlier_ratio": 0.45,
                "max_scan_match_mean_distance": 0.30,
                "signal_timeout": 1.0,
                "amcl_motion_translation_tolerance": 0.03,
                "amcl_motion_rotation_tolerance": 0.03,
                "stop_velocity": 0.03,
                "angular_speed": 0.18,
                "yaw_tolerance": math.radians(1.5),
                "segment_timeout": 8.0,
                "settle_time": 0.5,
                "segment_deltas_deg": [10.0, -20.0, 10.0],
                "max_total_rotation": math.radians(60.0),
                "max_attempt_duration": 30.0,
                "max_attempts": 2,
                "verification_samples": 3,
                "max_verify_position_jump": 0.5,
                "max_verify_yaw_jump": math.radians(20.0),
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)

        def _read_config(self) -> ActiveRelocalizationConfig:
            parameter = self.get_parameter
            deltas = tuple(math.radians(float(value)) for value in parameter("segment_deltas_deg").value)
            fields = {
                "suspect_samples": int(parameter("suspect_samples").value),
                "trigger_samples": int(parameter("trigger_samples").value),
                "healthy_recovery_samples": int(parameter("healthy_recovery_samples").value),
                "suspect_duration": float(parameter("suspect_duration").value),
                "trigger_duration": float(parameter("trigger_duration").value),
                "healthy_recovery_duration": float(parameter("healthy_recovery_duration").value),
                "required_signals": parse_required_signals(parameter("required_signals").value),
                "max_covariance": float(parameter("max_covariance").value),
                "min_lidar_quality": float(parameter("min_lidar_quality").value),
                "min_scan_match_score": float(parameter("min_scan_match_score").value),
                "min_scan_match_inlier_ratio": float(parameter("min_scan_match_inlier_ratio").value),
                "max_scan_match_mean_distance": float(parameter("max_scan_match_mean_distance").value),
                "signal_timeout": float(parameter("signal_timeout").value),
                "stop_velocity": float(parameter("stop_velocity").value),
                "angular_speed": float(parameter("angular_speed").value),
                "yaw_tolerance": float(parameter("yaw_tolerance").value),
                "segment_timeout": float(parameter("segment_timeout").value),
                "settle_time": float(parameter("settle_time").value),
                "segment_deltas": deltas,
                "max_total_rotation": float(parameter("max_total_rotation").value),
                "max_attempt_duration": float(parameter("max_attempt_duration").value),
                "max_attempts": int(parameter("max_attempts").value),
                "verification_samples": int(parameter("verification_samples").value),
                "max_verify_position_jump": float(parameter("max_verify_position_jump").value),
                "max_verify_yaw_jump": float(parameter("max_verify_yaw_jump").value),
            }
            return ActiveRelocalizationConfig(**fields)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _fresh(self, received_at: float | None, now: float) -> bool | None:
            if received_at is None:
                return None
            return now - received_at <= self._config.signal_timeout

        def _on_scan(self, _message: Any) -> None:
            self._scan_received_at = self._now()
            self._scan_sequence += 1

        def _on_odom(self, message: Any) -> None:
            self._odom_received_at = self._now()
            self._odom_yaw = _quaternion_yaw(message.pose.pose.orientation)
            try:
                position = message.pose.pose.position
                self._odom_linear = float(message.twist.twist.linear.x)
                self._odom_angular = float(message.twist.twist.angular.z)
                if self._odom_yaw is not None:
                    self._odom_pose = (float(position.x), float(position.y), self._odom_yaw)
            except (AttributeError, TypeError, ValueError):
                self._odom_linear = float("nan")
                self._odom_angular = float("nan")
            if self._amcl_received_at is not None and self._amcl_reference_odom_pose is None:
                self._amcl_reference_odom_pose = self._odom_pose

        def _on_imu(self, _message: Any) -> None:
            self._imu_received_at = self._now()

        def _on_amcl_pose(self, message: Any) -> None:
            self._amcl_received_at = self._now()
            self._amcl_reference_odom_pose = self._odom_pose
            try:
                self._amcl_covariance = float(message.pose.covariance[0])
                pose = message.pose.pose
                yaw = _quaternion_yaw(pose.orientation)
                self._amcl_pose = (float(pose.position.x), float(pose.position.y), yaw) if yaw is not None else None
            except (AttributeError, IndexError, TypeError, ValueError):
                self._amcl_covariance = None
                self._amcl_pose = None

        def _on_lidar_quality(self, message: Any) -> None:
            selected = self._select_status(message, "LIDAR")
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._lidar_quality = _float_value(values, "geometry_score")
            if self._lidar_quality is None:
                self._lidar_quality = _float_value(values, "valid_ratio")
            self._lidar_health_received_at = self._now()

        def _on_gnss_quality(self, message: Any) -> None:
            selected = self._select_status(message, "GNSS")
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._gnss_quality = _value(values, "current_state") or _value(values, "decision") or "UNKNOWN"
            self._gnss_health_received_at = self._now()

        def _on_match_quality(self, message: Any) -> None:
            selected = self._select_status(message, "MATCH")
            if selected is None:
                return
            received_time = self._now()
            self._match_quality_received_at = received_time
            self._scan_match_score = None
            self._scan_match_inlier_ratio = None
            self._scan_match_mean_distance = None
            values = {str(item.key): str(item.value) for item in selected.values}
            parsed = parse_match_quality_values(values, received_time, str(selected.message))
            self._scan_match_score = parsed.score
            self._scan_match_inlier_ratio = parsed.inlier_ratio
            self._scan_match_mean_distance = parsed.mean_distance
            self._candidate = parsed
            self._candidate_received_at = received_time
            self._candidate_sequence += 1

        @staticmethod
        def _select_status(message: Any, hint: str) -> Any | None:
            selected = None
            for status in message.status:
                if selected is None:
                    selected = status
                if hint in str(status.name).upper():
                    return status
            return selected

        def _on_manual_takeover(self, message: Any) -> None:
            if bool(message.data):
                self._manual_takeover = True
                self._publish_zero()
                self._tick()

        def _on_system_status(self, message: Any) -> None:
            try:
                payload = json.loads(str(message.data))
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            self._mission_status = payload
            self._mission_status_received_at = self._now()

        def _reset_runtime_state(self) -> None:
            self._controller = ActiveRelocalizationController(self._config)
            self._scan_received_at = None
            self._scan_sequence = 0
            self._consumed_scan_sequence = 0
            self._odom_received_at = None
            self._imu_received_at = None
            self._odom_pose = None
            self._odom_yaw = None
            self._odom_linear = 0.0
            self._odom_angular = 0.0
            self._amcl_received_at = None
            self._amcl_reference_odom_pose = None
            self._amcl_covariance = None
            self._amcl_pose = None
            self._lidar_quality = None
            self._lidar_health_received_at = None
            self._gnss_quality = None
            self._gnss_health_received_at = None
            self._candidate = None
            self._candidate_received_at = None
            self._candidate_sequence = 0
            self._consumed_candidate_sequence = 0
            self._scan_match_score = None
            self._scan_match_inlier_ratio = None
            self._scan_match_mean_distance = None
            self._match_quality_received_at = None
            self._manual_takeover = False

        def _mission_activity(self, now: float) -> MissionActivity:
            if not self._mission_gate_enabled:
                return ungated_activity()
            return self._mission_gate.evaluate(
                self._mission_status, self._mission_status_received_at, now
            )

        def _apply_mission_transition(self, activity: MissionActivity) -> None:
            previous = self._mission_phase
            if activity.phase == previous:
                return
            if previous is None:
                self._mission_phase = activity.phase
                return
            # Warmup -> active intentionally keeps observations gathered during
            # the grace interval.  Every other phase edge starts from a clean
            # controller so terminal states cannot leak into another mission.
            if not (previous == "WARMUP" and activity.phase == ACTIVE):
                self._reset_runtime_state()
            self._mission_phase = activity.phase

        def _health(self, now: float) -> LocalizationHealth:
            pose = self._amcl_pose
            return LocalizationHealth(
                timestamp=now,
                amcl_covariance=self._amcl_covariance,
                lidar_quality=self._lidar_quality,
                gnss_quality=self._gnss_quality,
                scan_match_score=self._scan_match_score,
                scan_match_inlier_ratio=self._scan_match_inlier_ratio,
                scan_match_mean_distance=self._scan_match_mean_distance,
                scan_fresh=self._fresh(self._scan_received_at, now),
                odom_fresh=self._fresh(self._odom_received_at, now),
                imu_fresh=self._fresh(self._imu_received_at, now),
                amcl_fresh=self._amcl_fresh(now),
                lidar_fresh=self._fresh(self._lidar_health_received_at, now),
                gnss_fresh=self._fresh(self._gnss_health_received_at, now),
                scan_match_fresh=self._fresh(self._match_quality_received_at, now),
                odom_linear_velocity=self._odom_linear,
                odom_angular_velocity=self._odom_angular,
                pose_x=None if pose is None else pose[0],
                pose_y=None if pose is None else pose[1],
                pose_yaw=None if pose is None else pose[2],
            )

        def _amcl_fresh(self, now: float) -> bool | None:
            return motion_aware_pose_fresh(
                self._fresh(self._amcl_received_at, now),
                self._amcl_reference_odom_pose,
                self._odom_pose,
                float(self.get_parameter("amcl_motion_translation_tolerance").value),
                float(self.get_parameter("amcl_motion_rotation_tolerance").value),
            )

        def _tick(self) -> None:
            now = self._now()
            activity = self._mission_activity(now)
            self._apply_mission_transition(activity)
            if not activity.mission_active:
                self._publish_zero()
                self._publish_gate_status(activity, now)
                self._last_tick = now
                return
            scan_updated = self._scan_sequence != self._consumed_scan_sequence
            if scan_updated:
                self._consumed_scan_sequence = self._scan_sequence
            candidate = None
            if self._candidate_sequence != self._consumed_candidate_sequence:
                candidate = self._candidate
                self._consumed_candidate_sequence = self._candidate_sequence
            event = SupervisorInput(
                now=now,
                health=self._health(now),
                current_yaw=self._odom_yaw,
                scan_updated=scan_updated,
                candidate=candidate,
                manual_takeover=self._manual_takeover,
            )
            output = self._controller.process(event)
            if output.failure_reason:
                self._last_failure_reason = str(output.failure_reason)
            command_is_nonzero = (
                abs(float(output.command_linear_x)) > 1e-9
                or abs(float(output.command_angular_z)) > 1e-9
            )
            allowed, gate_reason = recovery_motion_gate(
                bool(self.get_parameter("recovery_motion_armed").value), event.health
            )
            if command_is_nonzero and not allowed:
                self._last_motion_gate_reason = gate_reason
                self._publish_zero()
            else:
                self._last_motion_gate_reason = ""
                self._publish_command(output.command_linear_x, output.command_angular_z)
            if output.request_candidate:
                self._publish_seed(output.seed_pose)
            self._publish_status(output, now, activity)
            self._last_tick = now

        def _publish_zero(self) -> None:
            self._publish_command(0.0, 0.0)

        def _publish_command(self, linear: float, angular: float) -> None:
            message = Twist()
            message.linear.x = float(linear)
            message.angular.z = float(angular)
            self._cmd_pub.publish(message)

        def _publish_seed(self, pose: tuple[float, float, float] | None) -> None:
            if pose is None:
                self.get_logger().error("candidate requested but no valid seed pose is available")
                return
            message = PoseWithCovarianceStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(self.get_parameter("seed_frame_id").value)
            message.pose.pose.position.x = pose[0]
            message.pose.pose.position.y = pose[1]
            message.pose.pose.position.z = 0.0
            message.pose.pose.orientation.z = math.sin(pose[2] * 0.5)
            message.pose.pose.orientation.w = math.cos(pose[2] * 0.5)
            message.pose.covariance[0] = 0.5
            message.pose.covariance[7] = 0.5
            message.pose.covariance[35] = math.radians(20.0) ** 2
            self._seed_pub.publish(message)

        def _publish_gate_status(self, activity: MissionActivity, now: float) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "DG Active Relocalization Supervisor"
            status.level = (
                DiagnosticStatus.WARN
                if activity.phase == "WARMUP" or not activity.status_fresh
                else DiagnosticStatus.OK
            )
            status.message = activity.phase
            fields = {
                "state": activity.phase,
                "mission_phase": activity.phase,
                "mission_active": activity.mission_active,
                "mission_status_fresh": activity.status_fresh,
                "patrol_state": activity.patrol_state,
                "standby_reason": activity.standby_reason,
                "missing_components": ";".join(activity.missing_components),
                "last_terminal_state": activity.terminal_state,
                "last_failure_reason": self._last_failure_reason,
                "recovery_motion_armed": bool(self.get_parameter("recovery_motion_armed").value),
                "motion_gate_reason": self._last_motion_gate_reason,
                "command_angular_z": 0.0,
                "timestamp": now,
            }
            status.values = [KeyValue(key=str(key), value=str(value)) for key, value in fields.items()]
            array.status = [status]
            self._status_pub.publish(array)

        def _publish_status(self, output: Any, now: float, activity: MissionActivity) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "DG Active Relocalization Supervisor"
            if output.state in {"NORMAL", "RECOVERED"}:
                status.level = DiagnosticStatus.OK
            elif output.state in {"SUSPECTED", "ACTIVE_SCAN", "WAITING_CANDIDATE", "VERIFYING", "STOPPING"}:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR
            status.message = output.state
            fields = {
                "state": output.state,
                "attempt_id": output.attempt_id,
                "trigger_reason": output.trigger_reason,
                "elapsed": output.elapsed,
                "active_segment": output.active_segment,
                "verification_count": output.verification_count,
                "candidate_score": output.candidate_score,
                "candidate_inlier_ratio": output.candidate_inlier_ratio,
                "candidate_mean_distance": output.candidate_mean_distance,
                "lidar_health": output.lidar_health,
                "gnss_health": output.gnss_health,
                "amcl_health": output.amcl_health,
                "failure_reason": output.failure_reason,
                "manual_takeover": output.manual_takeover,
                "request_candidate": output.request_candidate,
                "candidate_request_time": output.candidate_request_time,
                "seed_source": output.seed_source,
                "seed_pose": output.seed_pose,
                "command_angular_z": output.command_angular_z if not self._last_motion_gate_reason else 0.0,
                "requested_command_angular_z": output.command_angular_z,
                "health_reasons": ";".join(output.health_reasons),
                "mission_phase": activity.phase,
                "mission_active": activity.mission_active,
                "mission_status_fresh": activity.status_fresh,
                "patrol_state": activity.patrol_state,
                "standby_reason": activity.standby_reason,
                "last_terminal_state": activity.terminal_state,
                "last_failure_reason": self._last_failure_reason,
                "recovery_motion_armed": bool(self.get_parameter("recovery_motion_armed").value),
                "motion_gate_reason": self._last_motion_gate_reason,
                "timestamp": now,
            }
            status.values = [KeyValue(key=str(key), value="" if value is None else str(value)) for key, value in fields.items()]
            array.status = [status]
            self._status_pub.publish(array)

        def destroy_node(self) -> bool:
            if rclpy.ok():
                self._publish_zero()
            return super().destroy_node()

    return ActiveRelocalizationNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the core is ROS-independent.", file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = create_node(args.node_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cleanup_ros_node(node, rclpy)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
