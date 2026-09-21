#!/usr/bin/env python3
"""ROS2 adapter for the unified DG-202611 navigation health status."""

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
from navigation_health_core import (  # noqa: E402
    NavigationHealthAggregator,
    NavigationHealthInput,
    parse_signal_diagnostic,
)
from pose_freshness import motion_aware_pose_fresh  # noqa: E402
from diagnostic_level import normalize_diagnostic_level  # noqa: E402
from mission_activity import (  # noqa: E402
    ACTIVE,
    MissionActivity,
    MissionActivityGate,
    ungated_activity,
)


def _float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _yaw_from_quaternion(orientation: Any) -> float | None:
    try:
        result = math.atan2(
            2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y)),
            1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _parse_required_signals(value: Any) -> tuple[str, ...]:
    if isinstance(value, str):
        value = (value,)
    return tuple(str(item).strip() for item in (value or ()) if str(item).strip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish unified DG-202611 navigation health.")
    parser.add_argument("--node-name", default="navigation_health_node", help="ROS node name")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import String

    class NavigationHealthNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("lidar_quality_topic", "/dg/lidar/quality")
            self.declare_parameter("gnss_quality_topic", "/dg/gnss/quality")
            self.declare_parameter("match_quality_topic", "/dg/relocalization/match_quality")
            self.declare_parameter("uwb_quality_topic", "/dg/uwb/quality")
            self.declare_parameter("relocalization_status_topic", "/dg/relocalization/status")
            self.declare_parameter("amcl_pose_topic", "/amcl_pose")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("imu_topic", "/imu/data")
            self.declare_parameter("status_topic", "/dg/navigation/status")
            self.declare_parameter("system_status_topic", "/inspection_ai/system_status")
            self.declare_parameter("mission_gate_enabled", False)
            self.declare_parameter("mission_status_timeout_sec", 3.0)
            self.declare_parameter("mission_activation_grace_sec", 2.0)
            self.declare_parameter("recovery_motion_armed", False)
            self.declare_parameter("freshness_timeout", 1.0)
            self.declare_parameter("amcl_motion_translation_tolerance", 0.03)
            self.declare_parameter("amcl_motion_rotation_tolerance", 0.03)
            self.declare_parameter("max_amcl_covariance", 0.5)
            self.declare_parameter("min_lidar_quality", 0.2)
            self.declare_parameter("post_recovery_stable_sec", 1.0)
            # An empty list is inferred as BYTE_ARRAY by rclpy on Humble;
            # use a blank string sentinel for the intended string array.
            self.declare_parameter("required_signals", [""])
            self.declare_parameter("timer_period", 0.1)

            self._timeout = float(self.get_parameter("freshness_timeout").value)
            self._min_lidar_quality = float(self.get_parameter("min_lidar_quality").value)
            self._required_signals = _parse_required_signals(self.get_parameter("required_signals").value)
            self._aggregator = NavigationHealthAggregator(
                max_amcl_covariance=float(self.get_parameter("max_amcl_covariance").value),
                freshness_timeout=self._timeout,
                min_lidar_quality=self._min_lidar_quality,
                post_recovery_stable_sec=float(
                    self.get_parameter("post_recovery_stable_sec").value
                ),
            )
            self._now_value = self._now()
            self._gnss_state: str | None = None
            self._gnss_received_at: float | None = None
            self._lidar_state: str | None = None
            self._lidar_received_at: float | None = None
            self._scan_match_state: str | None = None
            self._scan_match_received_at: float | None = None
            self._uwb_state: str | None = None
            self._uwb_received_at: float | None = None
            self._relocalization_state: str | None = None
            self._relocalization_received_at: float | None = None
            self._amcl_received_at: float | None = None
            self._amcl_reference_odom_pose: tuple[float, float, float] | None = None
            self._amcl_covariance: float | None = None
            self._odom_received_at: float | None = None
            self._odom_pose: tuple[float, float, float] | None = None
            self._scan_received_at: float | None = None
            self._imu_received_at: float | None = None

            self._status_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("status_topic").value), 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("lidar_quality_topic").value),
                lambda message: self._on_quality("lidar", message), 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("gnss_quality_topic").value),
                lambda message: self._on_quality("gnss", message), 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("match_quality_topic").value),
                lambda message: self._on_quality("scan_match", message), 10
            )
            self._mission_gate_enabled = bool(self.get_parameter("mission_gate_enabled").value)
            self._mission_gate = MissionActivityGate(
                status_timeout_sec=float(self.get_parameter("mission_status_timeout_sec").value),
                activation_grace_sec=float(self.get_parameter("mission_activation_grace_sec").value),
            )
            self._mission_status: dict[str, Any] | None = None
            self._mission_status_received_at: float | None = None
            self._mission_phase: str | None = None
            self._last_failure_reason = ""
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("uwb_quality_topic").value),
                lambda message: self._on_quality("uwb", message), 10
            )
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("relocalization_status_topic").value),
                self._on_relocalization, 10
            )
            self.create_subscription(
                PoseWithCovarianceStamped, str(self.get_parameter("amcl_pose_topic").value),
                self._on_amcl, 10
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
            )
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._on_scan,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Imu,
                str(self.get_parameter("imu_topic").value),
                self._on_imu,
                qos_profile_sensor_data,
            )
            if self._mission_gate_enabled:
                self.create_subscription(
                    String,
                    str(self.get_parameter("system_status_topic").value),
                    self._on_system_status,
                    10,
                )
            self._timer = self.create_timer(float(self.get_parameter("timer_period").value), self._tick)
            self.get_logger().info("unified DG-202611 navigation health side-channel active")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _fresh(self, received_at: float | None, now: float) -> bool | None:
            if received_at is None:
                return None
            return now - received_at <= self._timeout

        @staticmethod
        def _status(message: Any, hint: str) -> Any | None:
            selected = None
            for status in message.status:
                if selected is None:
                    selected = status
                if hint in str(status.name).upper():
                    return status
            return selected

        def _on_quality(self, signal: str, message: Any) -> None:
            selected = self._status(message, signal.upper())
            if selected is None:
                return
            received_at = self._now()
            values = {str(item.key): str(item.value) for item in selected.values}
            observation = parse_signal_diagnostic(
                signal,
                values,
                normalize_diagnostic_level(selected.level, default=3),
                received_at,
                received_at,
                self._timeout,
                self._min_lidar_quality,
            )
            if signal == "lidar":
                self._lidar_state, self._lidar_received_at = observation.state, received_at
            elif signal == "gnss":
                self._gnss_state, self._gnss_received_at = observation.state, received_at
            elif signal == "uwb":
                self._uwb_state, self._uwb_received_at = observation.state, received_at
            else:
                self._scan_match_state, self._scan_match_received_at = observation.state, received_at

        def _on_relocalization(self, message: Any) -> None:
            selected = self._status(message, "RELOCALIZATION")
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._relocalization_state = values.get("state") or selected.message
            self._relocalization_received_at = self._now()

        def _on_amcl(self, message: Any) -> None:
            self._amcl_received_at = self._now()
            self._amcl_reference_odom_pose = self._odom_pose
            try:
                self._amcl_covariance = _float_or_none(message.pose.covariance[0])
            except (AttributeError, IndexError, TypeError, ValueError):
                self._amcl_covariance = None

        def _on_odom(self, message: Any) -> None:
            self._odom_received_at = self._now()
            try:
                pose = message.pose.pose
                yaw = _yaw_from_quaternion(pose.orientation)
                self._odom_pose = (float(pose.position.x), float(pose.position.y), yaw) if yaw is not None else None
            except (AttributeError, TypeError, ValueError):
                self._odom_pose = None
            if self._amcl_received_at is not None and self._amcl_reference_odom_pose is None:
                self._amcl_reference_odom_pose = self._odom_pose

        def _on_scan(self, _message: Any) -> None:
            self._scan_received_at = self._now()

        def _on_imu(self, _message: Any) -> None:
            self._imu_received_at = self._now()

        def _on_system_status(self, message: Any) -> None:
            try:
                payload = json.loads(str(message.data))
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            self._mission_status = payload
            self._mission_status_received_at = self._now()

        def _new_aggregator(self) -> NavigationHealthAggregator:
            return NavigationHealthAggregator(
                max_amcl_covariance=float(self.get_parameter("max_amcl_covariance").value),
                freshness_timeout=self._timeout,
                min_lidar_quality=self._min_lidar_quality,
                post_recovery_stable_sec=float(
                    self.get_parameter("post_recovery_stable_sec").value
                ),
            )

        def _reset_runtime_state(self) -> None:
            self._aggregator = self._new_aggregator()
            self._gnss_state = None
            self._gnss_received_at = None
            self._lidar_state = None
            self._lidar_received_at = None
            self._scan_match_state = None
            self._scan_match_received_at = None
            self._uwb_state = None
            self._uwb_received_at = None
            self._relocalization_state = None
            self._relocalization_received_at = None
            self._amcl_received_at = None
            self._amcl_reference_odom_pose = None
            self._amcl_covariance = None
            self._odom_received_at = None
            self._odom_pose = None
            self._scan_received_at = None
            self._imu_received_at = None

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
            if not (previous == "WARMUP" and activity.phase == ACTIVE):
                self._reset_runtime_state()
            self._mission_phase = activity.phase

        def _tick(self) -> None:
            now = self._now()
            activity = self._mission_activity(now)
            self._apply_mission_transition(activity)
            if not activity.mission_active:
                self._publish_gate_status(activity, now)
                return
            output = self._aggregator.evaluate(
                NavigationHealthInput(
                    now=now,
                    gnss_state=self._gnss_state,
                    gnss_fresh=self._fresh(self._gnss_received_at, now),
                    lidar_state=self._lidar_state,
                    lidar_fresh=self._fresh(self._lidar_received_at, now),
                    amcl_covariance=self._amcl_covariance,
                    amcl_fresh=self._amcl_fresh(now),
                    odom_fresh=self._fresh(self._odom_received_at, now),
                    scan_fresh=self._fresh(self._scan_received_at, now),
                    imu_fresh=self._fresh(self._imu_received_at, now),
                    scan_match_state=self._scan_match_state,
                    scan_match_fresh=self._fresh(self._scan_match_received_at, now),
                    relocalization_state=self._relocalization_state,
                    relocalization_fresh=self._fresh(self._relocalization_received_at, now),
                    uwb_state=self._uwb_state,
                    uwb_fresh=self._fresh(self._uwb_received_at, now),
                    required_signals=self._required_signals,
                )
            )
            if output.transition:
                logger = self.get_logger()
                if output.transition_level == "ERROR":
                    logger.error(output.transition_message)
                elif output.transition_level == "WARN":
                    logger.warn(output.transition_message)
                else:
                    logger.info(output.transition_message)
            if output.overall_state in {"FAILED", "MANUAL_REQUIRED"}:
                self._last_failure_reason = ";".join(output.reasons)
            self._publish(output, activity)

        def _amcl_fresh(self, now: float) -> bool | None:
            return motion_aware_pose_fresh(
                self._fresh(self._amcl_received_at, now),
                self._amcl_reference_odom_pose,
                self._odom_pose,
                float(self.get_parameter("amcl_motion_translation_tolerance").value),
                float(self.get_parameter("amcl_motion_rotation_tolerance").value),
            )

        def _publish_gate_status(self, activity: MissionActivity, now: float) -> None:
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "DG Navigation Health"
            status.level = (
                DiagnosticStatus.WARN
                if activity.phase == "WARMUP" or not activity.status_fresh
                else DiagnosticStatus.OK
            )
            status.message = activity.phase
            fields = {
                "overall_state": activity.phase,
                "mission_phase": activity.phase,
                "mission_active": activity.mission_active,
                "mission_status_fresh": activity.status_fresh,
                "patrol_state": activity.patrol_state,
                "standby_reason": activity.standby_reason,
                "missing_components": ";".join(activity.missing_components),
                "last_terminal_state": activity.terminal_state,
                "last_failure_reason": self._last_failure_reason,
                "recovery_motion_armed": bool(self.get_parameter("recovery_motion_armed").value),
                "reasons": activity.standby_reason,
                "timestamp": now,
            }
            status.values = [KeyValue(key=str(key), value=str(value)) for key, value in fields.items()]
            message.status = [status]
            self._status_pub.publish(message)

        def _publish(self, output: Any, activity: MissionActivity) -> None:
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "DG Navigation Health"
            status.level = DiagnosticStatus.ERROR if output.overall_state in {"FAILED", "MANUAL_REQUIRED"} else DiagnosticStatus.WARN if output.overall_state not in {"NOMINAL", "RECOVERED"} else DiagnosticStatus.OK
            status.message = output.overall_state
            fields = {
                "gnss_state": output.gnss_state,
                "lidar_state": output.lidar_state,
                "amcl_state": output.amcl_state,
                "scan_match_state": output.scan_match_state,
                "relocalization_state": output.relocalization_state,
                "uwb_state": output.uwb_state,
                "odom_fresh": self._fresh(self._odom_received_at, output.timestamp),
                "scan_fresh": self._fresh(self._scan_received_at, output.timestamp),
                "imu_fresh": self._fresh(self._imu_received_at, output.timestamp),
                "overall_state": output.overall_state,
                "post_recovery_elapsed": output.post_recovery_elapsed,
                "post_recovery_required": output.post_recovery_required,
                "post_recovery_complete": output.post_recovery_complete,
                "reasons": ";".join(output.reasons),
                "mission_phase": activity.phase,
                "mission_active": activity.mission_active,
                "mission_status_fresh": activity.status_fresh,
                "patrol_state": activity.patrol_state,
                "standby_reason": activity.standby_reason,
                "last_terminal_state": activity.terminal_state,
                "last_failure_reason": self._last_failure_reason,
                "recovery_motion_armed": bool(self.get_parameter("recovery_motion_armed").value),
                "timestamp": output.timestamp,
            }
            status.values = [KeyValue(key=str(key), value=str(value)) for key, value in fields.items()]
            message.status = [status]
            self._status_pub.publish(message)

        def destroy_node(self) -> bool:
            return super().destroy_node()

    return NavigationHealthNode()


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
