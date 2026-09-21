#!/usr/bin/env python3
"""Optional ROS2 side-channel wrapper for the 2D LiDAR robustness POC."""

from __future__ import annotations

import argparse
import copy
import math
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402
from lidar_robust_features import (  # noqa: E402
    DynamicDetection2D,
    DynamicSector,
    RobustFeatureConfig,
    ScanFrame,
    dynamic_sector_from_detection,
    process_scan,
)
from pointcloud_reader import read_named_points  # noqa: E402


def _yaw_from_quaternion(quaternion: Any) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _relative_pose(previous: tuple[float, float, float], current: tuple[float, float, float]) -> tuple[float, float, float]:
    px, py, pyaw = previous
    cx, cy, cyaw = current
    dx_world = cx - px
    dy_world = cy - py
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    dx_previous = cosine * dx_world + sine * dy_world
    dy_previous = -sine * dx_world + cosine * dy_world
    dyaw = math.atan2(math.sin(cyaw - pyaw), math.cos(cyaw - pyaw))
    return dx_previous, dy_previous, dyaw


def _odom_pose(message: Any) -> tuple[float, float, float]:
    pose = message.pose.pose
    return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the side-channel DG-202611 2D LiDAR robustness node.")
    parser.add_argument("--node-name", default="lidar_robust_node", help="ROS node name")
    return parser


def _config_from_node(node: Any) -> RobustFeatureConfig:
    return RobustFeatureConfig(
        min_range=float(node.get_parameter("min_range").value),
        max_range=float(node.get_parameter("max_range").value),
        self_radius=float(node.get_parameter("self_radius").value),
        isolation_range_jump=float(node.get_parameter("isolation_range_jump").value),
        temporal_match_distance=float(node.get_parameter("temporal_match_distance").value),
        corner_deviation=float(node.get_parameter("corner_deviation").value),
        corner_neighbor_distance=float(node.get_parameter("corner_neighbor_distance").value),
        line_neighbor_distance=float(node.get_parameter("line_neighbor_distance").value),
        min_line_points=int(node.get_parameter("min_line_points").value),
    )


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    class LidarRobustNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("filtered_scan_topic", "/dg/lidar/filtered_scan")
            self.declare_parameter("stable_features_topic", "/dg/lidar/stable_features")
            self.declare_parameter("quality_topic", "/dg/lidar/quality")
            self.declare_parameter("min_range", 0.05)
            self.declare_parameter("max_range", 8.0)
            self.declare_parameter("self_radius", 0.18)
            self.declare_parameter("isolation_range_jump", 0.35)
            self.declare_parameter("temporal_match_distance", 0.12)
            self.declare_parameter("corner_deviation", math.radians(25.0))
            self.declare_parameter("corner_neighbor_distance", 0.35)
            self.declare_parameter("line_neighbor_distance", 0.25)
            self.declare_parameter("min_line_points", 3)
            self.declare_parameter("mmwave_dynamic_mask_enabled", False)
            self.declare_parameter("mmwave_observation_enabled", False)
            self.declare_parameter("mmwave_points_topic", "/dg/mmwave/points")
            self.declare_parameter("mmwave_expected_frame_id", "mmwave_link")
            self.declare_parameter("mmwave_timeout_sec", 0.25)
            self.declare_parameter("mmwave_to_lidar_x", 0.0)
            self.declare_parameter("mmwave_to_lidar_y", 0.0)
            self.declare_parameter("mmwave_to_lidar_yaw", 0.0)
            self.declare_parameter("mmwave_min_abs_velocity", 0.15)
            self.declare_parameter("mmwave_min_snr", 5.0)
            self.declare_parameter("mmwave_min_confidence", 0.5)
            self.declare_parameter("mmwave_angular_half_width", 0.08)
            self.declare_parameter("mmwave_radial_margin", 0.35)
            self.declare_parameter("event_assist_enabled", False)
            self.declare_parameter(
                "event_assist_quality_topic", "/dg/event_camera/assist_quality"
            )
            self.declare_parameter("event_assist_timeout_sec", 0.5)

            config = _config_from_node(self)
            self._config = config
            self._previous_frame: ScanFrame | None = None
            self._previous_odom_pose: tuple[float, float, float] | None = None
            self._latest_odom_pose: tuple[float, float, float] | None = None
            self._warned_without_odom = False
            self._mmwave_enabled = bool(self.get_parameter("mmwave_dynamic_mask_enabled").value)
            self._mmwave_observation_enabled = bool(
                self.get_parameter("mmwave_observation_enabled").value
            )
            self._mmwave_subscribed = self._mmwave_enabled or self._mmwave_observation_enabled
            self._mmwave_timeout_sec = float(self.get_parameter("mmwave_timeout_sec").value)
            self._mmwave_expected_frame_id = str(self.get_parameter("mmwave_expected_frame_id").value)
            self._mmwave_to_lidar = (
                float(self.get_parameter("mmwave_to_lidar_x").value),
                float(self.get_parameter("mmwave_to_lidar_y").value),
                float(self.get_parameter("mmwave_to_lidar_yaw").value),
            )
            self._mmwave_gate = {
                "min_abs_velocity": float(self.get_parameter("mmwave_min_abs_velocity").value),
                "min_snr": float(self.get_parameter("mmwave_min_snr").value),
                "min_confidence": float(self.get_parameter("mmwave_min_confidence").value),
                "angular_half_width": float(self.get_parameter("mmwave_angular_half_width").value),
                "radial_margin": float(self.get_parameter("mmwave_radial_margin").value),
            }
            numeric_parameters = (self._mmwave_timeout_sec, *self._mmwave_to_lidar, *self._mmwave_gate.values())
            if any(not math.isfinite(value) for value in numeric_parameters):
                raise ValueError("mmWave dynamic-mask parameters must be finite")
            if self._mmwave_timeout_sec <= 0.0 or any(value < 0.0 for value in self._mmwave_gate.values()):
                raise ValueError("mmWave timeout must be positive and gate thresholds non-negative")
            self._latest_dynamic_sectors: tuple[DynamicSector, ...] = ()
            self._mmwave_received_at: float | None = None
            self._mmwave_input_count = 0
            self._mmwave_reject_reason = "DISABLED" if not self._mmwave_subscribed else "NO_MESSAGE"
            self._mmwave_frame_id = ""
            self._event_assist_enabled = bool(
                self.get_parameter("event_assist_enabled").value
            )
            self._event_assist_timeout_sec = float(
                self.get_parameter("event_assist_timeout_sec").value
            )
            if (
                not math.isfinite(self._event_assist_timeout_sec)
                or self._event_assist_timeout_sec <= 0.0
            ):
                raise ValueError("event assist timeout must be finite and positive")
            self._event_assist_received_at: float | None = None
            self._event_assist_state = (
                "NO_MESSAGE" if self._event_assist_enabled else "DISABLED"
            )
            self._event_assist_declares_consumed = False
            self._event_assist_may_modify_lidar = False
            scan_topic = str(self.get_parameter("scan_topic").value)
            odom_topic = str(self.get_parameter("odom_topic").value)
            self._filtered_pub = self.create_publisher(LaserScan, str(self.get_parameter("filtered_scan_topic").value), 10)
            self._features_pub = self.create_publisher(PoseArray, str(self.get_parameter("stable_features_topic").value), 10)
            self._quality_pub = self.create_publisher(DiagnosticArray, str(self.get_parameter("quality_topic").value), 10)
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
            if self._mmwave_subscribed:
                mmwave_topic = str(self.get_parameter("mmwave_points_topic").value)
                self.create_subscription(PointCloud2, mmwave_topic, self._on_mmwave, qos_profile_sensor_data)
                mode = "dynamic mask" if self._mmwave_enabled else "observation"
                self.get_logger().info(
                    f"mmWave {mode} subscribed on {mmwave_topic}; expected frame "
                    f"{self._mmwave_expected_frame_id}"
                )
            if self._event_assist_enabled:
                event_assist_topic = str(
                    self.get_parameter("event_assist_quality_topic").value
                )
                self.create_subscription(
                    DiagnosticArray, event_assist_topic, self._on_event_assist, 10
                )
                self.get_logger().info(
                    f"event-camera assist diagnostics enabled on {event_assist_topic}; "
                    "scan modification remains calibration-gated"
                )
            self.get_logger().info(
                f"side-channel LiDAR robustness POC active: {scan_topic} + {odom_topic}; "
                "existing scan consumers are not replaced"
            )

        def _on_odom(self, message: Any) -> None:
            self._latest_odom_pose = _odom_pose(message)

        def _on_event_assist(self, message: Any) -> None:
            if not message.status:
                self._event_assist_state = "EMPTY_DIAGNOSTIC"
                self._event_assist_received_at = None
                return
            values = {
                str(item.key): str(item.value).strip().lower()
                for item in message.status[0].values
            }
            self._event_assist_state = values.get(
                "state", str(message.status[0].message)
            )
            self._event_assist_declares_consumed = (
                values.get("model2_consumes_event_assist", "false") == "true"
            )
            self._event_assist_may_modify_lidar = (
                values.get("may_modify_lidar", "false") == "true"
            )
            self._event_assist_received_at = time.monotonic()

        def _on_mmwave(self, message: Any) -> None:
            self._mmwave_frame_id = str(message.header.frame_id)
            self._mmwave_input_count = int(message.width) * int(message.height)
            if self._mmwave_frame_id != self._mmwave_expected_frame_id:
                self._latest_dynamic_sectors = ()
                self._mmwave_received_at = None
                self._mmwave_reject_reason = "FRAME_MISMATCH"
                return
            required_fields = ("x", "y", "radial_velocity", "snr_db", "confidence")
            available_fields = {field.name for field in message.fields}
            missing = [name for name in required_fields if name not in available_fields]
            if missing:
                self._latest_dynamic_sectors = ()
                self._mmwave_received_at = None
                self._mmwave_reject_reason = "MISSING_FIELDS:" + ",".join(missing)
                return
            sectors: list[DynamicSector] = []
            try:
                rows = read_named_points(message, required_fields)
                for row in rows:
                    values = tuple(row)
                    detection = DynamicDetection2D(
                        x=float(values[0]),
                        y=float(values[1]),
                        radial_velocity=float(values[2]),
                        snr=float(values[3]),
                        confidence=float(values[4]),
                    )
                    sector = dynamic_sector_from_detection(
                        detection,
                        self._mmwave_to_lidar,
                        **self._mmwave_gate,
                    )
                    if sector is not None:
                        sectors.append(sector)
            except (TypeError, ValueError, IndexError, RuntimeError) as exc:
                self._latest_dynamic_sectors = ()
                self._mmwave_received_at = None
                self._mmwave_reject_reason = f"PARSE_ERROR:{type(exc).__name__}"
                return
            self._latest_dynamic_sectors = tuple(sectors)
            self._mmwave_received_at = time.monotonic()
            self._mmwave_reject_reason = "ACCEPTED"

        def _on_scan(self, message: Any) -> None:
            frame = ScanFrame(
                ranges=list(message.ranges),
                angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment),
                range_min=float(message.range_min),
                range_max=float(message.range_max),
            )
            relative_pose = (0.0, 0.0, 0.0)
            if self._latest_odom_pose is not None and self._previous_odom_pose is not None:
                relative_pose = _relative_pose(self._previous_odom_pose, self._latest_odom_pose)
            elif self._previous_frame is not None and not self._warned_without_odom:
                self.get_logger().warning("no two odometry poses yet; temporal compensation is not available")
                self._warned_without_odom = True
            mmwave_fresh = (
                self._mmwave_subscribed
                and self._mmwave_received_at is not None
                and time.monotonic() - self._mmwave_received_at <= self._mmwave_timeout_sec
            )
            event_assist_fresh = (
                self._event_assist_enabled
                and self._event_assist_received_at is not None
                and time.monotonic() - self._event_assist_received_at
                <= self._event_assist_timeout_sec
            )
            external_dynamic_sectors = (
                self._latest_dynamic_sectors if self._mmwave_enabled and mmwave_fresh else ()
            )
            try:
                result = process_scan(
                    frame,
                    self._previous_frame,
                    relative_pose,
                    self._config,
                    external_dynamic_sectors,
                )
            except ValueError as exc:
                self.get_logger().error(f"invalid LaserScan for robustness POC: {exc}")
                return
            self._publish_filtered_scan(message, result)
            self._publish_features(message, result)
            quality = dict(result.quality)
            quality.update(
                {
                    "auxiliary_dynamic_mask_enabled": self._mmwave_enabled,
                    "mmwave_observation_enabled": self._mmwave_observation_enabled,
                    "mmwave_data_fresh": mmwave_fresh,
                    "mmwave_data_received": self._mmwave_received_at is not None,
                    "mmwave_dynamic_sector_count": (
                        len(self._latest_dynamic_sectors) if mmwave_fresh else 0
                    ),
                    "mmwave_dynamic_mask_applied": bool(
                        self._mmwave_enabled and external_dynamic_sectors
                    ),
                    "auxiliary_dynamic_data_fresh": mmwave_fresh,
                    "auxiliary_dynamic_input_count": self._mmwave_input_count,
                    "auxiliary_dynamic_frame_id": self._mmwave_frame_id,
                    "auxiliary_dynamic_reject_reason": (
                        self._mmwave_reject_reason
                        if mmwave_fresh or self._mmwave_reject_reason != "ACCEPTED"
                        else "STALE"
                    ),
                    "event_assist_enabled": self._event_assist_enabled,
                    "event_assist_data_fresh": event_assist_fresh,
                    "event_assist_state": self._event_assist_state,
                    "event_assist_consumed_by_model2": (
                        event_assist_fresh and self._event_assist_declares_consumed
                    ),
                    "event_assist_permission_to_modify_lidar": (
                        event_assist_fresh and self._event_assist_may_modify_lidar
                    ),
                    "event_assist_applied_to_scan": False,
                }
            )
            self._publish_quality(message, quality)
            self._previous_frame = frame
            self._previous_odom_pose = self._latest_odom_pose

        def _publish_filtered_scan(self, source: Any, result: Any) -> None:
            filtered = copy.deepcopy(source)
            remove_indices = set(result.rejected_indices)
            remove_indices.update(item.point.index for item in result.classifications if item.dynamic_candidate)
            filtered.ranges = [float("nan") if index in remove_indices else value for index, value in enumerate(source.ranges)]
            self._filtered_pub.publish(filtered)

        def _publish_features(self, source: Any, result: Any) -> None:
            message = PoseArray()
            message.header = source.header
            for feature in result.features:
                pose = message.poses.add() if hasattr(message.poses, "add") else None
                if pose is None:
                    from geometry_msgs.msg import Pose

                    pose = Pose()
                    message.poses.append(pose)
                pose.position.x = float(feature["x"])
                pose.position.y = float(feature["y"])
                pose.position.z = 0.0
                pose.orientation.w = 1.0
            self._features_pub.publish(message)

        def _publish_quality(self, source: Any, quality: dict[str, Any]) -> None:
            message = DiagnosticArray()
            message.header = source.header
            status = DiagnosticStatus()
            status.name = "dg/lidar/quality"
            status.level = DiagnosticStatus.OK if quality["valid_point_count"] else DiagnosticStatus.ERROR
            status.message = str(quality["temporal_status"])
            status.values = [KeyValue(key=str(key), value="" if value is None else str(value)) for key, value in sorted(quality.items())]
            message.status = [status]
            self._quality_pub.publish(message)

    return LidarRobustNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the pure core is ROS-independent.", file=sys.stderr)
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
