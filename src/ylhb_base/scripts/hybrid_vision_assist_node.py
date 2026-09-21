#!/usr/bin/env python3
"""Fuse live HVS health, features and masks into a gated MODEL-2 side channel."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from hybrid_vision_assist_core import (  # noqa: E402
    HybridVisionAssistConfig,
    HybridVisionObservation,
    evaluate_hybrid_vision_assist,
)
from ros_shutdown import cleanup_ros_node  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-name", default="hybrid_vision_assist_node")
    return parser


def _diagnostic_values(message: Any) -> dict[str, str]:
    if not message.status:
        return {}
    values = {str(item.key): str(item.value) for item in message.status[0].values}
    values.setdefault("state", str(message.status[0].message))
    return values


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

    class HybridVisionAssistNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("camera_quality_topic", "/dg/event_camera/quality")
            self.declare_parameter("event_features_topic", "/dg/event_camera/features")
            self.declare_parameter("motion_mask_topic", "/dg/event_camera/motion_mask")
            self.declare_parameter("camera_info_topic", "/dg/event_camera/camera_info")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("imu_topic", "/imu/data")
            self.declare_parameter("assist_quality_topic", "/dg/event_camera/assist_quality")
            self.declare_parameter("expected_frame_id", "event_camera_optical_frame")
            self.declare_parameter("input_timeout_sec", 0.5)
            self.declare_parameter("intrinsics_valid", False)
            self.declare_parameter("lidar_extrinsics_valid", False)
            self.declare_parameter("motion_compensation_ready", False)
            self.declare_parameter("max_linear_speed_for_uncompensated_mask", 0.02)
            self.declare_parameter("max_yaw_rate_for_uncompensated_mask", 0.03)

            self._config = HybridVisionAssistConfig(
                input_timeout_sec=float(self.get_parameter("input_timeout_sec").value),
                max_linear_speed_for_uncompensated_mask=float(
                    self.get_parameter("max_linear_speed_for_uncompensated_mask").value
                ),
                max_yaw_rate_for_uncompensated_mask=float(
                    self.get_parameter("max_yaw_rate_for_uncompensated_mask").value
                ),
            )
            self._expected_frame = str(self.get_parameter("expected_frame_id").value)
            self._intrinsics_declared_valid = bool(
                self.get_parameter("intrinsics_valid").value
            )
            self._lidar_extrinsics_valid = bool(
                self.get_parameter("lidar_extrinsics_valid").value
            )
            self._motion_compensation_ready = bool(
                self.get_parameter("motion_compensation_ready").value
            )
            self._camera_state = "UNKNOWN"
            self._camera_received: float | None = None
            self._features_received: float | None = None
            self._mask_received: float | None = None
            self._feature_count = 0
            self._mask_fraction = 0.0
            self._intrinsics_valid = False
            self._linear_speed = 0.0
            self._odom_yaw_rate = 0.0
            self._imu_yaw_rate = 0.0
            self._input_frame_valid = False

            self._publisher = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("assist_quality_topic").value), 10
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("camera_quality_topic").value),
                self._on_quality,
                10,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("event_features_topic").value),
                self._on_features,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("motion_mask_topic").value),
                self._on_mask,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
            )
            self.create_subscription(
                Imu, str(self.get_parameter("imu_topic").value), self._on_imu, qos_profile_sensor_data
            )
            self.create_timer(0.2, self._publish)
            self.get_logger().info(
                "hybrid vision assist started in calibration-gated mode; publishes_cmd_vel=false"
            )

        def _frame_ok(self, message: Any) -> bool:
            return str(message.header.frame_id) == self._expected_frame

        def _on_quality(self, message: Any) -> None:
            values = _diagnostic_values(message)
            self._camera_state = values.get("state", "UNKNOWN")
            self._camera_received = time.monotonic()

        def _on_features(self, message: Any) -> None:
            self._input_frame_valid = self._frame_ok(message)
            if self._input_frame_valid:
                self._feature_count = int(message.width) * int(message.height)
                self._features_received = time.monotonic()

        def _on_mask(self, message: Any) -> None:
            self._input_frame_valid = self._frame_ok(message)
            if not self._input_frame_valid or message.encoding != "mono8" or not message.data:
                return
            active = sum(1 for value in message.data if int(value) > 0)
            self._mask_fraction = active / len(message.data)
            self._mask_received = time.monotonic()

        def _on_camera_info(self, message: Any) -> None:
            self._input_frame_valid = self._frame_ok(message)
            self._intrinsics_valid = (
                self._input_frame_valid
                and int(message.width) > 0
                and int(message.height) > 0
                and math.isfinite(float(message.k[0]))
                and math.isfinite(float(message.k[4]))
                and float(message.k[0]) > 0.0
                and float(message.k[4]) > 0.0
            )

        def _on_odom(self, message: Any) -> None:
            twist = message.twist.twist
            self._linear_speed = math.hypot(float(twist.linear.x), float(twist.linear.y))
            self._odom_yaw_rate = float(twist.angular.z)

        def _on_imu(self, message: Any) -> None:
            self._imu_yaw_rate = float(message.angular_velocity.z)

        def _publish(self) -> None:
            now_sec = time.monotonic()
            decision = evaluate_hybrid_vision_assist(
                HybridVisionObservation(
                    now_sec=now_sec,
                    camera_quality_received_sec=self._camera_received,
                    features_received_sec=self._features_received,
                    mask_received_sec=self._mask_received,
                    camera_state=self._camera_state,
                    event_feature_count=self._feature_count,
                    motion_mask_active_fraction=self._mask_fraction,
                    camera_intrinsics_valid=(
                        self._intrinsics_declared_valid and self._intrinsics_valid
                    ),
                    lidar_extrinsics_valid=self._lidar_extrinsics_valid,
                    motion_compensation_ready=self._motion_compensation_ready,
                    robot_linear_speed=self._linear_speed,
                    robot_yaw_rate=max(abs(self._odom_yaw_rate), abs(self._imu_yaw_rate)),
                ),
                self._config,
            )
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "dg/event_camera/assist_quality"
            status.hardware_id = "apx003cc_hvs"
            status.message = decision.state
            status.level = (
                DiagnosticStatus.OK
                if decision.state in {"READY_IDLE", "SOFT_ASSIST_READY"}
                else DiagnosticStatus.WARN
            )
            values = {
                "state": decision.state,
                "reason": decision.reason,
                "camera_state": self._camera_state,
                "camera_fresh": decision.camera_fresh,
                "features_fresh": decision.features_fresh,
                "mask_fresh": decision.mask_fresh,
                "event_feature_count": self._feature_count,
                "motion_mask_active_fraction": self._mask_fraction,
                "camera_intrinsics_declared_valid": self._intrinsics_declared_valid,
                "camera_intrinsics_valid": self._intrinsics_valid,
                "lidar_extrinsics_valid": self._lidar_extrinsics_valid,
                "motion_compensation_ready": self._motion_compensation_ready,
                "model2_consumes_event_assist": decision.model2_consumes_event_assist,
                "may_modify_lidar": decision.may_modify_lidar,
                "event_fault_triggers_model3": False,
                "publishes_cmd_vel": False,
            }
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in sorted(values.items())
            ]
            message.status = [status]
            self._publisher.publish(message)

    return HybridVisionAssistNode()


def main(argv: list[str] | None = None) -> int:
    arguments, ros_arguments = build_parser().parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required for the hybrid vision assist node", file=sys.stderr)
        return 2
    rclpy.init(args=ros_arguments)
    node = create_node(arguments.node_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cleanup_ros_node(node, rclpy)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
