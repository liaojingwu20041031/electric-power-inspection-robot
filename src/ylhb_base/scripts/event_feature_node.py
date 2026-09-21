#!/usr/bin/env python3
"""Observation-only ROS 2 event feature and repeatability node."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402
from event_feature_core import (  # noqa: E402
    EventFeature,
    EventFeatureConfig,
    EventObservation,
    extract_event_features,
    feature_repeatability,
)
from pointcloud_reader import read_named_points  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-name", default="event_feature_node")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField

    class EventFeatureNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("events_topic", "/dg/event_camera/events")
            self.declare_parameter("features_topic", "/dg/event_camera/features")
            self.declare_parameter("quality_topic", "/dg/event_camera/feature_quality")
            self.declare_parameter("expected_frame_id", "event_camera_optical_frame")
            self.declare_parameter("image_width", 768)
            self.declare_parameter("image_height", 608)
            self.declare_parameter("cell_size_px", 12)
            self.declare_parameter("min_events_per_cell", 5)
            self.declare_parameter("min_unique_pixels", 3)
            self.declare_parameter("max_features", 512)
            self.declare_parameter("match_distance_px", 12.0)

            self._config = EventFeatureConfig(
                width=int(self.get_parameter("image_width").value),
                height=int(self.get_parameter("image_height").value),
                cell_size_px=int(self.get_parameter("cell_size_px").value),
                min_events_per_cell=int(self.get_parameter("min_events_per_cell").value),
                min_unique_pixels=int(self.get_parameter("min_unique_pixels").value),
                max_features=int(self.get_parameter("max_features").value),
                match_distance_px=float(self.get_parameter("match_distance_px").value),
            )
            self._expected_frame_id = str(self.get_parameter("expected_frame_id").value)
            self._previous_features: list[EventFeature] | None = None
            self._sequence = 0
            self._features_pub = self.create_publisher(
                PointCloud2, str(self.get_parameter("features_topic").value), 10
            )
            self._quality_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("quality_topic").value), 10
            )
            events_topic = str(self.get_parameter("events_topic").value)
            self.create_subscription(PointCloud2, events_topic, self._on_events, qos_profile_sensor_data)
            self.get_logger().info(
                f"event feature evidence active: input={events_topic}; observation_only=true"
            )

        def _publish_quality(self, source: Any, values: dict[str, Any], error: bool = False) -> None:
            message = DiagnosticArray()
            message.header = source.header
            status = DiagnosticStatus()
            status.name = "dg/event_camera/feature_quality"
            status.level = DiagnosticStatus.ERROR if error else DiagnosticStatus.OK
            status.message = str(values.get("repeatability_status", "UNKNOWN"))
            status.values = [
                KeyValue(key=str(key), value="" if value is None else str(value))
                for key, value in sorted(values.items())
            ]
            message.status = [status]
            self._quality_pub.publish(message)

        def _on_events(self, message: Any) -> None:
            self._sequence += 1
            frame_id = str(message.header.frame_id)
            if frame_id != self._expected_frame_id:
                self._publish_quality(
                    message,
                    {
                        "packet_sequence": self._sequence,
                        "event_count": int(message.width) * int(message.height),
                        "feature_count": 0,
                        "repeatability": None,
                        "repeatability_status": "FRAME_MISMATCH",
                        "frame_id": frame_id,
                    },
                    error=True,
                )
                self._previous_features = None
                return

            required_fields = ("pixel_x", "pixel_y", "polarity", "time_offset_us")
            available_fields = {field.name for field in message.fields}
            missing = [name for name in required_fields if name not in available_fields]
            if missing:
                self._publish_quality(
                    message,
                    {
                        "packet_sequence": self._sequence,
                        "event_count": int(message.width) * int(message.height),
                        "feature_count": 0,
                        "repeatability": None,
                        "repeatability_status": "MISSING_FIELDS:" + ",".join(missing),
                        "frame_id": frame_id,
                    },
                    error=True,
                )
                self._previous_features = None
                return

            try:
                observations = [
                    EventObservation(int(row[0]), int(row[1]), int(row[2]), int(row[3]))
                    for row in read_named_points(message, required_fields)
                ]
                features = extract_event_features(observations, self._config)
                repeatability = (
                    None
                    if self._previous_features is None
                    else feature_repeatability(
                        self._previous_features,
                        features,
                        self._config.match_distance_px,
                    )
                )
            except (TypeError, ValueError, IndexError, RuntimeError) as exc:
                self._publish_quality(
                    message,
                    {
                        "packet_sequence": self._sequence,
                        "event_count": int(message.width) * int(message.height),
                        "feature_count": 0,
                        "repeatability": None,
                        "repeatability_status": f"PARSE_ERROR:{type(exc).__name__}",
                        "frame_id": frame_id,
                    },
                    error=True,
                )
                self._previous_features = None
                return

            if self._previous_features is None:
                status = "NO_PREVIOUS_PACKET"
            elif repeatability is None:
                status = "ZERO_FEATURE_PAIR"
            else:
                status = "COMPARED"
            self._features_pub.publish(self._feature_cloud(message, features))
            self._publish_quality(
                message,
                {
                    "packet_sequence": self._sequence,
                    "event_count": len(observations),
                    "feature_count": len(features),
                    "repeatability": repeatability,
                    "repeatability_status": status,
                    "match_distance_px": self._config.match_distance_px,
                    "frame_id": frame_id,
                    "observation_only": True,
                },
            )
            self._previous_features = features

        @staticmethod
        def _feature_cloud(source: Any, features: list[EventFeature]) -> Any:
            message = PointCloud2()
            message.header = source.header
            message.height = 1
            message.width = len(features)
            message.fields = [
                PointField(name="pixel_x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="pixel_y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="event_count", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="positive_fraction", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            message.is_bigendian = False
            message.point_step = 16
            message.row_step = message.point_step * message.width
            message.is_dense = True
            message.data = b"".join(
                struct.pack("<ffff", item.x, item.y, item.event_count, item.positive_fraction)
                for item in features
            )
            return message

    return EventFeatureNode()


def main(argv: list[str] | None = None) -> int:
    arguments, ros_arguments = build_parser().parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required for the event feature node", file=sys.stderr)
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
