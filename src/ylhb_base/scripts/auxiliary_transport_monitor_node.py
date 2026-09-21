#!/usr/bin/env python3
"""Read-only cross-board transport monitor for auxiliary sensors.

This node proves that RDK-published auxiliary topics arrive on Jetson and are
fresh enough for the observation-only MODEL-2 side channel. It never publishes
``/cmd_vel`` and never changes LiDAR data.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-name", default="auxiliary_transport_monitor")
    return parser


def _stamp_to_seconds(stamp: Any) -> float | None:
    try:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def _timing_snapshot(
    stream: dict[str, Any],
    *,
    now_monotonic: float,
    now_ros_time: float | None,
) -> dict[str, float | None]:
    """Return auditable timing values for one received ROS message.

    ``age_sec`` uses the local monotonic clock and is therefore the stable
    freshness value even when a publisher uses a different ROS clock.  The
    header delta is best-effort: it is meaningful only when source and
    receiver ROS clocks share a time domain.
    """
    received_monotonic = stream.get("received_monotonic")
    age_sec = None
    if received_monotonic is not None:
        age_sec = max(0.0, float(now_monotonic) - float(received_monotonic))

    source_stamp = stream.get("source_stamp")
    received_ros_time = stream.get("received_ros_time")
    header_delta = None
    if source_stamp is not None and received_ros_time is not None:
        candidate = float(received_ros_time) - float(source_stamp)
        if math.isfinite(candidate):
            header_delta = candidate

    estimated_latency = None
    if header_delta is not None and header_delta >= 0.0:
        estimated_latency = header_delta

    header_age = None
    if source_stamp is not None and now_ros_time is not None:
        candidate = float(now_ros_time) - float(source_stamp)
        if math.isfinite(candidate) and candidate >= 0.0:
            header_age = candidate

    return {
        "header_stamp_sec": source_stamp,
        "received_wall_time_sec": stream.get("received_wall_time"),
        "received_ros_time_sec": received_ros_time,
        "age_sec": age_sec,
        "header_age_sec": header_age,
        "header_to_receive_delta_sec": header_delta,
        "estimated_latency_sec": estimated_latency,
    }


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2

    class AuxiliaryTransportMonitorNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("event_quality_topic", "/dg/event_camera/quality")
            self.declare_parameter("event_features_topic", "/dg/event_camera/features")
            self.declare_parameter("event_mask_topic", "/dg/event_camera/motion_mask")
            self.declare_parameter("mmwave_points_topic", "/dg/mmwave/points")
            self.declare_parameter("mmwave_quality_topic", "/dg/mmwave/quality")
            self.declare_parameter("output_topic", "/dg/auxiliary/transport_quality")
            self.declare_parameter("stale_timeout_sec", 1.0)
            self.declare_parameter("publish_rate_hz", 1.0)

            self._stale_timeout = max(0.1, float(self.get_parameter("stale_timeout_sec").value))
            output_topic = str(self.get_parameter("output_topic").value)
            self._publisher = self.create_publisher(DiagnosticArray, output_topic, 10)
            self._streams: dict[str, dict[str, Any]] = {
                "event_quality": self._new_stream(),
                "event_features": self._new_stream(),
                "event_mask": self._new_stream(),
                "mmwave_points": self._new_stream(),
                "mmwave_quality": self._new_stream(),
            }
            self._event_quality_state = "UNKNOWN"
            self._mmwave_quality_state = "UNKNOWN"

            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("event_quality_topic").value),
                self._on_event_quality,
                10,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("event_features_topic").value),
                lambda message: self._on_data("event_features", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("event_mask_topic").value),
                lambda message: self._on_data("event_mask", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("mmwave_points_topic").value),
                lambda message: self._on_data("mmwave_points", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("mmwave_quality_topic").value),
                self._on_mmwave_quality,
                10,
            )
            rate_hz = max(0.2, float(self.get_parameter("publish_rate_hz").value))
            self.create_timer(1.0 / rate_hz, self._publish)
            self.get_logger().info(
                "auxiliary transport monitor active; observation_only=true; publishes_cmd_vel=false"
            )

        @staticmethod
        def _new_stream() -> dict[str, Any]:
            return {
                "received_monotonic": None,
                "received_wall_time": None,
                "received_ros_time": None,
                "count": 0,
                "frame_id": "",
                "source_stamp": None,
                "sample_count": 0,
            }

        def _on_data(self, name: str, message: Any) -> None:
            stream = self._streams[name]
            stream["received_monotonic"] = time.monotonic()
            stream["received_wall_time"] = time.time()
            try:
                stream["received_ros_time"] = float(self.get_clock().now().nanoseconds) * 1e-9
            except (AttributeError, TypeError, ValueError):
                stream["received_ros_time"] = None
            stream["count"] += 1
            stream["frame_id"] = str(getattr(getattr(message, "header", None), "frame_id", ""))
            stream["source_stamp"] = _stamp_to_seconds(
                getattr(getattr(message, "header", None), "stamp", None)
            )
            stream["sample_count"] = int(getattr(message, "width", 0)) * int(
                getattr(message, "height", 0)
            )

        def _read_quality(self, name: str, message: Any) -> None:
            self._on_data(name, message)
            stream = self._streams[name]
            selected = message.status[0] if getattr(message, "status", None) else None
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            state = values.get("state") or str(selected.message or "UNKNOWN")
            stream["state"] = state
            if name == "event_quality":
                self._event_quality_state = state
            else:
                self._mmwave_quality_state = state

        def _on_event_quality(self, message: Any) -> None:
            self._read_quality("event_quality", message)

        def _on_mmwave_quality(self, message: Any) -> None:
            self._read_quality("mmwave_quality", message)

        def _fresh(self, name: str, now: float) -> bool:
            received = self._streams[name]["received_monotonic"]
            return received is not None and now - float(received) <= self._stale_timeout

        def _state(self, quality_name: str, data_name: str, now: float) -> str:
            quality_fresh = self._fresh(quality_name, now)
            data_fresh = self._fresh(data_name, now)
            if not quality_fresh and not data_fresh:
                return "UNSEEN"
            if quality_fresh and data_fresh:
                quality_state = str(self._streams[quality_name].get("state", "UNKNOWN"))
                if quality_state in {"FAULT", "ERROR"}:
                    return "FAULT"
                return "GOOD" if quality_state in {"GOOD", "STATIC_IDLE", "READY"} else quality_state
            return "STALE"

        def _publish(self) -> None:
            now = time.monotonic()
            event_state = self._state("event_quality", "event_features", now)
            mmwave_state = self._state("mmwave_quality", "mmwave_points", now)
            event_observable = (
                event_state in {"GOOD", "STATIC_IDLE", "READY"}
                and self._fresh("event_quality", now)
                and self._fresh("event_features", now)
                and self._fresh("event_mask", now)
            )
            mmwave_observable = mmwave_state in {"GOOD", "STATIC_IDLE", "READY"}

            message = DiagnosticArray()
            publish_time = self.get_clock().now()
            message.header.stamp = publish_time.to_msg()
            try:
                publish_ros_time = float(publish_time.nanoseconds) * 1e-9
            except (AttributeError, TypeError, ValueError):
                publish_ros_time = None
            status = DiagnosticStatus()
            status.name = "dg/auxiliary/transport_quality"
            status.hardware_id = "rdk_to_jetson_dds"
            status.level = DiagnosticStatus.OK if event_observable or mmwave_observable else DiagnosticStatus.WARN
            status.message = f"event={event_state};mmwave={mmwave_state}"
            values = {
                "event_camera_state": event_state,
                "event_camera_quality_state": self._event_quality_state,
                "event_camera_quality_fresh": self._fresh("event_quality", now),
                "event_camera_features_fresh": self._fresh("event_features", now),
                "event_camera_mask_fresh": self._fresh("event_mask", now),
                "event_camera_quality_messages": self._streams["event_quality"]["count"],
                "event_camera_feature_messages": self._streams["event_features"]["count"],
                "event_camera_feature_sample_count": self._streams["event_features"]["sample_count"],
                "mmwave_state": mmwave_state,
                "mmwave_quality_state": self._mmwave_quality_state,
                "mmwave_quality_fresh": self._fresh("mmwave_quality", now),
                "mmwave_points_fresh": self._fresh("mmwave_points", now),
                "mmwave_quality_messages": self._streams["mmwave_quality"]["count"],
                "mmwave_point_messages": self._streams["mmwave_points"]["count"],
                "mmwave_detection_count": self._streams["mmwave_points"]["sample_count"],
                "model2_event_assist_observable": event_observable,
                "model2_mmwave_observable": mmwave_observable,
                "observation_only": True,
                "publishes_cmd_vel": False,
                "changes_lidar_scan": False,
                "stale_timeout_sec": self._stale_timeout,
            }
            stream_prefixes = {
                "event_quality": "event_camera_quality",
                "event_features": "event_camera_features",
                "event_mask": "event_camera_mask",
                "mmwave_points": "mmwave_points",
                "mmwave_quality": "mmwave_quality",
            }
            for stream_name, prefix in stream_prefixes.items():
                timing = _timing_snapshot(
                    self._streams[stream_name],
                    now_monotonic=now,
                    now_ros_time=publish_ros_time,
                )
                values.update(
                    {f"{prefix}_{key}": value for key, value in timing.items()}
                )
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in sorted(values.items())
            ]
            message.status = [status]
            self._publisher.publish(message)

    return AuxiliaryTransportMonitorNode()


def main(argv: list[str] | None = None) -> int:
    arguments, ros_arguments = build_parser().parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required for the transport monitor", file=sys.stderr)
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
