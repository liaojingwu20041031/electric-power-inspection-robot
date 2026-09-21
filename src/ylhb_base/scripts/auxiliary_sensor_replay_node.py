#!/usr/bin/env python3
"""Replay measured event-camera or normalized mmWave CSV into ROS 2.

This adapter is observation-only.  It publishes the measured samples and a
diagnostic stream, and has no control topic, service client, or action client.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys
from typing import Any, Callable, Iterator

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402
from auxiliary_sensor_data import (  # noqa: E402
    EventPacket,
    MmwaveFrame,
    iter_event_packets,
    iter_mmwave_frames,
    summarize_event_packet,
    summarize_mmwave_frame,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay measured auxiliary sensor CSV")
    parser.add_argument("--node-name", default="auxiliary_sensor_replay_node")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField

    class AuxiliarySensorReplayNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("sensor_kind", "")
            self.declare_parameter("input_file", "")
            self.declare_parameter("loop", False)
            self.declare_parameter("replay_rate_hz", 50.0)
            self.declare_parameter("frame_id", "")
            self.declare_parameter("data_topic", "")
            self.declare_parameter("quality_topic", "")
            self.declare_parameter("source_kind", "MEASURED_REPLAY")
            self.declare_parameter("event_width", 768)
            self.declare_parameter("event_height", 608)
            self.declare_parameter("event_window_us", 20_000)
            self.declare_parameter("moving_speed_threshold", 0.15)

            self._sensor_kind = str(self.get_parameter("sensor_kind").value).strip().lower()
            if self._sensor_kind not in {"event_camera", "mmwave"}:
                raise ValueError("sensor_kind must be 'event_camera' or 'mmwave'")
            self._input_path = Path(str(self.get_parameter("input_file").value).strip())
            if not self._input_path.is_file():
                raise ValueError(f"measured input_file does not exist: {self._input_path}")
            self._loop = bool(self.get_parameter("loop").value)
            rate = float(self.get_parameter("replay_rate_hz").value)
            if rate <= 0.0:
                raise ValueError("replay_rate_hz must be positive")

            defaults = {
                "event_camera": (
                    "event_camera_optical_frame",
                    "/dg/event_camera/events",
                    "/dg/event_camera/quality",
                ),
                "mmwave": (
                    "mmwave_link",
                    "/dg/mmwave/points",
                    "/dg/mmwave/quality",
                ),
            }
            default_frame, default_data_topic, default_quality_topic = defaults[self._sensor_kind]
            self._frame_id = str(self.get_parameter("frame_id").value).strip() or default_frame
            data_topic = str(self.get_parameter("data_topic").value).strip() or default_data_topic
            quality_topic = str(self.get_parameter("quality_topic").value).strip() or default_quality_topic
            if not data_topic.startswith("/") or not quality_topic.startswith("/"):
                raise ValueError("data_topic and quality_topic must be absolute ROS topics")
            self._source_kind = str(self.get_parameter("source_kind").value).strip()
            if self._source_kind not in {
                "MEASURED_REPLAY",
                "SYNTHETIC_CONTRACT_FIXTURE",
            }:
                raise ValueError(
                    "source_kind must be MEASURED_REPLAY or SYNTHETIC_CONTRACT_FIXTURE"
                )

            self._event_width = int(self.get_parameter("event_width").value)
            self._event_height = int(self.get_parameter("event_height").value)
            self._event_window_us = int(self.get_parameter("event_window_us").value)
            self._moving_speed_threshold = float(
                self.get_parameter("moving_speed_threshold").value
            )
            self._source_factory: Callable[[], Iterator[EventPacket | MmwaveFrame]]
            if self._sensor_kind == "event_camera":
                self._source_factory = lambda: iter_event_packets(
                    self._input_path,
                    window_us=self._event_window_us,
                    width=self._event_width,
                    height=self._event_height,
                )
            else:
                self._source_factory = lambda: iter_mmwave_frames(self._input_path)

            self._source = self._source_factory()
            self._sequence = 0
            self._published_any = False
            self._data_pub = self.create_publisher(PointCloud2, data_topic, 10)
            self._quality_pub = self.create_publisher(DiagnosticArray, quality_topic, 10)
            self._timer = self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info(
                f"{self._sensor_kind} measured-data replay ready: {self._input_path}; "
                f"data={data_topic}; quality={quality_topic}; observation_only=true"
            )

        def _restart_or_stop(self) -> bool:
            if self._loop and self._published_any:
                self._source = self._source_factory()
                return True
            self._timer.cancel()
            if self._published_any:
                self.get_logger().info(f"{self._sensor_kind} measured-data replay complete")
            else:
                self.get_logger().error(
                    f"{self._sensor_kind} measured-data input contains no publishable records"
                )
            return False

        def _tick(self) -> None:
            try:
                packet = next(self._source)
            except StopIteration:
                if not self._restart_or_stop():
                    return
                try:
                    packet = next(self._source)
                except StopIteration:
                    self._timer.cancel()
                    self.get_logger().error("replay source became empty after restart")
                    return
            except (OSError, ValueError) as exc:
                self._timer.cancel()
                self.get_logger().error(f"measured-data replay rejected input: {exc}")
                return

            self._sequence += 1
            self._published_any = True
            stamp = self.get_clock().now().to_msg()
            if self._sensor_kind == "event_camera":
                assert isinstance(packet, EventPacket)
                cloud = self._event_cloud(packet, stamp)
                metrics = summarize_event_packet(
                    packet,
                    width=self._event_width,
                    height=self._event_height,
                )
                source_timestamp = packet.start_timestamp_us
                protocol = "SHIMETA_EVT2_CSV"
            else:
                assert isinstance(packet, MmwaveFrame)
                cloud = self._mmwave_cloud(packet, stamp)
                metrics = summarize_mmwave_frame(
                    packet,
                    moving_speed_threshold=self._moving_speed_threshold,
                )
                source_timestamp = packet.timestamp_us
                protocol = "NORMALIZED_CSV_VENDOR_DECODER_REQUIRED"
            self._data_pub.publish(cloud)
            self._publish_quality(stamp, metrics, source_timestamp, protocol)

        def _event_cloud(self, packet: EventPacket, stamp: Any) -> Any:
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = self._frame_id
            message.height = 1
            message.width = len(packet.samples)
            message.fields = [
                PointField(name="pixel_x", offset=0, datatype=PointField.UINT16, count=1),
                PointField(name="pixel_y", offset=2, datatype=PointField.UINT16, count=1),
                PointField(name="polarity", offset=4, datatype=PointField.UINT8, count=1),
                PointField(name="time_offset_us", offset=8, datatype=PointField.UINT32, count=1),
            ]
            message.is_bigendian = False
            message.point_step = 12
            message.row_step = message.point_step * message.width
            message.is_dense = True
            message.data = b"".join(
                struct.pack(
                    "<HHB3xI",
                    sample.x,
                    sample.y,
                    sample.polarity,
                    sample.timestamp_us - packet.start_timestamp_us,
                )
                for sample in packet.samples
            )
            return message

        def _mmwave_cloud(self, frame: MmwaveFrame, stamp: Any) -> Any:
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = self._frame_id
            message.height = 1
            message.width = len(frame.detections)
            message.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="radial_velocity", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="snr_db", offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name="confidence", offset=20, datatype=PointField.FLOAT32, count=1),
                PointField(name="track_id", offset=24, datatype=PointField.UINT32, count=1),
            ]
            message.is_bigendian = False
            message.point_step = 28
            message.row_step = message.point_step * message.width
            message.is_dense = True
            message.data = b"".join(
                struct.pack(
                    "<ffffffI",
                    item.x_m,
                    item.y_m,
                    item.z_m,
                    item.radial_velocity_mps,
                    item.snr_db,
                    item.confidence,
                    item.track_id,
                )
                for item in frame.detections
            )
            return message

        def _publish_quality(
            self,
            stamp: Any,
            metrics: dict[str, object],
            source_timestamp_us: int,
            protocol: str,
        ) -> None:
            array = DiagnosticArray()
            array.header.stamp = stamp
            status = DiagnosticStatus()
            status.name = (
                "DG Event Camera Input"
                if self._sensor_kind == "event_camera"
                else "DG mmWave Input"
            )
            has_data = int(metrics.get("event_count", metrics.get("detection_count", 0))) > 0
            # Use the same quality vocabulary as the live HVS bridge and the
            # transport monitor.  ``DATA_VALID`` is retained as an explicit
            # compatibility field below, but must not be used as the state:
            # MODEL-2 gates accept GOOD/STATIC_IDLE/READY only.
            quality_state = "GOOD" if has_data else "FAULT"
            status.level = DiagnosticStatus.OK if has_data else DiagnosticStatus.WARN
            status.message = quality_state
            values = {
                "sensor_kind": self._sensor_kind,
                "state": quality_state,
                "data_valid": has_data,
                "source_kind": self._source_kind,
                "source_timestamp_us": source_timestamp_us,
                "source_file": self._input_path.name,
                "source_protocol": protocol,
                "sequence": self._sequence,
                "frame_id": self._frame_id,
                "navigation_effect": "OBSERVATION_ONLY",
                **metrics,
            }
            status.values = [
                KeyValue(key=str(key), value=str(value)) for key, value in values.items()
            ]
            array.status = [status]
            self._quality_pub.publish(array)

    return AuxiliarySensorReplayNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    arguments, ros_arguments = parser.parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required only for the replay wrapper.", file=sys.stderr)
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
