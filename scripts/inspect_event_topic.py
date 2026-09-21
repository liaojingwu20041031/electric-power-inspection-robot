#!/usr/bin/env python3
"""Inspect one live event-camera PointCloud2 without publishing anything."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_SCRIPTS = SCRIPT_DIR.parent / "src" / "ylhb_base" / "scripts"
for candidate in (SCRIPT_DIR, SOURCE_SCRIPTS):
    if candidate.is_dir() and str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from pointcloud_reader import read_named_points  # noqa: E402


REQUIRED_FIELDS = ("pixel_x", "pixel_y", "polarity", "time_offset_us")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/dg/event_camera/events")
    parser.add_argument("--expected-frame-id", default="event_camera_optical_frame")
    parser.add_argument("--width", type=int, default=768)
    parser.add_argument("--height", type=int, default=608)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--sample-limit", type=int, default=20000)
    parser.add_argument("--output-json")
    return parser


def validate_message(message: Any, arguments: argparse.Namespace) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    frame_id = str(message.header.frame_id)
    if frame_id != arguments.expected_frame_id:
        errors.append(
            f"frame_id={frame_id!r}; expected {arguments.expected_frame_id!r}"
        )

    fields = {
        str(field.name): {
            "offset": int(field.offset),
            "datatype": int(field.datatype),
            "count": int(field.count),
        }
        for field in message.fields
    }
    missing = [name for name in REQUIRED_FIELDS if name not in fields]
    if missing:
        errors.append("missing PointCloud2 fields: " + ",".join(missing))

    declared_count = int(message.width) * int(message.height)
    expected_bytes = int(message.row_step) * max(1, int(message.height))
    actual_bytes = len(message.data)
    if declared_count <= 0:
        errors.append("message contains zero events")
    if int(message.point_step) <= 0:
        errors.append("point_step must be positive")
    if actual_bytes < expected_bytes:
        errors.append(
            f"data is truncated: {actual_bytes} bytes, expected at least {expected_bytes}"
        )

    sampled = 0
    minimum_offset: int | None = None
    maximum_offset: int | None = None
    positive_count = 0
    if not missing and declared_count > 0:
        try:
            rows = read_named_points(message, REQUIRED_FIELDS)
            for row in rows:
                x, y, polarity, offset = (int(value) for value in row)
                if not 0 <= x < arguments.width:
                    errors.append(f"pixel_x={x} outside [0,{arguments.width - 1}]")
                    break
                if not 0 <= y < arguments.height:
                    errors.append(f"pixel_y={y} outside [0,{arguments.height - 1}]")
                    break
                if polarity not in (0, 1):
                    errors.append(f"polarity={polarity}; expected 0 or 1")
                    break
                if offset < 0:
                    errors.append(f"time_offset_us={offset}; expected non-negative")
                    break
                sampled += 1
                positive_count += polarity
                minimum_offset = offset if minimum_offset is None else min(minimum_offset, offset)
                maximum_offset = offset if maximum_offset is None else max(maximum_offset, offset)
                if sampled >= arguments.sample_limit:
                    break
        except (IndexError, RuntimeError, TypeError, ValueError) as error:
            errors.append(f"PointCloud2 parse failed: {type(error).__name__}: {error}")

    if declared_count > arguments.sample_limit:
        warnings.append(
            f"validated the first {arguments.sample_limit} of {declared_count} events"
        )
    stamp = message.header.stamp
    stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
    if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
        errors.append("ROS header timestamp is missing or invalid")

    return {
        "schema_version": 1,
        "generated_utc": utc_now(),
        "status": "PASS" if not errors else "FAIL",
        "topic": arguments.topic,
        "message_type": "sensor_msgs/msg/PointCloud2",
        "frame_id": frame_id,
        "ros_stamp_sec": stamp_sec,
        "declared_event_count": declared_count,
        "validated_event_count": sampled,
        "image_width": arguments.width,
        "image_height": arguments.height,
        "point_step": int(message.point_step),
        "row_step": int(message.row_step),
        "data_bytes": actual_bytes,
        "fields": fields,
        "minimum_time_offset_us": minimum_offset,
        "maximum_time_offset_us": maximum_offset,
        "positive_fraction": None if sampled == 0 else positive_count / sampled,
        "publishes_messages": False,
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: list[str] | None = None) -> int:
    arguments, ros_arguments = build_parser().parse_known_args(argv)
    if not arguments.topic.startswith("/"):
        print("--topic must be absolute", file=sys.stderr)
        return 2
    if arguments.width <= 0 or arguments.height <= 0:
        print("--width and --height must be positive", file=sys.stderr)
        return 2
    if arguments.timeout_sec <= 0.0 or arguments.sample_limit <= 0:
        print("--timeout-sec and --sample-limit must be positive", file=sys.stderr)
        return 2
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
    except ImportError as error:
        print(f"ROS2 Python environment unavailable: {error}", file=sys.stderr)
        return 2

    received: list[Any] = []
    rclpy.init(args=ros_arguments)
    node = Node("dg_event_topic_inspector")
    node.create_subscription(
        PointCloud2,
        arguments.topic,
        lambda message: received.append(message),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + arguments.timeout_sec
    try:
        while not received and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if received:
        report = validate_message(received[0], arguments)
    else:
        report = {
            "schema_version": 1,
            "generated_utc": utc_now(),
            "status": "FAIL",
            "topic": arguments.topic,
            "publishes_messages": False,
            "errors": [f"no PointCloud2 received within {arguments.timeout_sec:.1f} seconds"],
            "warnings": [],
        }
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if arguments.output_json:
        Path(arguments.output_json).expanduser().write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
