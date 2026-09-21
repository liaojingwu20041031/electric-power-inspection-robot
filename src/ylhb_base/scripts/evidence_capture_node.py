#!/usr/bin/env python3
"""Write evaluator-ready CSV evidence from DG side-channel ROS topics.

The node only subscribes and writes files. It has no publishers, service
clients, action clients, or parameter clients and therefore cannot command the
robot. Raw messages remain preserved independently by ``ros2 bag record``.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Sequence


DIAGNOSTIC_TOPICS = (
    "/dg/lidar/quality",
    "/dg/gnss/quality",
    "/dg/uwb/quality",
    "/dg/event_camera/quality",
    "/dg/event_camera/feature_quality",
    "/dg/mmwave/quality",
    "/dg/fusion/status",
    "/dg/navigation/status",
    "/dg/relocalization/match_quality",
    "/dg/relocalization/status",
)

TERMINAL_RELOCALIZATION_STATES = {"RECOVERED", "FAILED", "MANUAL_REQUIRED"}
TERMINAL_PATROL_STATES = {"succeeded", "failed", "canceled", "cancelled"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_seconds(message: Any, fallback: float) -> float:
    try:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) / 1e9
        return value if value > 0.0 and math.isfinite(value) else fallback
    except (AttributeError, TypeError, ValueError):
        return fallback


def select_diagnostic(message: Any, hint: str = "") -> Any | None:
    selected = None
    upper_hint = hint.upper()
    for status in getattr(message, "status", ()):
        if selected is None:
            selected = status
        if upper_hint and upper_hint in str(getattr(status, "name", "")).upper():
            return status
    return selected


def diagnostic_values(status: Any | None) -> dict[str, str]:
    if status is None:
        return {}
    return {str(item.key): str(item.value) for item in getattr(status, "values", ())}


def finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def positive_int(value: Any) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def encode_feature_points(poses: Iterable[Any]) -> str:
    points = []
    for pose in poses:
        try:
            x = float(pose.position.x)
            y = float(pose.position.y)
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append([x, y])
    return json.dumps(points, ensure_ascii=True, separators=(",", ":"))


class CsvSink:
    def __init__(self, path: Path, fieldnames: Sequence[str]) -> None:
        self.path = path
        self._handle = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=fieldnames, extrasaction="ignore")
        self._writer.writeheader()
        self._handle.flush()

    def write(self, row: dict[str, Any]) -> None:
        self._writer.writerow(row)
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()


class RelocalizationAttemptTracker:
    """Collapse repeated status samples into one terminal row per attempt."""

    def __init__(self) -> None:
        self._terminal_attempts: set[int] = set()

    def consume(self, values: dict[str, str], received_timestamp: float) -> dict[str, Any] | None:
        attempt_id = positive_int(values.get("attempt_id"))
        state = str(values.get("state", "")).strip().upper()
        if attempt_id is None or state not in TERMINAL_RELOCALIZATION_STATES:
            return None
        if attempt_id in self._terminal_attempts:
            return None
        self._terminal_attempts.add(attempt_id)
        elapsed = finite_float(values.get("elapsed"))
        timestamp = finite_float(values.get("timestamp"))
        return {
            "timestamp": received_timestamp if timestamp is None else timestamp,
            "attempt_id": attempt_id,
            "outcome": state,
            "time_to_recovery": elapsed if state == "RECOVERED" else "",
            "failure_reason": str(values.get("failure_reason", "")).strip(),
        }


class PatrolLifecycleTracker:
    """Emit one start and one terminal event for the observed patrol run."""

    def __init__(self) -> None:
        self.started = False
        self.terminal = False

    def consume(self, payload: Any) -> str | None:
        if not isinstance(payload, dict):
            return None
        state = str(payload.get("state") or payload.get("patrol_mode_state") or "").strip().lower()
        if state in {"running", "returning_home"} and not self.started:
            self.started = True
            return "started"
        if self.started and state in TERMINAL_PATROL_STATES and not self.terminal:
            self.terminal = True
            return "terminal"
        return None


def create_node(
    node_name: str,
    output_dir: Path,
    ready_file: Path,
    start_file: Path,
    estimated_pose_topic: str,
    ground_truth: tuple[float, float, float | None] | None,
    ground_truth_frame: str,
    patrol_start_file: Path | None = None,
    patrol_terminal_file: Path | None = None,
) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    class EvidenceCaptureNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            output_dir.mkdir(parents=True, exist_ok=True)
            self._started_utc = utc_now()
            self._capture_started_utc: str | None = None
            self._counts: dict[str, int] = {}
            self._frame_sequence = 0
            self._ground_truth = ground_truth
            self._ground_truth_frame = ground_truth_frame
            self._estimated_pose_topic = estimated_pose_topic
            self._attempt_tracker = RelocalizationAttemptTracker()
            self._patrol_tracker = PatrolLifecycleTracker()
            self._sinks = {
                "diagnostics": CsvSink(
                    output_dir / "diagnostic_samples.csv",
                    (
                        "received_timestamp",
                        "message_timestamp",
                        "topic",
                        "status_name",
                        "level",
                        "message",
                        "values_json",
                    ),
                ),
                "lidar_features": CsvSink(
                    output_dir / "lidar_features.csv",
                    ("frame_id", "timestamp", "coordinate_frame", "feature_points", "valid_feature_count"),
                ),
                "lidar_quality": CsvSink(
                    output_dir / "lidar_quality.csv",
                    (
                        "timestamp",
                        "raw_point_count",
                        "valid_point_count",
                        "valid_ratio",
                        "outlier_count",
                        "dynamic_candidate_count",
                        "temporal_dynamic_candidate_count",
                        "auxiliary_dynamic_sector_count",
                        "auxiliary_dynamic_point_count",
                        "auxiliary_dynamic_mask_enabled",
                        "auxiliary_dynamic_data_fresh",
                        "auxiliary_dynamic_input_count",
                        "auxiliary_dynamic_frame_id",
                        "auxiliary_dynamic_reject_reason",
                        "stable_point_count",
                        "feature_count",
                        "angular_coverage",
                        "temporal_match_ratio",
                        "raw_temporal_match_ratio",
                        "geometry_score",
                        "temporal_status",
                    ),
                ),
                "relocalization_samples": CsvSink(
                    output_dir / "relocalization_status_samples.csv",
                    (
                        "timestamp",
                        "state",
                        "attempt_id",
                        "trigger_reason",
                        "elapsed",
                        "active_segment",
                        "verification_count",
                        "candidate_score",
                        "candidate_inlier_ratio",
                        "candidate_mean_distance",
                        "lidar_health",
                        "gnss_health",
                        "amcl_health",
                        "failure_reason",
                        "manual_takeover",
                        "request_candidate",
                        "candidate_request_time",
                        "seed_source",
                        "command_angular_z",
                        "health_reasons",
                    ),
                ),
                "relocalization_events": CsvSink(
                    output_dir / "relocalization_events.csv",
                    ("timestamp", "attempt_id", "outcome", "time_to_recovery", "failure_reason"),
                ),
                "fusion_status": CsvSink(
                    output_dir / "fusion_status_samples.csv",
                    (
                        "timestamp",
                        "fusion_mode",
                        "accepted_source",
                        "updated",
                        "output_frame_id",
                        "output_child_frame_id",
                        "uwb_alignment_ready",
                        "position_uncertainty",
                        "yaw_uncertainty",
                        "altitude",
                        "altitude_uncertainty",
                        "altitude_anchored",
                        "altitude_source",
                        "measurement_confidence",
                        "reasons",
                    ),
                ),
                "position": CsvSink(
                    output_dir / "position_samples.csv",
                    (
                        "timestamp",
                        "estimated_x",
                        "estimated_y",
                        "estimated_z",
                        "ground_truth_x",
                        "ground_truth_y",
                        "ground_truth_z",
                        "coordinate_frame",
                        "source_topic",
                    ),
                ),
            }

            self.create_subscription(PoseArray, "/dg/lidar/stable_features", self._on_features, 50)
            self.create_subscription(
                PoseWithCovarianceStamped, estimated_pose_topic, self._on_estimated_pose, 50
            )
            for topic in DIAGNOSTIC_TOPICS:
                self.create_subscription(
                    DiagnosticArray,
                    topic,
                    lambda message, source_topic=topic: self._on_diagnostic(source_topic, message),
                    50,
                )
            if patrol_start_file is not None or patrol_terminal_file is not None:
                self.create_subscription(String, "/patrol/status", self._on_patrol_status, 20)

            ready_file.parent.mkdir(parents=True, exist_ok=True)
            ready_file.write_text(
                json.dumps(
                    {
                        "ready_utc": utc_now(),
                        "node": node_name,
                        "publishes_motion_commands": False,
                        "estimated_pose_topic": estimated_pose_topic,
                        "ground_truth_enabled": ground_truth is not None,
                        "ground_truth_z_enabled": ground_truth is not None and ground_truth[2] is not None,
                        "ground_truth_frame": ground_truth_frame,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"evaluator-ready evidence capture active: {output_dir}")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _count(self, stream: str) -> None:
            self._counts[stream] = self._counts.get(stream, 0) + 1

        def _accepting(self) -> bool:
            if not start_file.is_file():
                return False
            if self._capture_started_utc is None:
                self._capture_started_utc = utc_now()
            return True

        def _on_features(self, message: Any) -> None:
            if not self._accepting():
                return
            self._frame_sequence += 1
            received = self._now()
            encoded = encode_feature_points(message.poses)
            count = len(json.loads(encoded))
            self._sinks["lidar_features"].write(
                {
                    "frame_id": self._frame_sequence,
                    "timestamp": stamp_seconds(message, received),
                    "coordinate_frame": str(message.header.frame_id),
                    "feature_points": encoded,
                    "valid_feature_count": count,
                }
            )
            self._count("lidar_features")

        @staticmethod
        def _write_signal(path: Path | None, payload: dict[str, Any]) -> None:
            if path is None:
                return
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            temporary.replace(path)

        def _on_patrol_status(self, message: Any) -> None:
            try:
                payload = json.loads(str(message.data))
            except (TypeError, ValueError):
                return
            event = self._patrol_tracker.consume(payload)
            if event == "started":
                self._write_signal(
                    patrol_start_file,
                    {"event": event, "observed_utc": utc_now(), "status": payload},
                )
            elif event == "terminal":
                self._write_signal(
                    patrol_terminal_file,
                    {"event": event, "observed_utc": utc_now(), "status": payload},
                )

        def _on_estimated_pose(self, message: Any) -> None:
            if not self._accepting():
                return
            self._count("estimated_pose_received")
            if self._ground_truth is None:
                return
            coordinate_frame = str(message.header.frame_id)
            if coordinate_frame != self._ground_truth_frame:
                self._count("estimated_pose_frame_mismatch")
                return
            try:
                x = float(message.pose.pose.position.x)
                y = float(message.pose.pose.position.y)
                z = float(message.pose.pose.position.z)
            except (AttributeError, TypeError, ValueError):
                self._count("estimated_pose_invalid")
                return
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                self._count("estimated_pose_invalid")
                return
            self._sinks["position"].write(
                {
                    "timestamp": stamp_seconds(message, self._now()),
                    "estimated_x": x,
                    "estimated_y": y,
                    "estimated_z": z,
                    "ground_truth_x": self._ground_truth[0],
                    "ground_truth_y": self._ground_truth[1],
                    "ground_truth_z": "" if self._ground_truth[2] is None else self._ground_truth[2],
                    "coordinate_frame": coordinate_frame,
                    "source_topic": self._estimated_pose_topic,
                }
            )
            self._count("position_samples")

        def _on_diagnostic(self, topic: str, message: Any) -> None:
            if not self._accepting():
                return
            received = self._now()
            message_timestamp = stamp_seconds(message, received)
            for status in message.status:
                values = diagnostic_values(status)
                self._sinks["diagnostics"].write(
                    {
                        "received_timestamp": received,
                        "message_timestamp": message_timestamp,
                        "topic": topic,
                        "status_name": str(status.name),
                        "level": int(status.level),
                        "message": str(status.message),
                        "values_json": json.dumps(values, ensure_ascii=False, sort_keys=True),
                    }
                )
                self._count(f"diagnostic:{topic}")

            selected = select_diagnostic(message)
            values = diagnostic_values(selected)
            if topic == "/dg/lidar/quality" and selected is not None:
                row = dict(values)
                row["timestamp"] = finite_float(values.get("timestamp")) or message_timestamp
                self._sinks["lidar_quality"].write(row)
                self._count("lidar_quality")
            elif topic == "/dg/fusion/status" and selected is not None:
                row = dict(values)
                row["timestamp"] = finite_float(values.get("timestamp")) or message_timestamp
                self._sinks["fusion_status"].write(row)
                self._count("fusion_status")
            elif topic == "/dg/relocalization/status" and selected is not None:
                row = dict(values)
                row["timestamp"] = finite_float(values.get("timestamp")) or message_timestamp
                self._sinks["relocalization_samples"].write(row)
                self._count("relocalization_samples")
                event = self._attempt_tracker.consume(values, received)
                if event is not None:
                    self._sinks["relocalization_events"].write(event)
                    self._count("relocalization_events")

        def close_evidence(self) -> None:
            for sink in self._sinks.values():
                sink.close()
            summary = {
                "schema_version": 1,
                "started_utc": self._started_utc,
                "capture_started_utc": self._capture_started_utc,
                "ended_utc": utc_now(),
                "publishes_motion_commands": False,
                "estimated_pose_topic": self._estimated_pose_topic,
                "ground_truth_enabled": self._ground_truth is not None,
                "ground_truth_z_enabled": self._ground_truth is not None and self._ground_truth[2] is not None,
                "ground_truth_frame": self._ground_truth_frame,
                "record_counts": dict(sorted(self._counts.items())),
            }
            (output_dir / "capture_summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )

    return EvidenceCaptureNode()


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Subscribe to DG evidence topics and write CSV files")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--ready-file", required=True)
    parser.add_argument("--start-file", required=True)
    parser.add_argument("--node-name", default="dg_evidence_capture_node")
    parser.add_argument("--estimated-pose-topic", default="/dg/fusion/pose")
    parser.add_argument("--ground-truth-x", type=float)
    parser.add_argument("--ground-truth-y", type=float)
    parser.add_argument("--ground-truth-z", type=float)
    parser.add_argument("--ground-truth-frame", default="map")
    parser.add_argument("--patrol-start-file")
    parser.add_argument("--patrol-terminal-file")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments, ros_args = create_parser().parse_known_args(argv)
    supplied = (arguments.ground_truth_x is not None, arguments.ground_truth_y is not None)
    if supplied[0] != supplied[1]:
        raise SystemExit("--ground-truth-x and --ground-truth-y must be supplied together")
    if arguments.ground_truth_z is not None and not all(supplied):
        raise SystemExit("--ground-truth-z requires --ground-truth-x and --ground-truth-y")
    if not arguments.estimated_pose_topic.startswith("/"):
        raise SystemExit("--estimated-pose-topic must be an absolute ROS topic")
    if not arguments.ground_truth_frame.strip():
        raise SystemExit("--ground-truth-frame must not be empty")
    ground_truth = (
        (
            float(arguments.ground_truth_x),
            float(arguments.ground_truth_y),
            None if arguments.ground_truth_z is None else float(arguments.ground_truth_z),
        )
        if all(supplied)
        else None
    )
    if ground_truth is not None and not all(
        value is None or math.isfinite(value) for value in ground_truth
    ):
        raise SystemExit("ground-truth coordinates must be finite")

    try:
        import rclpy
    except ImportError:
        print("ERROR: rclpy is unavailable; source the ROS 2 workspace first.", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = create_node(
        arguments.node_name,
        Path(arguments.output_dir).expanduser().resolve(),
        Path(arguments.ready_file).expanduser().resolve(),
        Path(arguments.start_file).expanduser().resolve(),
        arguments.estimated_pose_topic,
        ground_truth,
        arguments.ground_truth_frame.strip(),
        Path(arguments.patrol_start_file).expanduser().resolve() if arguments.patrol_start_file else None,
        Path(arguments.patrol_terminal_file).expanduser().resolve() if arguments.patrol_terminal_file else None,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_evidence()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
