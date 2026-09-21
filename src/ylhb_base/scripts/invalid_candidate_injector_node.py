#!/usr/bin/env python3
"""Publish a bounded, explicit invalid scan-match candidate for MODEL-3 testing.

This tool only publishes ``diagnostic_msgs/DiagnosticArray`` on the configured
match-quality topic. It must be used with the real scan-map matcher disabled and
the command arbiter disabled. It has no motion-message imports or interfaces.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any, Sequence


def bool_value(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def status_values(message: Any) -> dict[str, str]:
    selected = None
    for status in getattr(message, "status", ()):
        if selected is None:
            selected = status
        if "RELOCALIZATION" in str(getattr(status, "name", "")).upper():
            selected = status
            break
    if selected is None:
        return {}
    return {str(item.key): str(item.value) for item in getattr(selected, "values", ())}


def create_node(node_name: str, status_topic: str, match_topic: str, max_injections: int) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node

    class InvalidCandidateInjector(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.injection_count = 0
            self.last_injection_monotonic: float | None = None
            self._handled_requests: set[tuple[str, str]] = set()
            self._max_injections = max_injections
            self._publisher = self.create_publisher(DiagnosticArray, match_topic, 10)
            self.create_subscription(DiagnosticArray, status_topic, self._on_status, 10)
            self.get_logger().info(
                f"controlled invalid-candidate injector armed: {status_topic} -> {match_topic}; "
                f"limit={max_injections}"
            )

        def _on_status(self, message: Any) -> None:
            if self.injection_count >= self._max_injections:
                return
            values = status_values(message)
            if not bool_value(values.get("request_candidate", "false")):
                return
            request_key = (
                str(values.get("attempt_id", "")),
                str(values.get("candidate_request_time", "")),
            )
            if request_key in self._handled_requests:
                return
            self._handled_requests.add(request_key)
            now = self.get_clock().now()
            array = DiagnosticArray()
            array.header.stamp = now.to_msg()
            status = DiagnosticStatus()
            status.name = "DG Controlled Invalid Scan-to-Map Candidate"
            status.level = DiagnosticStatus.ERROR
            status.message = "CONTROLLED_INVALID_CANDIDATE"
            fields = {
                "accepted": False,
                "score": 0.0,
                "mean_distance": 1_000_000.0,
                "inlier_ratio": 0.0,
                "used_points": 0,
                "candidate_x": 0.0,
                "candidate_y": 0.0,
                "candidate_yaw": 0.0,
                "reason": "CONTROLLED_INVALID_CANDIDATE",
                "timestamp": now.nanoseconds / 1e9,
            }
            status.values = [
                KeyValue(key=str(key), value=str(value)) for key, value in fields.items()
            ]
            array.status = [status]
            self._publisher.publish(array)
            self.injection_count += 1
            self.last_injection_monotonic = time.monotonic()
            self.get_logger().warning(
                f"published controlled invalid candidate {self.injection_count}/{self._max_injections} "
                f"for attempt={request_key[0]}"
            )

    return InvalidCandidateInjector()


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-name", default="dg_invalid_candidate_injector")
    parser.add_argument("--status-topic", default="/dg/relocalization/status")
    parser.add_argument("--match-topic", default="/dg/relocalization/match_quality")
    parser.add_argument("--max-injections", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments, ros_args = create_parser().parse_known_args(argv)
    if not arguments.status_topic.startswith("/") or not arguments.match_topic.startswith("/"):
        raise SystemExit("status and match topics must be absolute ROS topic names")
    if arguments.max_injections < 1:
        raise SystemExit("--max-injections must be positive")
    if not math.isfinite(arguments.timeout_sec) or arguments.timeout_sec <= 0.0:
        raise SystemExit("--timeout-sec must be finite and positive")
    try:
        import rclpy
    except ImportError:
        print("ERROR: rclpy is unavailable; source the ROS 2 workspace first.", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = create_node(
        arguments.node_name,
        arguments.status_topic,
        arguments.match_topic,
        arguments.max_injections,
    )
    deadline = time.monotonic() + arguments.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.injection_count >= arguments.max_injections
                and node.last_injection_monotonic is not None
                and time.monotonic() - node.last_injection_monotonic >= 1.0
            ):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.injection_count < arguments.max_injections:
        print("ERROR: no matching MODEL-3 candidate request was observed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
