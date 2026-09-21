#!/usr/bin/env python3
"""ROS2 adapter for the side-channel GNSS quality gate POC."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ros_shutdown import cleanup_ros_node  # noqa: E402
from gnss_quality_gate import (  # noqa: E402
    ACCEPT,
    ACCEPT_DEGRADED,
    GnssGateConfig,
    GnssObservation,
    GnssQualityGate,
    parse_status_values,
)
from diagnostic_level import normalize_diagnostic_level  # noqa: E402


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_stamp(stamp: Any) -> bool:
    try:
        value = _stamp_to_seconds(stamp)
    except (AttributeError, TypeError, ValueError):
        return False
    return math.isfinite(value) and value > 0.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the side-channel DG-202611 GNSS quality gate node.")
    parser.add_argument("--node-name", default="gnss_quality_node", help="ROS node name")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    class GnssQualityNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("fix_topic", "/gps/fix")
            self.declare_parameter("status_topic", "/gps/rtk_status")
            self.declare_parameter("quality_topic", "/dg/gnss/quality")
            self.declare_parameter("accepted_fix_topic", "/dg/gnss/accepted_fix")
            self.declare_parameter("publish_accepted_fix", True)
            self.declare_parameter("min_satellites_good", 8)
            self.declare_parameter("min_satellites_degraded", 4)
            self.declare_parameter("max_hdop_good", 1.5)
            self.declare_parameter("max_hdop_degraded", 3.0)
            self.declare_parameter("max_differential_age", 3.0)
            self.declare_parameter("stale_timeout", 3.0)
            self.declare_parameter("status_timeout", 3.0)
            self.declare_parameter("max_jump_distance", 5.0)
            self.declare_parameter("max_implied_speed", 3.0)
            self.declare_parameter("recovery_good_samples", 3)
            self._config = self._read_config()
            self._gate = GnssQualityGate(self._config)
            self._latest_status: dict[str, Any] | None = None
            self._status_warned = False
            self._quality_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("quality_topic").value), 10
            )
            self._accepted_fix_pub = self.create_publisher(
                NavSatFix, str(self.get_parameter("accepted_fix_topic").value), 10
            )
            self.create_subscription(
                NavSatFix,
                str(self.get_parameter("fix_topic").value),
                self._on_fix,
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("status_topic").value),
                self._on_status,
                10,
            )
            self.get_logger().info(
                "GNSS quality gate side-channel active; /gps/fix is never replaced "
                "and accepted fixes are not connected to EKF"
            )

        def _read_config(self) -> GnssGateConfig:
            names = (
                "min_satellites_good",
                "min_satellites_degraded",
                "max_hdop_good",
                "max_hdop_degraded",
                "max_differential_age",
                "stale_timeout",
                "status_timeout",
                "max_jump_distance",
                "max_implied_speed",
                "recovery_good_samples",
            )
            values = {name: self.get_parameter(name).value for name in names}
            return GnssGateConfig(**values)

        def _on_status(self, message: Any) -> None:
            now = self.get_clock().now().nanoseconds / 1e9
            selected = None
            for status in message.status:
                if selected is None or "RTK" in str(status.name).upper():
                    selected = status
                if "RTK" in str(status.name).upper():
                    break
            if selected is None:
                self._latest_status = None
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            parsed = parse_status_values(
                values,
                normalize_diagnostic_level(selected.level, default=3),
            )
            message_stamp = _stamp_to_seconds(message.header.stamp) if _finite_stamp(message.header.stamp) else None
            self._latest_status = {
                "parsed": parsed,
                "message_stamp": message_stamp,
                "received_time": now,
            }

        def _on_fix(self, message: Any) -> None:
            now = self.get_clock().now().nanoseconds / 1e9
            fix_stamp = _stamp_to_seconds(message.header.stamp) if _finite_stamp(message.header.stamp) else now
            status = self._latest_status
            status_reasons: list[str] = []
            status_age: float | None = None
            stale = False
            if status is None:
                status_reasons.append("STATUS_MISSING")
            else:
                parsed = status["parsed"]
                status_reasons.extend(parsed.reasons)
                reference_stamp = status["message_stamp"]
                status_age = now - reference_stamp if reference_stamp is not None else now - status["received_time"]
                if status_age > self._config.status_timeout:
                    stale = True
                    status_reasons.append("STALE_STATUS")
                stale = stale or parsed.stale
            parsed = status["parsed"] if status is not None else None
            observation = GnssObservation(
                timestamp=fix_stamp,
                latitude=float(message.latitude),
                longitude=float(message.longitude),
                altitude=float(message.altitude) if math.isfinite(float(message.altitude)) else None,
                fix_available=int(message.status.status) != int(NavSatStatus.STATUS_NO_FIX),
                quality=parsed.quality if parsed is not None else None,
                satellites=parsed.satellites if parsed is not None else None,
                hdop=parsed.hdop if parsed is not None else None,
                differential_age=parsed.differential_age if parsed is not None else None,
                stale=stale,
                status_age=status_age,
                status_reasons=tuple(dict.fromkeys(status_reasons)),
            )
            result = self._gate.evaluate(observation, now=now)
            self._publish_quality(message, observation, result)
            publish_accepted = bool(self.get_parameter("publish_accepted_fix").value)
            if publish_accepted and result.decision in {ACCEPT, ACCEPT_DEGRADED}:
                self._accepted_fix_pub.publish(message)

        def _publish_quality(self, source: Any, observation: GnssObservation, result: Any) -> None:
            array = DiagnosticArray()
            array.header = source.header
            status = DiagnosticStatus()
            status.name = "DG GNSS Quality Gate"
            if result.current_state == "GOOD":
                status.level = DiagnosticStatus.OK
            elif result.current_state == "DEGRADED" or result.current_state == "RECOVERING":
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR
            status.message = f"{result.current_state}/{result.decision}"
            fields = {
                "previous_state": result.previous_state,
                "current_state": result.current_state,
                "decision": result.decision,
                "accepted": result.accepted,
                "reasons": ";".join(result.reasons),
                "distance_from_previous": result.distance_from_previous,
                "dt": result.dt,
                "implied_speed": result.implied_speed,
                "recovery_count": result.recovery_count,
                "quality": observation.quality,
                "satellites": observation.satellites,
                "hdop": observation.hdop,
                "differential_age": observation.differential_age,
                "stale": observation.stale,
                "status_age": observation.status_age,
            }
            status.values = [
                KeyValue(key=str(key), value="" if value is None else str(value))
                for key, value in fields.items()
            ]
            array.status = [status]
            self._quality_pub.publish(array)

    return GnssQualityNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the gate core is ROS-independent.", file=sys.stderr)
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
