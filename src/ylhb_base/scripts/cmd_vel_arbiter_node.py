#!/usr/bin/env python3
"""ROS2 adapter for the single-output DG-202611 command arbiter."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_arbiter_core import ArbiterConfig, ArbiterInput, CmdVelArbiter, TwistCommand  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Arbitrate manual, navigation and recovery velocity commands.")
    parser.add_argument("--node-name", default="cmd_vel_arbiter_node", help="ROS node name")
    return parser


def create_node(node_name: str) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool

    class CmdVelArbiterNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.declare_parameter("navigation_topic", "/cmd_vel_nav")
            self.declare_parameter("recovery_topic", "/cmd_vel_recovery")
            self.declare_parameter("output_topic", "/cmd_vel")
            self.declare_parameter("navigation_status_topic", "/dg/navigation/status")
            self.declare_parameter("manual_topic", "/dg/relocalization/manual_takeover")
            self.declare_parameter("manual_command_topic", "/cmd_vel_manual")
            self.declare_parameter("manual_authorization_topic", "/dg/control/manual_authorized")
            self.declare_parameter("source_timeout", 0.5)
            self.declare_parameter("switch_guard_duration", 0.2)
            self.declare_parameter("status_timeout", 1.0)
            self.declare_parameter("timer_period", 0.05)
            self._arbiter = CmdVelArbiter(
                ArbiterConfig(
                    source_timeout=float(self.get_parameter("source_timeout").value),
                    switch_guard_duration=float(self.get_parameter("switch_guard_duration").value),
                    status_timeout=float(self.get_parameter("status_timeout").value),
                )
            )
            self._now_value = self._now()
            self._nav_cmd: TwistCommand | None = None
            self._nav_received_at: float | None = None
            self._recovery_cmd: TwistCommand | None = None
            self._recovery_received_at: float | None = None
            self._manual_cmd: TwistCommand | None = None
            self._manual_received_at: float | None = None
            self._manual_authorized = False
            self._navigation_state = "DEGRADED"
            self._navigation_status_received_at: float | None = None
            self._manual_takeover = False
            self._output_pub = self.create_publisher(
                Twist, str(self.get_parameter("output_topic").value), 10
            )
            self.create_subscription(
                Twist, str(self.get_parameter("navigation_topic").value), self._on_nav, 10
            )
            self.create_subscription(
                Twist, str(self.get_parameter("recovery_topic").value), self._on_recovery, 10
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("navigation_status_topic").value),
                self._on_navigation_status,
                10,
            )
            self.create_subscription(
                Bool, str(self.get_parameter("manual_topic").value), self._on_manual, 10
            )
            self.create_subscription(
                Twist,
                str(self.get_parameter("manual_command_topic").value),
                self._on_manual_command,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("manual_authorization_topic").value),
                self._on_manual_authorization,
                10,
            )
            self._timer = self.create_timer(float(self.get_parameter("timer_period").value), self._tick)
            self.get_logger().info("DG command arbiter active; only this node publishes the integration /cmd_vel")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_nav(self, message: Any) -> None:
            self._nav_cmd = TwistCommand(float(message.linear.x), float(message.angular.z))
            self._nav_received_at = self._now()

        def _on_recovery(self, message: Any) -> None:
            self._recovery_cmd = TwistCommand(float(message.linear.x), float(message.angular.z))
            self._recovery_received_at = self._now()

        def _on_manual_command(self, message: Any) -> None:
            self._manual_cmd = TwistCommand(float(message.linear.x), float(message.angular.z))
            self._manual_received_at = self._now()

        def _on_manual_authorization(self, message: Any) -> None:
            self._manual_authorized = bool(message.data)
            if not self._manual_authorized:
                self._manual_cmd = None
                self._manual_received_at = None
                self._publish_zero()

        def _on_navigation_status(self, message: Any) -> None:
            selected = None
            for status in message.status:
                if "NAVIGATION HEALTH" in str(status.name).upper():
                    selected = status
                    break
                if selected is None:
                    selected = status
            if selected is None:
                return
            values = {str(item.key): str(item.value) for item in selected.values}
            self._navigation_state = values.get("overall_state") or str(selected.message) or "DEGRADED"
            self._navigation_status_received_at = self._now()

        def _on_manual(self, message: Any) -> None:
            self._manual_takeover = bool(message.data)
            if self._manual_takeover:
                self._publish_zero()
                self._tick()

        def _tick(self) -> None:
            now = self._now()
            output = self._arbiter.process(
                ArbiterInput(
                    now=now,
                    navigation_command=self._nav_cmd,
                    navigation_received_at=self._nav_received_at,
                    recovery_command=self._recovery_cmd,
                    recovery_received_at=self._recovery_received_at,
                    manual_command=self._manual_cmd,
                    manual_received_at=self._manual_received_at,
                    manual_authorized=self._manual_authorized,
                    navigation_state=self._navigation_state,
                    manual_takeover=self._manual_takeover,
                    navigation_status_received_at=self._navigation_status_received_at,
                )
            )
            message = Twist()
            message.linear.x = output.command.linear_x
            message.angular.z = output.command.angular_z
            self._output_pub.publish(message)

        def _publish_zero(self) -> None:
            message = Twist()
            self._output_pub.publish(message)

        def destroy_node(self) -> bool:
            self._publish_zero()
            return super().destroy_node()

    return CmdVelArbiterNode()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        import rclpy
    except ImportError:
        print("ROS2/rclpy is required only for the wrapper node; the core is ROS-independent.", file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = create_node(args.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS 2 may already have shut down the context from its signal handler.
        # Avoid a second shutdown call turning a normal Ctrl-C into an error.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
