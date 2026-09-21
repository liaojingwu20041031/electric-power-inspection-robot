#!/usr/bin/env python3
"""Replay the saved real UWB measurements into the MODEL-1 ROS interface."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

from uwb_data_model import UwbAnchorConfig
from uwb_input import build_logical_observations, load_replay_frames
from uwb_quality_gate import UwbQualityConfig, UwbQualityGate


class UwbReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_replay_node")
        self.declare_parameter("input_file", "")
        self.declare_parameter("playback_rate_hz", 25.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("anchor0_x", 0.0)
        self.declare_parameter("anchor0_y", 0.0)
        self.declare_parameter("anchor1_x", 2.40)
        self.declare_parameter("anchor1_y", 0.0)
        self.declare_parameter("anchor2_x", 1.20)
        self.declare_parameter("anchor2_y", 1.80)
        self.declare_parameter("pose_topic", "/dg/uwb/pose")
        self.declare_parameter("quality_topic", "/dg/uwb/quality")
        self.declare_parameter("frame_id", "uwb")
        self.declare_parameter("six_logical_slots", True)

        input_file = str(self.get_parameter("input_file").value).strip()
        if not input_file:
            raise ValueError("input_file is required when UWB replay is enabled")
        self._frames = load_replay_frames(input_file)
        self._frame_index = 0
        self._loop = bool(self.get_parameter("loop").value)
        rate = float(self.get_parameter("playback_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("playback_rate_hz must be positive")

        anchors = [
            UwbAnchorConfig(0, float(self.get_parameter("anchor0_x").value), float(self.get_parameter("anchor0_y").value), 0.0),
            UwbAnchorConfig(1, float(self.get_parameter("anchor1_x").value), float(self.get_parameter("anchor1_y").value), 0.0),
            UwbAnchorConfig(2, float(self.get_parameter("anchor2_x").value), float(self.get_parameter("anchor2_y").value), 0.0),
        ]
        self._gate = UwbQualityGate(UwbQualityConfig(), anchors)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), 10
        )
        self._quality_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("quality_topic").value), 10
        )
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"UWB measured-data replay ready: {input_file}, frames={len(self._frames)}, rate={rate:.1f} Hz"
        )

    def _tick(self) -> None:
        if self._frame_index >= len(self._frames):
            if self._loop:
                self._frame_index = 0
            else:
                self._timer.cancel()
                self.get_logger().info("UWB measured-data replay complete")
                return
        now = self.get_clock().now().nanoseconds / 1e9
        ranges = self._frames[self._frame_index]
        source_index = self._frame_index
        self._frame_index += 1
        observations = build_logical_observations(
            now,
            ranges,
            six_logical_slots=bool(self.get_parameter("six_logical_slots").value),
        )
        estimate, decision = self._gate.update(observations, now)
        self._publish_quality(estimate, decision, source_index)
        if estimate is not None and decision in ("ACCEPT", "ACCEPT_DEGRADED"):
            self._publish_pose(estimate)

    def _publish_pose(self, estimate) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.pose.pose.position.x = estimate.x
        message.pose.pose.position.y = estimate.y
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = estimate.covariance_xx
        message.pose.covariance[1] = estimate.covariance_xy
        message.pose.covariance[6] = estimate.covariance_xy
        message.pose.covariance[7] = estimate.covariance_yy
        for index in (14, 21, 28, 35):
            message.pose.covariance[index] = 1e6
        self._pose_pub.publish(message)

    def _publish_quality(self, estimate, decision: str, source_index: int) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "DG UWB"
        state = self._gate.state.state
        status.level = (
            DiagnosticStatus.OK
            if state == "GOOD"
            else DiagnosticStatus.WARN
            if state in ("DEGRADED", "RECOVERING")
            else DiagnosticStatus.ERROR
        )
        status.message = state
        fields = {
            "current_state": state,
            "decision": decision,
            "accepted": decision in ("ACCEPT", "ACCEPT_DEGRADED"),
            "unique_anchor_count": self._gate.state.unique_physical_count,
            "logical_anchor_count": 6 if bool(self.get_parameter("six_logical_slots").value) else 3,
            "physical_source_count": 3,
            "logical_physical_mapping": "A0:0,A1:1,A2:2,A3:0,A4:1,A5:2",
            "rejection_reason": self._gate.state.rejection_reason,
            "source_frame_index": source_index,
            "source_kind": "MEASURED_REPLAY",
        }
        if estimate is not None:
            fields.update(
                {
                    "x": estimate.x,
                    "y": estimate.y,
                    "residual_rms": estimate.residual_rms,
                    "geometry_score": estimate.geometry_metric,
                    "confidence": estimate.confidence,
                }
            )
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in fields.items()]
        array.status = [status]
        self._quality_pub.publish(array)


def main() -> None:
    rclpy.init()
    node = UwbReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
