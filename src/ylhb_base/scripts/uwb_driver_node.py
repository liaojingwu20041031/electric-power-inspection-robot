#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from uwb_data_model import UwbAnchorConfig
from uwb_input import build_logical_observations, parse_kt1_ranges
from uwb_quality_gate import UwbQualityGate, UwbQualityConfig


class UwbDriverNode(Node):

    def __init__(self):
        super().__init__("uwb_driver_node")

        # 串口参数
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)

        # 当前仅用于“跑通链路”的临时 Anchor 坐标
        # 硬件 A2 -> physical_0
        # 硬件 A3 -> physical_1
        # 硬件 A4 -> physical_2
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

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)

        anchors = [
            UwbAnchorConfig(
                physical_id=0,
                x=float(self.get_parameter("anchor0_x").value),
                y=float(self.get_parameter("anchor0_y").value),
                z=0.0,
            ),
            UwbAnchorConfig(
                physical_id=1,
                x=float(self.get_parameter("anchor1_x").value),
                y=float(self.get_parameter("anchor1_y").value),
                z=0.0,
            ),
            UwbAnchorConfig(
                physical_id=2,
                x=float(self.get_parameter("anchor2_x").value),
                y=float(self.get_parameter("anchor2_y").value),
                z=0.0,
            ),
        ]

        self.gate = UwbQualityGate(
            UwbQualityConfig(),
            anchors,
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            10,
        )

        self.quality_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("quality_topic").value),
            10,
        )

        self.ser = serial.Serial(
            port,
            baud,
            timeout=0,
        )
        self._rx_buffer = ""

        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info(
            f"UWB real driver started: {port} @ {baud}"
        )

    def read_serial(self):

        waiting = int(self.ser.in_waiting)
        if waiting <= 0:
            return
        self._rx_buffer += self.ser.read(waiting).decode("utf-8", errors="ignore")
        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            self.process_line(line)

    def process_line(self, line):
        ranges = parse_kt1_ranges(line)
        if ranges is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        observations = build_logical_observations(
            now,
            ranges,
            six_logical_slots=bool(self.get_parameter("six_logical_slots").value),
        )
        estimate, decision = self.gate.update(observations, now)
        self.publish_quality(estimate, decision)
        if estimate is not None and decision in ("ACCEPT", "ACCEPT_DEGRADED"):
            self.publish_pose(estimate)

    def publish_pose(self, estimate):

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)

        msg.pose.pose.position.x = estimate.x
        msg.pose.pose.position.y = estimate.y
        msg.pose.pose.position.z = 0.0

        # UWB 不提供 yaw
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance[0] = estimate.covariance_xx
        msg.pose.covariance[1] = estimate.covariance_xy
        msg.pose.covariance[6] = estimate.covariance_xy
        msg.pose.covariance[7] = estimate.covariance_yy

        # z / roll / pitch / yaw 均不由三基站 UWB 提供
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = 1e6

        self.pose_pub.publish(msg)

    def publish_quality(self, estimate, decision):

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "DG UWB"

        state = self.gate.state.state

        if state == "GOOD":
            status.level = DiagnosticStatus.OK
        elif state in ("DEGRADED", "RECOVERING"):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.ERROR

        status.message = state

        fields = {
            "current_state": state,
            "decision": decision,
            "accepted": decision in (
                "ACCEPT",
                "ACCEPT_DEGRADED",
            ),
            "unique_anchor_count":
                self.gate.state.unique_physical_count,
            "logical_anchor_count": 6 if bool(self.get_parameter("six_logical_slots").value) else 3,
            "physical_source_count": 3,
            "logical_physical_mapping": "A0:0,A1:1,A2:2,A3:0,A4:1,A5:2",
            "rejection_reason":
                self.gate.state.rejection_reason,
        }

        if estimate is not None:
            fields.update({
                "x": estimate.x,
                "y": estimate.y,
                "residual_rms":
                    estimate.residual_rms,
                "geometry_score":
                    estimate.geometry_metric,
                "confidence":
                    estimate.confidence,
            })

        status.values = [
            KeyValue(
                key=str(k),
                value=str(v),
            )
            for k, v in fields.items()
        ]

        array.status = [status]

        self.quality_pub.publish(array)


def main():

    rclpy.init()

    node = UwbDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
