#!/usr/bin/env python3
"""Display one ROS Image topic from the HVS bridge without extra GUI packages."""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class FusedImageViewer(Node):
    def __init__(self) -> None:
        super().__init__("dg_hvs_fused_viewer")
        self.declare_parameter("image_topic", "/dg/event_camera/fused/image")
        self.topic = str(self.get_parameter("image_topic").value)
        self._lock = threading.Lock()
        self._latest = None
        self._last_error = ""
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, self.topic, self._on_image, qos)
        self.create_timer(0.03, self._paint)
        self.get_logger().info("Displaying %s", self.topic)

    def _on_image(self, msg: Image) -> None:
        try:
            encoding = msg.encoding.lower()
            channels = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(
                encoding
            )
            if channels is None:
                raise ValueError(f"unsupported encoding: {msg.encoding}")
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((msg.height, msg.step))
            frame = rows[:, : msg.width * channels]
            if channels > 1:
                frame = frame.reshape((msg.height, msg.width, channels))
            else:
                frame = frame.reshape((msg.height, msg.width))
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding == "rgba8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif encoding == "bgra8":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            with self._lock:
                self._latest = frame.copy()
        except Exception as exc:  # Keep the viewer alive while reporting bad frames.
            error = str(exc)
            if error != self._last_error:
                self._last_error = error
                self.get_logger().error("Cannot decode image: %s", error)

    def _paint(self) -> None:
        with self._lock:
            frame = None if self._latest is None else self._latest.copy()
        if frame is None:
            return
        cv2.imshow("DG-202611 EVS+APS fused", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    viewer = FusedImageViewer()
    try:
        rclpy.spin(viewer)
    finally:
        viewer.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
