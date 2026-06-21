#!/usr/bin/env python3

import math
import sys


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angular_distance(start, end):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from std_srvs.srv import Empty

    class AmclSwingRelocalizationNode(Node):
        def __init__(self):
            super().__init__("amcl_swing_relocalization_node")
            self.declare_parameter("cycles", 2)
            self.declare_parameter("angular_speed", 0.18)
            self.declare_parameter("yaw_tolerance", math.radians(1.5))
            self.declare_parameter("segment_timeout", 8.0)
            self.declare_parameter("settle_seconds", 0.5)
            self.declare_parameter("control_period", 0.05)

            self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", lambda _msg: None, 1
            )
            self._nomotion_client = self.create_client(Empty, "/request_nomotion_update")
            self._yaw = None
            self._segments = []
            for _ in range(int(self.get_parameter("cycles").value)):
                self._segments.extend(
                    [
                        math.radians(10.0),
                        math.radians(-20.0),
                        math.radians(10.0),
                    ]
                )
            self._segment_index = 0
            self._target_yaw = None
            self._segment_started = None
            self._settle_until = None
            control_period = float(self.get_parameter("control_period").value)
            self._timer = self.create_timer(control_period, self._tick)
            self.get_logger().info("waiting for /odom before small AMCL swing relocalization")

        def _on_odom(self, msg):
            self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        def _zero_twist(self):
            self._cmd_pub.publish(Twist())

        def _request_nomotion_update(self):
            if not self._nomotion_client.service_is_ready():
                self.get_logger().warn(
                    "/request_nomotion_update is not available; continuing without it"
                )
                return
            request = Empty.Request()
            self._nomotion_client.call_async(request)

        def _start_next_segment(self):
            if self._segment_index >= len(self._segments):
                self._zero_twist()
                self.get_logger().info("AMCL swing relocalization complete; /cmd_vel is zero")
                raise SystemExit
            delta = self._segments[self._segment_index]
            self._target_yaw = self._yaw + delta
            self._segment_started = self.get_clock().now()
            self._segment_index += 1
            self.get_logger().info(
                f"swing segment {self._segment_index}/{len(self._segments)}: "
                f"{math.degrees(delta):.1f}deg"
            )

        def _tick(self):
            if self._yaw is None:
                self._zero_twist()
                return

            now = self.get_clock().now()
            if self._settle_until is not None:
                self._zero_twist()
                if now >= self._settle_until:
                    self._settle_until = None
                    self._request_nomotion_update()
                    self._start_next_segment()
                return

            if self._target_yaw is None:
                self._start_next_segment()
                return

            error = shortest_angular_distance(self._yaw, self._target_yaw)
            if abs(error) <= float(self.get_parameter("yaw_tolerance").value):
                self._zero_twist()
                self._target_yaw = None
                settle_seconds = float(self.get_parameter("settle_seconds").value)
                self._settle_until = now + Duration(seconds=settle_seconds)
                return

            elapsed = (now - self._segment_started).nanoseconds / 1e9
            if elapsed > float(self.get_parameter("segment_timeout").value):
                self._zero_twist()
                raise TimeoutError("timed out while executing AMCL swing segment")

            msg = Twist()
            msg.angular.z = math.copysign(float(self.get_parameter("angular_speed").value), error)
            self._cmd_pub.publish(msg)

        def destroy_node(self):
            self._zero_twist()
            super().destroy_node()

    rclpy.init(args=args)
    node = AmclSwingRelocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node._zero_twist()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
