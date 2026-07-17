from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


@dataclass(frozen=True)
class SkillLimits:
    max_angle_deg: float = 180.0
    max_distance_m: float = 0.5
    chassis_status_max_age_sec: float = 2.5


class BaseMotionSkillLogic:
    def __init__(self, limits: SkillLimits = SkillLimits()) -> None:
        self.limits = limits

    def validate(
        self,
        command: str,
        arguments: Dict[str, Any],
        system_mode: str,
        chassis_online: bool,
    ) -> str:
        if command == "stop_motion":
            return ""
        if system_mode in {"fault", "emergency_stop"}:
            return "system_mode blocks base motion"
        if not chassis_online:
            return "chassis offline"
        if command == "rotate_relative":
            angle = arguments.get("angle_deg")
            if isinstance(angle, bool) or not isinstance(angle, (int, float)):
                return "angle_deg must be number"
            if abs(float(angle)) > self.limits.max_angle_deg:
                return "angle_deg out of range"
            return ""
        if command == "move_relative":
            distance = arguments.get("distance_m")
            if isinstance(distance, bool) or not isinstance(distance, (int, float)):
                return "distance_m must be number"
            if abs(float(distance)) > self.limits.max_distance_m:
                return "distance_m out of range"
            return ""
        return f"unknown command: {command}"

    @staticmethod
    def chassis_status_online(status: str) -> bool:
        return status.strip().split(maxsplit=1)[0] == "online" if status.strip() else False


class BaseMotionSkillNode(Node):
    def __init__(self) -> None:
        super().__init__("base_motion_skill_node")
        self.declare_parameter("base_skill_command_topic", "/inspection_ai/base_skill_command")
        self.declare_parameter("base_skill_status_topic", "/inspection_ai/base_skill_status")
        self.declare_parameter("system_mode_topic", "/inspection_ai/system_mode")
        self.declare_parameter("zlac_status_topic", "/zlac8015d/status")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("max_distance_m", 0.5)
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("require_chassis_online", True)
        self.declare_parameter("chassis_status_max_age_sec", 2.5)
        self.declare_parameter("pose_loss_timeout_sec", 0.5)

        self.system_mode = "ready"
        self.last_chassis_status_at = 0.0
        self.last_chassis_status = ""
        self.logic = BaseMotionSkillLogic(
            SkillLimits(
                max_angle_deg=float(self.get_parameter("max_angle_deg").value),
                max_distance_m=float(self.get_parameter("max_distance_m").value),
                chassis_status_max_age_sec=float(self.get_parameter("chassis_status_max_age_sec").value),
            )
        )
        self.active: Dict[str, Any] | None = None
        self.tf_buffer = None
        self.tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception as exc:
            self.get_logger().warning(f"tf unavailable for base motion skill: {exc}")

        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter("base_skill_status_topic").value, 10)
        self.create_subscription(String, self.get_parameter("base_skill_command_topic").value, self.command_callback, 10)
        self.create_subscription(String, self.get_parameter("system_mode_topic").value, self.mode_callback, 10)
        self.create_subscription(String, self.get_parameter("zlac_status_topic").value, self.chassis_callback, 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick)
        self.publish_status("idle", "base motion skill ready")

    def mode_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.system_mode = str(payload.get("mode") or payload.get("system_mode") or msg.data)
        except json.JSONDecodeError:
            self.system_mode = msg.data

    def chassis_callback(self, msg: String) -> None:
        self.last_chassis_status = msg.data
        self.last_chassis_status_at = time.time()

    def chassis_online(self) -> bool:
        if not bool(self.get_parameter("require_chassis_online").value):
            return True
        age = time.time() - self.last_chassis_status_at
        return (
            self.last_chassis_status_at > 0.0
            and age <= self.logic.limits.chassis_status_max_age_sec
            and self.logic.chassis_status_online(self.last_chassis_status)
        )

    def command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.reject(f"invalid json: {exc}")
            return
        command = str(payload.get("command") or "")
        arguments = payload.get("arguments") or {}
        correlation = {
            key: str(payload.get(key) or "")
            for key in ("request_id", "run_id", "operation_id", "tool_call_id")
        }
        if command == "stop_motion":
            if self.active:
                self.stop("canceled")
            self.publish_status("done", "stopped", correlation)
            return
        if self.active:
            self.stop("canceled", "new base motion command rejected")
            self.reject("base motion already active", correlation)
            return
        reason = self.logic.validate(command, arguments, self.system_mode, self.chassis_online())
        if reason:
            self.reject(reason, correlation)
            return
        now = time.time()
        start_pose = self.current_pose()
        if start_pose is None:
            self.reject("pose unavailable", correlation)
            return
        self.active = {
            "command": command,
            "arguments": arguments,
            "started_at": now,
            "deadline": now + min(float(payload.get("timeout_sec") or self.get_parameter("timeout_sec").value), float(self.get_parameter("timeout_sec").value)),
            "start_pose": start_pose,
            "last_yaw": start_pose[2] if start_pose is not None else None,
            "accumulated_yaw": 0.0,
            "last_pose_at": now,
            "correlation": correlation,
        }
        self.publish_status("running", command, correlation)

    def tick(self) -> None:
        if not self.active:
            return
        if time.time() >= float(self.active["deadline"]):
            self.stop("timeout")
            return
        reason = self.logic.validate(
            self.active["command"], self.active["arguments"],
            self.system_mode, self.chassis_online(),
        )
        if reason:
            self.stop("failed", reason)
            return
        current_pose = self.current_pose()
        if current_pose is None:
            self.cmd_vel_pub.publish(Twist())
            if time.time() - float(self.active["last_pose_at"]) >= float(
                self.get_parameter("pose_loss_timeout_sec").value
            ):
                self.stop("failed", "pose unavailable")
            return
        self.active["last_pose_at"] = time.time()
        if self.reached_target(current_pose):
            self.stop("done")
            return
        twist = Twist()
        command = self.active["command"]
        arguments = self.active["arguments"]
        if command == "rotate_relative":
            angle = float(arguments["angle_deg"])
            twist.angular.z = math.copysign(float(self.get_parameter("angular_speed").value), angle)
        elif command == "move_relative":
            distance = float(arguments["distance_m"])
            twist.linear.x = math.copysign(float(self.get_parameter("linear_speed").value), distance)
        self.cmd_vel_pub.publish(twist)

    def reached_target(self, current_pose=None) -> bool:
        if not self.active:
            return False
        start_pose = self.active.get("start_pose")
        if current_pose is None:
            current_pose = self.current_pose()
        if start_pose is None or current_pose is None:
            return False
        command = self.active["command"]
        arguments = self.active["arguments"]
        if command == "rotate_relative":
            last_yaw = self.active.get("last_yaw")
            if last_yaw is None:
                self.active["last_yaw"] = current_pose[2]
                return False
            self.active["accumulated_yaw"] = float(self.active.get("accumulated_yaw") or 0.0) + self._angle_diff(current_pose[2], last_yaw)
            self.active["last_yaw"] = current_pose[2]
            wanted = math.radians(float(arguments["angle_deg"]))
            progress = float(self.active["accumulated_yaw"])
            return progress >= wanted if wanted >= 0.0 else progress <= wanted
        if command == "move_relative":
            wanted = float(arguments["distance_m"])
            dx = current_pose[0] - start_pose[0]
            dy = current_pose[1] - start_pose[1]
            progress = dx * math.cos(start_pose[2]) + dy * math.sin(start_pose[2])
            return progress >= wanted if wanted >= 0.0 else progress <= wanted
        return False

    def current_pose(self):
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("odom_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
            )
        except Exception:
            return None
        stamp = Time.from_msg(transform.header.stamp)
        if (
            stamp.nanoseconds > 0
            and (self.get_clock().now() - stamp).nanoseconds / 1e9
            > float(self.get_parameter("pose_loss_timeout_sec").value)
        ):
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (float(translation.x), float(translation.y), yaw)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def reject(self, reason: str, correlation=None) -> None:
        self.cmd_vel_pub.publish(Twist())
        self.publish_status("rejected", reason, correlation)

    def stop(self, status: str, message: str = "") -> None:
        correlation = dict((self.active or {}).get("correlation") or {})
        self.active = None
        self.cmd_vel_pub.publish(Twist())
        self.publish_status(status, message or status, correlation)

    def publish_status(self, status: str, message: str, correlation=None) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": "1.0",
                "status": status,
                "message": message,
                "timestamp": time.time(),
                **(correlation or {}),
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseMotionSkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
