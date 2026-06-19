import json
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ylhb_interfaces.msg import SayText


def system_mode_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class BasicMotionCommandNode(Node):
    def __init__(self) -> None:
        super().__init__('basic_motion_command_node')

        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('zlac_status_topic', '/zlac8015d/status')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('motion_duration_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('require_chassis_online', True)
        self.declare_parameter('chassis_status_max_age_sec', 2.5)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.motion_duration_sec = float(self.get_parameter('motion_duration_sec').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.require_chassis_online = bool(
            self.get_parameter('require_chassis_online').value)
        self.chassis_status_max_age_sec = max(
            0.1, float(self.get_parameter('chassis_status_max_age_sec').value))
        self.stop_at = 0.0
        self.system_mode = 'ready'
        self.active_twist: Optional[Twist] = None
        self.zlac_status = ''
        self.zlac_status_received_at = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.say_pub = self.create_publisher(
            SayText, self.get_parameter('say_text_topic').value, 10)
        self.create_subscription(
            String,
            self.get_parameter('text_command_topic').value,
            self.text_command_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('system_mode_topic').value,
            self.system_mode_callback,
            system_mode_qos(),
        )
        self.create_subscription(
            String,
            self.get_parameter('zlac_status_topic').value,
            self.zlac_status_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info(
            'Basic motion command node started: '
            f'publish_rate={self.publish_rate_hz:.1f}Hz, '
            f'require_chassis_online={self.require_chassis_online}.'
        )

    def system_mode_callback(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode in ('sleep', 'ready', 'mapping', 'running', 'fault'):
            self.system_mode = mode
        else:
            self.get_logger().warn(f'Ignoring unknown system_mode: {mode}')

    def text_command_callback(self, msg: String) -> None:
        text = self.command_text(msg.data)
        command = self.parse_motion_command(text)
        if command is None:
            return
        self.get_logger().info(
            f'Received motion command: text="{text}", system_mode={self.system_mode}'
        )
        if self.system_mode in ('sleep', 'fault') and not self.is_stop_command(text):
            self.get_logger().info(
                f'Ignoring motion command while system_mode={self.system_mode}: {text}'
            )
            return

        linear_x, angular_z = command
        if linear_x == 0.0 and angular_z == 0.0:
            self.stop_motion('stop command')
            return

        subscription_count = self.cmd_pub.get_subscription_count()
        status, status_age = self.chassis_status_summary()
        self.get_logger().info(
            f'Motion readiness: cmd_vel_subscribers={subscription_count}, '
            f'zlac_status={status}, zlac_status_age_sec={status_age:.2f}'
        )
        if subscription_count == 0:
            self.reject_motion('底盘控制节点尚未启动')
            return
        if self.require_chassis_online and not self.is_chassis_online():
            self.reject_motion('底盘未在线')
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.active_twist = twist
        self.cmd_pub.publish(twist)
        self.stop_at = time.monotonic() + self.motion_duration_sec
        self.get_logger().info(
            f'Publishing motion: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}, '
            f'duration_sec={self.motion_duration_sec:.2f}'
        )

    def command_text(self, data: str) -> str:
        raw = data.strip()
        if not raw.startswith('{'):
            return raw
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return raw
        if isinstance(payload, dict):
            return str(payload.get('text') or raw).strip()
        return raw

    def timer_callback(self) -> None:
        if self.active_twist is None:
            return
        if time.monotonic() >= self.stop_at:
            self.stop_motion('motion duration elapsed')
            return
        self.cmd_pub.publish(self.active_twist)

    def zlac_status_callback(self, msg: String) -> None:
        self.zlac_status = msg.data.strip()
        self.zlac_status_received_at = time.monotonic()

    def is_chassis_online(self) -> bool:
        if not self.zlac_status or self.zlac_status_received_at <= 0.0:
            return False
        age = time.monotonic() - self.zlac_status_received_at
        state = self.zlac_status.split(maxsplit=1)[0]
        return age <= self.chassis_status_max_age_sec and state == 'online'

    def chassis_status_summary(self) -> Tuple[str, float]:
        if not self.zlac_status or self.zlac_status_received_at <= 0.0:
            return 'missing', -1.0
        return (
            self.zlac_status.split(maxsplit=1)[0],
            time.monotonic() - self.zlac_status_received_at,
        )

    def reject_motion(self, reason: str) -> None:
        self.cmd_pub.publish(Twist())
        self.active_twist = None
        self.stop_at = 0.0
        self.get_logger().warn(f'Rejecting motion command: {reason}')
        self.say(reason)

    def stop_motion(self, reason: str) -> None:
        self.cmd_pub.publish(Twist())
        self.active_twist = None
        self.stop_at = 0.0
        self.get_logger().info(f'Motion stopped: {reason}')

    def say(self, text: str) -> None:
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = 'basic_motion'
        msg.priority = 7
        msg.interrupt = True
        msg.text = text
        self.say_pub.publish(msg)

    def parse_motion_command(self, text: str) -> Optional[Tuple[float, float]]:
        normalized = text.strip().replace(' ', '')
        if not normalized:
            return None
        if any(token in normalized for token in ('停止', '停下', '刹车')):
            return 0.0, 0.0
        if '前进' in normalized:
            return self.linear_speed, 0.0
        if '后退' in normalized:
            return -self.linear_speed, 0.0
        if '左转' in normalized:
            return 0.0, self.angular_speed
        if '右转' in normalized:
            return 0.0, -self.angular_speed
        return None

    def is_stop_command(self, text: str) -> bool:
        normalized = text.strip().replace(' ', '')
        return any(token in normalized for token in ('停止', '停下', '刹车'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BasicMotionCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
