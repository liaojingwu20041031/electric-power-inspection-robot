import math
import threading
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MobileRosBridge(Node):
    def __init__(self) -> None:
        super().__init__('mobile_bridge')
        self._declare_parameters()
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.text_command_topic = self.get_parameter('text_command_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.zlac_status_topic = self.get_parameter('zlac_status_topic').value
        self.zlac_fault_topic = self.get_parameter('zlac_fault_topic').value
        self.max_linear_speed = min(float(self.get_parameter('max_linear_speed').value), 0.15)
        self.max_angular_speed = min(float(self.get_parameter('max_angular_speed').value), 0.5)
        self.default_cmd_duration_ms = int(self.get_parameter('default_cmd_duration_ms').value)

        self._last_odom_time: Optional[float] = None
        self._last_scan_time: Optional[float] = None
        self._last_map_time: Optional[float] = None
        self._zlac_status = 'unknown'
        self._task_status = 'idle'
        self._stop_timer: Optional[threading.Timer] = None
        self._nav_goal_handle = None

        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._text_pub = self.create_publisher(String, self.text_command_topic, 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, self.zlac_status_topic, self._on_zlac_status, 10)
        self.create_subscription(String, self.zlac_fault_topic, self._on_zlac_fault, 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _declare_parameters(self) -> None:
        defaults = {
            'host': '0.0.0.0',
            'port': 8000,
            'cmd_vel_topic': '/cmd_vel',
            'text_command_topic': '/retail_ai/text_command',
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'map_topic': '/map',
            'zlac_status_topic': '/zlac8015d/status',
            'zlac_fault_topic': '/zlac8015d/fault',
            'status_rate_hz': 2.0,
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.5,
            'default_cmd_duration_ms': 300,
            'workspace_dir': '/home/nvidia/ros2_ws',
            'default_map_path': '/home/nvidia/ros2_ws/src/my_map',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_odom(self, _msg: Odometry) -> None:
        self._last_odom_time = time.time()

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_time = time.time()

    def _on_zlac_status(self, msg: String) -> None:
        self._zlac_status = msg.data or 'online'

    def _on_zlac_fault(self, msg: String) -> None:
        if msg.data:
            self._zlac_status = f'fault: {msg.data}'

    def _age(self, last_time: Optional[float]) -> Optional[float]:
        return None if last_time is None else round(time.time() - last_time, 3)

    def _topic_available(self, topic: str) -> bool:
        return topic in dict(self.get_topic_names_and_types())

    def _node_available(self, candidates: tuple[str, ...]) -> bool:
        names = set(self.get_node_names())
        return any(candidate in names for candidate in candidates)

    def robot_status(self) -> dict:
        return {
            'online': True,
            'can_status': 'online' if self._topic_available(self.cmd_vel_topic) else 'unknown',
            'zlac_status': self._zlac_status,
            'task_status': self._task_status,
            'mapping_status': 'running' if self._node_available(('slam_toolbox', 'async_slam_toolbox_node')) else 'not_running',
            'nav2_status': 'running' if self._node_available(('bt_navigator', 'controller_server', 'planner_server')) else 'not_running',
            'last_odom_age_sec': self._age(self._last_odom_time),
            'last_scan_age_sec': self._age(self._last_scan_time),
            'timestamp': time.time(),
        }

    def debug_status(self) -> dict:
        topics = {
            '/cmd_vel': self._topic_available(self.cmd_vel_topic),
            '/odom': self._topic_available(self.odom_topic),
            '/scan': self._topic_available(self.scan_topic),
            '/map': self._topic_available(self.map_topic),
        }
        nodes: Dict[str, bool] = {
            'zlac8015d_canopen_controller': self._node_available(('zlac8015d_canopen_controller',)),
            'slam_toolbox': self._node_available(('slam_toolbox', 'async_slam_toolbox_node')),
            'bt_navigator': self._node_available(('bt_navigator',)),
            'controller_server': self._node_available(('controller_server',)),
            'planner_server': self._node_available(('planner_server',)),
            'amcl': self._node_available(('amcl',)),
            'map_server': self._node_available(('map_server',)),
        }
        status = self.robot_status()
        return {
            'online': True,
            'topics': topics,
            'nodes': nodes,
            'last_odom_age_sec': status['last_odom_age_sec'],
            'last_scan_age_sec': status['last_scan_age_sec'],
            'zlac_status': self._zlac_status,
            'mapping_status': status['mapping_status'],
            'nav2_status': status['nav2_status'],
        }

    def publish_velocity(self, linear_x: float, angular_z: float, duration_ms: int) -> None:
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        duration_ms = max(50, min(3000, duration_ms or self.default_cmd_duration_ms))
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)
        if self._stop_timer:
            self._stop_timer.cancel()
        self._stop_timer = threading.Timer(duration_ms / 1000.0, self.stop_motion)
        self._stop_timer.daemon = True
        self._stop_timer.start()

    def stop_motion(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)

    def publish_text_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._task_status = text
        self._text_pub.publish(msg)

    def stop_all(self) -> None:
        self.stop_motion()
        self.publish_text_command('停止当前任务')

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return False
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = pose
        future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        self._nav_goal_handle = goal_handle
        return True

    def cancel_navigation(self) -> bool:
        if not self._nav_goal_handle:
            return False
        future = self._nav_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self._nav_goal_handle = None
        return future.result() is not None
