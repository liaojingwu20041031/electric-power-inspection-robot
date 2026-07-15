import math
import json
import os
import queue
import threading
import time
from datetime import datetime, timezone
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .map_snapshot import (
    occupancy_grid_metadata,
    occupancy_grid_to_png_snapshot,
)
from .network_status import NetworkStatusProvider

CHASSIS_SAFE_MAX_LINEAR_SPEED = 0.35
CHASSIS_SAFE_MAX_ANGULAR_SPEED = 0.55
TRUE_VALUES = {'1', 'true', 'yes', 'on'}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def initial_pose_qos_profile() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def map_qos_profile() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MobileRosBridge(Node):
    def __init__(self) -> None:
        super().__init__('mobile_bridge')
        self._declare_parameters()
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.text_command_topic = self.get_parameter(
            'text_command_topic'
        ).value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.zlac_status_topic = self.get_parameter('zlac_status_topic').value
        self.zlac_fault_topic = self.get_parameter('zlac_fault_topic').value
        self.system_mode_topic = self.get_parameter('system_mode_topic').value
        self.system_command_topic = self.get_parameter(
            'system_command_topic'
        ).value
        self.system_status_topic = self.get_parameter(
            'system_status_topic'
        ).value
        self.patrol_command_topic = self.get_parameter(
            'patrol_command_topic'
        ).value
        self.patrol_status_topic = self.get_parameter(
            'patrol_status_topic'
        ).value
        self.patrol_event_topic = self.get_parameter('patrol_event_topic').value
        self.max_linear_speed = self._safe_speed_limit(
            self.get_parameter('max_linear_speed').value,
            CHASSIS_SAFE_MAX_LINEAR_SPEED,
        )
        self.max_angular_speed = self._safe_speed_limit(
            self.get_parameter('max_angular_speed').value,
            CHASSIS_SAFE_MAX_ANGULAR_SPEED,
        )
        self.default_cmd_duration_ms = int(
            self.get_parameter('default_cmd_duration_ms').value
        )
        self.status_rate_hz = max(
            0.1,
            float(self.get_parameter('status_rate_hz').value),
        )
        self.map_stream_rate_hz = max(
            0.1,
            float(self.get_parameter('map_stream_rate_hz').value),
        )
        self.map_max_size_px = max(
            1,
            int(self.get_parameter('map_max_size_px').value),
        )
        self.require_token = bool(self.get_parameter('require_token').value)
        self.api_token = str(self.get_parameter('api_token').value)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.cloud_status_topic = str(self.get_parameter('cloud_status_topic').value)
        self.set_cloud_enabled_service_name = str(self.get_parameter('set_cloud_enabled_service_name').value)
        self.local_app_status_topic = str(self.get_parameter('local_app_status_topic').value)
        self.set_local_app_enabled_service_name = str(self.get_parameter('set_local_app_enabled_service_name').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.network_status = NetworkStatusProvider()

        self._last_odom_time: Optional[float] = None
        self._last_scan_time: Optional[float] = None
        self._last_map_time: Optional[float] = None
        self._last_imu_time: Optional[float] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._pose: Optional[dict] = None
        self._map_pose: Optional[dict] = None
        self._velocity: Optional[dict] = None
        self._scan_range_min: Optional[float] = None
        self._scan_range_max: Optional[float] = None
        self._zlac_status = 'unknown'
        self._task_status = 'idle'
        self._system_mode = 'unknown'
        self._system_status: dict = {}
        self._patrol_status: dict = {}
        self._platform_context: dict = {}
        self._cloud_command_queue: queue.Queue[dict] = queue.Queue()
        self._cloud_snapshot: dict = {}
        self._last_command_result_key = ''
        self._status_cache: dict = {}
        self._last_stop_motion_time = 0.0
        self._last_stop_text_time = 0.0
        self._stop_timer: Optional[threading.Timer] = None
        self._nav_goal_handle = None
        self.cloud_client = None
        self._local_app_enabled = os.environ.get(
            'YLHB_LOCAL_APP_ENABLED', 'true'
        ).strip().lower() in TRUE_VALUES
        self._local_app_http_available = False
        self._local_app_last_changed_at = _now()
        self._local_app_last_error = ''
        self._local_app_clients = {'status': 0, 'map': 0}
        self._local_app_lock = threading.Lock()

        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._text_pub = self.create_publisher(
            String,
            self.text_command_topic,
            10,
        )
        self._system_command_pub = self.create_publisher(
            String,
            self.system_command_topic,
            10,
        )
        self._patrol_command_pub = self.create_publisher(
            String,
            self.patrol_command_topic,
            10,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos_profile(),
        )
        self._cloud_status_pub = self.create_publisher(String, self.cloud_status_topic, initial_pose_qos_profile())
        self._local_app_status_pub = self.create_publisher(String, self.local_app_status_topic, initial_pose_qos_profile())
        self.create_service(SetBool, self.set_cloud_enabled_service_name, self._set_cloud_enabled)
        self.create_service(SetBool, self.set_local_app_enabled_service_name, self._set_local_app_enabled)
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._on_map,
            map_qos_profile(),
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10,
        )
        self.create_subscription(
            String,
            self.zlac_status_topic,
            self._on_zlac_status,
            10,
        )
        self.create_subscription(
            String,
            self.zlac_fault_topic,
            self._on_zlac_fault,
            10,
        )
        self.create_subscription(
            String,
            self.system_mode_topic,
            self._on_system_mode,
            initial_pose_qos_profile(),
        )
        self.create_subscription(
            String,
            self.system_status_topic,
            self._on_system_status,
            initial_pose_qos_profile(),
        )
        self.create_subscription(
            String,
            self.patrol_status_topic,
            self._on_patrol_status,
            initial_pose_qos_profile(),
        )
        self.create_subscription(
            String, self.patrol_event_topic, self._on_patrol_event,
            initial_pose_qos_profile(),
        )
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )
        self.create_timer(0.2, self._drain_cloud_commands)
        self.create_timer(0.5, self._refresh_cloud_snapshot)

    def _declare_parameters(self) -> None:
        defaults = {
            'host': '0.0.0.0',
            'port': 8000,
            'cmd_vel_topic': '/cmd_vel',
            'text_command_topic': '/inspection_ai/text_command',
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'map_topic': '/map',
            'imu_topic': '/imu/data',
            'amcl_pose_topic': '/amcl_pose',
            'zlac_status_topic': '/zlac8015d/status',
            'zlac_fault_topic': '/zlac8015d/fault',
            'system_mode_topic': '/inspection_ai/system_mode',
            'system_command_topic': '/inspection_ai/system_command',
            'system_status_topic': '/inspection_ai/system_status',
            'patrol_command_topic': '/patrol/command',
            'patrol_status_topic': '/patrol/status',
            'patrol_event_topic': '/patrol/event',
            'status_rate_hz': 2.0,
            'map_stream_rate_hz': 1.0,
            'map_max_size_px': 1024,
            'require_token': False,
            'api_token': '',
            'robot_id': '',
            'platform_api_token': '',
            'platform_storage_dir': '~/.local/share/ylhb/platform',
            'cloud_status_topic': '/mobile_bridge/cloud_status',
            'set_cloud_enabled_service_name': '/mobile_bridge/set_cloud_enabled',
            'local_app_status_topic': '/mobile_bridge/local_app_status',
            'set_local_app_enabled_service_name': '/mobile_bridge/set_local_app_enabled',
            'max_linear_speed': 0.30,
            'max_angular_speed': 0.55,
            'default_cmd_duration_ms': 300,
            'workspace_dir': '/home/nvidia/ros2_DL',
            'default_map_path': '/home/nvidia/ros2_DL/maps/my_map',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_time = time.time()
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        position = msg.pose.pose.position
        self._pose = {
            'frame': msg.header.frame_id or 'odom',
            'x': float(position.x),
            'y': float(position.y),
            'yaw': math.atan2(siny_cosp, cosy_cosp),
        }
        self._velocity = {
            'linear_x': float(msg.twist.twist.linear.x),
            'angular_z': float(msg.twist.twist.angular.z),
        }

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = time.time()
        self._scan_range_min = round(msg.range_min, 3)
        self._scan_range_max = round(msg.range_max, 3)

    def _on_map(self, msg: OccupancyGrid) -> None:
        if not self._mapping_map_source_available():
            return
        self._last_map_time = time.time()
        self._latest_map = msg

    def _on_imu(self, _msg: Imu) -> None:
        self._last_imu_time = time.time()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id != 'map':
            return
        orientation = msg.pose.pose.orientation
        position = msg.pose.pose.position
        self._map_pose = {
            'frame': 'map', 'x': float(position.x), 'y': float(position.y),
            'yaw': math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)),
        }

    def _on_zlac_status(self, msg: String) -> None:
        self._zlac_status = msg.data or 'online'

    def _on_zlac_fault(self, msg: String) -> None:
        if msg.data:
            self._zlac_status = f'fault: {msg.data}'

    def _on_system_mode(self, msg: String) -> None:
        self._system_mode = msg.data.strip() or 'unknown'

    def _parse_json_message(self, raw: str) -> dict:
        try:
            parsed = json.loads(raw)
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            return {}

    def _on_system_status(self, msg: String) -> None:
        self._system_status = self._parse_json_message(msg.data)
        result = self._system_status.get('command_result') or {}
        if not isinstance(result, dict) or result.get('event') not in {'command_rejected', 'command_failed'}:
            return
        key = json.dumps(result, ensure_ascii=False, sort_keys=True)
        if key == getattr(self, '_last_command_result_key', ''):
            return
        command_id = str(result.get('command_id') or '')
        store = getattr(self, 'platform_store', None)
        if not command_id or not store:
            return
        self._last_command_result_key = key
        existing = store.command(command_id)
        if existing and existing.get('state') in {'APPLIED', 'REJECTED', 'FAILED'}:
            return
        saved = store.append_event({
            **result,
            'schema_version': '1.0',
            'robot_id': getattr(self, 'platform_robot_id', getattr(self, 'robot_id', '')),
            'boot_id': getattr(self, 'platform_boot_id', ''),
        })
        store.set_command_state(command_id, 'REJECTED' if result['event'] == 'command_rejected' else 'FAILED', saved)

    def _on_patrol_status(self, msg: String) -> None:
        self._patrol_status = self._parse_json_message(msg.data)

    def _on_patrol_event(self, msg: String) -> None:
        event = self._parse_json_message(msg.data)
        if not event or not getattr(self, 'platform_store', None):
            return
        context = self._platform_context
        execution_id = str(event.get('execution_id') or context.get('active_execution_id') or '')
        if not execution_id:
            return
        command_id = str(event.get('command_id') if 'command_id' in event else context.get('active_command_id') or '')
        if not command_id and event.get('event') in {'route_paused', 'route_resumed', 'manual_takeover', 'route_canceled', 'command_rejected', 'command_failed'}:
            return
        saved = self.platform_store.append_event({
            **event, 'schema_version': '1.0',
            'robot_id': getattr(self, 'platform_robot_id', getattr(self, 'robot_id', '')),
            'boot_id': getattr(self, 'platform_boot_id', ''),
            'execution_id': execution_id,
            'deployment_id': str(event.get('deployment_id') or context.get('active_deployment_id') or ''),
            'request_id': str(event.get('request_id') if 'request_id' in event else context.get('active_request_id') or ''),
            'command_id': command_id,
        })
        if command_id:
            state = {
                'route_started': 'APPLIED', 'route_paused': 'APPLIED',
                'route_resumed': 'APPLIED', 'manual_takeover': 'APPLIED',
                'route_canceled': 'APPLIED', 'command_rejected': 'REJECTED',
                'command_failed': 'FAILED',
            }.get(str(saved.get('event') or ''))
            if state:
                self.platform_store.set_command_state(command_id, state, saved)
            elif saved.get('event') == 'route_failed':
                command = self.platform_store.command(command_id)
                if command and command.get('state') in {'ACKED', 'DISPATCHED'}:
                    failed = self.platform_store.append_event({
                        **saved,
                        'event': 'command_failed',
                        'error_code': 'ROUTE_FAILED_BEFORE_APPLIED',
                        'error_message': str(saved.get('reason') or saved.get('error') or 'route failed'),
                    })
                    self.platform_store.set_command_state(command_id, 'FAILED', failed)

    def _age(self, last_time: Optional[float]) -> Optional[float]:
        return None if last_time is None else round(time.time() - last_time, 3)

    @staticmethod
    def _safe_speed_limit(configured_limit: float, safety_limit: float) -> float:
        return min(float(configured_limit), safety_limit)

    def _cached_value(self, key: str, ttl_sec: float, builder):
        now = time.time()
        cached = self._status_cache.get(key)
        if cached and now - cached[0] < ttl_sec:
            return cached[1]
        value = builder()
        self._status_cache[key] = (now, value)
        return value

    def _cached_topic_names(self) -> set[str]:
        return self._cached_value(
            'topic_names',
            0.5,
            lambda: set(dict(self.get_topic_names_and_types())),
        )

    def _cached_node_names(self) -> set[str]:
        return self._cached_value(
            'node_names',
            0.5,
            lambda: set(self.get_node_names()),
        )

    def _topic_available(self, topic: str) -> bool:
        return topic in self._cached_topic_names()

    def _topic_has_publishers(self, topic: str) -> bool:
        return any(
            info.node_name
            for info in self.get_publishers_info_by_topic(topic)
        )

    def _node_available(self, candidates: tuple[str, ...]) -> bool:
        names = self._cached_node_names()
        return any(candidate in names for candidate in candidates)

    def _mapping_node_available(self) -> bool:
        return self._node_available(
            ('slam_toolbox', 'async_slam_toolbox_node')
        )

    def _mapping_map_source_available(self) -> bool:
        return (
            self._mapping_node_available()
            and not self._node_available(('map_server',))
        )

    def reset_mapping_map(self) -> None:
        self._last_map_time = None
        self._latest_map = None

    def has_mapping_map(self) -> bool:
        return (
            self._latest_map is not None
            and self._mapping_map_source_available()
        )

    def robot_status(self) -> dict:
        return self._cached_value('robot_status', 0.5, self._robot_status_uncached)

    def _robot_status_uncached(self) -> dict:
        network = self._network_status_snapshot()
        return {
            'online': True,
            'can_status': (
                'online'
                if self._topic_available(self.cmd_vel_topic)
                else 'unknown'
            ),
            'zlac_status': self._zlac_status,
            'task_status': self._task_status,
            'system_mode': self._system_mode,
            'mapping_status': (
                'running'
                if self._mapping_node_available()
                else 'not_running'
            ),
            'nav2_status': (
                'running'
                if self._node_available(
                    ('bt_navigator', 'controller_server', 'planner_server')
                )
                else 'not_running'
            ),
            'last_odom_age_sec': self._age(self._last_odom_time),
            'last_scan_age_sec': self._age(self._last_scan_time),
            'last_imu_age_sec': self._age(self._last_imu_time),
            'pose': self._pose,
            'mapPose': self._map_pose,
            'odomPose': self._pose,
            'velocity': self._velocity,
            'network': {
                'appEndpoints': network['appEndpoints'],
                'candidateEndpoints': network['candidateEndpoints'],
                'interfaces': network['interfaces'],
                'preferredAppEndpoint': network['preferredAppEndpoint'],
                'warnings': network['warnings'],
                'wifiReconnect': network['wifiReconnect'],
            },
            'timestamp': time.time(),
        }

    def debug_status(self) -> dict:
        return self._cached_value('debug_status', 0.5, self._debug_status_uncached)

    def _debug_status_uncached(self) -> dict:
        topics = {
            '/cmd_vel': self._topic_available(self.cmd_vel_topic),
            '/odom': self._topic_available(self.odom_topic),
            '/scan': self._topic_available(self.scan_topic),
            '/map': self._topic_available(self.map_topic),
            '/imu/data': self._topic_available(self.imu_topic),
        }
        nodes: Dict[str, bool] = {
            'zlac8015d_canopen_controller': self._node_available(
                ('zlac8015d_canopen_controller',)
            ),
            'slam_toolbox': self._node_available(
                ('slam_toolbox', 'async_slam_toolbox_node')
            ),
            'bt_navigator': self._node_available(('bt_navigator',)),
            'controller_server': self._node_available(('controller_server',)),
            'planner_server': self._node_available(('planner_server',)),
            'amcl': self._node_available(('amcl',)),
            'map_server': self._node_available(('map_server',)),
            'bringup': self._node_available(('robot_state_publisher',)),
            'rplidar_node': self._node_available(('rplidar_node',)),
            'imu': self._topic_has_publishers(self.imu_topic),
            'tf': self._topic_has_publishers('/tf'),
        }
        status = self.robot_status()
        return {
            'online': True,
            'topics': topics,
            'nodes': nodes,
            'last_odom_age_sec': status['last_odom_age_sec'],
            'last_scan_age_sec': status['last_scan_age_sec'],
            'last_map_age_sec': self._age(self._last_map_time),
            'last_imu_age_sec': self._age(self._last_imu_time),
            'scan_range_min': self._scan_range_min,
            'scan_range_max': self._scan_range_max,
            'zlac_status': self._zlac_status,
            'mapping_status': status['mapping_status'],
            'nav2_status': status['nav2_status'],
            'task_status': status['task_status'],
            'system_mode': status['system_mode'],
            'pose': self._pose,
            'velocity': self._velocity,
            'map_meta': self.map_metadata(),
        }

    def map_metadata(self) -> Optional[dict]:
        if not self.has_mapping_map():
            return None
        return occupancy_grid_metadata(self._latest_map)

    def map_snapshot(self, downsample: int = 1) -> Optional[dict]:
        if not self.has_mapping_map():
            return None
        return occupancy_grid_to_png_snapshot(
            self._latest_map,
            downsample=downsample,
            max_size_px=self.map_max_size_px,
        )

    def mapping_status(self, process: Optional[dict] = None) -> dict:
        status = self.robot_status()
        bringup_ready = (
            self._topic_available(self.odom_topic)
            and self._topic_available(self.scan_topic)
            and self._topic_available(self.imu_topic)
            and self._topic_has_publishers('/tf')
        )
        map_available = self.has_mapping_map()
        mapping_running = status['mapping_status'] == 'running' or bool(
            process and process.get('running')
        )
        if not bringup_ready:
            recommended_next_action = 'start_bringup'
        elif not mapping_running:
            recommended_next_action = 'start_mapping'
        elif not map_available:
            recommended_next_action = 'wait_for_map'
        else:
            recommended_next_action = 'continue_mapping_or_save'
        return {
            'mapping_status': status['mapping_status'],
            'bringup_ready': bringup_ready,
            'map_available': map_available,
            'recommended_next_action': recommended_next_action,
            'last_map_age_sec': (
                self._age(self._last_map_time) if map_available else None
            ),
            'map_meta': self.map_metadata(),
            'process': process,
        }

    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float,
        duration_ms: int,
    ) -> None:
        linear_x = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, linear_x),
        )
        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z),
        )
        duration_ms = max(
            50,
            min(3000, duration_ms or self.default_cmd_duration_ms),
        )
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)
        if linear_x == 0.0 and angular_z == 0.0:
            if self._stop_timer:
                self._stop_timer.cancel()
                self._stop_timer = None
            return
        if self._stop_timer:
            self._stop_timer.cancel()
        self._stop_timer = threading.Timer(
            duration_ms / 1000.0,
            self.stop_motion,
        )
        self._stop_timer.daemon = True
        self._stop_timer.start()

    def stop_motion(self, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_stop_motion_time < 0.15:
            return
        self._last_stop_motion_time = now
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None
        msg = Twist()
        self._cmd_pub.publish(msg)

    def publish_text_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._task_status = text
        self._text_pub.publish(msg)

    def stop_all(self) -> None:
        self.stop_motion(force=True)
        now = time.time()
        if now - self._last_stop_text_time >= 2.0:
            self._last_stop_text_time = now
            self.publish_text_command('停止当前任务')

    def publish_system_command(self, command: str, **extra) -> None:
        payload = {
            'schema_version': '1.0',
            'source': 'mobile_bridge',
            'command': command,
            **extra,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._system_command_pub.publish(msg)

    def enqueue_cloud_command(self, command: dict) -> None:
        self._cloud_command_queue.put(dict(command))

    def _record_cloud_queue_result(self, command: dict, state: str, event: str, code: str, message: str) -> None:
        command_id = str(command.get('commandId') or '')
        if not command_id or not getattr(self, 'platform_store', None):
            return
        result = {
            'event': event,
            'robot_id': getattr(self, 'platform_robot_id', getattr(self, 'robot_id', '')),
            'boot_id': getattr(self, 'platform_boot_id', ''),
            'command_id': command_id,
            'request_id': str(command.get('requestId') or ''),
            'execution_id': str(command.get('executionId') or ''),
            'deployment_id': str(command.get('deploymentId') or ''),
            'error_code': code,
            'error_message': message,
        }
        saved = self.platform_store.append_event(result)
        self.platform_store.set_command_state(command_id, state, saved)

    def _drain_cloud_commands(self) -> None:
        while True:
            try:
                command = self._cloud_command_queue.get_nowait()
            except queue.Empty:
                return
            command_type = str(command.get('type') or '').upper()
            command_id = str(command.get('commandId') or '')
            request_id = str(command.get('requestId') or '')
            execution_id = str(command.get('executionId') or '')
            deployment_id = str(command.get('deploymentId') or '')
            mapping = {'START': 'start_platform_patrol', 'PAUSE': 'pause_patrol', 'RESUME': 'resume_patrol', 'TAKEOVER': 'takeover_patrol', 'CANCEL': 'cancel_patrol'}
            if command_type not in mapping or not command_id or not request_id or not execution_id:
                self._record_cloud_queue_result(command, 'REJECTED', 'command_rejected', 'INVALID_COMMAND', 'cloud command queue item is incomplete')
                continue
            context = {**self._platform_context, 'active_command_id': command_id, 'active_request_id': request_id, 'active_execution_id': execution_id, 'active_deployment_id': deployment_id}
            if command_type == 'START':
                context.update({'active_route_revision_id': str(command.get('routeRevisionId') or ''), 'active_route_path': str(command.get('routePath') or ''), 'active_map_yaml_path': str(command.get('mapYamlPath') or ''), 'executor_route_id': str(command.get('executorRouteId') or '')})
            self.set_platform_context(context)
            try:
                self.publish_system_command(mapping[command_type], command_id=command_id, request_id=request_id, execution_id=execution_id, deployment_id=deployment_id, profile=str(command.get('profile') or 'inspection'), **context)
            except Exception as exc:
                self._record_cloud_queue_result(command, 'FAILED', 'command_failed', 'ROS_PUBLISH_FAILED', str(exc))
                continue
            try:
                self.platform_store.set_command_state(command_id, 'DISPATCHED')
            except Exception as exc:
                self._record_cloud_queue_result(command, 'FAILED', 'command_failed', 'DISPATCH_PERSIST_FAILED', str(exc))

    def _refresh_cloud_snapshot(self) -> None:
        status = self.robot_status()
        patrol = self.patrol_status()
        context = dict(self._platform_context)
        context.setdefault('active_execution_id', str(patrol.get('execution_id') or ''))
        context.setdefault('active_deployment_id', str(patrol.get('deployment_id') or ''))
        context.setdefault('active_request_id', str(patrol.get('request_id') or ''))
        context.setdefault('active_command_id', str(patrol.get('command_id') or ''))
        self._cloud_snapshot = {'state': str(patrol.get('state') or 'idle'), 'mapPose': status.get('mapPose'), 'odomPose': status.get('odomPose'), 'platformContext': context, 'health': {'odomAgeSec': status.get('last_odom_age_sec'), 'scanAgeSec': status.get('last_scan_age_sec'), 'imuAgeSec': status.get('last_imu_age_sec'), 'nav2': status.get('nav2_status'), 'systemMode': status.get('system_mode'), 'lastError': patrol.get('last_error') or self._system_status.get('last_error')}}
        if self.cloud_client:
            self.publish_cloud_status_now()
        self._publish_local_app_status()

    def publish_cloud_status_now(self, status: Optional[dict] = None) -> None:
        publisher = getattr(self, '_cloud_status_pub', None)
        if not self.cloud_client or publisher is None:
            return
        msg = String()
        msg.data = json.dumps(status or self.cloud_client.status(), ensure_ascii=False)
        publisher.publish(msg)

    def initialize_local_app_settings(self, store) -> None:
        override = store.bridge_setting(
            'local_app_enabled_override', ''
        ).strip().lower()
        if override in {'true', 'false'}:
            self._local_app_enabled = override == 'true'
        self._local_app_last_changed_at = _now()

    def is_local_app_enabled(self) -> bool:
        return bool(self._local_app_enabled)

    def set_local_app_http_available(self, available: bool, error: str = '') -> None:
        self._local_app_http_available = bool(available)
        self._local_app_last_error = str(error or '')
        self._publish_local_app_status()

    def local_app_client_connected(self, kind: str) -> None:
        lock = getattr(self, '_local_app_lock', None)
        if lock:
            with lock:
                self._local_app_clients[kind] = self._local_app_clients.get(kind, 0) + 1
        else:
            self._local_app_clients[kind] = self._local_app_clients.get(kind, 0) + 1
        self._publish_local_app_status()

    def local_app_client_disconnected(self, kind: str) -> None:
        lock = getattr(self, '_local_app_lock', None)
        if lock:
            with lock:
                self._local_app_clients[kind] = max(
                    0, self._local_app_clients.get(kind, 0) - 1
                )
        else:
            self._local_app_clients[kind] = max(
                0, self._local_app_clients.get(kind, 0) - 1
            )
        self._publish_local_app_status()

    def local_app_status_snapshot(self) -> dict:
        enabled = self.is_local_app_enabled()
        available = bool(self._local_app_http_available)
        if enabled and available:
            state = 'ENABLED'
        elif not enabled:
            state = 'DISABLED'
        else:
            state = 'DEGRADED'
        system_status = self.system_status()
        network = self._network_status_snapshot()
        app_url = str(system_status.get('mobile_bridge_url') or '')
        if not app_url:
            app_url = str(
                (network['preferredAppEndpoint'] or {}).get('url') or ''
            )
        if not app_url:
            app_url = f"http://127.0.0.1:{getattr(self, 'port', 8000)}"
        lock = getattr(self, '_local_app_lock', None)
        if lock:
            with lock:
                clients = dict(self._local_app_clients)
        else:
            clients = dict(self._local_app_clients)
        return {
            'enabled': enabled,
            'state': state,
            'httpAvailable': available,
            'appUrl': app_url,
            'authRequired': bool(getattr(self, 'require_token', False)),
            'activeStatusClients': clients.get('status', 0),
            'activeMapClients': clients.get('map', 0),
            'managedExternally': bool(
                system_status.get('mobile_bridge_managed_externally', True)
            ),
            'appEndpoints': network['appEndpoints'],
            'candidateEndpoints': network['candidateEndpoints'],
            'preferredAppEndpoint': network['preferredAppEndpoint'],
            'networkInterfaces': network['interfaces'],
            'networkWarnings': network['warnings'],
            'wifiReconnect': network['wifiReconnect'],
            'lastChangedAt': self._local_app_last_changed_at,
            'lastError': self._local_app_last_error,
        }

    def _network_status_snapshot(self) -> dict:
        provider = getattr(self, 'network_status', None)
        empty = {
            'appEndpoints': [],
            'candidateEndpoints': [],
            'preferredAppEndpoint': {},
            'interfaces': [],
            'warnings': [],
            'wifiReconnect': {'configured': False},
        }
        if provider is None:
            return empty
        try:
            snapshot = provider.snapshot()
            endpoints = provider.app_endpoints(
                getattr(self, 'host', '0.0.0.0'),
                getattr(self, 'port', 8000),
            )
            reconnect_reader = getattr(provider, 'wifi_reconnect_status', None)
            wifi_reconnect = (
                reconnect_reader()
                if callable(reconnect_reader)
                else {'configured': False}
            )
        except Exception:
            return empty
        return {
            'appEndpoints': endpoints,
            'candidateEndpoints': [
                {
                    'url': endpoint['url'],
                    'interface': endpoint['interface'],
                    'type': endpoint['type'],
                    'linkUp': bool(endpoint['available']),
                }
                for endpoint in endpoints
            ],
            'preferredAppEndpoint': {},
            'interfaces': list(snapshot.get('interfaces') or []),
            'warnings': list(snapshot.get('warnings') or []),
            'wifiReconnect': wifi_reconnect,
        }

    def set_local_app_enabled(self, enabled: bool) -> dict:
        enabled = bool(enabled)
        store = getattr(self, 'platform_store', None)
        if store is None:
            raise RuntimeError('bridge settings store is not ready')
        store.set_bridge_setting(
            'local_app_enabled_override', 'true' if enabled else 'false'
        )
        self._local_app_enabled = enabled
        self._local_app_last_changed_at = _now()
        self._local_app_last_error = ''
        status = self.local_app_status_snapshot()
        self._publish_local_app_status(status)
        return status

    def _publish_local_app_status(self, status: Optional[dict] = None) -> None:
        publisher = getattr(self, '_local_app_status_pub', None)
        if publisher is None:
            return
        msg = String()
        msg.data = json.dumps(
            status or self.local_app_status_snapshot(), ensure_ascii=False
        )
        publisher.publish(msg)

    def _set_local_app_enabled(self, request, response):
        try:
            status = self.set_local_app_enabled(bool(request.data))
            response.success = True
            response.message = str(status.get('state') or '')
        except Exception as exc:
            self._local_app_last_error = str(exc)
            response.success = False
            response.message = str(exc)
            self._publish_local_app_status()
        return response

    def _set_cloud_enabled(self, request, response):
        if not self.cloud_client:
            response.success = False
            response.message = 'cloud client is not ready'
            return response
        try:
            status = self.cloud_client.set_enabled(bool(request.data))
            response.success = bool(status.get('configured')) or not bool(request.data)
            response.message = str(status.get('state') or '')
        except Exception as exc:
            status = dict(self.cloud_client.status())
            status['lastError'] = f'cloud control failed: {type(exc).__name__}'
            response.success = False
            response.message = status['lastError']
        self.publish_cloud_status_now(status)
        return response

    def cloud_status_snapshot(self) -> dict:
        return dict(self._cloud_snapshot)

    def set_platform_context(self, context: dict) -> None:
        self._platform_context = dict(context)

    def has_system_supervisor(self) -> bool:
        return bool(self._system_status)

    def system_status(self) -> dict:
        return dict(self._system_status)

    def publish_patrol_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._patrol_command_pub.publish(msg)

    def patrol_status(self) -> dict:
        return dict(self._patrol_status)

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
