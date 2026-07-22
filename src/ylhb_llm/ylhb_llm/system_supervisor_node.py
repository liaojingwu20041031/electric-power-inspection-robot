import json
import math
import os
import signal
import shlex
import socket
import subprocess
import threading
import time
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes, SetInitialPose
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty
from ylhb_mobile_bridge.patrol_qos import patrol_status_qos_profile
from ylhb_mobile_bridge.patrol_route_store import (
    default_workspace_dir,
    load_route_file,
    resolve_route_file_path,
    validate_route_map_binding,
)
from .robot_recovery import RecoveryCatalog

try:
    from ylhb_3d_mapping import zed_3d_asset_manager
except ModuleNotFoundError:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'ylhb_3d_mapping'))
    from ylhb_3d_mapping import zed_3d_asset_manager


START_PROCESS_MESSAGES = {
    'bringup': '底盘与雷达已启动',
    'navigation': '导航已启动',
    'patrol_executor': '巡逻执行器已启动',
    'zed': 'ZED 相机已启动',
    '3d_mapping': '三维建图已启动',
    '3d_capture': '现场 SVO 采集已启动',
    '3d_reconstruct': '离线三维重建已启动',
    'perception': '感知节点已启动',
    'mobile_bridge': '移动端桥接已启动',
    'mapping': '建图已启动',
}

PATROL_BRINGUP_TO_NAVIGATION_DELAY_SEC = 3.0
PATROL_NAVIGATION_TO_EXECUTOR_DELAY_SEC = 20.0
PATROL_EXECUTOR_TO_START_DELAY_SEC = 6.0
LONG_RUNNING_COMMANDS = {
    'start_patrol_mode',
    'go_to_checkpoint',
    'stop_robot_stack',
    'stop_navigation',
    'stop_bringup',
    'stop_patrol_mode',
    'recover_component',
}
AGENT_COMPONENTS = {'bringup', 'navigation', 'perception', 'patrol_executor'}

STARTUP_STEP_LABELS = {
    'starting_bringup': '等待底盘传感器',
    'waiting_bringup': '等待底盘传感器',
    'waiting_odom': '等待底盘传感器',
    'waiting_scan': '等待底盘传感器',
    'waiting_tf': '等待底盘传感器',
    'waiting_after_bringup': '底盘启动后等待',
    'starting_navigation': '等待地图',
    'navigation_process_spawned': '导航进程已创建',
    'navigation_ready': '导航已就绪',
    'waiting_navigation': '等待地图',
    'waiting_map': '等待地图',
    'waiting_initialpose_subscribers': '等待地图',
    'waiting_after_navigation': '导航启动后等待',
    'generating_keepout_mask': '生成禁行区',
    'waiting_keepout_active': '等待禁行区',
    'starting_executor': '启动巡逻执行器',
    'executor_process_spawned': '巡逻执行器进程已创建',
    'executor_ready': '巡逻执行器已就绪',
    'waiting_executor': '发布初始位姿',
    'waiting_route_file': '发布初始位姿',
    'waiting_patrol_status': '发布初始位姿',
    'waiting_initial_pose_published': '发布初始位姿',
    'waiting_after_executor': '巡逻执行器启动后等待',
    'waiting_patrol_running': '等待巡逻执行器运行',
    'patrol_start_sent': '巡逻启动命令已发送',
    'patrol_command_sent': '巡逻启动命令已发送',
    'navigation_process_exited': '导航进程启动后退出',
    'executor_process_exited': '巡逻执行器启动后退出',
    'waiting_executor_response': '等待巡逻执行器响应',
    'waiting_nav2': '等待导航服务',
    'sending_goal': '发送导航目标',
    'retrying_goal': '导航目标重试',
    'returning_home': '返回初始点',
    'patrol_started': '巡逻运行中',
    'patrol_failed': '巡逻启动失败',
}

READINESS_ERROR_MESSAGES = {
    'odom': '等待 /odom 发布者超时',
    'scan': '等待 /scan 发布者超时',
    'tf': '等待 /tf 发布者超时',
    'map': '等待 /map 发布者超时',
    'map_to_odom': '等待 map->odom TF 超时',
    'initialpose_subscribers': '等待 /initialpose 订阅者超时',
    'nav2_action': 'Nav2 动作服务未就绪',
    'nav2_action_discovered': 'Nav2 动作服务未就绪',
    'nav2_components_loaded': 'Nav2 核心组件未加载',
    'nav2_active': 'Nav2 未激活',
    'keepout_active': 'Keepout lifecycle 尚未 active',
    'executor': '巡逻执行器未就绪',
    'route_file': '未找到正式巡逻路线文件',
    'patrol_status': '等待 /patrol/status 发布者超时',
}

LOCALIZATION_LIFECYCLE_NODES = ('/map_server', '/amcl')
NAVIGATION_LIFECYCLE_NODES = (
    '/controller_server',
    '/smoother_server',
    '/planner_server',
    '/behavior_server',
    '/bt_navigator',
    '/waypoint_follower',
    '/velocity_smoother',
)
KEEPOUT_LIFECYCLE_NODES = (
    '/keepout_global_mask_server',
    '/keepout_global_filter_info_server',
    '/keepout_local_mask_server',
    '/keepout_local_filter_info_server',
)
NAVIGATION_PROFILE_AUTO = 'auto'
NAVIGATION_PROFILE_NORMAL = 'normal'
NAVIGATION_PROFILE_KEEPOUT = 'keepout'
LIFECYCLE_MANAGER_LOCALIZATION = '/lifecycle_manager_localization/manage_nodes'
LIFECYCLE_MANAGER_NAVIGATION = '/lifecycle_manager_navigation/manage_nodes'
LIFECYCLE_MANAGER_KEEPOUT = '/lifecycle_manager_keepout/manage_nodes'
SHUTDOWN_ORDER = (
    'patrol_executor',
    'navigation',
    'perception',
    'zed',
    'mapping',
    'bringup',
    'mobile_bridge',
)
PATROL_SHUTDOWN_ORDER = (
    'patrol_executor',
    'navigation',
    'perception',
    'zed',
    'bringup',
)


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def workspace_path(*parts: str) -> str:
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    return os.path.join(workspace_dir, *parts)


def discover_jetson_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('8.8.8.8', 80))
        address = str(sock.getsockname()[0])
        if address and not address.startswith('127.'):
            return address
    except OSError:
        pass
    finally:
        sock.close()
    try:
        address = socket.gethostbyname(socket.gethostname())
        return address if address and not address.startswith('127.') else '127.0.0.1'
    except OSError:
        return '127.0.0.1'


def mobile_bridge_tcp_status(
    process_running: bool,
    connector=socket.create_connection,
    host: str = '127.0.0.1',
    port: int = 8000,
) -> str:
    if not process_running:
        return 'stopped'
    try:
        sock = connector((host, port), timeout=0.4)
        try:
            return 'tcp_ok'
        finally:
            sock.close()
    except Exception:
        return 'tcp_error'


def mobile_bridge_http_status(process_running: bool, **kwargs) -> str:
    return mobile_bridge_tcp_status(process_running, **kwargs)


class ManagedProcess:
    def __init__(self, name: str, command: str) -> None:
        self.name = name
        self.command = command
        self.process: Optional[subprocess.Popen] = None
        self.pgid: Optional[int] = None
        self.last_message = ''
        self.last_exit_code: Optional[int] = None
        self.last_started_at = 0.0
        self.last_error = ''
        self.last_command = command

    def poll_exit_code(self) -> Optional[int]:
        if self.process is None:
            return None
        exit_code = self.process.poll()
        if exit_code is not None:
            self.last_exit_code = exit_code
        return exit_code

    def is_running(self) -> bool:
        return self.process is not None and self.poll_exit_code() is None


class SystemSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('system_supervisor_node')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('workspace_dir', '')
        self.declare_parameter('route_directory', '')
        self.declare_parameter('ros_distro', 'humble')
        self.declare_parameter('map_output_dir', workspace_path('maps'))
        self.declare_parameter('mapping3d_output_dir', workspace_path('runs', '3d_capture'))
        self.declare_parameter('mapping3d_capture_dir', '')
        self.declare_parameter('mapping3d_reconstruct_dir', workspace_path('runs', '3d_reconstruct'))
        self.declare_parameter('default_navigation_map', workspace_path('maps', 'my_map.yaml'))
        self.declare_parameter('enable_keepout_navigation', True)
        self.declare_parameter('patrol_navigation_profile', NAVIGATION_PROFILE_AUTO)
        self.declare_parameter('keepout_mask_path', workspace_path('maps', 'keepout', 'keepout_mask_power_room_a.yaml'))
        self.declare_parameter('patrol_route_path', 'auto')
        self.declare_parameter('keepout_route_path', '')  # deprecated alias
        self.declare_parameter('perception_model_path', workspace_path('src', 'ylhb_perception', 'models', 'yolo26.engine'))
        self.declare_parameter('embedded_task_layer', True)
        self.declare_parameter('enable_voice', False)
        self.declare_parameter('enable_voice_session', False)
        self.declare_parameter('enable_capture_voice', False)
        self.declare_parameter('enable_tts', False)
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('audio_input_device', 'default')
        self.declare_parameter('audio_output_device', 'default')
        self.declare_parameter('asr_model', 'qwen3-asr-flash')
        self.declare_parameter('tts_model', 'qwen3-tts-flash')
        self.declare_parameter('tts_voice', 'Serena')
        self.declare_parameter('tts_language_type', 'Chinese')
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('patrol_sensor_freshness_sec', 1.0)
        self.declare_parameter('patrol_bringup_timeout_sec', 25.0)
        self.declare_parameter('patrol_navigation_timeout_sec', 35.0)
        self.declare_parameter('patrol_localization_timeout_sec', 25.0)
        self.declare_parameter('patrol_executor_timeout_sec', 15.0)
        self.declare_parameter('patrol_initial_pose_timeout_sec', 8.0)
        self.declare_parameter('patrol_amcl_timeout_sec', 10.0)
        self.declare_parameter('patrol_nav2_timeout_sec', 25.0)
        self.declare_parameter('patrol_command_timeout_sec', 8.0)
        self.declare_parameter('mobile_bridge_managed_externally', False)
        self.declare_parameter('auto_start_mobile_bridge', True)
        self.declare_parameter('recovery_config_file', '')

        workspace_dir = str(self.get_parameter('workspace_dir').value).strip()
        self.workspace_dir = os.path.expanduser(workspace_dir) if workspace_dir else str(default_workspace_dir())
        route_directory = str(self.get_parameter('route_directory').value).strip()
        self.route_directory = os.path.expanduser(route_directory) if route_directory else os.path.join(self.workspace_dir, 'maps')
        self.ros_distro = str(self.get_parameter('ros_distro').value)
        map_output_dir = str(self.get_parameter('map_output_dir').value).strip()
        self.map_output_dir = os.path.expanduser(map_output_dir) if map_output_dir else os.path.join(self.workspace_dir, 'maps')
        mapping3d_output_dir = str(self.get_parameter('mapping3d_output_dir').value).strip()
        self.mapping3d_output_dir = os.path.expanduser(mapping3d_output_dir) if mapping3d_output_dir else os.path.join(self.workspace_dir, 'runs', '3d_capture')
        capture_dir = str(self.get_parameter('mapping3d_capture_dir').value or self.mapping3d_output_dir)
        self.mapping3d_capture_dir = os.path.expanduser(capture_dir)
        self.mapping3d_output_dir = self.mapping3d_capture_dir
        reconstruct_dir = str(self.get_parameter('mapping3d_reconstruct_dir').value).strip()
        self.mapping3d_reconstruct_dir = os.path.expanduser(reconstruct_dir) if reconstruct_dir else os.path.join(self.workspace_dir, 'runs', '3d_reconstruct')
        navigation_map = str(self.get_parameter('default_navigation_map').value).strip()
        self.default_navigation_map = os.path.expanduser(navigation_map) if navigation_map else os.path.join(self.workspace_dir, 'maps', 'my_map.yaml')
        self.enable_keepout_navigation = bool(self.get_parameter('enable_keepout_navigation').value)
        self.patrol_navigation_profile = str(
            self.get_parameter('patrol_navigation_profile').value
            or NAVIGATION_PROFILE_AUTO
        )
        self.keepout_mask_path = os.path.expanduser(str(self.get_parameter('keepout_mask_path').value))
        keepout_output_dir = os.path.dirname(self.keepout_mask_path)
        self.keepout_global_mask_path = os.path.join(
            keepout_output_dir, 'keepout_global_mask.yaml'
        )
        self.keepout_local_mask_path = os.path.join(
            keepout_output_dir, 'keepout_local_mask.yaml'
        )
        patrol_route_path = str(self.get_parameter('patrol_route_path').value).strip()
        keepout_route_path = str(self.get_parameter('keepout_route_path').value).strip()
        self.patrol_route_request = patrol_route_path or keepout_route_path or 'auto'
        self.local_patrol_route_request = self.patrol_route_request
        self.local_default_navigation_map = self.default_navigation_map
        self.platform_context = {}
        resolved_route_file = resolve_route_file_path(
            self.patrol_route_request,
            self.route_directory,
            required=False,
        )
        self.patrol_route_path = str(resolved_route_file or '')
        self.keepout_route_path = self.patrol_route_path
        perception_model_path = str(self.get_parameter('perception_model_path').value).strip()
        self.perception_model_path = os.path.expanduser(perception_model_path) if perception_model_path else os.path.join(self.workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')
        self.get_logger().info(f'supervisor patrol route: resolved_file={self.patrol_route_path}')
        self.embedded_task_layer = bool(self.get_parameter('embedded_task_layer').value)
        self.enable_voice = bool(self.get_parameter('enable_voice').value)
        self.enable_voice_session = bool(self.get_parameter('enable_voice_session').value)
        self.enable_capture_voice = bool(self.get_parameter('enable_capture_voice').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)
        self.mobile_bridge_managed_externally = bool(self.get_parameter('mobile_bridge_managed_externally').value)
        self.auto_start_mobile_bridge = bool(self.get_parameter('auto_start_mobile_bridge').value)
        self.mobile_bridge_owner = 'systemd' if self.mobile_bridge_managed_externally else 'supervisor'
        self.mobile_bridge_started_by_supervisor = False
        self._mobile_bridge_auto_start_attempted = False
        self.mobile_bridge_ownership_conflict = False
        self.mobile_bridge_last_error = ''
        self.audio_device = str(self.get_parameter('audio_device').value)
        self.audio_input_device = str(self.get_parameter('audio_input_device').value)
        self.audio_output_device = str(self.get_parameter('audio_output_device').value)
        self.asr_model = str(self.get_parameter('asr_model').value)
        self.tts_model = str(self.get_parameter('tts_model').value)
        self.tts_voice = str(self.get_parameter('tts_voice').value)
        self.tts_language_type = str(self.get_parameter('tts_language_type').value)
        self.dashscope_base_url = str(self.get_parameter('dashscope_base_url').value)
        self.lock = threading.Lock()
        self.last_command = ''
        self.last_success = True
        self.last_message = 'system supervisor ready'
        self.command_result: Dict[str, Any] = {}
        self.agent_operation_context: Dict[str, str] = {}
        self.jetson_ip = discover_jetson_ip()
        self.mobile_bridge_url = f'http://{self.jetson_ip}:8000'
        self.mobile_bridge_http = 'stopped'
        self.current_system_mode = 'ready'
        self.patrol_mode_state = 'idle'
        self.patrol_error = ''
        self.patrol_warning = ''
        self.active_patrol_navigation_mode = NAVIGATION_PROFILE_NORMAL
        self.active_hard_keepout_count = 0
        self.patrol_navigation_assets_prepared = False
        self.startup_step = ''
        self.last_patrol_status: Dict[str, Any] = {}
        self.last_patrol_status_received_at = 0.0
        self.last_patrol_start_request_id = ''
        self.last_patrol_event: Dict[str, Any] = {}
        self.last_initial_pose_event: Dict[str, Any] = {}
        self.last_patrol_command_ack: Dict[str, Any] = {}
        self.last_odom_received_at = 0.0
        self.last_scan_received_at = 0.0
        self.last_scan_stamp_at = 0.0
        self.last_amcl_received_at = 0.0
        self.startup_generation = 0
        self.startup_id = ''
        self.startup_started_at = 0.0
        self.patrol_start_cancel_event = threading.Event()
        self.patrol_start_active = False
        self.set_initial_pose_client = None
        self.initial_pose_request_sent_at = 0.0
        self.initial_pose_service_ok = False
        self.initial_pose_confirmed_at = 0.0
        self.localization_lifecycle_started = False
        self.navigation_lifecycle_started = False
        self.keepout_lifecycle_started = False
        self.latest_mapping3d_status: Dict[str, Any] = {}
        self.latest_mapping3d_result: Dict[str, Any] = {}
        self.latest_scene_upload_status: Dict[str, Any] = {}
        self.scene_uploads_by_session: Dict[str, Dict[str, Any]] = {}
        self.inflight_commands = set()
        self.lifecycle_clients: Dict[str, Any] = {}
        self.lifecycle_manager_clients: Dict[str, Any] = {}
        self.lifecycle_states: Dict[str, str] = {}
        self.map_to_odom_stable_since = 0.0
        self.tf_buffer = None
        self.tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception as exc:
            self.log_info(f'tf listener unavailable for startup readiness: {exc}')

        self.processes: Dict[str, ManagedProcess] = {
            'bringup': ManagedProcess('bringup', 'ros2 launch ylhb_base bringup.launch.py'),
            'mapping': ManagedProcess('mapping', 'ros2 launch ylhb_base mapping.launch.py'),
            'navigation': ManagedProcess(
                'navigation',
                self.navigation_launch_command(),
            ),
            'zed': ManagedProcess('zed', 'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'),
            '3d_capture': ManagedProcess(
                '3d_capture',
                f'ros2 run ylhb_3d_mapping zed_svo_capture_node '
                f'--ros-args -p output_root:={self.mapping3d_output_dir} '
                f'-p auto_start:=true -p exit_on_finish:=true',
            ),
            'perception': ManagedProcess(
                'perception',
                f'ros2 launch ylhb_perception perception.launch.py '
                f'model_path:={self.perception_model_path} backend:=tensorrt half:=true',
            ),
            'llm': ManagedProcess(
                'llm',
                self.llm_launch_command(),
            ),
            'mobile_bridge': ManagedProcess(
                'mobile_bridge',
                'ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py',
            ),
            'patrol_executor': ManagedProcess(
                'patrol_executor',
                self.build_patrol_executor_command(),
            ),
        }
        recovery_config = str(self.get_parameter('recovery_config_file').value).strip()
        recovery_path = Path(recovery_config or 'agent_recovery.yaml').expanduser()
        if not recovery_path.is_absolute():
            recovery_path = get_package_share_path('ylhb_llm') / 'config' / recovery_path
        self.recovery_catalog = RecoveryCatalog.from_file(
            recovery_path, managed_processes=self.processes)
        self.recovery_last_attempt_at: Dict[str, float] = {}
        self.recovery_incidents: set[tuple[str, str]] = set()

        self.status_pub = self.create_publisher(
            String, self.get_parameter('system_status_topic').value, latched_qos())
        self.mode_pub = self.create_publisher(
            String, self.get_parameter('system_mode_topic').value, latched_qos())
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.patrol_command_pub = self.create_publisher(String, '/patrol/command', 10)
        self.mapping3d_command_pub = self.create_publisher(String, '/inspection_ai/mapping3d_capture_command', 10)
        self.scene_asset_ready_pub = self.create_publisher(
            String, '/inspection_ai/scene_asset_ready', latched_qos())
        self.scene_upload_command_pub = self.create_publisher(
            String, '/inspection_ai/scene_upload_command', 10)
        self.create_subscription(
            String,
            self.get_parameter('system_command_topic').value,
            self.command_callback,
            10,
        )
        self.create_subscription(
            String,
            '/patrol/status',
            self.patrol_status_callback,
            patrol_status_qos_profile(),
        )
        self.create_subscription(
            String,
            '/patrol/event',
            self.patrol_event_callback,
            patrol_status_qos_profile(),
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10
        )
        self.create_subscription(
            String,
            '/inspection_ai/mapping3d_status',
            self.mapping3d_status_callback,
            latched_qos(),
        )
        self.create_subscription(
            String,
            '/inspection_ai/mapping3d_result',
            self.mapping3d_result_callback,
            latched_qos(),
        )
        self.create_subscription(
            String,
            '/inspection_ai/scene_upload_status',
            self.scene_upload_status_callback,
            latched_qos(),
        )
        self.create_timer(1.0, self.publish_status)
        self._mobile_bridge_auto_start_scheduled = False
        self._mobile_bridge_auto_start_timer = self.create_timer(
            0.2, self.schedule_mobile_bridge_auto_start
        )
        self.publish_status()
        self.log_info('系统监督节点已启动')

    def schedule_mobile_bridge_auto_start(self) -> None:
        if self._mobile_bridge_auto_start_scheduled:
            return
        self._mobile_bridge_auto_start_scheduled = True
        self._mobile_bridge_auto_start_timer.cancel()
        if self.mobile_bridge_managed_externally or not self.auto_start_mobile_bridge:
            return
        threading.Thread(
            target=self.auto_start_mobile_bridge_once,
            name='mobile-bridge-auto-start',
            daemon=True,
        ).start()

    def mobile_bridge_external_instance_present(self) -> bool:
        if mobile_bridge_tcp_status(True) == 'tcp_ok':
            return True
        try:
            return 'mobile_bridge' in set(self.get_node_names())
        except Exception:
            return False

    def auto_start_mobile_bridge_once(self) -> None:
        if getattr(self, '_mobile_bridge_auto_start_attempted', False):
            return
        self._mobile_bridge_auto_start_attempted = True
        proc = self.processes.get('mobile_bridge')
        if self.mobile_bridge_managed_externally or (proc and proc.is_running()):
            return
        if self.mobile_bridge_external_instance_present():
            self.mobile_bridge_ownership_conflict = True
            self.mobile_bridge_last_error = '检测到外部 Mobile Bridge，Supervisor 未启动第二个实例'
            self.publish_status()
            return
        if self.start_process('mobile_bridge'):
            self.mobile_bridge_started_by_supervisor = True
            self.mobile_bridge_last_error = ''

    def start_owned_mobile_bridge(self) -> bool:
        if self.mobile_bridge_external_instance_present():
            self.mobile_bridge_ownership_conflict = True
            self.mobile_bridge_last_error = '检测到外部 Mobile Bridge，拒绝重复启动'
            self.set_result('start_mobile_bridge', False, self.mobile_bridge_last_error)
            return False
        self.mobile_bridge_ownership_conflict = False
        started = self.start_process('mobile_bridge')
        self.mobile_bridge_started_by_supervisor = bool(started)
        return started

    def recover_component(self, component: str, payload: Dict[str, Any]) -> None:
        correlation = {
            key: str(payload.get(key) or '')
            for key in ('run_id', 'operation_id', 'tool_call_id')
        }

        def finish(success: bool, result_status: str, message: str) -> None:
            self.set_result('recover_component', success, message)
            self.publish_component_operation_feedback(correlation, success, result_status)

        catalog = getattr(self, 'recovery_catalog', None)
        if catalog is None or component not in catalog.names():
            finish(False, 'rejected', '组件不在恢复白名单')
            return
        recovery = catalog.get(component)
        process_name = str(recovery.get('process') or '')
        if process_name not in getattr(self, 'processes', {}):
            finish(False, 'rejected', '恢复目标不是 Supervisor 受管进程')
            return
        if recovery.get('requires_no_active_patrol') and str(
            getattr(self, 'patrol_mode_state', 'idle')) in {
                'starting', 'command_sent', 'running', 'paused', 'returning_home',
            }:
            finish(False, 'rejected', '活动巡逻期间禁止恢复该组件')
            return
        if recovery.get('requires_supervisor_ownership') and str(
            getattr(self, 'mobile_bridge_owner', 'unknown')) != 'supervisor':
            finish(False, 'rejected', 'Mobile Bridge 由 systemd 管理，应由 systemd 处理')
            return
        now = time.monotonic()
        diagnostic_id = str(payload.get('diagnostic_id') or '')
        incident = (component, diagnostic_id)
        cooldown = float(recovery.get('cooldown_sec') or 0.0)
        if (
            (diagnostic_id and incident in getattr(self, 'recovery_incidents', set()))
            or now - float(getattr(self, 'recovery_last_attempt_at', {}).get(component, 0.0)) < cooldown
        ):
            finish(False, 'rejected', '该故障已尝试恢复，当前处于冷却期')
            return
        self.recovery_last_attempt_at[component] = now
        if diagnostic_id:
            self.recovery_incidents.add(incident)
        try:
            self.stop_process(process_name, report=False)
            if not self.start_process(process_name):
                finish(False, 'failed', f'{component} 恢复失败：进程未能启动')
                return
        except Exception as exc:
            finish(False, 'failed', f'{component} 恢复失败：{exc}')
            return
        process = self.processes[process_name]
        if not process.is_running() or str(getattr(process, 'last_error', '') or ''):
            finish(False, 'failed', f'{component} 启动后立即退出或报告错误')
            return
        if recovery.get('verification') == 'bridge_tcp_ok':
            if mobile_bridge_tcp_status(True) != 'tcp_ok':
                finish(False, 'failed', 'Mobile Bridge 进程已启动，但 TCP 端口不可达')
                return
            self.mobile_bridge_started_by_supervisor = True
        if component == 'perception':
            zed = self.processes.get('zed')
            if zed is not None and not zed.is_running():
                finish(True, 'succeeded', '感知进程已恢复，但相机输入尚未就绪')
                return
        finish(True, 'succeeded', f'{component} 已恢复并通过验证')

    def command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.set_result('invalid_json', False, f'Invalid system command JSON: {exc}')
            return
        if not isinstance(payload, dict):
            self.set_result('invalid_payload', False, 'System command must be a JSON object.')
            return

        command = str(payload.get('command') or '').strip()
        if not command:
            self.set_result('', False, 'Missing command.')
            return

        if command in LONG_RUNNING_COMMANDS:
            if not self.try_mark_command_inflight(command):
                self.set_result(command, True, '重复命令已忽略，命令正在执行')
                return
            target = self.handle_inflight_command
        else:
            target = self.handle_command
        threading.Thread(target=target, args=(command, payload), daemon=True).start()

    def try_mark_command_inflight(self, command: str) -> bool:
        with self.lock:
            if not hasattr(self, 'inflight_commands'):
                self.inflight_commands = set()
            if command in self.inflight_commands:
                return False
            self.inflight_commands.add(command)
            return True

    def handle_inflight_command(self, command: str, payload: Dict[str, Any]) -> None:
        try:
            self.handle_command(command, payload)
        finally:
            with self.lock:
                self.inflight_commands.discard(command)

    def patrol_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        if isinstance(payload, dict):
            self.last_patrol_status = payload
            self.last_patrol_status_received_at = time.time()
            state = str(payload.get('state') or payload.get('status') or '')
            if state == 'running':
                self.patrol_mode_state = 'running'
                phase = str(payload.get('navigation_phase') or 'target')
                self.startup_step = {
                    'waiting_nav2': 'waiting_nav2',
                    'sending_goal': 'sending_goal',
                    'retrying_goal': 'retrying_goal',
                    'target': 'patrol_started',
                    'return_home': 'returning_home',
                }.get(phase, 'patrol_started')
                self.patrol_error = ''
            elif state == 'failed':
                self.patrol_mode_state = 'failed'
                self.startup_step = 'patrol_failed'
            elif state in ('canceled', 'cancelled'):
                self.patrol_mode_state = 'canceled'
            elif state == 'succeeded':
                self.patrol_mode_state = 'succeeded'
            elif state == 'idle' and self.patrol_mode_state in ('starting', 'command_sent'):
                self.startup_step = 'waiting_executor_response'
            if (
                getattr(self, 'patrol_transaction_kind', '') in {'checkpoint', 'route'}
                and state in {'failed', 'succeeded', 'canceled', 'cancelled'}
            ):
                self.cleanup_checkpoint_transaction()
                self.patrol_transaction_kind = ''

    def patrol_event_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        if isinstance(payload, dict):
            self.last_patrol_event = payload
            if payload.get('event') == 'initial_pose_published':
                self.last_initial_pose_event = payload
            if payload.get('event') == 'command_accepted':
                self.last_patrol_command_ack = payload
            event = str(payload.get('event') or '')
            operation_id = str(payload.get('operation_id') or '')
            if operation_id and event in {
                'route_paused', 'route_resumed', 'route_canceled', 'command_rejected',
            }:
                success = event != 'command_rejected'
                self.last_agent_operation_feedback = {
                    'run_id': str(payload.get('run_id') or ''),
                    'operation_id': operation_id,
                    'tool_call_id': str(payload.get('tool_call_id') or ''),
                    'state': 'succeeded' if success else 'failed',
                    'status': 'succeeded' if success else 'failed',
                    'result_status': {
                        'route_paused': 'succeeded',
                        'route_resumed': 'running',
                        'route_canceled': 'canceled',
                    }.get(event, 'failed'),
                    'message': str(payload.get('error_message') or event),
                }
                self.publish_status()
                self.last_agent_operation_feedback = {}

    def odom_callback(self, _msg: Odometry) -> None:
        self.last_odom_received_at = time.time()

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan_received_at = time.time()
        self.last_scan_stamp_at = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )

    def amcl_pose_callback(self, _msg: PoseWithCovarianceStamped) -> None:
        self.last_amcl_received_at = time.time()

    def mapping3d_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        if isinstance(payload, dict):
            self.latest_mapping3d_status = payload

    def mapping3d_result_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        if isinstance(payload, dict):
            self.latest_mapping3d_result = payload
        if not isinstance(payload, dict) or payload.get('state') != 'succeeded':
            return
        model_path = Path(str(payload.get('output_file') or '')).expanduser()
        metadata_path = Path(str(payload.get('metadata_file') or '')).expanduser()
        try:
            root = Path(self.mapping3d_reconstruct_dir).expanduser().resolve()
            model_path = model_path.resolve(strict=True)
            metadata_path = metadata_path.resolve(strict=True)
        except OSError as exc:
            self.get_logger().error(f'3D reconstruction result is incomplete: {exc}')
            return
        if (
            not model_path.is_file()
            or not metadata_path.is_file()
            or not model_path.is_relative_to(root)
            or not metadata_path.is_relative_to(root)
        ):
            self.get_logger().error('3D reconstruction result paths are invalid')
            return
        ready = String()
        ready.data = json.dumps({
            'schemaVersion': '1.0',
            'state': 'succeeded',
            'assetKind': 'POINT_CLOUD',
            'format': 'PLY',
            'sourceSessionId': str(payload.get('session_id') or model_path.parent.name),
            'modelPath': str(model_path),
            'metadataPath': str(metadata_path),
        }, ensure_ascii=False)
        self.scene_asset_ready_pub.publish(ready)

    def scene_upload_status_callback(self, msg: String) -> None:
        payload = self._json_object(msg.data)
        if not payload:
            return
        self.latest_scene_upload_status = payload
        session_id = str(payload.get('sourceSessionId') or '')
        if session_id:
            self.scene_uploads_by_session[session_id] = payload

    @staticmethod
    def _json_object(raw: str) -> Dict[str, Any]:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return {}
        return payload if isinstance(payload, dict) else {}

    def handle_command(self, command: str, payload: Dict[str, Any]) -> None:
        correlation = {
            key: str(payload.get(key) or '')
            for key in ('run_id', 'operation_id', 'tool_call_id')
        }
        if command in {'start_patrol_mode', 'go_to_checkpoint', 'pause_patrol', 'resume_patrol', 'cancel_patrol', 'emergency_stop', 'recover_component'}:
            self.agent_operation_context = correlation if correlation['operation_id'] else {}
            self.last_agent_operation_feedback = {}
        if command == 'recover_component':
            self.recover_component(str(payload.get('component') or ''), payload)
            return
        if command in {'start_mobile_bridge', 'stop_mobile_bridge', 'restart_mobile_bridge'} and getattr(self, 'mobile_bridge_managed_externally', False):
            self.set_result(command, True, 'Mobile Bridge 由 systemd 常驻管理')
            return
        if command == 'start_mobile_bridge':
            self.start_owned_mobile_bridge()
            return
        if command == 'stop_mobile_bridge':
            if self.mobile_bridge_started_by_supervisor:
                self.stop_process('mobile_bridge')
                self.mobile_bridge_started_by_supervisor = False
            else:
                self.set_result(command, True, 'Supervisor 没有启动 Mobile Bridge')
            return
        if command == 'restart_mobile_bridge':
            if self.mobile_bridge_started_by_supervisor:
                self.stop_process('mobile_bridge')
                self.mobile_bridge_started_by_supervisor = False
            self.start_owned_mobile_bridge()
            return
        if command == 'start_platform_patrol':
            required = ('active_execution_id', 'active_deployment_id', 'active_request_id', 'active_route_revision_id', 'active_route_path', 'active_map_yaml_path', 'executor_route_id')
            if any(not str(payload.get(key) or '').strip() for key in required):
                self.command_result = {
                    'event': 'command_rejected',
                    'command_id': str(payload.get('command_id') or ''),
                    'request_id': str(payload.get('active_request_id') or payload.get('request_id') or ''),
                    'execution_id': str(payload.get('active_execution_id') or payload.get('execution_id') or ''),
                    'deployment_id': str(payload.get('active_deployment_id') or payload.get('deployment_id') or ''),
                    'error_code': 'INCOMPLETE_SUPERVISOR_CONTEXT',
                    'error_message': '平台巡逻上下文不完整',
                }
                self.set_result(command, False, '平台巡逻上下文不完整')
                return
            self.platform_context = {key: str(payload[key]) for key in required}
            self.platform_context['active_command_id'] = str(payload.get('command_id') or '')
            self.patrol_route_request = self.platform_context['active_route_path']
            self.default_navigation_map = self.platform_context['active_map_yaml_path']
            self.start_patrol_mode(str(payload.get('profile') or 'inspection'), self.platform_context['executor_route_id'])
            return
        if command == 'start_patrol_mode':
            self.platform_context = {}
            self.start_patrol_mode(
                str(payload.get('profile') or 'navigation'),
                str(payload.get('route_id') or ''),
            )
            return
        if command == 'go_to_checkpoint':
            self.platform_context = {}
            self.start_patrol_mode(
                str(payload.get('profile') or 'navigation'),
                target_id=str(payload.get('target_id') or ''),
            )
            return
        patrol_commands = {
            'pause_patrol': 'pause',
            'resume_patrol': 'resume',
            'cancel_patrol': 'cancel',
        }
        if command == 'reload_patrol_route':
            self.reload_patrol_route()
            return
        if command in patrol_commands:
            patrol_command = patrol_commands[command]
            if patrol_command == 'cancel':
                self.cancel_patrol_start()
            self.publish_patrol_command(patrol_command, request_id=str(payload.get('request_id') or ''), command_id=str(payload.get('command_id') or ''))
            self.set_result(command, True, f'已发送巡逻命令: {patrol_command}')
            return
        if command == 'takeover_patrol':
            self.publish_patrol_command('takeover', request_id=str(payload.get('request_id') or ''), command_id=str(payload.get('command_id') or ''))
            self.set_result(command, True, '已发送人工接管命令')
            return
        if command == 'stop_patrol_mode':
            self.cancel_patrol_start()
            self.publish_patrol_command('cancel')
            self.publish_zero_velocity()
            time.sleep(0.2)
            self.cleanup_patrol_stack()
            self.reset_patrol_mode_state()
            self.publish_mode('ready')
            self.set_result(command, True, '巡逻、导航、感知与底盘已停止')
            return
        if command == 'start_3d_mapping':
            self.start_3d_mapping()
            return
        if command == 'stop_3d_mapping':
            self.stop_3d_mapping()
            return
        if command == 'reconstruct_latest_3d_map':
            self.reconstruct_3d_map('quality_safe', command, str(payload.get('session_id') or ''))
            return
        if command == 'reconstruct_fast_3d_map':
            self.reconstruct_3d_map('fast_check', command, str(payload.get('session_id') or ''))
            return
        if command == 'reconstruct_quality_3d_map':
            self.reconstruct_3d_map('quality_plus', command, str(payload.get('session_id') or ''))
            return
        if command in ('list_3d_assets', 'rename_3d_asset', 'delete_3d_asset', 'set_latest_3d_capture', 'set_latest_3d_reconstruct'):
            self.handle_3d_asset_command(command, payload)
            return
        if command in {'enqueue_scene_upload', 'retry_scene_upload'}:
            session_id = str(payload.get('session_id') or '').strip()
            task_id = str(payload.get('task_id') or '').strip()
            if command == 'enqueue_scene_upload' and not session_id:
                self.set_result(command, False, '缺少三维重建 session_id')
                return
            if command == 'retry_scene_upload' and not task_id:
                self.set_result(command, False, '缺少三维上传 task_id')
                return
            message = String()
            message.data = json.dumps({
                'schemaVersion': '1.0',
                'action': 'enqueue' if command == 'enqueue_scene_upload' else 'retry',
                'sessionId': session_id,
                'taskId': task_id,
            }, ensure_ascii=False)
            self.scene_upload_command_pub.publish(message)
            self.set_result(command, True, '三维上传命令已发送')
            return
        if command == 'export_3d_map':
            self.export_3d_map()
            return
        if command.startswith('start_'):
            name = command[len('start_'):]
            if name in self.processes:
                was_running = self.processes[name].is_running()
                success = self.start_process(name)
                if name == 'bringup' and success:
                    success = self.wait_for_core_sensors(
                        self.patrol_timeout('patrol_bringup_timeout_sec', 25.0))
                    if not success and not was_running:
                        self.stop_process(name, report=False)
                if name == 'mapping':
                    self.publish_mode('mapping')
                if name in AGENT_COMPONENTS and correlation['operation_id']:
                    self.publish_component_operation_feedback(
                        correlation,
                        bool(success),
                        'already_running' if was_running and success else 'started' if success else 'failed',
                    )
                return
        if command.startswith('stop_'):
            name = command[len('stop_'):]
            if name in self.processes:
                was_running = self.processes[name].is_running()
                self.stop_process(name)
                success = not self.processes[name].is_running()
                if name == 'mapping':
                    self.publish_mode('ready')
                if name in AGENT_COMPONENTS and correlation['operation_id']:
                    self.publish_component_operation_feedback(
                        correlation,
                        success,
                        'already_stopped' if not was_running and success else 'stopped' if success else 'failed',
                    )
                return
        if command == 'restart_navigation':
            self.stop_process('navigation')
            self.start_navigation_process()
            return
        if command == 'restart_perception':
            self.stop_process('perception')
            self.start_process('perception')
            return
        if command == 'save_map':
            self.save_map(str(payload.get('map_name') or '').strip())
            return
        if command == 'emergency_stop':
            self.emergency_stop()
            return
        if command == 'start_robot_stack':
            self.start_robot_stack()
            return
        if command == 'stop_robot_stack':
            self.stop_robot_stack()
            return
        if command == 'return_ready':
            self.publish_mode('ready')
            self.set_result(command, True, '已返回准备状态')
            return
        self.set_result(command, False, f'Unknown system command: {command}')

    def start_process(self, name: str) -> bool:
        if name == 'llm' and self.embedded_task_layer:
            self.set_result(
                'start_llm',
                True,
                self.voice_summary('AI task layer is embedded in inspection launch'),
            )
            return True
        if name == 'navigation':
            return self.start_navigation_process()
        return self.start_process_raw(name)

    def start_process_raw(self, name: str) -> bool:
        proc = self.processes[name]
        with self.lock:
            if proc.is_running():
                self.set_result_locked(f'start_{name}', True, f'{START_PROCESS_MESSAGES.get(name, name)}，已在运行')
                return True
        if proc.pgid is not None and self.process_group_alive(proc.pgid):
            success, message = self.terminate_process_group(proc)
            if not success:
                self.set_result(f'start_{name}', False, message)
                return False
        with self.lock:
            cmd = self.wrap_command(proc.command)
            proc.last_command = proc.command
            try:
                proc.process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=self.workspace_dir,
                    preexec_fn=os.setsid,
                )
                proc.pgid = proc.process.pid
            except Exception as exc:
                proc.last_error = f'failed to start: {exc}'
                proc.last_message = proc.last_error
                self.set_result_locked(f'start_{name}', False, f'{name} 启动失败: {exc}')
                return False
            proc.last_exit_code = None
            proc.last_error = ''
            proc.last_started_at = time.time()
            proc.last_message = f'started pid={proc.process.pid}'
        deadline = time.monotonic() + (2.5 if name == 'patrol_executor' else 0.75)
        while time.monotonic() < deadline:
            time.sleep(0.1)
            with self.lock:
                exit_code = proc.poll_exit_code()
                if exit_code is None:
                    continue
                if name == 'patrol_executor':
                    mode = 'platform' if getattr(self, 'platform_context', {}) else 'local'
                    proc.last_error = (
                        f'patrol_executor {mode} mode launch 参数构建失败，'
                        f'exit code={exit_code}'
                    )
                else:
                    proc.last_error = f'{name} process exited immediately, exit code={exit_code}'
                proc.last_message = proc.last_error
                self.set_result_locked(f'start_{name}', False, proc.last_error)
                return False
        with self.lock:
            self.set_result_locked(f'start_{name}', True, f'{name} 启动命令已发送')
        return True

    def start_navigation_process(self) -> bool:
        if not getattr(self, 'patrol_navigation_assets_prepared', False) and not self.prepare_patrol_navigation_assets():
            self.set_result('start_navigation', False, '导航资源准备失败: ' + self.patrol_error)
            return False
        return self.start_process_raw('navigation')

    def stop_process(
        self,
        name: str,
        *,
        ros_cleanup: bool = True,
        report: bool = True,
    ) -> None:
        if name in ('bringup', 'navigation', 'patrol_executor'):
            self.cancel_patrol_start()
        if name == 'llm' and self.embedded_task_layer:
            if report and self.ros_context_valid():
                self.set_result(
                    'stop_llm',
                    True,
                    'AI task layer is embedded in inspection launch; keep it running with UI and voice.',
                )
            return
        proc = self.processes[name]
        group_alive = proc.pgid is not None and self.process_group_alive(proc.pgid)
        if ros_cleanup and group_alive and self.ros_context_valid():
            self.cleanup_process_ros_interfaces(name)
        success, message = self.terminate_process_group(proc)
        if report and self.ros_context_valid():
            self.set_result(f'stop_{name}', success, message)

    def cleanup_process_ros_interfaces(self, name: str) -> None:
        if name == 'navigation':
            self.shutdown_started_lifecycles()
        if name == 'bringup':
            self.stop_lidar_motor()

    def shutdown_started_lifecycles(self) -> None:
        if getattr(self, 'keepout_lifecycle_started', False):
            keepout_unconfigured = all(
                self.lifecycle_node_state(name) == State.PRIMARY_STATE_UNCONFIGURED
                for name in KEEPOUT_LIFECYCLE_NODES
            )
            if not keepout_unconfigured:
                self.manage_lifecycle_nodes(
                    LIFECYCLE_MANAGER_KEEPOUT,
                    ManageLifecycleNodes.Request.SHUTDOWN,
                    timeout_sec=3.0,
                    required=False,
                )
            self.keepout_lifecycle_started = False
        if getattr(self, 'navigation_lifecycle_started', False):
            nav_unconfigured = all(
                self.lifecycle_node_state(name) == State.PRIMARY_STATE_UNCONFIGURED
                for name in NAVIGATION_LIFECYCLE_NODES
            )
            if not nav_unconfigured:
                self.manage_lifecycle_nodes(
                    LIFECYCLE_MANAGER_NAVIGATION,
                    ManageLifecycleNodes.Request.SHUTDOWN,
                    timeout_sec=3.0,
                    required=False,
                )
            self.navigation_lifecycle_started = False
        if getattr(self, 'localization_lifecycle_started', False):
            loc_unconfigured = all(
                self.lifecycle_node_state(name) == State.PRIMARY_STATE_UNCONFIGURED
                for name in LOCALIZATION_LIFECYCLE_NODES
            )
            if not loc_unconfigured:
                self.manage_lifecycle_nodes(
                    LIFECYCLE_MANAGER_LOCALIZATION,
                    ManageLifecycleNodes.Request.SHUTDOWN,
                    timeout_sec=3.0,
                    required=False,
                )
            self.localization_lifecycle_started = False

    def terminate_process_group(self, proc: ManagedProcess) -> tuple[bool, str]:
        with self.lock:
            pgid = proc.pgid
            process = proc.process
        if process is not None:
            process.poll()
        if pgid is None or not self.process_group_alive(pgid):
            with self.lock:
                proc.process = None
                proc.pgid = None
                proc.last_message = 'stopped'
            return True, f'{proc.name} already stopped'

        try:
            for sig, timeout_sec in (
                (signal.SIGINT, 6.0),
                (signal.SIGTERM, 2.0),
                (signal.SIGKILL, 2.0),
            ):
                os.killpg(pgid, sig)
                deadline = time.monotonic() + timeout_sec
                while self.process_group_alive(pgid) and time.monotonic() < deadline:
                    if process is not None:
                        process.poll()
                    time.sleep(0.05)
                if not self.process_group_alive(pgid):
                    break
            if self.process_group_alive(pgid):
                raise RuntimeError(f'process group {pgid} survived SIGKILL')
            if process is not None:
                process.poll()
            with self.lock:
                proc.process = None
                proc.pgid = None
                proc.last_message = 'stopped'
            return True, f'{proc.name} stopped'
        except ProcessLookupError:
            with self.lock:
                proc.process = None
                proc.pgid = None
                proc.last_message = 'stopped'
            return True, f'{proc.name} stopped'
        except Exception as exc:
            return False, f'Failed to stop {proc.name}: {exc}'

    @staticmethod
    def process_group_alive(pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def cleanup_patrol_stack(self, *, ros_cleanup: bool = True) -> None:
        for name in PATROL_SHUTDOWN_ORDER:
            if name in self.processes:
                self.stop_process(name, ros_cleanup=ros_cleanup, report=False)

    def ros_context_valid(self) -> bool:
        try:
            return rclpy.ok(context=self.context)
        except Exception:
            return False

    def stop_lidar_motor(self) -> bool:
        try:
            client = self.create_client(Empty, '/stop_motor')
            if not client.wait_for_service(timeout_sec=0.5):
                self.log_info('LiDAR stop_motor service unavailable; stopping bringup process anyway')
                return False
            future = client.call_async(Empty.Request())
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if future.done():
                    return True
                time.sleep(0.02)
            self.log_info('LiDAR stop_motor service call timed out; stopping bringup process anyway')
            return False
        except Exception as exc:
            self.log_info(f'LiDAR stop_motor service call failed: {exc}')
            return False

    def publish_zero_velocity(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def start_robot_stack(self) -> None:
        for name in ('bringup', 'zed', 'perception'):
            if not self.start_process(name):
                self.set_result('start_robot_stack', False, f'{name} 启动失败')
                return
            time.sleep(0.3)
        if not self.start_navigation_process():
            self.set_result('start_robot_stack', False, '导航启动失败: ' + self.patrol_error)
            return
        if not self.start_process('llm'):
            self.set_result('start_robot_stack', False, 'llm 启动失败')
            return
        self.set_result('start_robot_stack', True, '巡检节点启动命令已发送')

    def stop_robot_stack(self) -> None:
        self.cancel_patrol_start()
        self.publish_patrol_command('cancel')
        self.publish_zero_velocity()
        self.cleanup_patrol_stack()
        self.reset_patrol_mode_state()
        self.publish_mode('ready')
        self.set_result('stop_robot_stack', True, '巡检运动、导航和感知节点已停止，AI/UI 保持运行')

    def reset_patrol_mode_state(self) -> None:
        self.patrol_mode_state = 'idle'
        self.startup_step = ''
        self.patrol_error = ''
        self.patrol_warning = ''

    def reload_patrol_route(self) -> None:
        if self.patrol_mode_state in ('running', 'paused', 'returning_home') or self.startup_step == 'returning_home':
            self.set_result('reload_patrol_route', False, '巡逻运行中，请先取消巡逻。')
            return
        navigation_running = bool(self.processes.get('navigation') and self.processes['navigation'].is_running())
        if not self.prepare_patrol_navigation_assets():
            self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
            return
        if navigation_running:
            self.stop_process('navigation')
            self.startup_generation = getattr(self, 'startup_generation', 0) + 1
            generation = self.startup_generation
            self.startup_id = f'patrol_reload_{int(time.time() * 1000)}_{generation}'
            self.startup_started_at = time.time()
            self.patrol_start_cancel_event = threading.Event()
            self.initial_pose_service_ok = False
            self.initial_pose_confirmed_at = 0.0
            self.localization_lifecycle_started = False
            self.navigation_lifecycle_started = False
            self.keepout_lifecycle_started = False
            if not self.start_process_raw('navigation'):
                self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
                return
            if not self.wait_for_lifecycle_manager_services(15.0):
                self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
                return
            if not self.wait_for_nav2_components_loaded(15.0):
                self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
                return
            if not self.manage_lifecycle_nodes(
                LIFECYCLE_MANAGER_LOCALIZATION,
                ManageLifecycleNodes.Request.STARTUP,
                timeout_sec=10.0,
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: localization lifecycle 启动失败')
                return
            self.localization_lifecycle_started = True
            if not self.wait_for_lifecycle_nodes_active(
                LOCALIZATION_LIFECYCLE_NODES,
                self.patrol_timeout('patrol_localization_timeout_sec', 25.0),
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: AMCL 尚未 active')
                return
            if not self.wait_for_navigation_ready(self.patrol_timeout('patrol_navigation_timeout_sec', 35.0)):
                self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
                return
            if self.active_patrol_navigation_mode == NAVIGATION_PROFILE_KEEPOUT:
                if not self.manage_lifecycle_nodes(
                    LIFECYCLE_MANAGER_KEEPOUT,
                    ManageLifecycleNodes.Request.STARTUP,
                    timeout_sec=12.0,
                ):
                    self.set_result('reload_patrol_route', False, '路线刷新失败: Keepout lifecycle 启动失败')
                    return
                self.keepout_lifecycle_started = True
                if not self.wait_for_lifecycle_nodes_active(
                    KEEPOUT_LIFECYCLE_NODES, 12.0
                ):
                    self.set_result('reload_patrol_route', False, '路线刷新失败: Keepout lifecycle 尚未 active')
                    return
            if not self.initialize_amcl_with_confirmation(generation):
                self.set_result('reload_patrol_route', False, '路线刷新失败: AMCL 初始定位失败')
                return
            if not self.wait_for_stable_map_to_odom(
                timeout_sec=10.0,
                stable_duration_sec=1.0,
                after_amcl=self.initial_pose_request_sent_at,
                generation=generation,
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: map->odom 未稳定')
                return
            if not self.manage_lifecycle_nodes(
                LIFECYCLE_MANAGER_NAVIGATION,
                ManageLifecycleNodes.Request.STARTUP,
                timeout_sec=12.0,
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: navigation lifecycle 启动失败')
                return
            self.navigation_lifecycle_started = True
            if not self.wait_for_lifecycle_nodes_active(
                NAVIGATION_LIFECYCLE_NODES,
                self.patrol_timeout('patrol_nav2_timeout_sec', 25.0),
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: Nav2 节点尚未全部 active')
                return
            if (
                self.active_patrol_navigation_mode == NAVIGATION_PROFILE_KEEPOUT
                and not self.wait_for_keepout_runtime_ready(12.0)
            ):
                self.set_result('reload_patrol_route', False, '路线刷新失败: ' + self.patrol_error)
                return
        self.publish_patrol_command('reload')
        self.set_result('reload_patrol_route', True, '巡逻路线已刷新')

    def start_patrol_mode(
        self,
        profile: str = 'navigation',
        route_id: str = '',
        target_id: str = '',
    ) -> None:
        if not getattr(self, 'platform_context', {}):
            self.patrol_route_request = getattr(self, 'local_patrol_route_request', '')
            self.default_navigation_map = getattr(
                self, 'local_default_navigation_map', getattr(self, 'default_navigation_map', ''))
        profile = profile if profile in ('navigation', 'inspection') else 'navigation'
        with getattr(self, 'lock', threading.Lock()):
            if getattr(self, 'patrol_start_active', False):
                self.set_result('start_patrol_mode', True, '巡逻启动正在进行')
                return
            self.patrol_start_active = True
            self.startup_generation = getattr(self, 'startup_generation', 0) + 1
            generation = self.startup_generation
            self.startup_id = f'patrol_start_{int(time.time() * 1000)}_{generation}'
            self.startup_started_at = time.time()
            self.patrol_start_cancel_event = threading.Event()
            self.last_patrol_status = {}
            self.last_initial_pose_event = {}
            self.last_patrol_command_ack = {}
            self.last_patrol_start_request_id = ''
            self.last_amcl_received_at = 0.0
            self.set_initial_pose_client = None
            self.initial_pose_request_sent_at = 0.0
            self.initial_pose_service_ok = False
            self.initial_pose_confirmed_at = 0.0
            self.localization_lifecycle_started = False
            self.navigation_lifecycle_started = False
            self.keepout_lifecycle_started = False
            self.patrol_transaction_kind = 'checkpoint' if target_id else 'route'
            self.patrol_transaction_owned_components = []
            self.patrol_transaction_preexisting_components = [
                name for name in PATROL_SHUTDOWN_ORDER
                if name in getattr(self, 'processes', {})
                and self.processes[name].is_running()
            ]
        try:
            self._start_patrol_transaction(profile, route_id, generation, target_id)
        finally:
            if self.is_current_patrol_start(generation):
                self.patrol_start_active = False

    def is_current_patrol_start(self, generation: int) -> bool:
        return (
            generation == getattr(self, 'startup_generation', 0)
            and not getattr(self, 'patrol_start_cancel_event', threading.Event()).is_set()
        )

    def cancel_patrol_start(self) -> None:
        event = getattr(self, 'patrol_start_cancel_event', None)
        if event is not None:
            event.set()
        self.startup_generation = getattr(self, 'startup_generation', 0) + 1
        self.patrol_start_active = False

    def _start_patrol_transaction(
        self,
        profile: str,
        route_id: str,
        generation: int,
        target_id: str = '',
    ) -> None:
        self.patrol_mode_state = 'starting'
        self.patrol_error = ''
        self.patrol_warning = ''
        self.patrol_navigation_assets_prepared = False
        route_request = getattr(self, 'patrol_route_request', '')
        if route_request:
            try:
                self.patrol_route_path = str(resolve_route_file_path(
                    route_request, self.route_directory,
                ))
                self.keepout_route_path = self.patrol_route_path
            except ValueError as exc:
                self.fail_patrol_start(str(exc), generation=generation)
                return
        if not self.prepare_patrol_navigation_assets():
            self.fail_patrol_start(self.patrol_error or '导航资源准备失败', generation=generation)
            return
        self.patrol_navigation_assets_prepared = True
        self.log_info(
            'patrol navigation '
            f'profile={self.patrol_navigation_profile} '
            f'mode={self.active_patrol_navigation_mode} '
            f'enabled hard_keepout count={self.active_hard_keepout_count} '
            f'launch={os.path.basename(self.navigation_launch_file())} '
            f'params={self.navigation_params_file_name()} '
            f'keepout required={self.keepout_required()}'
        )
        self.log_info('start_patrol_mode: 按手动流程启动巡逻')

        def gate(ok: bool, error: str) -> bool:
            if not self.is_current_patrol_start(generation):
                return False
            if ok:
                return True
            self.fail_patrol_start(self.patrol_error or error, generation)
            return False

        # 1. bringup
        self.startup_step = 'starting_bringup'
        if not self.start_process('bringup'):
            self.fail_patrol_start('底盘与雷达启动失败', generation=generation)
            return
        # 2. core sensors
        if not gate(self.wait_for_core_sensors(self.patrol_timeout('patrol_bringup_timeout_sec', 25.0)), '底盘与传感器未就绪'):
            return
        # 3. inspection peripherals
        if profile == 'inspection':
            self.startup_step = 'starting_inspection'
            for name in ('zed', 'perception'):
                if not self.start_process(name):
                    self.fail_patrol_start(f'{name} 启动失败', generation=generation)
                    return

        # 4. navigation bringup, autostart=false
        self.startup_step = 'starting_navigation'
        if not self.start_navigation_process():
            self.fail_patrol_start(self.patrol_error or '导航进程启动失败', generation=generation)
            return
        self.startup_step = 'navigation_process_spawned'
        # 5. Nav2 components loaded (lifecycle services discoverable)
        if not gate(self.wait_for_lifecycle_manager_services(15.0), 'Nav2 lifecycle manager 服务未就绪'):
            return
        if not gate(self.wait_for_nav2_components_loaded(15.0), 'Nav2 核心组件未加载'):
            return
        # 6. STARTUP localization lifecycle
        if not gate(
            self.manage_lifecycle_nodes(
                LIFECYCLE_MANAGER_LOCALIZATION,
                ManageLifecycleNodes.Request.STARTUP,
                timeout_sec=10.0,
            ),
            'localization lifecycle 启动失败',
        ):
            return
        self.localization_lifecycle_started = True
        # 7. localization nodes active (map_server + AMCL)
        if not gate(
            self.wait_for_lifecycle_nodes_active(
                LOCALIZATION_LIFECYCLE_NODES,
                self.patrol_timeout('patrol_localization_timeout_sec', 25.0),
            ),
            'AMCL 尚未 active',
        ):
            return
        # 8. /map publisher and /initialpose subscriber
        if not gate(
            self.wait_for_navigation_ready(
                self.patrol_timeout('patrol_navigation_timeout_sec', 35.0)
            ),
            '地图或 initialpose 订阅尚未就绪',
        ):
            return
        # 9. Keepout lifecycle is only part of the keepout profile.
        if self.active_patrol_navigation_mode == NAVIGATION_PROFILE_KEEPOUT:
            if not gate(
                self.manage_lifecycle_nodes(
                    LIFECYCLE_MANAGER_KEEPOUT,
                    ManageLifecycleNodes.Request.STARTUP,
                    timeout_sec=12.0,
                ),
                'Keepout lifecycle 启动失败',
            ):
                return
            self.keepout_lifecycle_started = True
            if not gate(
                self.wait_for_lifecycle_nodes_active(KEEPOUT_LIFECYCLE_NODES, 12.0),
                'Keepout lifecycle 尚未 active',
            ):
                return
        # 10. AMCL initial pose via /set_initial_pose service
        if not gate(
            self.initialize_amcl_with_confirmation(generation),
            'AMCL 初始定位失败',
        ):
            return
        # 11. stable map->odom
        if not gate(
            self.wait_for_stable_map_to_odom(
                timeout_sec=10.0,
                stable_duration_sec=1.0,
                after_amcl=self.initial_pose_request_sent_at,
                generation=generation,
            ),
            'map->odom 未稳定',
        ):
            return
        # 12. STARTUP navigation lifecycle (after localization+initial pose)
        if not gate(
            self.manage_lifecycle_nodes(
                LIFECYCLE_MANAGER_NAVIGATION,
                ManageLifecycleNodes.Request.STARTUP,
                timeout_sec=12.0,
            ),
            'navigation lifecycle 启动失败',
        ):
            return
        self.navigation_lifecycle_started = True
        # 13. navigation nodes active
        if not gate(
            self.wait_for_lifecycle_nodes_active(
                NAVIGATION_LIFECYCLE_NODES,
                self.patrol_timeout('patrol_nav2_timeout_sec', 25.0),
            ),
            'Nav2 节点尚未全部 active',
        ):
            return
        # 14. Keepout runtime data is only required by the keepout profile.
        if self.active_patrol_navigation_mode == NAVIGATION_PROFILE_KEEPOUT:
            if not gate(
                self.wait_for_keepout_runtime_ready(12.0),
                'global/local costmap 尚未收到 keepout info 和 mask',
            ):
                return
        self.startup_step = 'navigation_ready'
        # 15. patrol executor (no auto initial pose)
        self.startup_step = 'starting_executor'
        started_at = time.time()
        executor = getattr(self, 'processes', {}).get('patrol_executor')
        if executor is not None:
            try:
                executor.command = self.build_patrol_executor_command()
            except (TypeError, ValueError) as exc:
                mode = 'platform' if getattr(self, 'platform_context', {}) else 'local'
                self.fail_patrol_start(
                    f'patrol_executor {mode} mode launch 参数构建失败: {exc}',
                    generation=generation,
                )
                return
        if not self.start_process('patrol_executor'):
            self.fail_patrol_start(
                getattr(executor, 'last_error', '') or '巡逻执行器启动失败',
                generation=generation,
            )
            return
        # 16. executor /patrol/status and /patrol/command
        self.startup_step = 'executor_process_spawned'
        heartbeat_ok = self.wait_for_patrol_status_heartbeat(started_at, self.patrol_timeout('patrol_initial_pose_timeout_sec', 8.0))
        subscriber_ok = self.wait_for_patrol_command_subscriber(self.patrol_timeout('patrol_initial_pose_timeout_sec', 8.0))
        executor_ok = self.wait_for_patrol_executor_ready(self.patrol_timeout('patrol_executor_timeout_sec', 15.0))
        self.log_patrol_start_readiness()
        gate_warnings = []
        if not executor_ok:
            gate_warnings.append('巡逻执行器未就绪')
        if not heartbeat_ok:
            gate_warnings.append('未确认 /patrol/status heartbeat')
        if not subscriber_ok:
            gate_warnings.append('未确认 /patrol/command 订阅者')
        if gate_warnings:
            self.fail_patrol_start('；'.join(gate_warnings), generation=generation)
            return
        # 17. send patrol start
        self.startup_step = 'executor_ready'
        command = 'go_to_target' if target_id else 'start'
        request_id = str(
            getattr(self, 'platform_context', {}).get('active_request_id')
            or f"{self.startup_id}_command"
        )
        if target_id:
            self.publish_patrol_command(command, request_id=request_id, target_id=target_id)
        elif route_id:
            self.publish_patrol_command(command, request_id=request_id, route_id=route_id)
        else:
            self.publish_patrol_command(command, request_id=request_id)
        self.startup_step = 'patrol_command_sent'
        if not self.wait_for_patrol_command_ack(request_id, self.patrol_timeout('patrol_command_timeout_sec', 8.0), generation):
            if self.is_current_patrol_start(generation):
                self.publish_patrol_command(
                    command,
                    request_id=request_id,
                    route_id=route_id,
                    target_id=target_id,
                )
            if not self.wait_for_patrol_command_ack(request_id, self.patrol_timeout('patrol_command_timeout_sec', 8.0), generation):
                self.fail_patrol_start(self.patrol_error or '巡逻执行器未确认启动命令', generation=generation)
                return
        if self.patrol_mode_state == 'starting':
            self.patrol_mode_state = 'command_sent'
            self.startup_step = 'patrol_command_sent'
        self.patrol_transaction_owned_components = [
            name for name in PATROL_SHUTDOWN_ORDER
            if name not in getattr(self, 'patrol_transaction_preexisting_components', [])
            and name in getattr(self, 'processes', {})
            and self.processes[name].is_running()
        ]
        self.set_result(
            'go_to_checkpoint' if target_id else 'start_patrol_mode',
            True,
            '检查点导航命令已发送' if target_id else '巡逻启动命令已发送',
        )

    def load_patrol_initial_pose(self) -> PoseWithCovarianceStamped:
        route = load_route_file(self.patrol_route_path)
        start_pose = route.get('start_pose') or {}
        pose = start_pose.get('pose') or {}
        covariance = start_pose.get('covariance') or {}
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = Time().to_msg()
        message.pose.pose.position.x = float(pose.get('x') or 0.0)
        message.pose.pose.position.y = float(pose.get('y') or 0.0)
        yaw = float(pose.get('yaw') or 0.0)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = float(covariance.get('x') or 0.0)
        message.pose.covariance[7] = float(covariance.get('y') or 0.0)
        message.pose.covariance[35] = float(covariance.get('yaw') or 0.0)
        return message

    def request_amcl_initial_pose(self, generation: int, timeout_sec: float = 4.0) -> bool:
        if not self.is_current_patrol_start(generation):
            return False
        if not all(
            self.lifecycle_node_is_active(name) for name in LOCALIZATION_LIFECYCLE_NODES
        ):
            self.patrol_error = 'AMCL lifecycle 不在 active 状态'
            return False
        client = self.set_initial_pose_client
        if client is None:
            client = self.create_client(SetInitialPose, '/set_initial_pose')
            self.set_initial_pose_client = client
        if not client.wait_for_service(timeout_sec=2.0):
            self.patrol_error = '/set_initial_pose 服务不可用'
            return False
        request = SetInitialPose.Request()
        request.pose = self.load_patrol_initial_pose()
        pose_msg = request.pose.pose.pose
        yaw = 2.0 * math.atan2(pose_msg.orientation.z, pose_msg.orientation.w)
        self.log_info(
            'AMCL initial pose request: '
            f'x={pose_msg.position.x:.3f} y={pose_msg.position.y:.3f} '
            f'yaw={yaw:.3f} attempt=1'
        )
        self.initial_pose_request_sent_at = time.time()
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self.is_current_patrol_start(generation):
                return False
            if future.done():
                try:
                    future.result()
                except Exception as exc:
                    self.patrol_error = f'AMCL 初始位姿服务调用失败: {exc}'
                    return False
                self.initial_pose_service_ok = True
                self.log_info('AMCL set_initial_pose service completed')
                return True
            time.sleep(0.02)
        self.patrol_error = 'AMCL 初始位姿服务调用超时'
        return False

    def initialize_amcl_with_confirmation(self, generation: int) -> bool:
        for attempt in range(1, 3):
            if not self.is_current_patrol_start(generation):
                return False
            self.log_info(f'AMCL 初始定位 attempt={attempt}')
            sent_at = time.time()
            if not self.request_amcl_initial_pose(generation):
                return False
            if self.wait_for_fresh_amcl(timeout_sec=4.0, after=sent_at, generation=generation):
                self.initial_pose_confirmed_at = getattr(self, 'last_amcl_received_at', 0.0)
                self.log_info('fresh /amcl_pose confirmed')
                return True
            self.log_info(
                'AMCL 未在本次初始位姿请求后发布 /amcl_pose，准备重试 '
                f'attempt={attempt + 1}'
            )
        self.patrol_error = (
            'AMCL 已接收初始位姿服务请求，但未发布新的 /amcl_pose'
        )
        return False

    def fail_patrol_start(self, error: str, generation: Optional[int] = None) -> None:
        if generation is not None and generation != getattr(self, 'startup_generation', generation):
            return
        self.patrol_mode_state = 'failed'
        self.startup_step = 'patrol_failed'
        self.patrol_error = error
        self.cancel_patrol_start()
        if getattr(self, 'patrol_transaction_kind', '') == 'checkpoint':
            self.cleanup_checkpoint_transaction()
        else:
            self.cleanup_patrol_stack()
        self.patrol_mode_state = 'failed'
        self.startup_step = 'patrol_failed'
        self.patrol_error = error
        context = getattr(self, 'platform_context', {})
        if context.get('active_command_id'):
            self.command_result = {
                'event': 'command_failed',
                'command_id': context.get('active_command_id', ''),
                'request_id': context.get('active_request_id', ''),
                'execution_id': context.get('active_execution_id', ''),
                'deployment_id': context.get('active_deployment_id', ''),
                'error_code': 'PATROL_START_FAILED',
                'error_message': error,
            }
        correlation = dict(getattr(self, 'agent_operation_context', {}))
        self.last_agent_operation_feedback = (
            {
                **correlation,
                'state': 'failed',
                'status': 'failed',
                'message': error,
            }
            if correlation.get('operation_id') else {}
        )
        self.set_result(
            'go_to_checkpoint'
            if getattr(self, 'patrol_transaction_kind', '') == 'checkpoint'
            else 'start_patrol_mode',
            False,
            '巡逻启动失败: ' + error,
        )
        self.last_agent_operation_feedback = {}

    def publish_patrol_command(
        self,
        command: str,
        request_id: str = '',
        route_id: str = '',
        command_id: str = '',
        target_id: str = '',
    ) -> None:
        if not request_id:
            request_id = (
                f"patrol_start_{int(time.time() * 1000)}"
                if command == 'start'
                else f"patrol_{command}_{int(time.time() * 1000)}"
            )
        if command == 'start':
            self.last_patrol_start_request_id = request_id
        msg = String()
        payload = {
            'schema_version': '1.0',
            'command': command,
            'source': 'system_supervisor',
            'timestamp': time.time(),
            'request_id': request_id,
            **getattr(self, 'agent_operation_context', {}),
        }
        if route_id:
            payload['route_id'] = route_id
        if target_id:
            payload['target_id'] = target_id
        if command == 'start' and not command_id:
            command_id = str(getattr(self, 'platform_context', {}).get('active_command_id') or '')
        if command_id:
            payload['command_id'] = command_id
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.patrol_command_pub.publish(msg)

    def cleanup_checkpoint_transaction(self) -> None:
        owned = list(getattr(self, 'patrol_transaction_owned_components', []))
        if not owned:
            preexisting = set(getattr(self, 'patrol_transaction_preexisting_components', []))
            owned = [
                name for name in PATROL_SHUTDOWN_ORDER
                if name not in preexisting
                and name in getattr(self, 'processes', {})
                and self.processes[name].is_running()
            ]
        for name in owned:
            if name in getattr(self, 'processes', {}):
                self.stop_process(name, report=False)
        self.patrol_transaction_owned_components = []

    def active_hard_keepout_zones(self, route: Dict[str, Any]) -> List[Dict[str, Any]]:
        return [
            zone
            for zone in route.get('keepout_zones', [])
            if zone.get('enabled') is True and zone.get('type') == 'hard_keepout'
        ]

    def resolve_patrol_navigation_mode(self, route: Dict[str, Any]) -> str:
        requested = str(
            getattr(self, 'patrol_navigation_profile', NAVIGATION_PROFILE_AUTO)
            or NAVIGATION_PROFILE_AUTO
        )
        zones = self.active_hard_keepout_zones(route)
        if requested == NAVIGATION_PROFILE_AUTO:
            return NAVIGATION_PROFILE_KEEPOUT if zones else NAVIGATION_PROFILE_NORMAL
        if requested == NAVIGATION_PROFILE_NORMAL:
            if zones:
                raise ValueError('normal profile cannot ignore enabled hard_keepout zones')
            return NAVIGATION_PROFILE_NORMAL
        if requested == NAVIGATION_PROFILE_KEEPOUT:
            if not getattr(self, 'enable_keepout_navigation', False):
                raise ValueError('keepout profile requested but keepout capability is disabled')
            return NAVIGATION_PROFILE_KEEPOUT
        raise ValueError(f'invalid patrol_navigation_profile: {requested}')

    def keepout_required(self, mode: Optional[str] = None) -> bool:
        return (mode or getattr(self, 'active_patrol_navigation_mode', '')) == NAVIGATION_PROFILE_KEEPOUT

    def navigation_launch_file(self, mode: Optional[str] = None) -> str:
        return (
            'navigation_keepout.launch.py'
            if self.keepout_required(mode)
            else 'navigation.launch.py'
        )

    def navigation_params_file_name(self, mode: Optional[str] = None) -> str:
        return (
            'nav2_params_keepout.yaml'
            if self.keepout_required(mode)
            else 'nav2_params.yaml'
        )

    def navigation_launch_command(self, mode: Optional[str] = None) -> str:
        mode = mode or getattr(self, 'active_patrol_navigation_mode', NAVIGATION_PROFILE_NORMAL)
        default_map = self.default_navigation_map
        if mode == NAVIGATION_PROFILE_KEEPOUT:
            return (
                'ros2 launch ylhb_base navigation_keepout.launch.py '
                f'map:={default_map} '
                f'params_file:={self.workspace_dir}/src/ylhb_base/config/nav2_params_keepout.yaml '
                f'keepout_global_mask:={self.keepout_global_mask_path} '
                f'keepout_local_mask:={self.keepout_local_mask_path} '
                'autostart:=false'
            )
        return (
            'ros2 launch ylhb_base navigation.launch.py '
            f'map:={default_map} '
            f'params_file:={self.workspace_dir}/src/ylhb_base/config/nav2_params.yaml '
            'autostart:=false'
        )

    def build_patrol_executor_command(self) -> str:
        context = dict(getattr(self, 'platform_context', {}) or {})
        route_file_path = str(getattr(self, 'patrol_route_request', '') or '').strip()
        route_directory = str(getattr(self, 'route_directory', '') or '').strip()
        startup_id = str(getattr(self, 'startup_id', '') or 'pending').strip()
        if not route_file_path or not route_directory:
            raise ValueError('route_file_path 和 route_directory 不能为空')

        parts = [
            'ros2',
            'launch',
            'ylhb_mobile_bridge',
            'patrol_executor.launch.py',
            f'route_file_path:={route_file_path}',
            f'route_directory:={route_directory}',
            'auto_start:=false',
            'publish_initial_pose_on_startup:=false',
            f'startup_id:={startup_id}',
        ]
        optional = {
            'execution_id': context.get('active_execution_id'),
            'deployment_id': context.get('active_deployment_id'),
            'platform_request_id': context.get('active_request_id'),
            'platform_command_id': context.get('active_command_id'),
        }
        for name, raw_value in optional.items():
            value = str(raw_value or '').strip()
            if value:
                parts.append(f'{name}:={value}')
        return ' '.join(shlex.quote(part) for part in parts)

    def prepare_keepout_navigation(self) -> bool:
        if not hasattr(self, 'default_navigation_map'):
            return True
        return self.prepare_patrol_navigation_assets()

    def prepare_patrol_navigation_assets(self) -> bool:
        try:
            if not os.path.isfile(self.default_navigation_map):
                raise ValueError(f'map yaml missing: {self.default_navigation_map}')
            if not os.path.isfile(self.patrol_route_path):
                raise ValueError(f'patrol route missing: {self.patrol_route_path}')
            route = load_route_file(self.patrol_route_path)
            validate_route_map_binding(route, self.default_navigation_map)
            mode = self.resolve_patrol_navigation_mode(route)
        except Exception as exc:
            self.patrol_error = str(exc)
            return False
        self.active_patrol_navigation_mode = mode
        self.active_hard_keepout_count = len(self.active_hard_keepout_zones(route))
        if getattr(self, 'processes', None) and 'navigation' in self.processes:
            self.processes['navigation'].command = self.navigation_launch_command(mode)
        safety = self.run_route_safety_check(mode)
        if safety == 'unsafe':
            self.patrol_error = '巡逻路线安全校验失败'
            return False
        if safety == 'error':
            return False
        if mode == NAVIGATION_PROFILE_NORMAL:
            self.patrol_error = ''
            return True
        if self.check_keepout_setup():
            if not self._log_keepout_zone_status():
                return False
            self.patrol_error = ''
            return True
        error = self.patrol_error
        config_error_keywords = ('keepout_filter', 'keepout plugin', 'filter info topic')
        if any(kw in error for kw in config_error_keywords):
            return False
        self.startup_step = 'generating_keepout_mask'
        if not self.generate_keepout_mask() or not all(
            os.path.exists(path)
            for path in (
                self.keepout_global_mask_path,
                self.keepout_local_mask_path,
            )
        ):
            self.patrol_error = self.patrol_error or 'global/local keepout mask missing'
            return False
        if not self.check_keepout_setup():
            return False
        if not self._log_keepout_zone_status():
            return False
        self.patrol_error = ''
        return True

    def _log_keepout_zone_status(self) -> bool:
        metadata_path = os.path.join(
            os.path.dirname(self.keepout_global_mask_path), 'keepout_masks.metadata.json'
        )
        try:
            metadata = json.loads(Path(metadata_path).read_text(encoding='utf-8'))
            zones = metadata.get('zones') or {}
            global_mask = metadata['global_mask']
            local_mask = metadata['local_mask']
            hard_padding = ','.join(
                f"{zone['hard_padding_m']:.3f}" for zone in zones.values()
            ) or 'n/a'
        except (KeyError, OSError, TypeError, ValueError) as exc:
            self.patrol_error = f'keepout mask metadata invalid: {exc}'
            return False
        if zones and (
            global_mask.get('weighted_cells', 0) != 0
            or local_mask.get('weighted_cells', 0) != 0
        ):
            self.patrol_error = 'keepout masks must not contain weighted cells'
            return False
        self.log_info(
            'keepout masks ready: '
            f'zones={len(zones)} hard_padding={hard_padding}m '
            f"global_hard_cells={global_mask.get('hard_cells', 0)} "
            f"local_hard_cells={local_mask.get('hard_cells', 0)}"
        )
        return True

    def run_route_safety_check(self, mode: Optional[str] = None) -> str:
        nav2_params_name = self.navigation_params_file_name(mode)
        command = [
            'python3', os.path.join(self.workspace_dir, 'scripts', 'validate_route_safety.py'),
            '--map', self.default_navigation_map,
            '--route', self.patrol_route_path,
            '--nav2-params', os.path.join(self.workspace_dir, 'src', 'ylhb_base', 'config', nav2_params_name),
            '--report',
        ]
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            report = json.loads(result.stdout)
        except json.JSONDecodeError:
            report = {}
        if result.returncode == 1:
            self.patrol_error = '；'.join(report.get('failures') or [result.stdout.strip()])
            return 'unsafe'
        if result.returncode != 0:
            self.patrol_error = f'route safety validation failed: {result.stdout.strip()}'
            return 'error'
        if report.get('status') == 'warning':
            self.patrol_warning = 'warning: ' + '；'.join(report.get('warnings') or [])
            self.patrol_error = self.patrol_warning
            self.log_info(self.patrol_warning)
            return 'warning'
        self.patrol_warning = ''
        return 'ok'

    def generate_keepout_mask(self) -> bool:
        output_dir = os.path.dirname(self.keepout_global_mask_path)
        command = [
            'python3',
            os.path.join(self.workspace_dir, 'scripts', 'generate_keepout_mask.py'),
            '--map', self.default_navigation_map,
            '--route', self.patrol_route_path,
            '--nav2-params', os.path.join(
                self.workspace_dir, 'src', 'ylhb_base', 'config',
                'nav2_params_keepout.yaml',
            ),
            '--output-dir', output_dir,
        ]
        try:
            result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except Exception as exc:
            self.patrol_error = f'keepout mask generation failed: {exc}'
            return False
        if result.returncode != 0:
            self.patrol_error = f'keepout mask generation failed: {result.stdout.strip()}'
            return False
        self.patrol_error = ''
        return True

    def check_keepout_setup(self) -> bool:
        command = [
            'python3', os.path.join(self.workspace_dir, 'scripts', 'check_keepout_setup.py'),
            '--map', self.default_navigation_map,
            '--route', self.patrol_route_path,
            '--nav2-params', os.path.join(self.workspace_dir, 'src', 'ylhb_base', 'config', 'nav2_params_keepout.yaml'),
            '--output-dir', os.path.dirname(self.keepout_global_mask_path),
        ]
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode != 0:
            self.patrol_error = f'keepout setup failed: {result.stdout.strip()}'
            return False
        self.patrol_error = ''
        return True

    def publish_3d_mapping_command(self, command: str) -> None:
        msg = String()
        msg.data = json.dumps({
            'schema_version': '1.0',
            'command': command,
            'source': 'system_supervisor',
            'timestamp': time.time(),
            'request_id': f"3d_mapping_{command}_{int(time.time() * 1000)}",
        }, ensure_ascii=False)
        self.mapping3d_command_pub.publish(msg)

    def start_3d_mapping(self) -> None:
        blockers = [
            name for name in ('zed', 'perception')
            if self.processes.get(name) and self.processes[name].is_running()
        ]
        if blockers:
            self.set_result(
                'start_3d_mapping',
                False,
                '请先停止 ZED wrapper/感知进程: ' + ', '.join(blockers),
            )
            return
        self.start_process('3d_capture')
        self.set_result('start_3d_mapping', True, '现场 SVO 采集已启动')

    def stop_3d_mapping(self) -> None:
        proc = self.processes.get('3d_capture')
        if not proc or not proc.is_running():
            self.set_result('stop_3d_mapping', True, '3d_capture already stopped')
            return
        self.publish_3d_mapping_command('stop')
        terminal = self.wait_for_mapping3d_terminal(8.0)
        if terminal == 'timeout' or terminal not in ('succeeded', 'failed', 'idle', 'stopped'):
            self.stop_process('3d_capture')
        latest = self.read_latest_json(self.mapping3d_output_dir)
        svo_file = str(latest.get('svo_file') or '')
        message = 'SVO 采集已停止'
        if svo_file:
            message = f'SVO 采集已停止，最新文件: {svo_file}'
        self.set_result('stop_3d_mapping', True, message)

    def reconstruct_3d_map(self, profile: str, command: str, session_id: str = '') -> None:
        latest = self.read_latest_json(self.mapping3d_output_dir) if not session_id else self.load_json_safe(
            os.path.join(self.mapping3d_output_dir, session_id, 'metadata.json')
        )
        if not latest.get('svo_file'):
            self.set_result(command, False, '请先完成一次现场采集')
            return
        proc = self.processes.get('3d_reconstruct')
        if proc and proc.is_running():
            self.set_result(command, True, '三维重建已在运行')
            return
        reconstruct_root = getattr(self, 'mapping3d_reconstruct_dir', workspace_path('runs', '3d_reconstruct'))
        input_arg = f'session:={session_id}' if session_id else 'input:=latest'
        self.processes['3d_reconstruct'] = ManagedProcess(
            '3d_reconstruct',
            f'ros2 run ylhb_3d_mapping zed_svo_reconstruct '
            f'{input_arg} capture_root:={self.mapping3d_output_dir} '
            f'output_root:={reconstruct_root} profile:={profile}',
        )
        self.start_process('3d_reconstruct')
        self.set_result(command, True, f'离线三维重建已启动: {profile}')

    def asset_root(self, asset_type: str) -> str:
        if asset_type in ('reconstruct', 'reconstructs', '3d_reconstruct'):
            return getattr(self, 'mapping3d_reconstruct_dir', workspace_path('runs', '3d_reconstruct'))
        return getattr(self, 'mapping3d_capture_dir', getattr(self, 'mapping3d_output_dir', workspace_path('runs', '3d_capture')))

    def handle_3d_asset_command(self, command: str, payload: Dict[str, Any]) -> None:
        asset_type = str(payload.get('asset_type') or 'capture')
        session_id = str(payload.get('session_id') or '').strip()
        try:
            if command == 'list_3d_assets':
                result = {
                    'captures': zed_3d_asset_manager.list_assets(self.asset_root('capture'), 'capture')[:10],
                    'reconstructs': zed_3d_asset_manager.list_assets(self.asset_root('reconstruct'), 'reconstruct')[:10],
                }
            elif command == 'rename_3d_asset':
                result = zed_3d_asset_manager.rename_asset(
                    self.asset_root(asset_type),
                    session_id,
                    str(payload.get('display_name') or session_id),
                )
            elif command == 'delete_3d_asset':
                upload = getattr(self, 'scene_uploads_by_session', {}).get(
                    session_id, {}) if asset_type == 'reconstruct' else {}
                if str(upload.get('status') or '') in {
                    'PENDING', 'UPLOADING', 'FAILED_RETRYABLE',
                    'CREDENTIAL_BLOCKED',
                }:
                    raise ValueError('该三维资产仍有活动上传任务，暂不能删除')
                result = zed_3d_asset_manager.delete_asset(self.asset_root(asset_type), session_id)
            elif command == 'set_latest_3d_capture':
                result = zed_3d_asset_manager.set_latest_asset(self.asset_root('capture'), session_id)
            elif command == 'set_latest_3d_reconstruct':
                result = zed_3d_asset_manager.set_latest_asset(self.asset_root('reconstruct'), session_id)
            else:
                result = {}
            self.set_result(command, True, json.dumps(result, ensure_ascii=False))
        except Exception as exc:
            self.set_result(command, False, str(exc))

    def export_3d_map(self) -> None:
        self.set_result(
            'export_3d_map',
            False,
            '最终模型请离线执行: ./scripts/run_on_jetson.sh zed_3d_reconstruct input:=<capture.svo2>',
        )

    def read_latest_json(self, root: str) -> Dict[str, Any]:
        return self.load_json_safe(os.path.join(os.path.expanduser(root), 'latest.json'))

    def load_json_safe(self, path: str) -> Dict[str, Any]:
        try:
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as handle:
                data = json.load(handle)
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def wait_for_mapping3d_terminal(self, timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        last_state = ''
        while time.monotonic() < deadline:
            status = getattr(self, 'latest_mapping3d_status', {}) or {}
            last_state = str(status.get('state') or '')
            if last_state in ('succeeded', 'failed', 'idle', 'stopped'):
                return last_state
            time.sleep(0.2)
        return last_state or 'timeout'

    def build_patrol_readiness(self) -> Dict[str, bool]:
        processes = getattr(self, 'processes', {})
        bringup = processes.get('bringup')
        navigation = processes.get('navigation')
        sensors = self.core_sensor_status()
        keepout_required = self.keepout_required()
        keepout_lifecycle_active = self.lifecycle_nodes_cached_active(
            KEEPOUT_LIFECYCLE_NODES
        )
        readiness = {
            'bringup': bool(bringup and bringup.is_running()),
            'navigation': bool(navigation and navigation.is_running()),
            'executor': self.is_patrol_executor_ready(),
            'route_file': self.has_patrol_route_file(),
            'odom': sensors['odom_publisher'] and sensors['odom_fresh'],
            'scan': sensors['scan_publisher'] and sensors['scan_fresh'],
            'tf': sensors['odom_to_base'] and sensors['base_to_laser'],
            'map': self.topic_has_publishers('/map'),
            'map_to_odom': self.has_map_to_odom(),
            'initialpose_subscribers': self.topic_has_subscribers('/initialpose'),

            'localization_active': self.lifecycle_nodes_cached_active(
                LOCALIZATION_LIFECYCLE_NODES
            ),
            'nav2_components_loaded': self.nav2_components_loaded(),
            'nav2_action_discovered': self.has_nav2_action(),
            'nav2_active': self.compute_nav2_active(),
            'keepout_required': keepout_required,
            'keepout_lifecycle_active': keepout_lifecycle_active,
            'keepout_ready': not keepout_required or (
                keepout_lifecycle_active and self.has_keepout_runtime_ready()
            ),

            'patrol_status': self.topic_has_publishers('/patrol/status'),
            'initial_pose_published': self.has_initial_pose_published(),
            'set_initial_pose_service': self.service_exists('/set_initial_pose'),
            'initial_pose_service_ok': bool(getattr(self, 'initial_pose_service_ok', False)),
            'amcl_pose_confirmed': getattr(self, 'initial_pose_confirmed_at', 0.0) > 0.0,
            'initial_pose': {
                'service_available': self.service_exists('/set_initial_pose'),
                'request_sent': getattr(self, 'initial_pose_request_sent_at', 0.0) > 0.0,
                'service_completed': bool(getattr(self, 'initial_pose_service_ok', False)),
                'amcl_pose_confirmed': getattr(self, 'initial_pose_confirmed_at', 0.0) > 0.0,
                'request_sent_at': getattr(self, 'initial_pose_request_sent_at', 0.0),
                'amcl_confirmed_at': getattr(self, 'initial_pose_confirmed_at', 0.0),
            },
        }
        return readiness

    def build_light_patrol_readiness(self) -> Dict[str, bool]:
        processes = getattr(self, 'processes', {})
        bringup = processes.get('bringup')
        navigation = processes.get('navigation')
        executor = processes.get('patrol_executor')
        keepout_required = self.keepout_required()
        return {
            'bringup': bool(bringup and bringup.is_running()),
            'navigation': bool(navigation and navigation.is_running()),
            'executor': bool(executor and executor.is_running()),
            'route_file': self.has_patrol_route_file(),
            'odom': False,
            'scan': False,
            'tf': False,
            'map': False,
            'map_to_odom': False,
            'initialpose_subscribers': False,
            'localization_active': False,
            'nav2_components_loaded': False,
            'nav2_action_discovered': False,
            'nav2_active': False,
            'keepout_required': keepout_required,
            'keepout_lifecycle_active': False,
            'keepout_ready': not keepout_required,
            'patrol_status': False,
            'initial_pose_published': False,
            'set_initial_pose_service': False,
            'initial_pose_service_ok': False,
            'amcl_pose_confirmed': False,
            'initial_pose': {
                'service_available': False,
                'request_sent': False,
                'service_completed': False,
                'amcl_pose_confirmed': False,
                'request_sent_at': 0.0,
                'amcl_confirmed_at': 0.0,
            },
        }

    def wait_for_patrol_readiness(self, timeout_sec: float = 25.0) -> bool:
        required_keys = (
            'bringup',
            'navigation',
            'executor',
            'route_file',
            'odom',
            'scan',
            'tf',
            'nav2_action_discovered',
            'patrol_status',
        )
        deadline = time.monotonic() + timeout_sec
        last_missing: List[str] = []
        while time.monotonic() < deadline:
            readiness = self.build_patrol_readiness()
            last_missing = [key for key in required_keys if not readiness.get(key)]
            if not last_missing:
                self.patrol_error = ''
                return True
            self.startup_step = f"waiting_{last_missing[0]}"
            self.patrol_error = '等待巡逻依赖: ' + ', '.join(last_missing)
            time.sleep(0.25)
        self.patrol_error = '巡逻依赖等待超时: ' + ', '.join(last_missing)
        return False

    def wait_for_core_sensors(self, timeout_sec: float = 25.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.process_exited_before_readiness('bringup'):
                return False
            sensors = self.core_sensor_status()
            if all((
                sensors['odom_publisher'],
                sensors['odom_fresh'],
                sensors['scan_publisher'],
                sensors['scan_fresh'],
                sensors['odom_to_base'],
                sensors['base_to_laser'],
            )):
                self.patrol_error = ''
                return True
            if not sensors['odom_publisher'] or not sensors['odom_fresh']:
                self.startup_step = 'waiting_odom'
            elif not sensors['scan_publisher'] or not sensors['scan_fresh']:
                self.startup_step = 'waiting_scan'
            else:
                self.startup_step = 'waiting_tf'
            self.patrol_error = self.core_sensor_wait_message(sensors)
            time.sleep(0.25)
        return False

    def wait_for_navigation_ready(self, timeout_sec: float = 35.0) -> bool:
        return self.wait_for_readiness_keys(
            ('navigation', 'map', 'initialpose_subscribers'),
            timeout_sec,
            error_prefix='导航等待超时',
        )

    def wait_for_nav2_action_ready(self, timeout_sec: float = 25.0) -> bool:
        return self.wait_for_readiness_keys(
            ('nav2_action_discovered',),
            timeout_sec,
            error_prefix='Nav2 动作服务未就绪',
        )

    def wait_for_map_to_odom(self, timeout_sec: float = 10.0) -> bool:
        return self.wait_for_readiness_keys(
            ('map_to_odom',),
            timeout_sec,
            error_prefix='等待 map->odom TF 超时',
        )

    def wait_for_stable_map_to_odom(
        self,
        timeout_sec: float = 10.0,
        stable_duration_sec: float = 1.0,
        after_amcl: float = 0.0,
        generation: Optional[int] = None,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if generation is not None and not self.is_current_patrol_start(generation):
                return False
            amcl_fresh = getattr(self, 'last_amcl_received_at', 0.0) >= after_amcl
            sensors_fresh = self.has_fresh_core_sensors()
            transform_ready = self.has_map_to_odom()
            laser_in_map_ready = self.has_transform('map', 'laser_link')
            stable_sec = self.map_to_odom_stable_sec()
            if (
                amcl_fresh
                and sensors_fresh
                and transform_ready
                and laser_in_map_ready
                and stable_sec >= stable_duration_sec
            ):
                self.patrol_error = ''
                return True
            self.startup_step = 'waiting_map_to_odom'
            if not amcl_fresh:
                self.patrol_error = 'fresh /amcl_pose 尚未收到'
            elif not sensors_fresh:
                self.patrol_error = '/odom、/scan 或基础 TF 已不新鲜'
            elif not transform_ready:
                self.patrol_error = 'map->odom 尚未建立'
            elif not laser_in_map_ready:
                self.patrol_error = 'map->laser_link 完整 TF 链尚未建立'
            else:
                self.patrol_error = (
                    f'map->odom 尚未连续稳定 {stable_duration_sec:.1f}s '
                    f'(当前 {stable_sec:.1f}s)'
                )
            time.sleep(0.1)
        return False

    def wait_for_lifecycle_manager_services(self, timeout_sec: float) -> bool:
        required = [
            LIFECYCLE_MANAGER_LOCALIZATION,
            LIFECYCLE_MANAGER_NAVIGATION,
        ]
        if self.active_patrol_navigation_mode == NAVIGATION_PROFILE_KEEPOUT:
            required.append(LIFECYCLE_MANAGER_KEEPOUT)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            missing = [name for name in required if not self.service_exists(name)]
            if not missing:
                self.patrol_error = ''
                return True
            self.startup_step = 'waiting_nav2'
            self.patrol_error = 'lifecycle manager 服务未就绪: ' + ', '.join(missing)
            time.sleep(0.2)
        return False

    def manage_lifecycle_nodes(
        self,
        service_name: str,
        command: int,
        timeout_sec: float,
        required: bool = True,
    ) -> bool:
        try:
            client = self.lifecycle_manager_clients.get(service_name)
            if client is None:
                client = self.create_client(ManageLifecycleNodes, service_name)
                self.lifecycle_manager_clients[service_name] = client
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if client.wait_for_service(timeout_sec=min(remaining, 1.0)):
                    break
                time.sleep(0.1)
            else:
                if required:
                    self.patrol_error = f'lifecycle manager 服务不可用: {service_name}'
                return False
            request = ManageLifecycleNodes.Request()
            request.command = command
            future = client.call_async(request)
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    if response is not None and response.success:
                        return True
                    if required:
                        self.patrol_error = f'lifecycle manager 命令失败: {service_name}'
                    return False
                time.sleep(0.02)
        except Exception as exc:
            if required:
                self.patrol_error = f'lifecycle manager 调用失败 {service_name}: {exc}'
            return False
        if required:
            self.patrol_error = f'lifecycle manager 调用超时: {service_name}'
        return False

    def wait_for_lifecycle_nodes_active(
        self,
        node_names: Iterable[str],
        timeout_sec: float,
    ) -> bool:
        required = tuple(node_names)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            missing = [
                node_name
                for node_name in required
                if not self.lifecycle_node_is_active(node_name)
            ]
            if not missing:
                self.patrol_error = ''
                return True
            first = missing[0].lstrip('/')
            self.startup_step = 'waiting_nav2'
            self.patrol_error = f'{first} 尚未 active'
            time.sleep(0.2)
        return False

    def wait_for_nav2_active_ready(self, timeout_sec: float = 25.0) -> bool:
        return self.wait_for_readiness_keys(
            ('nav2_active',),
            timeout_sec,
            error_prefix='Nav2 lifecycle 未激活',
        )

    def wait_for_keepout_active_ready(self, timeout_sec: float = 12.0) -> bool:
        if not self.keepout_required():
            return True
        return self.wait_for_lifecycle_nodes_active(
            KEEPOUT_LIFECYCLE_NODES,
            timeout_sec,
        )

    def wait_for_patrol_executor_ready(self, timeout_sec: float = 15.0) -> bool:
        if not hasattr(self, 'processes'):
            return True
        return self.wait_for_readiness_keys(
            ('executor', 'route_file', 'patrol_status'),
            timeout_sec,
            error_prefix='巡逻执行器等待超时',
        )

    def wait_for_initial_pose_published(self, timeout_sec: float = 8.0, generation: Optional[int] = None) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if generation is not None and not self.is_current_patrol_start(generation):
                return False
            if self.process_exited_before_readiness('patrol_executor'):
                return False
            if self.has_initial_pose_published(generation):
                self.patrol_error = ''
                return True
            self.startup_step = 'waiting_initial_pose_published'
            self.patrol_error = '等待巡逻执行器发布初始位姿'
            time.sleep(0.1)
        self.patrol_error = '等待初始位姿发布超时'
        return False

    def wait_for_fresh_amcl(self, timeout_sec: float, after: float, generation: int) -> bool:
        if not hasattr(self, 'processes'):
            return True
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self.is_current_patrol_start(generation):
                return False
            if getattr(self, 'last_amcl_received_at', 0.0) >= after:
                return True
            self.startup_step = 'waiting_amcl_pose'
            self.patrol_error = '等待初始位姿后的 /amcl_pose'
            time.sleep(0.1)
        return False

    def wait_for_localization_ready(self, timeout_sec: float) -> bool:
        if not hasattr(self, 'processes'):
            return True
        return self.wait_for_readiness_keys(
            ('navigation', 'map', 'initialpose_subscribers', 'localization_active'),
            timeout_sec,
            error_prefix='定位服务未就绪',
        )

    def wait_for_patrol_command_ack(self, request_id: str, timeout_sec: float, generation: int) -> bool:
        if not hasattr(self, 'processes'):
            return True
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self.is_current_patrol_start(generation):
                return False
            ack = getattr(self, 'last_patrol_command_ack', {}) or {}
            if ack.get('event') == 'command_accepted' and ack.get('request_id') == request_id:
                return True
            self.startup_step = 'waiting_executor_response'
            self.patrol_error = '等待巡逻执行器确认启动命令'
            time.sleep(0.1)
        return False

    def wait_for_readiness_keys(
        self,
        required_keys: Iterable[str],
        timeout_sec: float,
        error_prefix: str,
    ) -> bool:
        required = tuple(required_keys)
        deadline = time.monotonic() + timeout_sec
        last_missing: List[str] = []
        while time.monotonic() < deadline:
            for name in self.readiness_process_names(required):
                if self.process_exited_before_readiness(name):
                    return False
            readiness = self.build_patrol_readiness()
            last_missing = [key for key in required if not readiness.get(key)]
            if not last_missing:
                self.patrol_error = ''
                return True
            self.startup_step = f"waiting_{last_missing[0]}"
            self.patrol_error = self.readiness_wait_message(last_missing)
            time.sleep(0.25)
        self.patrol_error = self.readiness_timeout_message(error_prefix, last_missing)
        return False

    def readiness_process_names(self, required: Iterable[str]) -> Iterable[str]:
        required = set(required)
        navigation_keys = {
            'navigation', 'map', 'initialpose_subscribers', 'nav2_action',
            'nav2_action_discovered', 'nav2_components_loaded',
            'nav2_active', 'keepout_active', 'map_to_odom', 'localization_active',
        }
        if required.intersection(navigation_keys):
            yield 'navigation'
        if required.intersection({
            'executor', 'route_file', 'patrol_status', 'initial_pose_published', 'map_to_odom',
        }):
            yield 'patrol_executor'

    def process_exited_before_readiness(self, name: str) -> bool:
        proc = getattr(self, 'processes', {}).get(name)
        if proc is None or getattr(proc, 'process', None) is None:
            return False
        exit_code = proc.poll_exit_code()
        if exit_code is None:
            return False
        error = f'{name} process exited before readiness, exit code={exit_code}'
        proc.last_error = error
        proc.last_message = error
        self.patrol_error = error
        self.startup_step = f'{name}_process_exited'
        return True

    def readiness_wait_message(self, missing: List[str]) -> str:
        if len(missing) == 1 and missing[0] in READINESS_ERROR_MESSAGES:
            return '等待巡逻依赖: ' + READINESS_ERROR_MESSAGES[missing[0]]
        return '等待巡逻依赖: ' + ', '.join(
            READINESS_ERROR_MESSAGES.get(key, key) for key in missing
        )

    def readiness_timeout_message(self, error_prefix: str, missing: List[str]) -> str:
        if len(missing) == 1 and missing[0] in READINESS_ERROR_MESSAGES:
            return READINESS_ERROR_MESSAGES[missing[0]]
        if error_prefix:
            return f"{error_prefix}: " + ', '.join(
                READINESS_ERROR_MESSAGES.get(key, key) for key in missing
            )
        return '等待巡逻依赖超时: ' + ', '.join(
            READINESS_ERROR_MESSAGES.get(key, key) for key in missing
        )

    def wait_for_patrol_status(self, timeout_sec: float = 10.0) -> str:
        deadline = time.monotonic() + timeout_sec
        last_state = ''
        while time.monotonic() < deadline:
            status = getattr(self, 'last_patrol_status', {}) or {}
            last_state = str(status.get('state') or status.get('status') or '')
            if last_state in ('running', 'failed', 'succeeded', 'canceled', 'cancelled'):
                if last_state == 'failed':
                    self.patrol_error = str(
                        status.get('message') or status.get('error') or '巡逻执行器报告失败'
                    )
                return last_state
            time.sleep(0.2)
        self.patrol_error = '等待 /patrol/status 进入 running 超时'
        return last_state or 'timeout'

    def is_patrol_executor_ready(self) -> bool:
        process = self.processes.get('patrol_executor')
        process_running = bool(process and process.is_running())
        try:
            subscribers = self.get_subscriptions_info_by_topic('/patrol/command')
        except Exception:
            subscribers = []
        return process_running and bool(subscribers)

    def has_patrol_route_file(self) -> bool:
        try:
            route = load_route_file(self.patrol_route_path)
            validate_route_map_binding(route, self.default_navigation_map)
            return route.get('version') == 3
        except Exception:
            return False

    def topic_has_publishers(self, topic: str) -> bool:
        try:
            return bool(self.get_publishers_info_by_topic(topic))
        except Exception:
            return False

    def topic_has_subscribers(self, topic: str) -> bool:
        try:
            return bool(self.get_subscriptions_info_by_topic(topic))
        except Exception:
            return False

    def has_initial_pose_published(self, generation: Optional[int] = None) -> bool:
        event = getattr(self, 'last_initial_pose_event', {}) or {}
        if event.get('event') != 'initial_pose_published':
            return False
        if generation is None:
            return True
        return (
            event.get('startup_id') == getattr(self, 'startup_id', '')
            and os.path.abspath(str(event.get('route_path') or '')) == self.patrol_route_path
            and float(event.get('timestamp') or 0.0) >= getattr(self, 'startup_started_at', 0.0)
        )

    def patrol_timeout(self, name: str, default: float) -> float:
        try:
            return float(self.get_parameter(name).value)
        except Exception:
            return default

    def has_fresh_core_sensors(self) -> bool:
        sensors = self.core_sensor_status()
        return all((
            sensors['odom_publisher'],
            sensors['odom_fresh'],
            sensors['scan_publisher'],
            sensors['scan_fresh'],
            sensors['odom_to_base'],
            sensors['base_to_laser'],
        ))

    def core_sensor_status(self) -> Dict[str, Any]:
        freshness = self.patrol_timeout('patrol_sensor_freshness_sec', 1.0)
        now = time.time()
        last_odom = getattr(self, 'last_odom_received_at', 0.0)
        odom_age = now - last_odom if last_odom else None
        scan = self.scan_freshness_status()
        return {
            'freshness_sec': freshness,
            'odom_publisher': self.topic_has_publishers('/odom'),
            'odom_age_sec': odom_age,
            'odom_fresh': odom_age is not None and odom_age <= freshness,
            'scan_publisher': self.topic_has_publishers('/scan'),
            'scan_age_sec': scan['scan_age_sec'],
            'scan_stamp_age_sec': scan['scan_stamp_age_sec'],
            'scan_fresh': scan['scan_fresh'],
            'odom_to_base': self.has_transform('odom', 'base_footprint'),
            'base_to_laser': self.has_transform('base_footprint', 'laser_link'),
        }

    def scan_freshness_status(self) -> Dict[str, Any]:
        freshness = self.patrol_timeout('patrol_sensor_freshness_sec', 1.0)
        now = time.time()
        received = getattr(self, 'last_scan_received_at', 0.0)
        stamped = getattr(self, 'last_scan_stamp_at', received)
        receive_age = now - received if received else None
        stamp_age = now - stamped if stamped else None
        return {
            'scan_age_sec': receive_age,
            'scan_stamp_age_sec': stamp_age,
            'scan_fresh': bool(
                receive_age is not None
                and stamp_age is not None
                and receive_age <= freshness
                and -0.1 <= stamp_age <= freshness
            ),
        }

    @staticmethod
    def core_sensor_wait_message(status: Dict[str, Any]) -> str:
        def age_text(value: Any) -> str:
            return '未收到' if value is None else f'{float(value):.2f}s'

        return (
            '传感器门控未就绪: '
            f"/odom发布者={'有' if status['odom_publisher'] else '无'}, "
            f"odom消息年龄={age_text(status['odom_age_sec'])}; "
            f"/scan发布者={'有' if status['scan_publisher'] else '无'}, "
            f"scan消息年龄={age_text(status['scan_age_sec'])}; "
            f"odom→base_footprint={'有' if status['odom_to_base'] else '无'}; "
            f"base_footprint→laser_link={'有' if status['base_to_laser'] else '无'}; "
            f"新鲜度要求≤{float(status['freshness_sec']):.2f}s"
        )

    def has_base_sensor_tf(self) -> bool:
        return self.has_transform('odom', 'base_footprint') and self.has_transform(
            'base_footprint', 'laser_link'
        )

    def has_transform(self, target_frame: str, source_frame: str) -> bool:
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return False
        try:
            return bool(tf_buffer.can_transform(target_frame, source_frame, Time()))
        except Exception:
            return False

    def has_map_to_odom(self) -> bool:
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            self.map_to_odom_stable_since = 0.0
            return False
        try:
            ready = bool(tf_buffer.can_transform('map', 'odom', Time()))
        except Exception:
            ready = False
        if ready:
            if not getattr(self, 'map_to_odom_stable_since', 0.0):
                self.map_to_odom_stable_since = time.monotonic()
        else:
            self.map_to_odom_stable_since = 0.0
        return ready

    def map_to_odom_stable_sec(self) -> float:
        since = getattr(self, 'map_to_odom_stable_since', 0.0)
        return max(0.0, time.monotonic() - since) if since else 0.0

    def lifecycle_node_state(self, node_name: str) -> int:
        try:
            client = self.lifecycle_clients.get(node_name)
            if client is None:
                client = self.create_client(GetState, f'{node_name}/get_state')
                self.lifecycle_clients[node_name] = client
            if not client.wait_for_service(timeout_sec=0.1):
                self.lifecycle_states[node_name] = 'unavailable'
                return State.PRIMARY_STATE_UNKNOWN
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    if response is not None:
                        state = response.current_state
                        self.lifecycle_states[node_name] = state.label
                        return state.id
                    break
                time.sleep(0.02)
        except Exception:
            pass
        self.lifecycle_states[node_name] = 'unknown'
        return State.PRIMARY_STATE_UNKNOWN

    def is_nav2_active(self) -> bool:
        return all(
            self.lifecycle_node_is_active(name)
            for name in NAVIGATION_LIFECYCLE_NODES
        )

    def is_localization_active(self) -> bool:
        return all(
            self.lifecycle_node_is_active(name)
            for name in LOCALIZATION_LIFECYCLE_NODES
        )

    def is_keepout_active(self) -> bool:
        return all(
            self.lifecycle_node_is_active(name)
            for name in KEEPOUT_LIFECYCLE_NODES
        )

    def lifecycle_node_is_active(self, node_name: str) -> bool:
        return self.lifecycle_node_state(node_name) == State.PRIMARY_STATE_ACTIVE

    def lifecycle_nodes_cached_active(self, node_names: Iterable[str]) -> bool:
        return all(
            self.lifecycle_states.get(name) == 'active'
            for name in node_names
        )

    def service_exists(self, service_name: str) -> bool:
        try:
            return any(
                name == service_name
                for name, _types in self.get_service_names_and_types()
            )
        except Exception:
            return False

    def nav2_components_loaded(self) -> bool:
        required_services = tuple(
            f'{name}/get_state'
            for name in (
                *LOCALIZATION_LIFECYCLE_NODES,
                *NAVIGATION_LIFECYCLE_NODES,
            )
        )
        return all(self.service_exists(name) for name in required_services)

    def wait_for_nav2_components_loaded(self, timeout_sec: float = 15.0) -> bool:
        required_services = tuple(
            f'{name}/get_state'
            for name in (
                *LOCALIZATION_LIFECYCLE_NODES,
                *NAVIGATION_LIFECYCLE_NODES,
            )
        )
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            missing = [name for name in required_services if not self.service_exists(name)]
            if not missing:
                self.patrol_error = ''
                return True
            self.startup_step = 'waiting_nav2'
            self.patrol_error = 'Nav2 组件未加载: ' + ', '.join(missing)
            time.sleep(0.25)
        return False

    def compute_nav2_active(self) -> bool:
        return self.lifecycle_nodes_cached_active(NAVIGATION_LIFECYCLE_NODES)

    def has_nav2_action(self) -> bool:
        action_topics = (
            '/navigate_to_pose/_action/status',
            '/navigate_to_pose/_action/feedback',
            '/navigate_to_pose/_action/send_goal',
        )
        return any(
            self.topic_has_publishers(topic) or self.topic_has_subscribers(topic)
            for topic in action_topics
        )

    def has_keepout_runtime_ready(self) -> bool:
        if not self.keepout_required():
            return True
        return all(self.keepout_subscription_status().values())

    def keepout_subscription_status(self) -> Dict[str, bool]:
        if not self.keepout_required():
            return {}
        global_info = '/keepout_global_filter_info'
        global_mask = '/keepout_global_mask'
        local_info = '/keepout_local_filter_info'
        local_mask = '/keepout_local_mask'
        return {
            'global_info_subscribed': self.topic_has_publishers(global_info)
            and self.costmap_subscribes(
                global_info, 'global_costmap'
            ),
            'global_mask_subscribed': self.topic_has_publishers(global_mask)
            and self.costmap_subscribes(
                global_mask, 'global_costmap'
            ),
            'local_info_subscribed': self.topic_has_publishers(local_info)
            and self.costmap_subscribes(
                local_info, 'local_costmap'
            ),
            'local_mask_subscribed': self.topic_has_publishers(local_mask)
            and self.costmap_subscribes(
                local_mask, 'local_costmap'
            ),
        }

    def costmap_subscribes(self, topic: str, costmap_name: str) -> bool:
        try:
            subscriptions = self.get_subscriptions_info_by_topic(topic)
        except Exception:
            return False
        for subscription in subscriptions:
            node_name = str(getattr(subscription, 'node_name', '') or '')
            node_namespace = str(getattr(subscription, 'node_namespace', '') or '')
            full_name = f'{node_namespace}/{node_name}'
            if costmap_name in full_name:
                return True
        return False

    def wait_for_keepout_runtime_ready(
        self,
        timeout_sec: float = 12.0,
    ) -> bool:
        if not self.keepout_required():
            return True
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            status = self.keepout_subscription_status()
            if all(status.values()):
                self.patrol_error = ''
                return True

            self.startup_step = 'waiting_keepout_active'
            missing = [name for name, ready in status.items() if not ready]
            self.patrol_error = 'Keepout 尚未就绪: ' + ', '.join(missing)
            time.sleep(0.25)

        self.patrol_error = '禁行区数据未送达 global/local costmap'
        return False

    def log_patrol_start_readiness(self) -> None:
        try:
            readiness = self.build_patrol_readiness()
            fields = (
                'map', 'localization_active', 'set_initial_pose_service',
                'initial_pose_service_ok', 'amcl_pose_confirmed', 'map_to_odom',
                'nav2_active',
            )
            summary = ', '.join(f'{key}={bool(readiness.get(key))}' for key in fields)
            self.log_info(f'patrol start readiness: {summary}')
        except Exception as exc:
            self.log_info(f'patrol start readiness unavailable: {exc}')

    def wait_for_patrol_command_subscriber(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.process_exited_before_readiness('patrol_executor'):
                return False
            if self.is_patrol_executor_ready():
                return True
            time.sleep(0.1)
        return False

    def wait_for_patrol_status_heartbeat(self, since: float, timeout_sec: float = 5.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.process_exited_before_readiness('patrol_executor'):
                return False
            if getattr(self, 'last_patrol_status_received_at', 0.0) >= since:
                return True
            time.sleep(0.1)
        return False

    def wait_for_patrol_executor(self, timeout_sec: float = 2.0) -> bool:
        return self.wait_for_patrol_command_subscriber(timeout_sec)

    def save_map(self, map_name: str) -> None:
        safe_name = ''.join(c for c in map_name if c.isalnum() or c in ('_', '-')).strip('_-')
        if not safe_name:
            safe_name = time.strftime('inspection_map_%Y%m%d_%H%M')
        os.makedirs(self.map_output_dir, exist_ok=True)
        map_prefix = os.path.join(self.map_output_dir, safe_name)
        cmd = self.wrap_command(
            f'ros2 run nav2_map_server map_saver_cli -f {map_prefix} '
            '--ros-args -p save_map_timeout:=10.0'
        )
        try:
            completed = subprocess.run(
                cmd,
                shell=True,
                executable='/bin/bash',
                cwd=self.workspace_dir,
                timeout=30.0,
                check=False,
                capture_output=True,
                text=True,
            )
        except Exception as exc:
            self.set_result('save_map', False, f'保存地图失败: {exc}')
            return
        if completed.returncode == 0:
            self.set_result('save_map', True, f'地图已保存: {map_prefix}.yaml')
        else:
            detail = (completed.stderr or completed.stdout or '').strip().splitlines()
            message = '\n'.join(detail[-5:]) if detail else f'map_saver_cli exited {completed.returncode}'
            self.set_result('save_map', False, f'保存地图失败: {message}')

    def emergency_stop(self) -> None:
        self.cancel_patrol_start()
        self.publish_mode('fault')
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        correlation = dict(getattr(self, 'agent_operation_context', {}))
        self.last_agent_operation_feedback = (
            {**correlation, 'state': 'succeeded', 'status': 'succeeded'}
            if correlation.get('operation_id') else {}
        )
        self.set_result('emergency_stop', True, '软件急停已发送')

    def publish_mode(self, mode: str) -> None:
        self.current_system_mode = str(mode)
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def wrap_command(self, command: str) -> str:
        return (
            f'source /opt/ros/{self.ros_distro}/setup.bash && '
            f'if [ -f "{self.workspace_dir}/install/setup.bash" ]; then '
            f'source "{self.workspace_dir}/install/setup.bash"; fi && '
            f'exec {command}'
        )

    def llm_launch_command(self) -> str:
        return (
            'ros2 launch ylhb_llm llm.launch.py '
            'enable_display_ui:=false enable_system_supervisor:=false '
            f'enable_voice:={str(self.enable_voice).lower()} '
            f'enable_voice_session:={str(self.enable_voice_session).lower()} '
            f'enable_capture_voice:={str(self.enable_capture_voice).lower()} '
            f'enable_tts:={str(self.enable_tts).lower()} '
            f'audio_device:={self.audio_device} '
            f'audio_input_device:={self.audio_input_device} '
            f'audio_output_device:={self.audio_output_device} '
            f'asr_model:={self.asr_model} '
            f'tts_model:={self.tts_model} '
            f'tts_voice:={self.tts_voice} '
            f'tts_language_type:={self.tts_language_type} '
            f'dashscope_base_url:={self.dashscope_base_url}'
        )

    def voice_summary(self, prefix: str) -> str:
        return (
            f'{prefix}; voice={self.enable_voice}, session={self.enable_voice_session}, '
            f'capture={self.enable_capture_voice}, tts={self.enable_tts}, '
            f'input={self.audio_input_device}, output={self.audio_output_device}'
        )

    def set_result(self, command: str, success: bool, message: str) -> None:
        with self.lock:
            self.set_result_locked(command, success, message)

    def set_result_locked(self, command: str, success: bool, message: str) -> None:
        self.last_command = command
        self.last_success = bool(success)
        self.last_message = message
        self.log_info(f'{command}: {message}')
        self.publish_status_locked()

    def publish_component_operation_feedback(
        self,
        correlation: Dict[str, str],
        success: bool,
        result_status: str,
    ) -> None:
        self.last_agent_operation_feedback = {
            **correlation,
            'state': 'succeeded' if success else 'failed',
            'status': 'succeeded' if success else 'failed',
            'result_status': result_status,
        }
        self.publish_status()
        self.last_agent_operation_feedback = {}

    def log_info(self, message: str) -> None:
        try:
            self.get_logger().info(message)
        except AttributeError:
            pass

    def publish_status(self) -> None:
        mobile_bridge = self.processes.get('mobile_bridge')
        tcp_status = mobile_bridge_tcp_status(True)
        if tcp_status == 'tcp_error' and not (
            getattr(self, 'mobile_bridge_managed_externally', False)
            or (mobile_bridge and mobile_bridge.is_running())
        ):
            tcp_status = 'stopped'
        self.mobile_bridge_tcp = tcp_status
        self.mobile_bridge_http = {'tcp_ok': 'http_ok', 'tcp_error': 'http_error'}.get(tcp_status, tcp_status)
        with self.lock:
            self.publish_status_locked()

    def build_status_payload(self) -> Dict[str, Any]:
        mobile_bridge = self.processes.get('mobile_bridge')
        process_running = bool(mobile_bridge and mobile_bridge.is_running())
        tcp_status = getattr(self, 'mobile_bridge_tcp', {
            'http_ok': 'tcp_ok', 'http_error': 'tcp_error'
        }.get(getattr(self, 'mobile_bridge_http', 'stopped'), 'stopped'))
        if getattr(self, 'mobile_bridge_ownership_conflict', False):
            mobile_bridge_core_state = 'ownership_conflict'
        elif getattr(self, 'mobile_bridge_managed_externally', False):
            mobile_bridge_core_state = 'running' if tcp_status == 'tcp_ok' else 'stopped'
        elif process_running and tcp_status == 'tcp_ok':
            mobile_bridge_core_state = 'running'
        elif process_running:
            mobile_bridge_core_state = 'starting' if time.time() - mobile_bridge.last_started_at < 8.0 else 'unreachable'
        elif tcp_status == 'tcp_ok':
            mobile_bridge_core_state = 'ownership_conflict'
        else:
            mobile_bridge_core_state = 'stopped'
        payload = {
            'schema_version': '1.0',
            'timestamp': time.time(),
            'last_command': self.last_command,
            'success': self.last_success,
            'message': self.last_message,
            **getattr(self, 'last_agent_operation_feedback', {}),
        }
        for name, proc in self.processes.items():
            if name == 'llm' and self.embedded_task_layer:
                payload[name] = 'embedded'
            else:
                payload[name] = 'running' if proc.is_running() else 'stopped'
        capture = self.processes.get('3d_capture')
        reconstruct = self.processes.get('3d_reconstruct')
        payload['3d_mapping'] = (
            'running'
            if (capture and capture.is_running()) or (reconstruct and reconstruct.is_running())
            else 'stopped'
        )
        patrol_state = getattr(self, 'patrol_mode_state', 'idle')
        if patrol_state in ('starting', 'command_sent', 'running'):
            patrol_readiness = self.build_patrol_readiness()
        else:
            patrol_readiness = self.build_light_patrol_readiness()
        keepout_required = self.keepout_required()
        keepout_lifecycle_active = self.lifecycle_nodes_cached_active(
            KEEPOUT_LIFECYCLE_NODES
        ) if keepout_required else False
        keepout_status = {
            'required': keepout_required,
            'ready': not keepout_required or self.has_keepout_runtime_ready(),
            'lifecycle_active': keepout_lifecycle_active,
        }
        if keepout_required:
            keepout_status.update(self.keepout_subscription_status())
        system_mode = getattr(self, 'current_system_mode', 'ready')
        if patrol_state in {'starting', 'command_sent', 'running', 'paused', 'returning_home'}:
            system_mode = 'inspection' if payload.get('perception') == 'running' else 'patrol'
        elif payload.get('mapping') == 'running':
            system_mode = 'mapping'
        elif payload.get('navigation') == 'running':
            system_mode = 'navigation'
        payload.update({
            'system_mode': system_mode,
            'mobile_bridge_owner': getattr(self, 'mobile_bridge_owner', 'systemd' if getattr(self, 'mobile_bridge_managed_externally', False) else 'supervisor'),
            'mobile_bridge_core_state': mobile_bridge_core_state,
            'mobile_bridge_tcp': tcp_status,
            'mobile_bridge_http': self.mobile_bridge_http,
            'mobile_bridge_url': self.mobile_bridge_url,
            'mobile_bridge_managed_externally': getattr(self, 'mobile_bridge_managed_externally', False),
            'mobile_bridge_last_error': (
                getattr(self, 'mobile_bridge_last_error', '')
                or getattr(mobile_bridge, 'last_error', '')
                or ('ylhb-mobile-bridge.service 未运行或 8000 不可达' if getattr(self, 'mobile_bridge_managed_externally', False) and mobile_bridge_core_state != 'running' else '')
            ),
            'mobile_bridge_started_by_supervisor': getattr(self, 'mobile_bridge_started_by_supervisor', False),
            'command_result': getattr(self, 'command_result', {}),
            'jetson_ip': self.jetson_ip,
            'patrol_mode_state': patrol_state,
            'patrol_readiness': patrol_readiness,
            'patrol_error': getattr(self, 'patrol_error', ''),
            'patrol_warning': getattr(self, 'patrol_warning', ''),
            'patrol_navigation_profile': getattr(
                self, 'patrol_navigation_profile', NAVIGATION_PROFILE_AUTO
            ),
            'patrol_navigation_mode': getattr(
                self, 'active_patrol_navigation_mode', NAVIGATION_PROFILE_NORMAL
            ),
            'enabled_hard_keepout_count': getattr(self, 'active_hard_keepout_count', 0),
            'navigation_launch_file': self.navigation_launch_file(),
            'nav2_params_file': self.navigation_params_file_name(),
            'keepout_required': keepout_required,
            'keepout_ready': keepout_status['ready'],
            'keepout_lifecycle_active': keepout_lifecycle_active,
            'lifecycle': {
                name.lstrip('/'): getattr(self, 'lifecycle_states', {}).get(
                    name, 'unknown'
                )
                for name in (
                    *LOCALIZATION_LIFECYCLE_NODES,
                    *NAVIGATION_LIFECYCLE_NODES,
                    *KEEPOUT_LIFECYCLE_NODES,
                )
            },
            'tf': {
                'odom_to_base': self.has_transform('odom', 'base_footprint'),
                'base_to_laser': self.has_transform('base_footprint', 'laser_link'),
                'map_to_odom': self.has_map_to_odom(),
                'map_to_laser': self.has_transform('map', 'laser_link'),
                'map_to_odom_stable_sec': self.map_to_odom_stable_sec(),
            },
            'keepout': keepout_status,
            'startup_step': getattr(self, 'startup_step', ''),
            'startup_step_label': STARTUP_STEP_LABELS.get(
                getattr(self, 'startup_step', ''),
                getattr(self, 'startup_step', ''),
            ),
            'navigation_process_running': bool(
                self.processes.get('navigation') and self.processes['navigation'].is_running()
            ),
            'navigation_exit_code': getattr(self.processes.get('navigation'), 'last_exit_code', None),
            'navigation_last_error': getattr(self.processes.get('navigation'), 'last_error', ''),
            'last_patrol_status': getattr(self, 'last_patrol_status', {}),
            'patrol_diagnostics': {
                'last_patrol_event': getattr(self, 'last_patrol_event', {}),
                'last_initial_pose_event': getattr(self, 'last_initial_pose_event', {}),
                'last_patrol_start_request_id': getattr(self, 'last_patrol_start_request_id', ''),
                'last_patrol_command_ack': getattr(self, 'last_patrol_command_ack', {}),
                'startup_id': getattr(self, 'startup_id', ''),
                'startup_started_at': getattr(self, 'startup_started_at', 0.0),
                'startup_generation': getattr(self, 'startup_generation', 0),
                'sensor_gate': self.core_sensor_status(),
                'odom_age_sec': max(0.0, time.time() - getattr(self, 'last_odom_received_at', 0.0)),
                'scan_age_sec': max(0.0, time.time() - getattr(self, 'last_scan_received_at', 0.0)),
                'amcl_age_sec': max(0.0, time.time() - getattr(self, 'last_amcl_received_at', 0.0)),
            },
            'latest_mapping3d_status': getattr(self, 'latest_mapping3d_status', {}),
            'latest_mapping3d_result': getattr(self, 'latest_mapping3d_result', {}),
            'latest_scene_upload_status': getattr(self, 'latest_scene_upload_status', {}),
            'scene_uploads_by_session': getattr(self, 'scene_uploads_by_session', {}),
            'latest_3d_capture': self.read_latest_json(
                getattr(self, 'mapping3d_capture_dir', getattr(self, 'mapping3d_output_dir', workspace_path('runs', '3d_capture')))
            ),
            'latest_3d_reconstruct': self.read_latest_json(
                getattr(self, 'mapping3d_reconstruct_dir', workspace_path('runs', '3d_reconstruct'))
            ),
        })
        payload['mobile_bridge'] = 'running' if mobile_bridge_core_state == 'running' else 'stopped'
        capture_dir = getattr(self, 'mapping3d_capture_dir', getattr(self, 'mapping3d_output_dir', workspace_path('runs', '3d_capture')))
        reconstruct_dir = getattr(self, 'mapping3d_reconstruct_dir', workspace_path('runs', '3d_reconstruct'))
        payload['mapping3d_assets'] = {
            'captures': zed_3d_asset_manager.list_assets(capture_dir, 'capture')[:10],
            'reconstructs': zed_3d_asset_manager.list_assets(reconstruct_dir, 'reconstruct')[:10],
        }
        payload['mapping3d_storage_summary'] = zed_3d_asset_manager.storage_summary(capture_dir, reconstruct_dir)
        return payload

    def publish_status_locked(self) -> None:
        payload = self.build_status_payload()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.cancel_patrol_start()
        ros_cleanup = self.ros_context_valid()
        if ros_cleanup:
            try:
                self.publish_patrol_command('cancel')
                self.publish_zero_velocity()
            except Exception:
                ros_cleanup = False
        extras = [
            name
            for name in self.processes
            if name not in SHUTDOWN_ORDER
        ]
        shutdown_order = (
            *SHUTDOWN_ORDER[:-2],
            *extras,
            *SHUTDOWN_ORDER[-2:],
        )
        for name in shutdown_order:
            if name == 'mobile_bridge' and (
                getattr(self, 'mobile_bridge_managed_externally', False)
                or not getattr(self, 'mobile_bridge_started_by_supervisor', False)
            ):
                continue
            try:
                self.stop_process(name, ros_cleanup=ros_cleanup, report=False)
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SystemSupervisorNode()
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
