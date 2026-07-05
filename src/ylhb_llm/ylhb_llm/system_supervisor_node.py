import json
import os
import signal
import socket
import subprocess
import threading
import time
from typing import Any, Dict, Iterable, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty
from ylhb_mobile_bridge.patrol_qos import patrol_status_qos_profile


START_PROCESS_MESSAGES = {
    'bringup': '底盘与雷达已启动',
    'navigation': '导航已启动',
    'patrol_executor': '巡逻执行器已启动',
    'zed': 'ZED 相机已启动',
    'perception': '感知节点已启动',
    'mobile_bridge': '移动端桥接已启动',
    'mapping': '建图已启动',
}

PATROL_BRINGUP_TO_NAVIGATION_DELAY_SEC = 3.0
PATROL_NAVIGATION_TO_EXECUTOR_DELAY_SEC = 20.0
PATROL_EXECUTOR_TO_START_DELAY_SEC = 6.0
LONG_RUNNING_COMMANDS = {
    'start_patrol_mode',
    'stop_robot_stack',
    'stop_navigation',
    'stop_bringup',
    'stop_patrol_mode',
}

STARTUP_STEP_LABELS = {
    'starting_bringup': '等待底盘传感器',
    'waiting_bringup': '等待底盘传感器',
    'waiting_odom': '等待底盘传感器',
    'waiting_scan': '等待底盘传感器',
    'waiting_tf': '等待底盘传感器',
    'waiting_after_bringup': '底盘启动后等待',
    'starting_navigation': '等待地图',
    'waiting_navigation': '等待地图',
    'waiting_map': '等待地图',
    'waiting_initialpose_subscribers': '等待地图',
    'waiting_after_navigation': '导航启动后等待',
    'starting_executor': '发布初始位姿',
    'waiting_executor': '发布初始位姿',
    'waiting_route_file': '发布初始位姿',
    'waiting_patrol_status': '发布初始位姿',
    'waiting_initial_pose_published': '发布初始位姿',
    'waiting_after_executor': '巡逻执行器启动后等待',
    'waiting_patrol_running': '等待巡逻执行器运行',
    'patrol_start_sent': '巡逻启动命令已发送',
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
    'nav2_active': 'Nav2 lifecycle 未激活',
    'executor': '巡逻执行器未就绪',
    'route_file': '未找到正式巡逻路线文件',
    'patrol_status': '等待 /patrol/status 发布者超时',
}


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
        self.last_message = ''

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None


class SystemSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('system_supervisor_node')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('workspace_dir', os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL')))
        self.declare_parameter('ros_distro', 'humble')
        self.declare_parameter('map_output_dir', workspace_path('maps'))
        self.declare_parameter('default_navigation_map', workspace_path('maps', 'my_map.yaml'))
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

        self.workspace_dir = os.path.expanduser(str(self.get_parameter('workspace_dir').value))
        self.ros_distro = str(self.get_parameter('ros_distro').value)
        self.map_output_dir = os.path.expanduser(str(self.get_parameter('map_output_dir').value))
        self.default_navigation_map = os.path.expanduser(str(self.get_parameter('default_navigation_map').value))
        self.perception_model_path = os.path.expanduser(str(self.get_parameter('perception_model_path').value))
        self.embedded_task_layer = bool(self.get_parameter('embedded_task_layer').value)
        self.enable_voice = bool(self.get_parameter('enable_voice').value)
        self.enable_voice_session = bool(self.get_parameter('enable_voice_session').value)
        self.enable_capture_voice = bool(self.get_parameter('enable_capture_voice').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)
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
        self.jetson_ip = discover_jetson_ip()
        self.mobile_bridge_url = f'http://{self.jetson_ip}:8000'
        self.mobile_bridge_http = 'stopped'
        self.patrol_mode_state = 'idle'
        self.patrol_error = ''
        self.startup_step = ''
        self.last_patrol_status: Dict[str, Any] = {}
        self.last_patrol_status_received_at = 0.0
        self.last_patrol_start_request_id = ''
        self.last_patrol_event: Dict[str, Any] = {}
        self.last_initial_pose_event: Dict[str, Any] = {}
        self.inflight_commands = set()
        self.lifecycle_clients: Dict[str, Any] = {}
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
                f'ros2 launch ylhb_base navigation.launch.py map:={self.default_navigation_map}',
            ),
            'zed': ManagedProcess('zed', 'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'),
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
                'ros2 launch ylhb_mobile_bridge patrol_executor.launch.py '
                'auto_start:=false publish_initial_pose_on_startup:=true',
            ),
        }

        self.status_pub = self.create_publisher(
            String, self.get_parameter('system_status_topic').value, latched_qos())
        self.mode_pub = self.create_publisher(
            String, self.get_parameter('system_mode_topic').value, latched_qos())
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.patrol_command_pub = self.create_publisher(String, '/patrol/command', 10)
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
        self.create_timer(1.0, self.publish_status)
        self.publish_status()
        self.log_info('系统监督节点已启动')

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

    def patrol_event_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        if isinstance(payload, dict):
            self.last_patrol_event = payload
            if payload.get('event') == 'initial_pose_published':
                self.last_initial_pose_event = payload

    def handle_command(self, command: str, payload: Dict[str, Any]) -> None:
        if command == 'start_patrol_mode':
            self.start_patrol_mode(
                str(payload.get('profile') or 'navigation'),
                str(payload.get('route_id') or ''),
            )
            return
        patrol_commands = {
            'pause_patrol': 'pause',
            'resume_patrol': 'resume',
            'cancel_patrol': 'cancel',
            'reload_patrol_route': 'reload',
        }
        if command in patrol_commands:
            patrol_command = patrol_commands[command]
            self.publish_patrol_command(patrol_command)
            self.set_result(command, True, f'已发送巡逻命令: {patrol_command}')
            return
        if command == 'stop_patrol_mode':
            self.stop_process('patrol_executor')
            self.reset_patrol_mode_state()
            self.set_result(command, True, '巡逻模式已停止')
            return
        if command.startswith('start_'):
            name = command[len('start_'):]
            if name in self.processes:
                self.start_process(name)
                if name == 'mapping':
                    self.publish_mode('mapping')
                return
        if command.startswith('stop_'):
            name = command[len('stop_'):]
            if name in self.processes:
                self.stop_process(name)
                if name == 'mapping':
                    self.publish_mode('ready')
                return
        if command == 'restart_mobile_bridge':
            self.stop_process('mobile_bridge')
            self.start_process('mobile_bridge')
            return
        if command == 'restart_navigation':
            self.stop_process('navigation')
            self.start_process('navigation')
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

    def start_process(self, name: str) -> None:
        if name == 'llm' and self.embedded_task_layer:
            self.set_result(
                'start_llm',
                True,
                self.voice_summary('AI task layer is embedded in inspection launch'),
            )
            return
        proc = self.processes[name]
        with self.lock:
            if proc.is_running():
                self.set_result_locked(f'start_{name}', True, f'{START_PROCESS_MESSAGES.get(name, name)}，已在运行')
                return
            cmd = self.wrap_command(proc.command)
            proc.process = subprocess.Popen(
                cmd,
                shell=True,
                executable='/bin/bash',
                cwd=self.workspace_dir,
                preexec_fn=os.setsid,
            )
            proc.last_message = f'started pid={proc.process.pid}'
            self.set_result_locked(f'start_{name}', True, START_PROCESS_MESSAGES.get(name, f'{name} 已启动'))

    def stop_process(self, name: str) -> None:
        if name == 'llm' and self.embedded_task_layer:
            self.set_result(
                'stop_llm',
                True,
                'AI task layer is embedded in inspection launch; keep it running with UI and voice.',
            )
            return
        proc = self.processes[name]
        with self.lock:
            if not proc.is_running():
                self.set_result_locked(f'stop_{name}', True, f'{name} already stopped')
                return
            if name == 'bringup':
                self.stop_lidar_motor()
            assert proc.process is not None
            pid = proc.process.pid
            try:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
                try:
                    proc.process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(pid), signal.SIGKILL)
                    proc.process.wait(timeout=2.0)
                proc.last_message = 'stopped'
                self.set_result_locked(f'stop_{name}', True, f'{name} stopped')
            except Exception as exc:
                self.set_result_locked(f'stop_{name}', False, f'Failed to stop {name}: {exc}')

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

    def start_robot_stack(self) -> None:
        for name in ('bringup', 'zed', 'perception', 'navigation', 'llm'):
            self.start_process(name)
            time.sleep(0.3)
        self.set_result('start_robot_stack', True, '巡检节点启动命令已发送')

    def stop_robot_stack(self) -> None:
        names = ('patrol_executor', 'navigation', 'perception', 'zed', 'bringup')
        if all(not self.processes[name].is_running() for name in names):
            self.reset_patrol_mode_state()
            self.publish_mode('ready')
            self.set_result('stop_robot_stack', True, '巡检运动、导航和感知节点已停止，AI/UI 保持运行')
            return
        for name in names:
            self.stop_process(name)
            time.sleep(0.2)
        self.reset_patrol_mode_state()
        self.publish_mode('ready')
        self.set_result('stop_robot_stack', True, '巡检运动、导航和感知节点已停止，AI/UI 保持运行')

    def reset_patrol_mode_state(self) -> None:
        self.patrol_mode_state = 'idle'
        self.startup_step = ''
        self.patrol_error = ''

    def start_patrol_mode(self, profile: str = 'navigation', route_id: str = '') -> None:
        profile = profile if profile in ('navigation', 'inspection') else 'navigation'
        self.patrol_mode_state = 'starting'
        self.patrol_error = ''
        self.log_info('start_patrol_mode: 按手动流程启动巡逻')

        self.startup_step = 'starting_bringup'
        self.start_process('bringup')
        self.startup_step = 'waiting_after_bringup'
        time.sleep(PATROL_BRINGUP_TO_NAVIGATION_DELAY_SEC)

        if profile == 'inspection':
            self.startup_step = 'starting_inspection'
            self.start_process('zed')
            self.start_process('perception')

        self.startup_step = 'starting_navigation'
        self.start_process('navigation')
        self.startup_step = 'waiting_after_navigation'
        time.sleep(PATROL_NAVIGATION_TO_EXECUTOR_DELAY_SEC)
        navigation_ok = self.wait_for_navigation_ready(35.0)
        navigation_error = self.patrol_error

        self.startup_step = 'starting_executor'
        started_at = time.time()
        self.start_process('patrol_executor')
        self.startup_step = 'waiting_after_executor'
        time.sleep(PATROL_EXECUTOR_TO_START_DELAY_SEC)
        heartbeat_ok = self.wait_for_patrol_status_heartbeat(started_at, 8.0)
        subscriber_ok = self.wait_for_patrol_command_subscriber(5.0)
        initial_pose_ok = self.wait_for_initial_pose_published(8.0)
        map_to_odom_ok = self.wait_for_map_to_odom(10.0)
        nav2_active_ok = self.wait_for_nav2_active_ready(25.0)
        self.log_patrol_start_readiness()
        gate_warnings = []
        if not initial_pose_ok:
            gate_warnings.append('未确认初始位姿已发布')
        if not map_to_odom_ok:
            gate_warnings.append('未确认 map->odom TF')
        if not nav2_active_ok:
            gate_warnings.append('未确认 Nav2 lifecycle active')
        if gate_warnings:
            self.patrol_mode_state = 'failed'
            self.startup_step = 'patrol_failed'
            self.patrol_error = '；'.join(gate_warnings)
            self.set_result('start_patrol_mode', False, '巡逻启动门控失败: ' + self.patrol_error)
            return
        request_id = f"patrol_start_{int(time.time() * 1000)}"
        if route_id:
            self.publish_patrol_command('start', request_id=request_id, route_id=route_id)
        else:
            self.publish_patrol_command('start', request_id=request_id)
        self.startup_step = 'patrol_start_sent'
        time.sleep(5.0)
        status_state = str((self.last_patrol_status or {}).get('state') or '')
        if status_state in ('idle', 'unavailable'):
            if route_id:
                self.publish_patrol_command('start', request_id=request_id, route_id=route_id)
            else:
                self.publish_patrol_command('start', request_id=request_id)
        if self.patrol_mode_state == 'starting':
            self.patrol_mode_state = 'command_sent'
            self.startup_step = 'patrol_start_sent'
        warnings = []
        if not navigation_ok:
            warnings.append(navigation_error or '导航依赖未完全就绪')
        if not heartbeat_ok:
            warnings.append('未确认 /patrol/status heartbeat')
        if not subscriber_ok:
            warnings.append('未确认 /patrol/command 订阅者')
        if warnings:
            self.patrol_error = 'warning: ' + '；'.join(warnings)
        message = '已按手动流程发送巡逻启动命令'
        if warnings:
            message = f"{message}（{self.patrol_error}）"
        self.set_result('start_patrol_mode', True, message)

    def publish_patrol_command(self, command: str, request_id: str = '', route_id: str = '') -> None:
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
        }
        if route_id:
            payload['route_id'] = route_id
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.patrol_command_pub.publish(msg)

    def build_patrol_readiness(self) -> Dict[str, bool]:
        bringup = self.processes.get('bringup')
        navigation = self.processes.get('navigation')
        executor = self.processes.get('patrol_executor')
        readiness = {
            'bringup': bool(bringup and bringup.is_running()),
            'navigation': bool(navigation and navigation.is_running()),
            'executor': self.is_patrol_executor_ready(),
            'route_file': self.has_patrol_route_file(),
            'odom': self.topic_has_publishers('/odom'),
            'scan': self.topic_has_publishers('/scan'),
            'tf': self.topic_has_publishers('/tf'),
            'map': self.topic_has_publishers('/map'),
            'map_to_odom': self.has_map_to_odom(),
            'initialpose_subscribers': self.topic_has_subscribers('/initialpose'),
            'nav2_action': self.has_nav2_action(),
            'nav2_active': self.is_nav2_active(),
            'patrol_status': self.topic_has_publishers('/patrol/status'),
            'initial_pose_published': self.has_initial_pose_published(),
        }
        return readiness

    def build_light_patrol_readiness(self) -> Dict[str, bool]:
        bringup = self.processes.get('bringup')
        navigation = self.processes.get('navigation')
        executor = self.processes.get('patrol_executor')
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
            'nav2_action': False,
            'nav2_active': False,
            'patrol_status': False,
            'initial_pose_published': False,
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
            'nav2_action',
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
        return self.wait_for_readiness_keys(
            ('bringup', 'odom', 'scan', 'tf'),
            timeout_sec,
            error_prefix='底盘与传感器等待超时',
        )

    def wait_for_navigation_ready(self, timeout_sec: float = 35.0) -> bool:
        return self.wait_for_readiness_keys(
            ('navigation', 'map', 'initialpose_subscribers'),
            timeout_sec,
            error_prefix='导航等待超时',
        )

    def wait_for_nav2_action_ready(self, timeout_sec: float = 25.0) -> bool:
        return self.wait_for_readiness_keys(
            ('nav2_action',),
            timeout_sec,
            error_prefix='Nav2 动作服务未就绪',
        )

    def wait_for_map_to_odom(self, timeout_sec: float = 10.0) -> bool:
        return self.wait_for_readiness_keys(
            ('map_to_odom',),
            timeout_sec,
            error_prefix='等待 map->odom TF 超时',
        )

    def wait_for_nav2_active_ready(self, timeout_sec: float = 25.0) -> bool:
        return self.wait_for_readiness_keys(
            ('nav2_active',),
            timeout_sec,
            error_prefix='Nav2 lifecycle 未激活',
        )

    def wait_for_patrol_executor_ready(self, timeout_sec: float = 15.0) -> bool:
        return self.wait_for_readiness_keys(
            ('executor', 'route_file', 'patrol_status'),
            timeout_sec,
            error_prefix='巡逻执行器等待超时',
        )

    def wait_for_initial_pose_published(self, timeout_sec: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.has_initial_pose_published():
                self.patrol_error = ''
                return True
            self.startup_step = 'waiting_initial_pose_published'
            self.patrol_error = '等待巡逻执行器发布初始位姿'
            time.sleep(0.1)
        self.patrol_error = '等待初始位姿发布超时'
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
            return any(
                name.startswith('route_patrol_') and name.endswith('.json')
                for name in os.listdir(os.path.dirname(self.default_navigation_map))
            )
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

    def has_initial_pose_published(self) -> bool:
        event = getattr(self, 'last_initial_pose_event', {}) or {}
        return event.get('event') == 'initial_pose_published'

    def has_map_to_odom(self) -> bool:
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return False
        try:
            return bool(tf_buffer.can_transform('map', 'odom', Time()))
        except Exception:
            return False

    def is_nav2_active(self) -> bool:
        required_nodes = ('/bt_navigator', '/planner_server', '/controller_server')
        return all(self.lifecycle_node_is_active(name) for name in required_nodes)

    def lifecycle_node_is_active(self, node_name: str) -> bool:
        try:
            client = self.lifecycle_clients.get(node_name)
            if client is None:
                client = self.create_client(GetState, f'{node_name}/get_state')
                self.lifecycle_clients[node_name] = client
            if not client.wait_for_service(timeout_sec=0.1):
                return False
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                if future.done():
                    return future.result().current_state.label == 'active'
                time.sleep(0.02)
        except Exception:
            return False
        return False

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

    def log_patrol_start_readiness(self) -> None:
        try:
            readiness = self.build_patrol_readiness()
            fields = ('map', 'initial_pose_published', 'map_to_odom', 'nav2_active')
            summary = ', '.join(f'{key}={bool(readiness.get(key))}' for key in fields)
            self.log_info(f'patrol start readiness: {summary}')
        except Exception as exc:
            self.log_info(f'patrol start readiness unavailable: {exc}')

    def wait_for_patrol_command_subscriber(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.is_patrol_executor_ready():
                return True
            time.sleep(0.1)
        return False

    def wait_for_patrol_status_heartbeat(self, since: float, timeout_sec: float = 5.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
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
        self.publish_mode('fault')
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.set_result('emergency_stop', True, '软件急停已发送')

    def publish_mode(self, mode: str) -> None:
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

    def log_info(self, message: str) -> None:
        try:
            self.get_logger().info(message)
        except AttributeError:
            pass

    def publish_status(self) -> None:
        mobile_bridge = self.processes.get('mobile_bridge')
        self.mobile_bridge_http = mobile_bridge_tcp_status(
            bool(mobile_bridge and mobile_bridge.is_running())
        )
        with self.lock:
            self.publish_status_locked()

    def build_status_payload(self) -> Dict[str, Any]:
        payload = {
            'schema_version': '1.0',
            'timestamp': time.time(),
            'last_command': self.last_command,
            'success': self.last_success,
            'message': self.last_message,
        }
        for name, proc in self.processes.items():
            if name == 'llm' and self.embedded_task_layer:
                payload[name] = 'embedded'
            else:
                payload[name] = 'running' if proc.is_running() else 'stopped'
        patrol_state = getattr(self, 'patrol_mode_state', 'idle')
        if patrol_state in ('starting', 'command_sent', 'running'):
            patrol_readiness = self.build_patrol_readiness()
        else:
            patrol_readiness = self.build_light_patrol_readiness()
        payload.update({
            'mobile_bridge_http': self.mobile_bridge_http,
            'mobile_bridge_url': self.mobile_bridge_url,
            'jetson_ip': self.jetson_ip,
            'patrol_mode_state': patrol_state,
            'patrol_readiness': patrol_readiness,
            'patrol_error': getattr(self, 'patrol_error', ''),
            'startup_step': getattr(self, 'startup_step', ''),
            'startup_step_label': STARTUP_STEP_LABELS.get(
                getattr(self, 'startup_step', ''),
                getattr(self, 'startup_step', ''),
            ),
            'last_patrol_status': getattr(self, 'last_patrol_status', {}),
            'patrol_diagnostics': {
                'last_patrol_event': getattr(self, 'last_patrol_event', {}),
                'last_initial_pose_event': getattr(self, 'last_initial_pose_event', {}),
                'last_patrol_start_request_id': getattr(self, 'last_patrol_start_request_id', ''),
            },
        })
        return payload

    def publish_status_locked(self) -> None:
        payload = self.build_status_payload()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        for name in list(self.processes):
            try:
                self.stop_process(name)
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
