import json
import os
import signal
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def workspace_path(*parts: str) -> str:
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_ws'))
    return os.path.join(workspace_dir, *parts)


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
        self.declare_parameter('workspace_dir', os.environ.get('WS_DIR', os.path.expanduser('~/ros2_ws')))
        self.declare_parameter('ros_distro', 'humble')
        self.declare_parameter('map_output_dir', workspace_path('src', 'maps'))
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
        }

        self.status_pub = self.create_publisher(
            String, self.get_parameter('system_status_topic').value, latched_qos())
        self.mode_pub = self.create_publisher(
            String, self.get_parameter('system_mode_topic').value, latched_qos())
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            String,
            self.get_parameter('system_command_topic').value,
            self.command_callback,
            10,
        )
        self.create_timer(1.0, self.publish_status)
        self.publish_status()
        self.get_logger().info('System supervisor node started.')

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

        threading.Thread(target=self.handle_command, args=(command, payload), daemon=True).start()

    def handle_command(self, command: str, payload: Dict[str, Any]) -> None:
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
                self.set_result_locked(f'start_{name}', True, f'{name} already running')
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
            self.set_result_locked(f'start_{name}', True, f'{name} started')

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

    def start_robot_stack(self) -> None:
        for name in ('bringup', 'zed', 'perception', 'navigation', 'llm'):
            self.start_process(name)
            time.sleep(0.3)
        self.set_result('start_robot_stack', True, '巡检节点启动命令已发送')

    def stop_robot_stack(self) -> None:
        for name in ('navigation', 'perception', 'zed', 'bringup'):
            self.stop_process(name)
            time.sleep(0.2)
        self.publish_mode('ready')
        self.set_result('stop_robot_stack', True, '巡检运动、导航和感知节点已停止，AI/UI 保持运行')

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
        self.get_logger().info(f'{command}: {message}')
        self.publish_status_locked()

    def publish_status(self) -> None:
        with self.lock:
            self.publish_status_locked()

    def publish_status_locked(self) -> None:
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
