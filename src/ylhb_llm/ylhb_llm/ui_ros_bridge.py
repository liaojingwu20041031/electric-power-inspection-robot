import json
import threading
import time
from typing import Any, Dict

from geometry_msgs.msg import Twist
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from ylhb_mobile_bridge.patrol_qos import patrol_status_qos_profile

from ylhb_interfaces.msg import SayText, TaskEvent, TaskStatus, VoiceStatus


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class UiSignals(QObject):
    systemStatus = pyqtSignal(dict)
    localAppStatus = pyqtSignal(dict)
    cloudStatus = pyqtSignal(dict)
    bridgeAvailability = pyqtSignal(dict)
    localAppControlResult = pyqtSignal(bool, bool, str)
    cloudControlResult = pyqtSignal(bool, bool, str)
    taskContext = pyqtSignal(dict)
    taskEvent = pyqtSignal(object)
    taskStatus = pyqtSignal(object)
    sayText = pyqtSignal(object)
    voiceStatus = pyqtSignal(object)
    voiceSessionStatus = pyqtSignal(dict)
    voiceServiceResult = pyqtSignal(str, bool, str)
    agentStatus = pyqtSignal(dict)
    agentEvent = pyqtSignal(dict)
    agentChat = pyqtSignal(dict)
    localizedObjects = pyqtSignal(str)
    patrolStatus = pyqtSignal(dict)
    patrolEvent = pyqtSignal(dict)
    mapping3dStatus = pyqtSignal(dict)
    mapping3dResult = pyqtSignal(dict)


class InspectionDisplayRosBridge(Node):
    def __init__(self, signals: UiSignals) -> None:
        super().__init__('inspection_display_ui_node')
        self.signals = signals
        parameters = {
            'text_command_topic': '/inspection_ai/text_command',
            'agent_request_topic': '/inspection_ai/agent_request',
            'system_mode_topic': '/inspection_ai/system_mode',
            'system_command_topic': '/inspection_ai/system_command',
            'system_status_topic': '/inspection_ai/system_status',
            'cloud_status_topic': '/mobile_bridge/cloud_status',
            'set_cloud_enabled_service_name': '/mobile_bridge/set_cloud_enabled',
            'local_app_status_topic': '/mobile_bridge/local_app_status',
            'set_local_app_enabled_service_name': '/mobile_bridge/set_local_app_enabled',
            'task_event_topic': '/inspection_ai/task_event',
            'task_status_topic': '/inspection_ai/task_status',
            'say_text_topic': '/inspection_ai/say_text',
            'task_context_status_topic': '/inspection_ai/task_context_status',
            'voice_status_topic': '/inspection_ai/voice_status',
            'voice_session_status_topic': '/inspection_ai/voice_session_status',
            'agent_status_topic': '/inspection_ai/agent_status',
            'agent_event_topic': '/inspection_ai/agent_event',
            'agent_chat_topic': '/inspection_ai/agent_chat',
            'start_voice_session_service_name': '/inspection_ai/start_voice_session',
            'stop_voice_session_service_name': '/inspection_ai/stop_voice_session',
            'capture_voice_service_name': '/inspection_ai/capture_voice',
            'localized_objects_topic': '/perception/localized_objects',
            'patrol_status_topic': '/patrol/status',
            'patrol_event_topic': '/patrol/event',
            'patrol_command_topic': '/patrol/command',
            'mapping3d_status_topic': '/inspection_ai/mapping3d_status',
            'mapping3d_result_topic': '/inspection_ai/mapping3d_result',
            'cmd_vel_topic': '/cmd_vel',
            'initial_system_mode': 'ready',
            'fullscreen': True,
            'display': ':0',
            'force_local_display': True,
            'ui_safe_margin_left': 28,
            'ui_safe_margin_right': 28,
            'ui_safe_margin_top': 24,
            'ui_safe_margin_bottom': 28,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.text_pub = self.create_publisher(String, self._param('text_command_topic'), 10)
        self.agent_request_pub = self.create_publisher(String, self._param('agent_request_topic'), 10)
        self.system_mode_pub = self.create_publisher(String, self._param('system_mode_topic'), latched_qos())
        self.system_command_pub = self.create_publisher(String, self._param('system_command_topic'), 10)
        self.patrol_command_pub = self.create_publisher(String, self._param('patrol_command_topic'), 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self._param('cmd_vel_topic'), 10)
        self.create_subscription(String, self._param('system_status_topic'), self._system_status, latched_qos())
        self.create_subscription(String, self._param('local_app_status_topic'), self._local_app_status, latched_qos())
        self.create_subscription(String, self._param('cloud_status_topic'), self._cloud_status, latched_qos())
        self.create_subscription(String, self._param('task_context_status_topic'), self._task_context, latched_qos())
        self.create_subscription(TaskEvent, self._param('task_event_topic'), signals.taskEvent.emit, 10)
        self.create_subscription(TaskStatus, self._param('task_status_topic'), signals.taskStatus.emit, 10)
        self.create_subscription(SayText, self._param('say_text_topic'), signals.sayText.emit, 10)
        self.create_subscription(VoiceStatus, self._param('voice_status_topic'), signals.voiceStatus.emit, 10)
        self.create_subscription(String, self._param('voice_session_status_topic'), self._voice_session_status, latched_qos())
        self.create_subscription(String, self._param('agent_status_topic'), self._agent_status, latched_qos())
        self.create_subscription(String, self._param('agent_event_topic'), self._agent_event, 10)
        self.create_subscription(String, self._param('agent_chat_topic'), self._agent_chat, 10)
        self.create_subscription(String, self._param('localized_objects_topic'), self._localized_objects, 10)
        self.create_subscription(
            String,
            self._param('patrol_status_topic'),
            self._patrol_status,
            patrol_status_qos_profile(),
        )
        self.create_subscription(
            String,
            self._param('patrol_event_topic'),
            self._patrol_event,
            patrol_status_qos_profile(),
        )
        self.create_subscription(String, self._param('mapping3d_status_topic'), self._mapping3d_status, latched_qos())
        self.create_subscription(String, self._param('mapping3d_result_topic'), self._mapping3d_result, latched_qos())
        self.voice_clients = {
            'start': self.create_client(Trigger, self._param('start_voice_session_service_name')),
            'stop': self.create_client(Trigger, self._param('stop_voice_session_service_name')),
            'capture': self.create_client(Trigger, self._param('capture_voice_service_name')),
        }
        self.cloud_enabled_client = self.create_client(SetBool, self._param('set_cloud_enabled_service_name'))
        self.local_app_enabled_client = self.create_client(SetBool, self._param('set_local_app_enabled_service_name'))
        self.create_timer(1.0, self.publish_bridge_availability)
        self.publish_system_mode(str(self.get_parameter('initial_system_mode').value))

    def publish_bridge_availability(self) -> None:
        try:
            payload = {
                'cloudServiceReady': bool(self.cloud_enabled_client.service_is_ready()),
                'localAppServiceReady': bool(self.local_app_enabled_client.service_is_ready()),
                'cloudStatusPublishers': len(self.get_publishers_info_by_topic(self._param('cloud_status_topic'))),
                'localAppStatusPublishers': len(self.get_publishers_info_by_topic(self._param('local_app_status_topic'))),
                'checkedAt': time.time(),
            }
        except Exception as exc:
            payload = {
                'cloudServiceReady': False,
                'localAppServiceReady': False,
                'cloudStatusPublishers': 0,
                'localAppStatusPublishers': 0,
                'checkedAt': time.time(),
                'error': type(exc).__name__,
            }
        self.signals.bridgeAvailability.emit(payload)

    def _param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    @staticmethod
    def parse_json(text: str) -> Dict[str, Any]:
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {'value': value}
        except json.JSONDecodeError:
            return {'raw': text}

    def _system_status(self, msg: String) -> None:
        self.signals.systemStatus.emit(self.parse_json(msg.data))

    def _cloud_status(self, msg: String) -> None:
        self.signals.cloudStatus.emit(self.parse_json(msg.data))

    def _local_app_status(self, msg: String) -> None:
        self.signals.localAppStatus.emit(self.parse_json(msg.data))

    def _task_context(self, msg: String) -> None:
        self.signals.taskContext.emit(self.parse_json(msg.data))

    def _localized_objects(self, msg: String) -> None:
        self.signals.localizedObjects.emit(msg.data)

    def _patrol_status(self, msg: String) -> None:
        self.signals.patrolStatus.emit(self.parse_json(msg.data))

    def _patrol_event(self, msg: String) -> None:
        self.signals.patrolEvent.emit(self.parse_json(msg.data))

    def _mapping3d_status(self, msg: String) -> None:
        self.signals.mapping3dStatus.emit(self.parse_json(msg.data))

    def _mapping3d_result(self, msg: String) -> None:
        self.signals.mapping3dResult.emit(self.parse_json(msg.data))

    def _agent_status(self, msg: String) -> None:
        self.signals.agentStatus.emit(self.parse_json(msg.data))

    def _agent_event(self, msg: String) -> None:
        self.signals.agentEvent.emit(self.parse_json(msg.data))

    def _agent_chat(self, msg: String) -> None:
        self.signals.agentChat.emit(self.parse_json(msg.data))

    def _voice_session_status(self, msg: String) -> None:
        self.signals.voiceSessionStatus.emit(self.parse_json(msg.data))

    def publish_text_command(self, text: str, source: str = 'ui') -> None:
        self._publish_json(self.text_pub, {
            'schema_version': '1.0', 'source': source, 'text': text, 'timestamp': time.time(),
        })

    def publish_agent_request(self, text: str, client_msg_id: str = '', source: str = 'ui') -> None:
        self._publish_json(self.agent_request_pub, {
            'schema_version': '1.0',
            'source': source,
            'text': text,
            'client_msg_id': client_msg_id,
            'input_type': 'text',
            'timestamp': time.time(),
        })

    def publish_system_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.system_mode_pub.publish(msg)

    def publish_system_command(self, command: str, **extra: Any) -> None:
        payload = {
            'schema_version': '1.0', 'command': command, 'source': 'ui', 'timestamp': time.time(),
        }
        payload.update(extra)
        self._publish_json(self.system_command_pub, payload)

    def publish_patrol_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.patrol_command_pub.publish(msg)

    def publish_twist(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def call_voice_service(self, name: str) -> None:
        client = self.voice_clients[name]
        if not client.service_is_ready():
            self.signals.voiceServiceResult.emit(name, False, '语音服务不可用')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done, service_name=name: self._voice_service_done(service_name, done))

    def call_cloud_enabled(self, enabled: bool) -> None:
        if not self.cloud_enabled_client.service_is_ready():
            self.signals.cloudControlResult.emit(
                bool(enabled), False, '云平台控制服务不可用'
            )
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.cloud_enabled_client.call_async(request)
        self._watch_set_bool(
            future,
            bool(enabled),
            self.signals.cloudControlResult,
            self._cloud_control_done,
            '云平台控制请求超时',
        )

    def call_local_app_enabled(self, enabled: bool) -> None:
        if not self.local_app_enabled_client.service_is_ready():
            self.signals.localAppControlResult.emit(
                bool(enabled), False, '本地 APP 控制服务不可用'
            )
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.local_app_enabled_client.call_async(request)
        self._watch_set_bool(
            future,
            bool(enabled),
            self.signals.localAppControlResult,
            self._local_app_control_done,
            '本地 APP 控制请求超时',
        )

    @staticmethod
    def _watch_set_bool(
        future, enabled: bool, signal, done_callback, timeout_message: str
    ) -> None:
        lock = threading.Lock()
        finished = False

        def finish(success: bool, message: str) -> None:
            nonlocal finished
            with lock:
                if finished:
                    return
                finished = True
            signal.emit(enabled, success, message)

        timer = threading.Timer(5.0, lambda: finish(False, timeout_message))
        timer.daemon = True
        timer.start()
        future.add_done_callback(
            lambda done: done_callback(done, finish, timer)
        )

    @staticmethod
    def _local_app_control_done(future, finish, timer) -> None:
        timer.cancel()
        try:
            result = future.result()
            finish(bool(result.success), str(result.message))
        except Exception as exc:
            finish(False, str(exc))

    @staticmethod
    def _cloud_control_done(future, finish, timer) -> None:
        timer.cancel()
        try:
            result = future.result()
            finish(bool(result.success), str(result.message))
        except Exception as exc:
            finish(False, str(exc))

    def _voice_service_done(self, name: str, future) -> None:
        try:
            result = future.result()
            self.signals.voiceServiceResult.emit(name, bool(result.success), str(result.message))
        except Exception as exc:
            self.signals.voiceServiceResult.emit(name, False, str(exc))

    @staticmethod
    def _publish_json(publisher, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)
