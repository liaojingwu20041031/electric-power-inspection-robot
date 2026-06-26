import json
import time
from typing import Any, Dict

from geometry_msgs.msg import Twist
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
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
    taskContext = pyqtSignal(dict)
    taskEvent = pyqtSignal(object)
    taskStatus = pyqtSignal(object)
    sayText = pyqtSignal(object)
    voiceStatus = pyqtSignal(object)
    localizedObjects = pyqtSignal(str)
    patrolStatus = pyqtSignal(dict)
    patrolEvent = pyqtSignal(dict)


class InspectionDisplayRosBridge(Node):
    def __init__(self, signals: UiSignals) -> None:
        super().__init__('inspection_display_ui_node')
        self.signals = signals
        parameters = {
            'text_command_topic': '/inspection_ai/text_command',
            'system_mode_topic': '/inspection_ai/system_mode',
            'system_command_topic': '/inspection_ai/system_command',
            'system_status_topic': '/inspection_ai/system_status',
            'task_event_topic': '/inspection_ai/task_event',
            'task_status_topic': '/inspection_ai/task_status',
            'say_text_topic': '/inspection_ai/say_text',
            'task_context_status_topic': '/inspection_ai/task_context_status',
            'voice_status_topic': '/inspection_ai/voice_status',
            'start_voice_session_service_name': '/inspection_ai/start_voice_session',
            'stop_voice_session_service_name': '/inspection_ai/stop_voice_session',
            'capture_voice_service_name': '/inspection_ai/capture_voice',
            'localized_objects_topic': '/perception/localized_objects',
            'patrol_status_topic': '/patrol/status',
            'patrol_event_topic': '/patrol/event',
            'patrol_command_topic': '/patrol/command',
            'cmd_vel_topic': '/cmd_vel',
            'initial_system_mode': 'ready',
            'fullscreen': True,
            'display': ':0',
            'force_local_display': True,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.text_pub = self.create_publisher(String, self._param('text_command_topic'), 10)
        self.system_mode_pub = self.create_publisher(String, self._param('system_mode_topic'), latched_qos())
        self.system_command_pub = self.create_publisher(String, self._param('system_command_topic'), 10)
        self.patrol_command_pub = self.create_publisher(String, self._param('patrol_command_topic'), 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self._param('cmd_vel_topic'), 10)
        self.create_subscription(String, self._param('system_status_topic'), self._system_status, latched_qos())
        self.create_subscription(String, self._param('task_context_status_topic'), self._task_context, latched_qos())
        self.create_subscription(TaskEvent, self._param('task_event_topic'), signals.taskEvent.emit, 10)
        self.create_subscription(TaskStatus, self._param('task_status_topic'), signals.taskStatus.emit, 10)
        self.create_subscription(SayText, self._param('say_text_topic'), signals.sayText.emit, 10)
        self.create_subscription(VoiceStatus, self._param('voice_status_topic'), signals.voiceStatus.emit, 10)
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
        self.voice_clients = {
            'start': self.create_client(Trigger, self._param('start_voice_session_service_name')),
            'stop': self.create_client(Trigger, self._param('stop_voice_session_service_name')),
            'capture': self.create_client(Trigger, self._param('capture_voice_service_name')),
        }
        self.publish_system_mode(str(self.get_parameter('initial_system_mode').value))

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

    def _task_context(self, msg: String) -> None:
        self.signals.taskContext.emit(self.parse_json(msg.data))

    def _localized_objects(self, msg: String) -> None:
        self.signals.localizedObjects.emit(msg.data)

    def _patrol_status(self, msg: String) -> None:
        self.signals.patrolStatus.emit(self.parse_json(msg.data))

    def _patrol_event(self, msg: String) -> None:
        self.signals.patrolEvent.emit(self.parse_json(msg.data))

    def publish_text_command(self, text: str, source: str = 'ui') -> None:
        self._publish_json(self.text_pub, {
            'schema_version': '1.0', 'source': source, 'text': text, 'timestamp': time.time(),
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
        if client.service_is_ready():
            client.call_async(Trigger.Request())

    @staticmethod
    def _publish_json(publisher, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)
