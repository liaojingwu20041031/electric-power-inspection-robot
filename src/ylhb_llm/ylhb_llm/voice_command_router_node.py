import json
import time
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ylhb_interfaces.msg import SayText, TaskStatus

from .ros_params import declare_string_array_parameter
from .voice_stability import VoiceIntent, VoiceRoutingPolicy, classify_voice_intent, normalize_voice_text


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class VoiceCommandRouterNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_command_router_node')
        self.declare_parameter('voice_command_event_topic', '/inspection_ai/voice_command_event')
        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('agent_request_topic', '/inspection_ai/agent_request')
        self.declare_parameter('agent_event_topic', '/inspection_ai/agent_event')
        self.declare_parameter('task_context_status_topic', '/inspection_ai/task_context_status')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('task_status_topic', '/inspection_ai/task_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        declare_string_array_parameter(self, 'motion_aliases')
        declare_string_array_parameter(self, 'system_commands')
        declare_string_array_parameter(self, 'voice_close_words')
        declare_string_array_parameter(self, 'safety_words')
        declare_string_array_parameter(self, 'cancel_words')
        declare_string_array_parameter(self, 'system_feedback_words')
        declare_string_array_parameter(self, 'general_qa_words')
        declare_string_array_parameter(self, 'inspection_words')
        declare_string_array_parameter(self, 'background_words')
        declare_string_array_parameter(self, 'followup_words')
        declare_string_array_parameter(self, 'incomplete_motion_words')
        self.declare_parameter('ignore_unknown_voice', True)
        self.declare_parameter('enable_inspection_agent', True)
        self.declare_parameter('publish_legacy_text_command', False)

        self.system_mode = 'ready'
        self.task_context: Dict[str, Any] = {}
        self.recent_utterances = set()
        self.recent_utterance_order = []
        self.motion_aliases = self.parse_motion_aliases([str(v) for v in self.get_parameter('motion_aliases').value])
        self.routing_policy = self.load_routing_policy()
        self.ignore_unknown_voice = bool(self.get_parameter('ignore_unknown_voice').value)
        self.enable_inspection_agent = bool(self.get_parameter('enable_inspection_agent').value)
        self.publish_legacy_text_command = bool(self.get_parameter('publish_legacy_text_command').value)

        self.text_pub = self.create_publisher(String, self.get_parameter('text_command_topic').value, 10)
        self.system_command_pub = self.create_publisher(String, self.get_parameter('system_command_topic').value, 10)
        self.agent_request_pub = self.create_publisher(String, self.get_parameter('agent_request_topic').value, 10)
        self.agent_event_pub = self.create_publisher(String, self.get_parameter('agent_event_topic').value, 10)
        self.say_pub = self.create_publisher(SayText, self.get_parameter('say_text_topic').value, 10)
        self.create_subscription(String, self.get_parameter('voice_command_event_topic').value, self.voice_event_callback, 10)
        self.create_subscription(String, self.get_parameter('task_context_status_topic').value, self.task_context_callback, transient_qos())
        self.create_subscription(String, self.get_parameter('system_mode_topic').value, self.system_mode_callback, transient_qos())
        self.create_subscription(TaskStatus, self.get_parameter('task_status_topic').value, self.task_status_callback, 10)
        self.get_logger().info('巡检语音命令路由节点已启动。')

    def voice_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid voice command event JSON: {exc}')
            return
        text = normalize_voice_text(str(event.get('text') or ''))
        if not text:
            return
        dedupe_key = f"{event.get('session_id') or ''}:{event.get('utterance_id') or ''}"
        if dedupe_key in self.recent_utterances:
            return
        self.remember_utterance(dedupe_key)

        interaction_phase = str(event.get('interaction_phase') or 'wake_command')
        intent = self.classify(text, interaction_phase)
        if intent.route == 'ignore':
            return
        if intent.route == 'voice_close':
            self.publish_text_command(intent.text, event, 'voice_close')
            self.say('voice_router', intent.feedback, priority=7, interrupt=True)
            return
        if intent.route == 'system_command':
            if intent.system_command == 'emergency_stop':
                self.publish_system_command(intent.system_command, intent.text, event)
                self.publish_agent_event('emergency_stop', True, 'executed', '语音急停直达 supervisor')
                self.say('voice_router', intent.feedback, priority=8, interrupt=True)
                return
            self.publish_agent_request(intent.text, event, intent.route, intent.system_command)
            if self.publish_legacy_text_command:
                self.publish_text_command(intent.text, event, intent.route)
            return
        if intent.route == 'unsupported_motion':
            self.say('voice_router', intent.feedback, priority=7)
            return
        if intent.route in ('global_safety', 'global_cancel', 'motion', 'inspection_command', 'general_qa', 'system_feedback'):
            if self.enable_inspection_agent:
                self.publish_agent_request(intent.text, event, intent.route)
                if self.publish_legacy_text_command:
                    self.publish_text_command(intent.text, event, intent.route)
            else:
                self.publish_text_command(intent.text, event, intent.route)
            if intent.feedback:
                self.say('voice_router', intent.feedback, priority=7, interrupt=intent.route == 'global_safety')
            return
        if self.enable_inspection_agent:
            self.publish_agent_request(intent.text, event, intent.route)
            if self.publish_legacy_text_command:
                self.publish_text_command(intent.text, event, intent.route)
        else:
            self.publish_text_command(intent.text, event, intent.route)

    def classify(self, text: str, interaction_phase: str) -> VoiceIntent:
        intent = classify_voice_intent(text, self.routing_policy, interaction_phase, self.ignore_unknown_voice)
        if intent.route == 'ignore' and interaction_phase != 'context_followup':
            custom_motion = self.normalize_custom_motion(text)
            if custom_motion:
                return VoiceIntent('motion', custom_motion)
        return intent

    def publish_text_command(self, text: str, event: Dict[str, Any], route: str) -> None:
        command = {
            'schema_version': '1.0',
            'source': 'voice',
            'route': route,
            'session_id': str(event.get('session_id') or ''),
            'utterance_id': str(event.get('utterance_id') or ''),
            'text': text,
            'raw_asr_text': str(event.get('raw_asr_text') or text),
            'awakened': bool(event.get('awakened')),
            'contains_wake_phrase': bool(event.get('contains_wake_phrase')),
            'interaction_phase': str(event.get('interaction_phase') or 'wake_command'),
            'confidence': float(event.get('confidence') or 0.0),
            'timestamp': float(event.get('timestamp') or time.time()),
        }
        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.text_pub.publish(msg)

    def publish_system_command(self, command: str, text: str, event: Dict[str, Any]) -> None:
        payload = {
            'schema_version': '1.0',
            'source': 'voice',
            'command': command,
            'session_id': str(event.get('session_id') or ''),
            'utterance_id': str(event.get('utterance_id') or ''),
            'text': text,
            'timestamp': float(event.get('timestamp') or time.time()),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.system_command_pub.publish(msg)

    def publish_agent_request(
        self,
        text: str,
        event: Dict[str, Any],
        route: str,
        legacy_system_command: str = '',
    ) -> None:
        payload = {
            'schema_version': '1.0',
            'source': 'voice',
            'route': route,
            'session_id': str(event.get('session_id') or ''),
            'utterance_id': str(event.get('utterance_id') or ''),
            'text': text,
            'raw_asr_text': str(event.get('raw_asr_text') or text),
            'interaction_phase': str(event.get('interaction_phase') or 'wake_command'),
            'confidence': float(event.get('confidence') or 0.0),
            'legacy_system_command': legacy_system_command,
            'timestamp': float(event.get('timestamp') or time.time()),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.agent_request_pub.publish(msg)

    def publish_agent_event(self, tool_name: str, ok: bool, status: str, message: str) -> None:
        payload = {
            'schema_version': '1.0',
            'tool_name': tool_name,
            'ok': ok,
            'status': status,
            'message': message,
            'data': {},
            'error_code': '',
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.agent_event_pub.publish(msg)

    def task_context_callback(self, msg: String) -> None:
        try:
            self.task_context = json.loads(msg.data)
        except json.JSONDecodeError:
            self.task_context = {}

    def system_mode_callback(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode:
            self.system_mode = mode

    def task_status_callback(self, _msg: TaskStatus) -> None:
        return

    def remember_utterance(self, key: str) -> None:
        self.recent_utterances.add(key)
        self.recent_utterance_order.append(key)
        while len(self.recent_utterance_order) > 100:
            old = self.recent_utterance_order.pop(0)
            self.recent_utterances.discard(old)

    def parse_motion_aliases(self, values: List[str]) -> List[Tuple[str, str]]:
        aliases: List[Tuple[str, str]] = []
        for value in values:
            alias, separator, canonical = value.partition(':')
            if separator and alias.strip() and canonical.strip():
                aliases.append((alias.strip(), canonical.strip()))
        aliases.sort(key=lambda item: len(item[0]), reverse=True)
        return aliases

    def load_routing_policy(self) -> VoiceRoutingPolicy:
        return VoiceRoutingPolicy(
            system_commands=self.parse_system_commands(self.string_list_parameter('system_commands')),
            voice_close_words=tuple(self.string_list_parameter('voice_close_words')),
            safety_words=tuple(self.string_list_parameter('safety_words')),
            cancel_words=tuple(self.string_list_parameter('cancel_words')),
            system_feedback_words=tuple(self.string_list_parameter('system_feedback_words')),
            general_qa_words=tuple(self.string_list_parameter('general_qa_words')),
            inspection_words=tuple(self.string_list_parameter('inspection_words')),
            background_words=tuple(self.string_list_parameter('background_words')),
            followup_words=tuple(self.string_list_parameter('followup_words')),
            motion_aliases=tuple(self.motion_aliases),
            incomplete_motion_words=tuple(self.string_list_parameter('incomplete_motion_words')),
        )

    def string_list_parameter(self, name: str) -> List[str]:
        return [str(value) for value in self.get_parameter(name).value if str(value)]

    def parse_system_commands(self, values: List[str]) -> Dict[str, Tuple[str, str]]:
        commands: Dict[str, Tuple[str, str]] = {}
        for value in values:
            parts = value.split('|', maxsplit=2)
            if len(parts) == 3 and all(part.strip() for part in parts):
                phrase, command, feedback = (part.strip() for part in parts)
                commands[phrase] = (command, feedback)
        return commands

    def normalize_custom_motion(self, text: str) -> str:
        for alias, canonical in self.motion_aliases:
            if alias == text or alias in text:
                return canonical
        return ''

    def say(self, task_id: str, text: str, priority: int = 5, interrupt: bool = False) -> None:
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.priority = int(priority)
        msg.interrupt = bool(interrupt)
        msg.text = text
        self.say_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceCommandRouterNode()
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
