import json

from std_msgs.msg import String

from ylhb_llm.voice_command_router_node import VoiceCommandRouterNode
from ylhb_llm.voice_stability import VoiceRoutingPolicy


class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def make_router():
    node = VoiceCommandRouterNode.__new__(VoiceCommandRouterNode)
    node.recent_utterances = set()
    node.recent_utterance_order = []
    node.ignore_unknown_voice = True
    node.enable_inspection_agent = True
    node.publish_legacy_text_command = False
    node.motion_aliases = [('前进', '前进')]
    node.routing_policy = VoiceRoutingPolicy(
        system_commands={'急停': ('emergency_stop', '已发送紧急停止命令。')},
        safety_words=('停止',),
        general_qa_words=('你能做什么',),
        inspection_words=('开始巡检',),
        motion_aliases=(('前进', '前进'),),
        incomplete_motion_words=('旋转',),
    )
    node.text_pub = FakePub()
    node.system_command_pub = FakePub()
    node.agent_request_pub = FakePub()
    node.agent_event_pub = FakePub()
    node.say_messages = []
    node.say = lambda task_id, text, priority=5, interrupt=False: node.say_messages.append((task_id, text, priority, interrupt))
    return node


def voice_msg(text):
    msg = String()
    msg.data = json.dumps({
        'text': text,
        'raw_asr_text': text,
        'session_id': 's1',
        'utterance_id': text,
        'interaction_phase': 'wake_command',
        'confidence': 0.8,
        'timestamp': 123.0,
    }, ensure_ascii=False)
    return msg


def test_normal_voice_goes_to_agent_request_without_speaking_or_legacy_text():
    node = make_router()

    node.voice_event_callback(voice_msg('开始巡检'))

    assert node.agent_request_pub.messages[0]['text'] == '开始巡检'
    assert node.agent_request_pub.messages[0]['input_type'] == 'voice'
    assert node.text_pub.messages == []
    assert node.say_messages == []


def test_emergency_stop_direct_path_publishes_system_command_and_agent_event():
    node = make_router()

    node.voice_event_callback(voice_msg('急停'))

    assert node.system_command_pub.messages[0]['command'] == 'emergency_stop'
    assert node.agent_event_pub.messages[0]['tool_name'] == 'emergency_stop'
    assert node.agent_event_pub.messages[0]['status'] == 'sent'
    assert node.say_messages == []


def test_unsupported_motion_only_speaks_hint():
    node = make_router()

    node.voice_event_callback(voice_msg('旋转'))

    assert node.agent_request_pub.messages == []
    assert node.say_messages[0][1] == '请说左转或右转。'
