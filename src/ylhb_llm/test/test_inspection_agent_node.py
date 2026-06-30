import json
from types import SimpleNamespace

from ylhb_llm.agent_schema import validate_decision
from ylhb_llm.agent_state import AgentState
from ylhb_llm.inspection_agent_node import InspectionAgentNode, decide_local


class FakeQwen:
    def __init__(self, raw='', available=True):
        self.raw = raw
        self._available = available
        self.calls = 0

    def available(self):
        return self._available

    def chat_completion(self, **_kwargs):
        self.calls += 1
        return self.raw


class FakeTools:
    def __init__(self):
        self.events = []
        self.says = []
        self.executed = []

    def execute(self, decision, policy):
        self.executed.append(decision)
        return {'status': 'sent', 'tool_name': decision['tool_call']['name']}

    def publish_event(self, payload):
        self.events.append(payload)

    def say(self, decision, priority=5, interrupt=False):
        self.says.append((decision, priority, interrupt))


def make_node(qwen):
    node = InspectionAgentNode.__new__(InspectionAgentNode)
    node.state = AgentState()
    node.qwen = qwen
    node.chat_model = 'fake'
    node.request_timeout_sec = 1.0
    node.enable_llm_fallback = True
    node.llm_json_retry_count = 0
    node.tools = FakeTools()
    node.publish_status = lambda: None
    node.get_logger = lambda: SimpleNamespace(info=lambda _msg: None)
    return node


def decision_json(response_type='tool_call', tool='generate_local_status_reply', command=''):
    data = {
        'schema_version': '1.0',
        'decision_id': 'd1',
        'response_type': response_type,
        'intent': 'mock',
        'safety_level': 'normal',
        'tool_call': {'name': tool, 'arguments': {}},
        'speak': {'reply_key': 'mock', 'text': '收到。', 'priority': 5, 'interrupt': False},
        'final_answer': '收到。',
        'need_confirm': False,
        'reason_cn': 'mock',
    }
    if command:
        data['tool_call']['arguments'] = {'command': command}
    return json.dumps(data, ensure_ascii=False)


def test_local_rules_only_cover_empty_and_emergency_stop():
    assert validate_decision(decide_local({'text': ''}, {}))['response_type'] == 'ignore'
    emergency = validate_decision(decide_local({'text': '急停'}, {}))
    assert emergency['tool_call']['name'] == 'emergency_stop'
    assert emergency['speak']['interrupt'] is True
    assert decide_local({'text': '开始巡逻'}, {}) is None
    assert decide_local({'text': '后退'}, {}) is None
    assert decide_local({'text': '你能够做什么'}, {}) is None


def test_llm_final_answer_is_spoken():
    node = make_node(FakeQwen(decision_json('final_answer')))

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"你能够做什么"}'))

    assert node.tools.executed[0]['response_type'] == 'final_answer'
    assert node.tools.says


def test_llm_start_patrol_executes_system_tool():
    node = make_node(FakeQwen(decision_json(tool='start_patrol_mode')))

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"开始巡逻"}'))

    assert node.tools.executed[0]['tool_call']['name'] == 'start_patrol_mode'


def test_llm_motion_uses_structured_motion_tool():
    node = make_node(FakeQwen(decision_json(tool='send_motion_command', command='后退')))

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"后退"}'))

    assert node.tools.executed[0]['tool_call'] == {
        'name': 'send_motion_command',
        'arguments': {'command': '后退'},
    }


def test_llm_unavailable_publishes_event_and_says_without_tool_execution():
    node = make_node(FakeQwen(available=False))

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"后退"}'))

    assert node.tools.executed == []
    assert node.tools.events[0]['ok'] is False
    assert node.tools.says[0][0]['speak']['text'] == '语言模型暂不可用，未执行动作。'
