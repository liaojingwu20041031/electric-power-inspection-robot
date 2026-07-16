import json
import queue
import threading
from types import SimpleNamespace

from ylhb_llm.agent_state import AgentState
from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.inspection_agent_node import InspectionAgentNode, decide_local
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator


class FakeRuntime:
    def __init__(self):
        self.calls = []

    def run_turn(self, request):
        self.calls.append(request)
        return {
            'decision': {
                'intent': 'assistant_chat',
                'tool_call': {'name': 'generate_local_status_reply', 'arguments': {}},
                'speak': {'text': '收到。'},
            },
            'result': {'ok': True, 'status': 'ok', 'message': '收到。'},
            'assistant_text': '收到。',
            'display_text': '屏幕显示完整回答。',
            'speech_text': '收到。',
            'role': 'assistant',
        }


class RaisingRuntime:
    def run_turn(self, _request):
        raise RuntimeError('DashScope HTTP 400: bad payload')


class WaitingRuntime(FakeRuntime):
    def __init__(self, operation_id):
        super().__init__()
        self.operation_id = operation_id
        self.resume_calls = []

    def run_turn(self, request):
        self.calls.append(request)
        return {
            'state': 'waiting_feedback',
            'pending_operation_id': self.operation_id,
            'decision': {'intent': 'rotate_relative', 'tool_call': {'name': 'rotate_relative'}, 'speak': {}},
            'result': {'ok': True, 'status': 'sent', 'message': '已发送'},
            'assistant_text': '等待真实反馈。',
            'role': 'tool',
        }

    def resume_turn(self, operation):
        self.resume_calls.append(operation)
        return {
            'state': 'finished',
            'decision': {'intent': 'assistant_chat', 'tool_call': {'name': 'generate_local_status_reply'}, 'speak': {}},
            'result': {'ok': True, 'status': 'ok', 'message': '已完成'},
            'assistant_text': '已根据真实反馈完成。',
            'role': 'assistant',
        }


class FakeTools:
    def __init__(self):
        self.events = []
        self.says = []

    def publish_event(self, payload):
        self.events.append(payload)

    def say(self, decision, priority=5, interrupt=False):
        self.says.append((decision, priority, interrupt))


class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def make_node(runtime=None, now=100.0):
    node = InspectionAgentNode.__new__(InspectionAgentNode)
    node.state = AgentState()
    node.tools = FakeTools()
    node.agent_runtime = runtime or FakeRuntime()
    node.chat_pub = FakePub()
    node.status_pub = FakePub()
    node.seen_request_ids = set()
    node.seen_request_order = __import__('collections').deque(maxlen=256)
    node.request_queue = queue.Queue()
    node.pending_turn_context = None
    node.pending_turn_lock = threading.Lock()
    node.operation_manager = AgentOperationManager(clock=lambda: now)
    node.status_aggregator = RobotStatusAggregator(clock=lambda: now)
    node.last_error_tts = {}
    node.agent_spec = SimpleNamespace(summary=lambda: {'name': 'inspection_agent'})
    node.publish_status = lambda: InspectionAgentNode.publish_status(node)
    node.logger_messages = []
    node.get_logger = lambda: SimpleNamespace(
        info=lambda msg: node.logger_messages.append(('info', msg)),
        error=lambda msg: node.logger_messages.append(('error', msg)),
    )
    node._now = now
    return node


def test_local_rules_only_cover_empty_stop_and_emergency_stop():
    assert decide_local({'text': ''}, {})['response_type'] == 'ignore'
    assert decide_local({'text': '急停'}, {})['tool_call']['name'] == 'emergency_stop'
    assert decide_local({'text': '停止'}, {})['tool_call']['name'] == 'stop_motion'
    assert decide_local({'text': '开始巡逻'}, {}) is None
    assert decide_local({'text': '你能够做什么'}, {}) is None


def test_request_callback_dedupes_client_msg_id_and_publishes_chat():
    runtime = FakeRuntime()
    node = make_node(runtime)
    msg = SimpleNamespace(data='{"text":"你好","client_msg_id":"c1"}')

    InspectionAgentNode.request_callback(node, msg)
    InspectionAgentNode.request_callback(node, msg)
    InspectionAgentNode.process_next_request(node)

    assert len(runtime.calls) == 1
    roles = [item['role'] for item in node.chat_pub.messages]
    assert roles == ['user', 'assistant']
    assert node.chat_pub.messages[-1]['text'] == '屏幕显示完整回答。'
    assert node.status_pub.messages[-1]['agent_spec_summary']['name'] == 'inspection_agent'


def test_request_id_is_used_for_voice_dedupe():
    runtime = FakeRuntime()
    node = make_node(runtime)

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"你好","request_id":"utt_1"}'))
    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"你好","request_id":"utt_1"}'))
    InspectionAgentNode.process_next_request(node)

    assert len(runtime.calls) == 1


def test_request_callback_returns_before_planner_runs():
    runtime = FakeRuntime()
    node = make_node(runtime)

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"查询状态","client_msg_id":"c3"}'))

    assert runtime.calls == []
    assert node.request_queue.qsize() == 1


def test_same_agent_error_tts_is_throttled_for_three_seconds(monkeypatch):
    values = iter([100.0, 101.0, 104.1])
    monkeypatch.setattr('time.monotonic', lambda: next(values))
    node = make_node()

    node.say_error_throttled('boom')
    node.say_error_throttled('boom')
    node.say_error_throttled('boom')

    assert len(node.tools.says) == 2


def test_planner_exception_publishes_real_error_to_chat_status_and_log():
    node = make_node(RaisingRuntime())

    InspectionAgentNode.request_callback(node, SimpleNamespace(data='{"text":"自我介绍一下","client_msg_id":"c2"}'))
    InspectionAgentNode.process_next_request(node)

    system_chat = node.chat_pub.messages[-1]
    assert system_chat['role'] == 'system'
    assert 'Planner 调用失败：RuntimeError: DashScope HTTP 400: bad payload' in system_chat['text']
    assert node.status_pub.messages[-1]['last_error'] == 'RuntimeError: DashScope HTTP 400: bad payload'
    assert ('error', 'agent planner error: RuntimeError: DashScope HTTP 400: bad payload') in node.logger_messages
    assert node.tools.says[0][0]['speak']['text'] == '语言模型暂不可用，未执行动作。'


def test_stopped_voice_turn_is_displayed_but_not_spoken():
    node = make_node()
    node.state.voice_status = {'enabled': False, 'state': 'OFF'}

    InspectionAgentNode.process_request(
        node, {'source': 'voice', 'text': '查询状态'}, 'utt_1', '')

    assert node.chat_pub.messages[-1]['text'] == '屏幕显示完整回答。'
    assert node.tools.says == []


def test_terminal_operation_feedback_requeues_and_resumes_same_turn():
    node = make_node()
    operation = node.operation_manager.create('run_1', 'call_1', 'rotate_relative', {}, 12.0)
    node.operation_manager.mark_sent(operation.operation_id)
    runtime = WaitingRuntime(operation.operation_id)
    node.agent_runtime = runtime

    InspectionAgentNode.process_request(node, {'text': '左转十度'}, 'turn_1', 'client_1')
    InspectionAgentNode.update_operation_from_feedback(node, {
        'operation_id': operation.operation_id,
        'state': 'succeeded',
        'message': '底盘已完成转向',
    })
    InspectionAgentNode.process_next_request(node)

    assert len(runtime.resume_calls) == 1
    assert runtime.resume_calls[0]['state'] == 'succeeded'
    assert node.chat_pub.messages[-1]['text'] == '已根据真实反馈完成。'


def test_emergency_request_uses_entry_fast_path_without_queueing():
    runtime = FakeRuntime()
    node = make_node(runtime)

    InspectionAgentNode.request_callback(
        node, SimpleNamespace(data='{"text":"急停","client_msg_id":"urgent_1"}'))

    assert len(runtime.calls) == 1
    assert node.request_queue.empty()
