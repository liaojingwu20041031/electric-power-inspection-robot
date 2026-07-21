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


def test_local_rules_only_cover_empty_input():
    assert decide_local({'text': ''}, {})['response_type'] == 'ignore'
    assert decide_local({'text': '急停'}, {}) is None
    assert decide_local({'text': '停止'}, {}) is None
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


def test_patrol_running_finishes_start_route_but_not_checkpoint_navigation():
    node = make_node()
    start = node.operation_manager.create('run_1', 'call_1', 'start_route', {}, 120.0)
    target = node.operation_manager.create('run_2', 'call_2', 'go_to_checkpoint', {}, 120.0)
    node.operation_manager.mark_sent(start.operation_id)
    node.operation_manager.mark_sent(target.operation_id)

    InspectionAgentNode.update_operation_from_feedback(node, {
        'operation_id': start.operation_id, 'state': 'running',
    })
    InspectionAgentNode.update_operation_from_feedback(node, {
        'operation_id': target.operation_id, 'state': 'running',
    })

    assert node.operation_manager.get(start.operation_id)['state'] == 'succeeded'
    assert node.operation_manager.get(target.operation_id)['state'] == 'running'


def test_voice_status_nested_feedback_completes_voice_session_operation():
    node = make_node()
    operation = node.operation_manager.create(
        'run_1', 'call_1', 'end_voice_conversation', {}, 2.0)
    node.operation_manager.mark_sent(operation.operation_id)

    InspectionAgentNode.voice_status_callback(node, SimpleNamespace(data=json.dumps({
        'state': 'WAIT_WAKE',
        'agent_operation_feedback': {
            'operation_id': operation.operation_id,
            'state': 'succeeded',
            'message': '已结束当前对话',
        },
    })))

    assert node.operation_manager.get(operation.operation_id)['state'] == 'succeeded'


def test_emergency_request_uses_same_planner_queue_as_other_tools():
    runtime = FakeRuntime()
    node = make_node(runtime)

    InspectionAgentNode.request_callback(
        node, SimpleNamespace(data='{"text":"急停","client_msg_id":"urgent_1"}'))

    assert runtime.calls == []
    assert node.request_queue.qsize() == 1


def test_plain_text_chassis_status_preserves_offline_state():
    node = make_node()

    InspectionAgentNode.chassis_status_callback(
        node, SimpleNamespace(data='offline heartbeat_age=-1 feedback_age=-1'))

    assert node.status_aggregator.get('chassis_status')['state'] == 'offline'


def test_status_snapshot_preserves_full_system_status_and_is_mode_aware():
    aggregator = RobotStatusAggregator(clock=lambda: 10.0)
    aggregator.update('system_status', {
        'system_mode': 'ready', 'navigation': 'stopped',
        'patrol_mode_state': 'idle', 'custom_supervisor_field': {'value': 1},
    }, now=10.0)

    snapshot = aggregator.snapshot()
    summary = aggregator.mode_aware_summary()

    assert snapshot['system_status']['custom_supervisor_field'] == {'value': 1}
    assert snapshot['system_status']['fresh'] is True
    assert summary['health'] == 'ok'


def test_new_status_callbacks_preserve_fault_app_cloud_and_imu():
    node = make_node()

    InspectionAgentNode.chassis_fault_callback(node, SimpleNamespace(data='undervoltage'))
    InspectionAgentNode.local_app_status_callback(node, SimpleNamespace(data='{"enabled":false}'))
    InspectionAgentNode.cloud_status_callback(node, SimpleNamespace(data='{"enabled":true,"connected":false}'))
    InspectionAgentNode.imu_callback(node, SimpleNamespace())

    assert node.status_aggregator.raw('chassis_fault')['text'] == 'undervoltage'
    assert node.status_aggregator.raw('local_app_status')['enabled'] is False
    assert node.status_aggregator.raw('cloud_status')['connected'] is False
    assert node.status_aggregator.get('imu')['fresh'] is True


def test_health_monitor_debounces_issue_and_does_not_auto_recover_by_default():
    now = [0.0]
    node = make_node()
    node.enable_agent_health_monitor = True
    node.enable_agent_auto_recovery = False
    node.health_issue_debounce_sec = 5.0
    node.health_clock = lambda: now[0]
    node.health_issue_first_seen = {}
    node.health_published_incidents = set()
    node.health_status_pub = FakePub()
    node.diagnostic_event_pub = FakePub()
    node.system_pub = FakePub()
    node.diagnostic_engine = SimpleNamespace(run_self_check=lambda _scope: {
        'diagnostic_id': 'diag1', 'overall': 'warning',
        'issues': [{'code': 'PERCEPTION_STOPPED', 'component': 'perception', 'recoverable': True, 'recovery_component': 'perception'}],
    })

    InspectionAgentNode.health_monitor_tick(node)
    now[0] = 6.0
    InspectionAgentNode.health_monitor_tick(node)
    InspectionAgentNode.health_monitor_tick(node)

    assert len(node.diagnostic_event_pub.messages) == 1
    assert node.system_pub.messages == []


def test_health_monitor_auto_recovery_uses_operation_and_high_level_command_once():
    now = [100.0]
    node = make_node(now=100.0)
    node.enable_agent_health_monitor = True
    node.enable_agent_auto_recovery = True
    node.health_issue_debounce_sec = 0.0
    node.health_recovery_cooldown_sec = 60.0
    node.health_clock = lambda: now[0]
    node.health_issue_first_seen = {'PERCEPTION_STOPPED:perception': 90.0}
    node.health_published_incidents = set()
    node.health_auto_recovered_incidents = set()
    node.health_last_recovery_at = {}
    node.health_status_pub = FakePub()
    node.diagnostic_event_pub = FakePub()
    node.system_pub = FakePub()
    node.recovery_catalog = SimpleNamespace(
        names=lambda: ['perception'],
        get=lambda _name: {
            'auto_allowed': True, 'requires_no_active_patrol': True,
            'cooldown_sec': 60.0, 'timeout_sec': 25.0,
        },
    )
    report = {
        'diagnostic_id': 'diag1', 'overall': 'warning',
        'issues': [{'code': 'PERCEPTION_STOPPED', 'component': 'perception', 'recoverable': True, 'recovery_component': 'perception'}],
    }
    node.diagnostic_engine = SimpleNamespace(run_self_check=lambda _scope: report)

    InspectionAgentNode.health_monitor_tick(node)
    InspectionAgentNode.health_monitor_tick(node)

    assert len(node.system_pub.messages) == 1
    assert node.system_pub.messages[0]['command'] == 'recover_component'
    assert node.system_pub.messages[0]['diagnostic_id'] == 'diag1'
    assert len(node.operation_manager.list_active(now=100.0)) == 1
