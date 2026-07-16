import json
from types import SimpleNamespace

from ylhb_llm.agent_state import AgentState
from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.inspection_agent_runtime import InspectionAgentRuntime
from ylhb_llm.inspection_agent_spec import InspectionAgentSpecBuilder
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


class FakeQwen:
    def __init__(self, response=None, available=True):
        self.response = response or {
            'message': {'role': 'assistant', 'content': '我可以巡逻和查询状态。'},
            'content': '我可以巡逻和查询状态。',
            'tool_calls': [],
        }
        self._available = available
        self.calls = 0
        self.requests = []

    def available(self):
        return self._available

    def chat_tools(self, **kwargs):
        self.calls += 1
        self.requests.append(kwargs)
        if isinstance(self.response, list):
            return self.response.pop(0)
        return self.response


class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def make_runtime(qwen, enabled=True):
    route_toolpack = RouteToolPack(RouteCatalog({
        'routes': [{'id': 'route_1', 'name': '路线', 'target_ids': ['target_003']}],
        'targets': [{'id': 'target_003', 'name': '巡检点3'}],
    }))
    schemas = {
        **route_toolpack.tool_schemas(),
        'generate_local_status_reply': {'properties': {}, 'required': []},
        'get_robot_summary': {'properties': {}, 'required': [], 'risk_level': 'read_only'},
        'emergency_stop': {'properties': {}, 'required': []},
        'cancel_patrol': {'properties': {}, 'required': []},
        'stop_motion': {'properties': {}, 'required': []},
        'rotate_relative': {'properties': {'angle_deg': {'type': 'number', 'minimum': -180, 'maximum': 180}}, 'required': ['angle_deg']},
    }
    pubs = SimpleNamespace(system=FakePub(), motion=FakePub(), say=FakePub(), event=FakePub(), patrol=FakePub(), base=FakePub())
    state = AgentState()
    tools = AgentTools(
        SimpleNamespace(),
        state,
        pubs.system,
        pubs.motion,
        pubs.say,
        pubs.event,
        patrol_pub=pubs.patrol,
        base_skill_pub=pubs.base,
        route_toolpack=route_toolpack,
        tool_schemas=schemas,
    )
    spec = InspectionAgentSpecBuilder(route_toolpack, schemas, tools.registry).build()
    runtime = InspectionAgentRuntime(qwen, tools, state, spec, schemas, route_toolpack=route_toolpack, enabled=enabled)
    return runtime, pubs


def tool_response(name, arguments, call_id='call_1'):
    message = {
        'role': 'assistant',
        'content': '',
        'tool_calls': [{
            'id': call_id,
            'type': 'function',
            'function': {'name': name, 'arguments': json.dumps(arguments, ensure_ascii=False)},
        }],
    }
    return {'message': message, 'content': '', 'tool_calls': message['tool_calls']}


def final_response(content='已完成。'):
    return {'message': {'role': 'assistant', 'content': content}, 'content': content, 'tool_calls': []}


def test_capability_question_uses_llm_when_enabled():
    runtime, pubs = make_runtime(FakeQwen())

    result = runtime.run_turn({'text': '能够做什么'})

    assert result['role'] == 'assistant'
    assert '巡逻' in result['assistant_text']
    assert pubs.base.messages == []


def test_planner_disabled_returns_system_without_tool():
    runtime, pubs = make_runtime(FakeQwen(available=False), enabled=False)

    result = runtime.run_turn({'text': '能够做什么'})

    assert result['role'] == 'system'
    assert '不可用' in result['assistant_text']
    assert '巡逻状态' in result['assistant_text']
    assert pubs.event.messages == []


def test_rotate_relative_tool_call_executes_base_skill():
    runtime, pubs = make_runtime(FakeQwen(tool_response('rotate_relative', {'angle_deg': 180})))

    runtime.run_turn({'text': '转个一百八十度'})

    assert pubs.base.messages[0]['command'] == 'rotate_relative'
    assert pubs.base.messages[0]['arguments']['angle_deg'] == 180
    assert pubs.motion.messages == []


def test_go_to_checkpoint_alias_is_normalized():
    runtime, pubs = make_runtime(FakeQwen(tool_response('go_to_checkpoint', {'target_id': '巡检点3'})))

    runtime.run_turn({'text': '去巡检点3'})

    assert pubs.patrol.messages[0]['target_id'] == 'target_003'


def test_emergency_stop_does_not_call_llm():
    qwen = FakeQwen()
    runtime, pubs = make_runtime(qwen)

    runtime.run_turn({'text': '急停'})

    assert qwen.calls == 0
    assert pubs.system.messages[0]['command'] == 'emergency_stop'


def test_stop_motion_does_not_call_llm_when_planner_unavailable():
    qwen = FakeQwen(available=False)
    runtime, pubs = make_runtime(qwen, enabled=False)

    runtime.run_turn({'text': '停止'})

    assert qwen.calls == 0
    assert pubs.base.messages[0]['command'] == 'stop_motion'


def test_tool_call_message_and_result_keep_server_call_id():
    qwen = FakeQwen([
        tool_response('get_system_status', {}, call_id='call_status_1'),
        final_response('状态已读取。'),
    ])
    runtime, _pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '机器人状态怎么样？'})

    assert result['assistant_text'] == '状态已读取。'
    messages = qwen.requests[1]['messages']
    assert messages[1]['tool_calls'][0]['id'] == 'call_status_1'
    assert messages[2]['role'] == 'tool'
    assert messages[2]['tool_call_id'] == 'call_status_1'
    assert messages[2]['name'] == 'get_system_status'


def test_tool_failure_is_returned_to_model_for_followup_status_query():
    qwen = FakeQwen([
        tool_response('go_to_checkpoint', {'target_id': 'target_003'}, call_id='call_move_1'),
        tool_response('get_system_status', {}, call_id='call_status_2'),
        final_response('导航工具不可用，已查询系统状态。'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tools.patrol_pub = None

    result = runtime.run_turn({'text': '去巡检点3'})

    assert qwen.calls == 3
    assert result['assistant_text'] == '导航工具不可用，已查询系统状态。'
    assert pubs.patrol.messages == []


def test_third_identical_side_effect_call_is_blocked():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'),
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_2'),
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_3'),
    ])
    runtime, pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '左转十度'})

    assert len(pubs.base.messages) == 2
    assert result['result']['error_code'] == 'repeated_tool_call'


def test_runtime_forwards_run_and_tool_call_id_to_operation_manager():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'),
        final_response(),
    ])
    runtime, _pubs = make_runtime(qwen)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})

    operation = manager.list_active(now=10.0)[0]
    assert operation['run_id'] == 'run_1'
    assert operation['tool_call_id'] == 'call_rotate_1'


def test_full_display_answer_uses_short_speech_answer():
    answer = '## 状态\n' + ('设备运行正常，' * 20) + '完整数据保留在屏幕。'
    runtime, _pubs = make_runtime(FakeQwen(final_response(answer)))

    result = runtime.run_turn({'text': '汇报详细状态'})

    assert result['display_text'] == answer
    assert result['assistant_text'] == answer
    assert len(result['speech_text']) <= 60
    assert result['decision']['speak']['text'] == result['speech_text']


def test_history_keeps_six_complete_turns_and_tool_pairs():
    runtime, _pubs = make_runtime(FakeQwen())
    runtime.history_max_turns = 6
    runtime.history_max_chars = 10_000
    messages = []
    for index in range(8):
        messages.extend([
            {'role': 'user', 'content': f'user-{index}'},
            {'role': 'assistant', 'content': '', 'tool_calls': [{'id': f'call-{index}'}]},
            {'role': 'tool', 'tool_call_id': f'call-{index}', 'content': f'result-{index}'},
            {'role': 'assistant', 'content': f'answer-{index}'},
        ])
    runtime.messages = messages

    runtime.trim_history()

    users = [item['content'] for item in runtime.messages if item['role'] == 'user']
    assert users == [f'user-{index}' for index in range(2, 8)]
    assert runtime.messages[1]['tool_calls'][0]['id'] == 'call-2'
    assert runtime.messages[2]['tool_call_id'] == 'call-2'


def test_history_character_limit_drops_oldest_complete_turn_only():
    runtime, _pubs = make_runtime(FakeQwen())
    runtime.history_max_turns = 6
    runtime.history_max_chars = 40
    runtime.messages = [
        {'role': 'user', 'content': 'old-user'},
        {'role': 'assistant', 'content': 'old-answer' * 4},
        {'role': 'user', 'content': 'new-user'},
        {'role': 'assistant', 'content': 'new-answer'},
    ]

    runtime.trim_history()

    assert runtime.messages == [
        {'role': 'user', 'content': 'new-user'},
        {'role': 'assistant', 'content': 'new-answer'},
    ]


def test_realtime_status_requires_real_tool_evidence_once():
    qwen = FakeQwen([
        final_response('机器人状态正常。'),
        final_response('机器人状态还是正常。'),
    ])
    runtime, _pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '机器人当前状态怎么样？'})

    assert qwen.calls == 2
    assert qwen.requests[0]['tool_choice'] == 'auto'
    assert qwen.requests[1]['tool_choice'] == 'required'
    assert result['result']['ok'] is False
    assert result['result']['error_code'] == 'missing_tool_evidence'
    assert '真实工具结果' in result['assistant_text']


def test_action_sent_waits_for_terminal_feedback_before_planning_continues():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'),
        final_response('已根据真实反馈完成转向。'),
    ])
    runtime, _pubs = make_runtime(qwen)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})

    assert qwen.calls == 1
    assert waiting['state'] == 'waiting_feedback'
    operation_id = waiting['pending_operation_id']
    manager.update(operation_id, 'succeeded', {
        'ok': True,
        'status': 'succeeded',
        'message': '底盘已完成转向',
        'operation_id': operation_id,
    }, now=11.0)

    finished = runtime.resume_turn(manager.get(operation_id, now=11.0))

    assert qwen.calls == 2
    assert finished['state'] == 'finished'
    assert qwen.requests[1]['tool_choice'] == 'auto'
    assert finished['assistant_text'] == '已根据真实反馈完成转向。'
    tool_message = [
        message for message in qwen.requests[1]['messages']
        if message.get('role') == 'tool'
    ][-1]
    assert tool_message['tool_call_id'] == 'call_rotate_1'
    assert json.loads(tool_message['content'])['status'] == 'succeeded'


def test_cancel_patrol_uses_local_fast_path_without_planner():
    qwen = FakeQwen()
    runtime, pubs = make_runtime(qwen)
    runtime.state.patrol_status = {'state': 'running'}

    result = runtime.run_turn({'text': '取消巡逻'})

    assert qwen.calls == 0
    assert pubs.system.messages[0]['command'] == 'cancel_patrol'
    assert result['result']['status'] == 'sent'


def test_fast_cancel_uses_independent_operation_and_targets_active_goal():
    qwen = FakeQwen(tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'))
    runtime, pubs = make_runtime(qwen)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager
    waiting = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})
    runtime.state.patrol_status = {'state': 'running'}

    runtime.run_turn({'text': '取消巡逻'})

    cancel_payload = pubs.system.messages[-1]
    assert cancel_payload['operation_id'] != waiting['pending_operation_id']
    assert cancel_payload['target_operation_id'] == waiting['pending_operation_id']
