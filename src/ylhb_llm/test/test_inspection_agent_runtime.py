import json
from types import SimpleNamespace

from ylhb_llm.agent_state import AgentState
from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.inspection_agent_runtime import InspectionAgentRuntime
from ylhb_llm.inspection_agent_spec import InspectionAgentSpecBuilder
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator
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
        'generate_local_status_reply': {
            'properties': {}, 'required': [], 'model_visible': True,
            'risk_level': 'read_only',
        },
        'get_robot_summary': {'properties': {}, 'required': [], 'risk_level': 'read_only'},
        'start_patrol_mode': {'properties': {}, 'required': [], 'model_visible': False},
        'start_component': {
            'properties': {'component': {'type': 'string', 'enum': ['bringup']}},
            'required': ['component'],
            'side_effect': 'component_lifecycle',
            'risk_level': 'system_control',
            'result_schema': {'status': ['started', 'already_running', 'failed', 'timeout']},
        },
        'stop_component': {
            'properties': {'component': {'type': 'string', 'enum': ['bringup']}},
            'required': ['component'],
            'model_visible': False,
            'side_effect': 'component_lifecycle',
            'risk_level': 'system_control',
            'preconditions': ['started_this_turn'],
            'result_schema': {'status': ['stopped', 'already_stopped', 'failed', 'timeout']},
        },
        'end_voice_conversation': {
            'properties': {}, 'required': [], 'side_effect': 'voice_session_control',
            'suppress_speech_after_call': True,
        },
        'close_voice_mode': {
            'properties': {}, 'required': [], 'side_effect': 'voice_session_control',
            'suppress_speech_after_call': True,
        },
        'emergency_stop': {'properties': {}, 'required': [], 'side_effect': 'robot_stop'},
        'cancel_patrol': {'properties': {}, 'required': [], 'side_effect': 'patrol_cancel'},
        'stop_motion': {'properties': {}, 'required': [], 'side_effect': 'robot_stop'},
        'rotate_relative': {
            'properties': {'angle_deg': {'type': 'number', 'minimum': -180, 'maximum': 180}},
            'required': ['angle_deg'], 'side_effect': 'robot_motion',
        },
        'move_relative': {
            'properties': {'distance_m': {'type': 'number', 'minimum': -0.5, 'maximum': 0.5}},
            'required': ['distance_m'], 'side_effect': 'robot_motion',
        },
    }
    pubs = SimpleNamespace(system=FakePub(), motion=FakePub(), say=FakePub(), event=FakePub(), patrol=FakePub(), base=FakePub(), voice=FakePub())
    state = AgentState()
    aggregator = RobotStatusAggregator(default_max_age_sec=60.0, clock=lambda: 10.0)
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
        status_aggregator=aggregator,
        voice_session_pub=pubs.voice,
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


def test_voice_interaction_metadata_is_passed_to_planner_context():
    qwen = FakeQwen(final_response('请再说明目标。'))
    runtime, _pubs = make_runtime(qwen)

    runtime.run_turn({
        'text': '回来',
        'interaction_phase': 'context_followup',
        'contains_wake_phrase': True,
    })

    context = next(
        item['content'] for item in qwen.requests[0]['messages']
        if str(item.get('content') or '').startswith('INTERNAL_INTERACTION_CONTEXT ')
    )
    assert '"interaction_phase":"context_followup"' in context
    assert '"contains_wake_phrase":true' in context


def test_planner_disabled_returns_system_without_tool():
    runtime, pubs = make_runtime(FakeQwen(available=False), enabled=False)

    result = runtime.run_turn({'text': '能够做什么'})

    assert result['role'] == 'system'
    assert '不可用' in result['assistant_text']
    assert pubs.event.messages == []


def test_rotate_relative_tool_call_executes_base_skill():
    runtime, pubs = make_runtime(FakeQwen(tool_response('rotate_relative', {'angle_deg': 180})))
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)

    runtime.run_turn({'text': '转个一百八十度'})

    assert runtime.planner.requests[0]['tool_choice'] == 'required'
    assert pubs.base.messages[0]['command'] == 'rotate_relative'
    assert pubs.base.messages[0]['arguments']['angle_deg'] == 180
    assert pubs.motion.messages == []


def test_go_to_checkpoint_alias_is_normalized():
    runtime, pubs = make_runtime(FakeQwen(tool_response('go_to_checkpoint', {'target_id': '巡检点3'})))

    runtime.run_turn({'text': '去巡检点3'})

    assert pubs.system.messages[0]['command'] == 'go_to_checkpoint'
    assert pubs.system.messages[0]['target_id'] == 'target_003'
    assert pubs.patrol.messages == []


def test_emergency_stop_is_selected_by_model_contract():
    qwen = FakeQwen(tool_response('emergency_stop', {}))
    runtime, pubs = make_runtime(qwen)
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)

    runtime.run_turn({'text': '急停'})

    assert qwen.calls == 1
    assert pubs.system.messages[0]['command'] == 'emergency_stop'


def test_stop_motion_is_not_guessed_when_planner_unavailable():
    qwen = FakeQwen(available=False)
    runtime, pubs = make_runtime(qwen, enabled=False)

    runtime.run_turn({'text': 'OK行停止'})

    assert qwen.calls == 0
    assert pubs.base.messages == []


def test_tool_call_message_and_result_keep_server_call_id():
    qwen = FakeQwen([
        tool_response('get_system_status', {}, call_id='call_status_1'),
        final_response('状态已读取。'),
    ])
    runtime, _pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '机器人状态怎么样？'})

    assert result['assistant_text'] == '状态已读取。'
    messages = qwen.requests[1]['messages']
    assistant = next(item for item in messages if item.get('tool_calls'))
    tool = next(item for item in messages if item.get('role') == 'tool')
    assert assistant['tool_calls'][0]['id'] == 'call_status_1'
    assert tool['tool_call_id'] == 'call_status_1'
    assert tool['name'] == 'get_system_status'


def test_nonrecoverable_tool_failure_finishes_without_starting_components():
    qwen = FakeQwen([
        tool_response('go_to_checkpoint', {'target_id': 'target_003'}, call_id='call_move_1'),
        tool_response('get_system_status', {}, call_id='call_status_2'),
        final_response('导航工具不可用，已查询系统状态。'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['go_to_checkpoint']['preconditions'] = ['no_active_patrol']
    runtime.state.patrol_status = {'state': 'running'}

    result = runtime.run_turn({'text': '去巡检点3'})

    assert qwen.calls == 1
    assert result['result']['error_code'] == 'precondition_failed'
    assert pubs.system.messages == []


def test_side_effect_waits_for_feedback_before_model_can_repeat_it():
    qwen = FakeQwen(tool_response(
        'rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'))
    runtime, pubs = make_runtime(qwen)
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)

    result = runtime.run_turn({'text': '左转十度'})

    assert len(pubs.base.messages) == 1
    assert qwen.calls == 1
    assert result['state'] == 'waiting_feedback'


def test_repeated_status_queries_are_allowed_while_state_is_changing():
    qwen = FakeQwen([
        tool_response('get_robot_summary', {}, call_id='summary_1'),
        tool_response('get_robot_summary', {}, call_id='summary_2'),
        tool_response('get_robot_summary', {}, call_id='summary_3'),
        final_response('状态查询完成。'),
    ])
    runtime, _pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '持续确认机器人当前状态'})

    assert qwen.calls == 4
    assert result['assistant_text'] == '状态查询完成。'


def test_operation_feedback_refreshes_summary_when_schema_requests_it():
    qwen = FakeQwen(final_response('恢复结果已确认。'))
    runtime, _pubs = make_runtime(qwen)
    runtime.tool_schemas['recover_component'] = {
        'properties': {'component': {'type': 'string'}}, 'required': ['component'],
        'side_effect': 'component_recovery', 'refresh_summary_after': True,
    }
    runtime.pending_turn = {
        'pending_operation_id': 'op1',
        'pending_call': SimpleNamespace(id='call1', name='recover_component', arguments={'component': 'perception'}),
        'request': {'text': '恢复感知'}, 'run_id': 'run1', 'request_id': 'req1',
        'tool_results': [], 'decision': {}, 'side_effect_tools': 1,
        'previous_call_key': '', 'identical_call_count': 0, 'steps_used': 1,
        'required_retry_used': False, 'force_tool_once': False, 'started_components': [],
        'progress_announced': False, 'preparation_seen': False,
        'target_tool': 'recover_component', 'terminal_side_effect_tools': [],
    }

    runtime.resume_turn({
        'operation_id': 'op1', 'tool_name': 'recover_component', 'state': 'succeeded',
        'arguments': {'component': 'perception'}, 'result': {'message': '恢复成功'},
    })

    assert any(
        str(message.get('content') or '').startswith('INTERNAL_FRESH_ROBOT_SUMMARY')
        for message in runtime.messages
    )


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


def test_realtime_status_uses_real_tool_evidence():
    qwen = FakeQwen([
        tool_response('get_robot_summary', {}, call_id='summary'),
        final_response('机器人状态已查询。'),
    ])
    runtime, _pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '机器人当前状态怎么样？'})

    assert qwen.calls == 2
    assert qwen.requests[0]['tool_choice'] == 'required'
    assert result['result']['ok'] is True
    assert result['assistant_text'] == '机器人状态已查询。'


def test_action_sent_waits_for_terminal_feedback_before_planning_continues():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'),
        tool_response('rotate_relative', {'angle_deg': 9}, call_id='call_rotate_duplicate'),
        final_response('已根据真实反馈完成转向。'),
    ])
    runtime, _pubs = make_runtime(qwen)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})

    assert qwen.calls == 1
    assert waiting['state'] == 'waiting_feedback'
    assert waiting['speech_text'] == '已开始执行，等待结果。'
    operation_id = waiting['pending_operation_id']
    manager.update(operation_id, 'succeeded', {
        'ok': True,
        'status': 'succeeded',
        'message': '底盘已完成转向',
        'operation_id': operation_id,
    }, now=11.0)

    finished = runtime.resume_turn(manager.get(operation_id, now=11.0))

    assert qwen.calls == 3
    assert finished['state'] == 'finished'
    assert qwen.requests[2]['tool_choice'] == 'auto'
    assert finished['assistant_text'] == '已根据真实反馈完成转向。'
    assert len(_pubs.base.messages) == 1
    tool_message = [
        message for message in qwen.requests[2]['messages']
        if message.get('role') == 'tool'
    ]
    assert tool_message[-2]['tool_call_id'] == 'call_rotate_1'
    assert json.loads(tool_message[-2]['content'])['status'] == 'succeeded'
    assert json.loads(tool_message[-1]['content'])['error_code'] == 'duplicate_terminal_action'


def test_waiting_progress_is_announced_only_once_per_turn():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='rotate_before_start'),
        tool_response('start_component', {'component': 'bringup'}, call_id='start_bringup'),
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='rotate_after_start'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['rotate_relative']['preconditions'] = [
        'bringup_running', 'chassis_online', 'sensor_fresh',
    ]
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    first = runtime.run_turn({'text': '准备后左转十度', 'run_id': 'run_1'})
    first_id = first['pending_operation_id']
    manager.update(first_id, 'succeeded', {
        'ok': True, 'status': 'succeeded', 'result_status': 'started',
        'operation_id': first_id,
    }, now=10.0)
    runtime.state.system_status = {'bringup': 'running'}
    for source, payload in (
        ('system_status', {'bringup': 'running'}),
        ('chassis_status', {'state': 'online'}),
        ('scan', {'state': 'ok'}),
        ('odom', {'state': 'ok'}),
    ):
        runtime.tools.status_aggregator.update(source, payload, now=10.0)

    second = runtime.resume_turn(manager.get(first_id, now=10.0))

    assert first['speech_text'] == '已开始执行，等待结果。'
    assert second['state'] == 'waiting_feedback'
    assert second['speech_text'] == ''
    assert second['decision']['speak'] == {}
    assert qwen.calls == 3
    summaries = [
        item for item in qwen.requests[-1]['messages']
        if str(item.get('content') or '').startswith('INTERNAL_FRESH_ROBOT_SUMMARY ')
    ]
    assert len(summaries) == 2
    first_tools = {
        item['function']['name'] for item in qwen.requests[0]['tools']
    }
    recovery_tools = {
        item['function']['name'] for item in qwen.requests[1]['tools']
    }
    assert 'start_component' not in first_tools
    assert 'start_component' in recovery_tools
    assert [item['command'] for item in pubs.system.messages] == ['start_bringup']
    assert pubs.base.messages[-1]['command'] == 'rotate_relative'


def test_voice_end_intent_is_a_silent_tool_command():
    qwen = FakeQwen([
        tool_response('end_voice_conversation', {}, call_id='end_voice'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)

    waiting = runtime.run_turn({'text': '那不聊了', 'source': 'voice', 'run_id': 'run_1'})

    assert waiting['state'] == 'waiting_feedback'
    assert waiting['speech_text'] == ''
    assert waiting['decision']['speak'] == {}
    assert pubs.voice.messages[-1]['command'] == 'end_voice_conversation'

    operation_id = waiting['pending_operation_id']
    runtime.tools.operation_manager.update(operation_id, 'succeeded', {
        'ok': True, 'status': 'succeeded', 'result_status': 'succeeded',
        'message': '已结束当前对话', 'operation_id': operation_id,
    }, now=10.0)

    finished = runtime.resume_turn(runtime.tools.operation_manager.get(operation_id, now=10.0))

    assert qwen.calls == 1
    assert finished['state'] == 'finished'
    assert finished['assistant_text'] == '已结束当前对话'


def test_out_of_range_motion_is_rejected_before_any_component_start():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 360}, call_id='rotate'),
        tool_response('start_component', {'component': 'bringup'}, call_id='start_bringup'),
    ])
    runtime, pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '转三百六十度', 'run_id': 'run_1'})

    assert qwen.calls == 1
    assert result['result']['error_code'] == 'invalid_tool_arguments'
    assert pubs.system.messages == []
    assert pubs.base.messages == []


def test_out_of_range_long_move_is_not_split_or_prepared():
    qwen = FakeQwen([
        tool_response('move_relative', {'distance_m': -20}, call_id='move'),
        tool_response('start_component', {'component': 'bringup'}, call_id='start_bringup'),
    ])
    runtime, pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '后退二十米', 'run_id': 'run_1'})

    assert qwen.calls == 1
    assert result['result']['status'] == 'rejected'
    assert result['result']['error_code'] == 'invalid_tool_arguments'
    assert pubs.system.messages == []
    assert pubs.base.messages == []


def test_failed_motion_preconditions_only_allow_reported_recovery_component():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='rotate'),
        tool_response('start_component', {'component': 'perception'}, call_id='start_perception'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['rotate_relative']['preconditions'] = [
        'bringup_running', 'chassis_online', 'sensor_fresh',
    ]

    result = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})

    assert qwen.calls == 2
    assert result['result']['error_code'] == 'invalid_tool_arguments'
    assert pubs.system.messages == []


def test_running_bringup_with_offline_chassis_stops_without_guessing_other_components():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='rotate'),
        tool_response('start_component', {'component': 'navigation'}, call_id='start_navigation'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['rotate_relative']['preconditions'] = [
        'bringup_running', 'chassis_online', 'sensor_fresh',
    ]
    runtime.tools.status_aggregator.update('system_status', {'bringup': 'running'}, now=10.0)

    result = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})

    assert qwen.calls == 1
    assert result['result']['error_code'] == 'precondition_failed'
    assert result['result']['data']['recovery_components'] == []
    assert pubs.system.messages == []


def test_cancel_patrol_uses_model_tool_contract():
    qwen = FakeQwen(tool_response('cancel_patrol', {}))
    runtime, pubs = make_runtime(qwen)
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.state.patrol_status = {'state': 'running'}

    result = runtime.run_turn({'text': '取消巡逻'})

    assert qwen.calls == 1
    assert pubs.system.messages[0]['command'] == 'cancel_patrol'
    assert result['result']['status'] == 'sent'


def test_pending_base_motion_only_exposes_stop_motion_and_targets_active_goal():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='call_rotate_1'),
        tool_response('stop_motion', {}, call_id='call_stop_1'),
    ])
    runtime, pubs = make_runtime(qwen)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager
    waiting = runtime.run_turn({'text': '左转十度', 'run_id': 'run_1'})
    runtime.run_turn({'text': '停止'})

    stop_payload = pubs.base.messages[-1]
    assert stop_payload['operation_id'] != waiting['pending_operation_id']
    assert stop_payload['target_operation_id'] == waiting['pending_operation_id']


def test_openai_tools_hide_internal_adapters_and_explain_contract_metadata():
    runtime, _pubs = make_runtime(FakeQwen())
    runtime.tool_schemas['rotate_relative'].update({
        'description': '相对旋转机器人',
        'preconditions': ['bringup_running', 'chassis_online', 'sensor_fresh'],
        'side_effect': 'robot_motion',
        'risk_level': 'reversible_motion',
        'timeout_sec': 12.0,
        'result_schema': {'status': ['accepted', 'running', 'succeeded', 'failed', 'timeout']},
    })

    tools = {item['function']['name']: item['function'] for item in runtime.openai_tools()}

    assert 'start_patrol_mode' not in tools
    assert 'generate_local_status_reply' in tools
    assert 'stop_component' not in tools
    assert 'start_route' in tools
    assert tools['start_component']['parameters']['properties']['component']['enum'] == ['bringup']
    description = tools['rotate_relative']['description']
    for text in ('什么时候使用', '不要使用', '前置条件', 'bringup_running', '结果状态', 'running', '重新查询'):
        assert text in description


def test_component_start_is_not_action_evidence_and_terminal_auto_stops_owned_component():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 10}, call_id='rotate_before_start'),
        tool_response('start_component', {'component': 'bringup'}, call_id='start_bringup'),
        final_response('bringup 已启动。'),
        final_response('bringup 已启动。'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['rotate_relative']['preconditions'] = [
        'bringup_running', 'chassis_online', 'sensor_fresh',
    ]
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '启动底盘后左转十度', 'run_id': 'run_1'})
    start_id = waiting['pending_operation_id']
    manager.update(start_id, 'succeeded', {
        'ok': True, 'status': 'succeeded', 'result_status': 'started',
        'message': 'bringup 已启动', 'operation_id': start_id,
    }, now=10.0)

    cleaning = runtime.resume_turn(manager.get(start_id, now=10.0))
    stop_id = cleaning['pending_operation_id']
    manager.update(stop_id, 'succeeded', {
        'ok': True, 'status': 'succeeded', 'result_status': 'stopped',
        'message': 'bringup 已停止', 'operation_id': stop_id,
    }, now=10.0)

    finished = runtime.resume_turn(manager.get(stop_id, now=10.0))

    assert finished['result']['error_code'] == 'missing_tool_evidence'
    assert [item['command'] for item in pubs.system.messages] == [
        'start_bringup', 'stop_bringup',
    ]
    assert pubs.base.messages == []
    assert runtime.state.state == 'finished'
    assert all('回收本轮启动组件' not in step['summary'] for step in runtime.state.steps)


def test_component_already_running_is_not_added_to_owned_stop_ledger():
    context = {'started_components': []}

    InspectionAgentRuntime._update_component_ledger(
        context,
        SimpleNamespace(name='start_component', arguments={'component': 'bringup'}),
        {'status': 'succeeded', 'data': {'component_status': 'already_running'}},
    )

    assert context['started_components'] == []


def test_terminal_operation_result_keeps_original_tool_arguments():
    result = InspectionAgentRuntime._operation_result({
        'operation_id': 'op_rotate',
        'tool_name': 'rotate_relative',
        'arguments': {'angle_deg': 30},
        'state': 'succeeded',
        'result': {'ok': True, 'status': 'succeeded', 'message': '旋转完成'},
    })

    assert result['data']['arguments'] == {'angle_deg': 30}


def test_direct_component_start_without_recovery_credential_is_rejected():
    qwen = FakeQwen(tool_response('start_component', {'component': 'bringup'}))
    runtime, pubs = make_runtime(qwen)

    result = runtime.run_turn({'text': '启动底盘'})

    assert result['result']['status'] == 'rejected'
    assert result['result']['error_code'] == 'invalid_tool_arguments'
    assert pubs.system.messages == []


def test_first_planning_step_does_not_expose_component_start():
    qwen = FakeQwen(final_response('请说明动作。'))
    runtime, _pubs = make_runtime(qwen)

    runtime.run_turn({'text': '左旋转三十度'})

    tools = {item['function']['name'] for item in qwen.requests[0]['tools']}
    assert 'rotate_relative' in tools
    assert 'start_component' not in tools


def test_unknown_route_is_a_rejected_tool_result_not_agent_error():
    runtime, pubs = make_runtime(FakeQwen(
        tool_response('start_route', {'route_id': '不存在路线'})))

    result = runtime.run_turn({'text': '巡逻不存在路线'})

    assert result['result']['status'] == 'rejected'
    assert result['result']['error_code'] == 'invalid_tool_arguments'
    assert pubs.system.messages == []


def test_first_target_tool_stays_locked_after_component_recovery():
    qwen = FakeQwen([
        tool_response('rotate_relative', {'angle_deg': 60}, call_id='rotate'),
        tool_response('start_component', {'component': 'bringup'}, call_id='start'),
        tool_response('move_relative', {'distance_m': 0.1}, call_id='changed_target'),
    ])
    runtime, pubs = make_runtime(qwen)
    runtime.tool_schemas['rotate_relative']['preconditions'] = ['bringup_running']
    runtime.tools.operation_manager = AgentOperationManager(clock=lambda: 10.0)

    first = runtime.run_turn({'text': '左转六十度'})
    operation_id = first['pending_operation_id']
    runtime.tools.operation_manager.update(operation_id, 'succeeded', {
        'result_status': 'already_running', 'operation_id': operation_id,
    })

    result = runtime.resume_turn(runtime.tools.operation_manager.get(operation_id))

    assert result['result']['status'] == 'rejected'
    assert result['result']['error_code'] == 'target_tool_changed'
    assert pubs.base.messages == []
