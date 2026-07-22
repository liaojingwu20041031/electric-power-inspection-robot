import json
from types import SimpleNamespace

import pytest

from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.agent_policy import authorize
from ylhb_llm.agent_state import AgentState
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.inspection_agent_runtime import (
    AgentTurnContext,
    FinalResponse,
    InspectionAgentRuntime,
    Observation,
    RuntimeLease,
    ToolCall,
)
from ylhb_llm.inspection_agent_spec import InspectionAgentSpecBuilder
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


class FakePlanner:
    def __init__(self, responses=None, available=True):
        self.responses = list(responses or [final_response('我可以巡逻和查询状态。')])
        self._available = available
        self.calls = 0
        self.requests = []

    def available(self):
        return self._available

    def chat_tools(self, **kwargs):
        self.calls += 1
        self.requests.append(kwargs)
        response = self.responses.pop(0) if self.responses else final_response('无法继续执行。')
        response = json.loads(json.dumps(response, ensure_ascii=False))
        message = response.get('message') or {}
        final_call = next((
            item for item in message.get('tool_calls') or []
            if (item.get('function') or {}).get('name') == 'submit_final_response'
        ), None)
        if final_call is not None:
            arguments = (final_call.get('function') or {}).get('arguments') or '{}'
            arguments = json.loads(arguments) if isinstance(arguments, str) else dict(arguments)
            if arguments.get('evidence_refs') == 'AUTO':
                observations = self._observations(kwargs)
                arguments['evidence_refs'] = [item['observation_id'] for item in observations]
                if arguments.get('task_state') == 'AUTO':
                    arguments['task_state'] = self._task_state(observations)
                final_call['function']['arguments'] = json.dumps(arguments, ensure_ascii=False)
            return response
        try:
            final = json.loads(str(message.get('content') or ''))
        except json.JSONDecodeError:
            return response
        if final.get('evidence_refs') != 'AUTO':
            return response
        observations = self._observations(kwargs)
        final['evidence_refs'] = [item['observation_id'] for item in observations]
        if final.get('task_state') == 'AUTO':
            final['task_state'] = self._task_state(observations)
        message['content'] = json.dumps(final, ensure_ascii=False)
        response['content'] = message['content']
        return response

    @staticmethod
    def _observations(kwargs):
        observations = []
        for item in kwargs.get('messages') or []:
            if item.get('role') != 'tool':
                continue
            try:
                value = json.loads(item.get('content') or '{}')
            except json.JSONDecodeError:
                continue
            if value.get('observation_id'):
                observations.append(value)
        return observations

    @staticmethod
    def _task_state(observations):
        if not observations:
            return 'informational'
        state = observations[-1]['state']
        return (
            'informational' if state in {'ok', 'warning'}
            else 'failed' if state == 'rejected'
            else state
        )


class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


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


def final_response(answer='已完成。', task_state='AUTO', evidence_refs='AUTO'):
    return tool_response('submit_final_response', {
        'answer': answer,
        'evidence_refs': evidence_refs,
        'task_state': task_state,
    }, 'final_response')


def make_runtime(planner=None, clock=lambda: 10.0):
    route_toolpack = RouteToolPack(RouteCatalog({
        'routes': [{'id': 'route_1', 'name': '路线', 'target_ids': ['target_003']}],
        'targets': [{'id': 'target_003', 'name': '巡检点3'}],
    }))
    schemas = {
        **route_toolpack.tool_schemas(),
        'start_patrol_mode': {
            'properties': {}, 'required': [], 'model_visible': False,
            'executor': 'system', 'side_effect': 'robot_navigation',
            'risk_level': 'mission',
        },
        'generate_local_status_reply': {
            'properties': {}, 'required': [], 'model_visible': False,
            'risk_level': 'read_only', 'side_effect': 'none',
        },
        'get_system_status': {
            'properties': {}, 'required': [], 'risk_level': 'read_only',
            'side_effect': 'none', 'executor': 'local', 'freshness_ttl_sec': 5.0,
        },
        'get_robot_summary': {
            'properties': {}, 'required': [], 'risk_level': 'read_only',
            'side_effect': 'none', 'executor': 'local', 'freshness_ttl_sec': 5.0,
        },
        'start_component': {
            'properties': {'component': {'type': 'string', 'enum': ['bringup']}},
            'required': ['component'], 'model_visible': False,
            'side_effect': 'component_lifecycle', 'risk_level': 'system_control',
        },
        'stop_component': {
            'properties': {'component': {'type': 'string', 'enum': ['bringup']}},
            'required': ['component'], 'model_visible': False,
            'side_effect': 'component_lifecycle', 'risk_level': 'system_control',
        },
        'prepare_robot_runtime': {
            'properties': {}, 'required': [], 'executor': 'system',
            'side_effect': 'runtime_prepare', 'risk_level': 'system_control',
            'conflict_group': 'robot_runtime',
            'terminal_states': ['succeeded', 'failed', 'rejected', 'timeout'],
        },
        'release_robot_runtime': {
            'properties': {}, 'required': [], 'executor': 'system',
            'model_visible': False, 'side_effect': 'runtime_cleanup',
            'risk_level': 'system_control', 'conflict_group': 'robot_runtime',
        },
        'rotate_relative': {
            'properties': {'angle_deg': {'type': 'number', 'minimum': -180, 'maximum': 180}},
            'required': ['angle_deg'], 'executor': 'base_skill',
            'side_effect': 'robot_motion', 'risk_level': 'reversible_motion',
            'conflict_group': 'robot_motion',
            'terminal_states': ['succeeded', 'failed', 'canceled', 'timeout'],
        },
        'move_relative': {
            'properties': {'distance_m': {'type': 'number', 'minimum': -0.5, 'maximum': 0.5}},
            'required': ['distance_m'], 'executor': 'base_skill',
            'side_effect': 'robot_motion', 'risk_level': 'reversible_motion',
            'conflict_group': 'robot_motion',
            'terminal_states': ['succeeded', 'failed', 'canceled', 'timeout'],
        },
        'stop_3d_capture': {
            'properties': {}, 'required': [], 'executor': 'system',
            'side_effect': 'mapping3d_capture', 'risk_level': 'mission',
            'conflict_group': 'mapping3d_capture',
        },
        'reconstruct_3d_model': {
            'properties': {'profile': {'type': 'string'}}, 'required': ['profile'],
            'executor': 'system', 'side_effect': 'mapping3d_reconstruct',
            'risk_level': 'mission', 'conflict_group': 'mapping3d_reconstruct',
        },
        'upload_3d_model': {
            'properties': {'session_id': {'type': 'string'}}, 'required': ['session_id'],
            'executor': 'system', 'side_effect': 'scene_upload',
            'risk_level': 'mission', 'conflict_group': 'scene_upload',
            'terminal_states': ['submitted', 'failed', 'rejected', 'timeout'],
        },
        'end_voice_conversation': {
            'properties': {}, 'required': [], 'executor': 'voice_session',
            'side_effect': 'voice_session_control', 'risk_level': 'session_control',
            'suppress_speech_after_call': True,
        },
        'close_voice_mode': {
            'properties': {}, 'required': [], 'executor': 'voice_session',
            'side_effect': 'voice_session_control', 'risk_level': 'session_control',
            'suppress_speech_after_call': True,
        },
        'emergency_stop': {
            'properties': {}, 'required': [], 'executor': 'system',
            'side_effect': 'robot_stop', 'risk_level': 'emergency',
        },
        'cancel_patrol': {
            'properties': {}, 'required': [], 'executor': 'system',
            'side_effect': 'patrol_cancel', 'risk_level': 'emergency',
            'conflict_group': 'robot_navigation',
        },
        'stop_motion': {
            'properties': {}, 'required': [], 'executor': 'base_skill',
            'side_effect': 'robot_stop', 'risk_level': 'emergency',
        },
    }
    pubs = SimpleNamespace(**{
        name: FakePub() for name in
        ('system', 'motion', 'say', 'event', 'patrol', 'base', 'voice')
    })
    state = AgentState()
    aggregator = RobotStatusAggregator(default_max_age_sec=60.0, clock=clock)
    tools = AgentTools(
        SimpleNamespace(), state, pubs.system, pubs.motion, pubs.say, pubs.event,
        patrol_pub=pubs.patrol, base_skill_pub=pubs.base,
        route_toolpack=route_toolpack, tool_schemas=schemas,
        status_aggregator=aggregator, voice_session_pub=pubs.voice,
    )
    spec = InspectionAgentSpecBuilder(route_toolpack, schemas, tools.registry).build()
    runtime = InspectionAgentRuntime(
        planner or FakePlanner(), tools, state, spec, schemas,
        route_toolpack=route_toolpack, clock=clock,
    )
    return runtime, pubs


def complete(manager, waiting, result_status='succeeded', message='操作完成', now=11.0):
    operation_id = waiting['pending_operation_id']
    manager.update(operation_id, 'succeeded', {
        'ok': True, 'status': 'succeeded', 'result_status': result_status,
        'message': message,
    }, now=now)
    return manager.get(operation_id, now=now)


def context_with(observations):
    return AgentTurnContext(
        run_id='run_1', request={}, history=[], observations=observations,
        active_operations=[], available_tools=[], runtime_lease=None,
    )


def test_all_nonempty_chinese_inputs_use_planner_without_python_business_routing():
    for text in ('你好', '开始巡逻', '上传三维模型', 'APP 连不上', '机器人正常吗'):
        planner = FakePlanner([final_response('收到。', 'informational', [])])
        runtime, _pubs = make_runtime(planner)

        result = runtime.run_turn({'text': text})

        assert result['role'] == 'assistant'
        assert planner.calls == 1


def test_planner_requires_protocol_final_response_tool():
    planner = FakePlanner([final_response('收到。', 'informational', [])])
    runtime, _pubs = make_runtime(planner)

    runtime.run_turn({'text': '普通问题'})

    request = planner.requests[0]
    tools = {item['function']['name']: item['function'] for item in request['tools']}
    assert request['tool_choice'] == 'required'
    assert tools['submit_final_response']['parameters']['required'] == [
        'answer', 'evidence_refs', 'task_state',
    ]


def test_llm_can_serially_call_different_side_effect_tools():
    planner = FakePlanner([
        tool_response('stop_3d_capture', {}, 'stop_capture'),
        tool_response('reconstruct_3d_model', {'profile': 'fast_check'}, 'reconstruct'),
    ])
    runtime, pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    first = runtime.run_turn({'text': '停止采集后重建', 'run_id': 'run_1'})
    second = runtime.resume_turn(complete(manager, first))

    assert first['state'] == second['state'] == 'waiting_feedback'
    assert [item['command'] for item in pubs.system.messages] == [
        'stop_3d_capture', 'reconstruct_3d_model',
    ]


def test_active_conflict_group_blocks_concurrent_side_effect():
    planner = FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move'),
        final_response('已有运动正在执行。'),
    ])
    runtime, pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager
    active = manager.create(
        'other_run', 'rotate', 'rotate_relative', {'angle_deg': 10}, 20.0,
        idempotency_key='other_run:rotate')
    manager.mark_sent(active.operation_id, now=10.0)

    result = runtime.run_turn({'text': '向前一点', 'run_id': 'run_1'})

    assert result['observations'][0]['error_code'] == 'operation_conflict'
    assert pubs.base.messages == []


def test_terminal_operation_allows_later_side_effect_in_same_conflict_group():
    runtime, pubs = make_runtime(FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move'),
    ]))
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager
    old = manager.create('old', 'rotate', 'rotate_relative', {}, 20.0)
    manager.mark_sent(old.operation_id, now=10.0)
    manager.update(old.operation_id, 'succeeded', now=10.0)

    waiting = runtime.run_turn({'text': '向前一点', 'run_id': 'run_1'})

    assert waiting['state'] == 'waiting_feedback'
    assert len(pubs.base.messages) == 1


def test_side_effect_operation_ros_payload_and_observation_share_idempotency_key():
    planner = FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move'),
        final_response('移动完成。'),
    ])
    runtime, pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '向前一点', 'run_id': 'run_1'})
    operation = manager.get(waiting['pending_operation_id'], now=10.0)
    result = runtime.resume_turn(complete(manager, waiting))

    assert operation['idempotency_key'] == 'run_1:move'
    assert pubs.base.messages[0]['idempotency_key'] == operation['idempotency_key']
    assert result['observations'][0]['idempotency_key'] == operation['idempotency_key']


def test_duplicate_idempotency_key_returns_existing_operation_without_republishing():
    runtime, pubs = make_runtime()
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager
    call = ToolCall('move', 'move_relative', {'distance_m': 0.1})
    decision = runtime._decision_from_tool_call(call, 'run_1', '', 'run_1:move')
    policy = authorize(decision, runtime._policy_context(), runtime.tool_schemas)

    first = runtime.tools.execute(decision, policy)
    second = runtime.tools.execute(decision, policy)

    assert first['data']['operation_id'] == second['data']['operation_id']
    assert len(pubs.base.messages) == 1


def test_every_observation_has_timestamp_and_freshness():
    runtime, _pubs = make_runtime(FakePlanner([
        tool_response('get_system_status', {}, 'status'),
        final_response('已读取状态。'),
    ]))

    result = runtime.run_turn({'text': '读取状态'})

    observation = result['observations'][0]
    assert observation['observed_at'] == 10.0
    assert observation['freshness'] == 'unknown'
    assert result['display_text'].endswith(
        '\n\n调用工具：\n- `get_system_status`：ok')
    assert runtime.state.steps[-1]['summary'] == '调用 get_system_status：ok'


def test_freshness_uses_source_time_and_configured_ttl():
    runtime, _pubs = make_runtime()
    call = ToolCall('status', 'get_system_status', {})

    fresh = runtime._observe(call, {
        'ok': True, 'status': 'ok', 'message': 'ok', 'data': {'observed_at': 8.0},
    })
    stale = runtime._observe(call, {
        'ok': True, 'status': 'ok', 'message': 'ok', 'data': {'observed_at': 4.0},
    })

    assert fresh.freshness == 'fresh'
    assert stale.freshness == 'stale'


def test_stale_status_cannot_support_current_ok_final():
    runtime, _pubs = make_runtime()
    observation = Observation(
        'obs_1', 'get_system_status', True, 'ok', 'old', {}, '', True,
        1.0, 'stale')

    with pytest.raises(ValueError, match='stale or unknown'):
        runtime._validate_final(FinalResponse('当前正常', ['obs_1'], 'ok'), context_with([observation]))


@pytest.mark.parametrize('state', ['submitted', 'running'])
def test_nonterminal_side_effect_cannot_support_succeeded_final(state):
    runtime, _pubs = make_runtime()
    observation = Observation(
        'obs_1', 'upload_3d_model', True, state, state, {}, '', False,
        10.0, 'fresh', idempotency_key='run:call')

    with pytest.raises(ValueError, match='incompatible'):
        runtime._validate_final(
            FinalResponse('已成功完成', ['obs_1'], 'succeeded'),
            context_with([observation]),
        )


def test_async_feedback_resumes_the_same_agent_turn_context():
    planner = FakePlanner([
        tool_response('stop_3d_capture', {}, 'stop_capture'),
        tool_response('reconstruct_3d_model', {'profile': 'fast_check'}, 'reconstruct'),
    ])
    runtime, _pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    first = runtime.run_turn({'text': '停止后重建', 'run_id': 'run_1'})
    context_id = id(runtime.pending_turn)
    runtime.resume_turn(complete(manager, first))

    assert id(runtime.pending_turn) == context_id
    assert runtime.pending_turn.observations[0].tool_name == 'stop_3d_capture'


def test_real_failure_is_preserved_instead_of_missing_tool_evidence():
    planner = FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move'),
        final_response('机器人运行环境未就绪。'),
    ])
    runtime, pubs = make_runtime(planner)
    runtime.tool_schemas['move_relative']['preconditions'] = ['robot_ready']

    result = runtime.run_turn({'text': '向前一点'})

    assert result['observations'][0]['error_code'] == 'precondition_failed'
    assert result['result']['status'] == 'failed'
    assert result['observations'][0]['error_code'] != 'missing_tool_evidence'
    assert pubs.base.messages == []


def test_internal_and_low_level_tools_are_hidden_from_openai():
    runtime, _pubs = make_runtime()
    names = {item['function']['name'] for item in runtime.openai_tools()}

    assert 'prepare_robot_runtime' in names
    assert 'start_component' not in names
    assert 'stop_component' not in names
    assert 'release_robot_runtime' not in names
    assert 'generate_local_status_reply' not in names
    assert 'start_patrol_mode' not in names


def test_runtime_lease_hides_repeated_prepare_and_refreshes_on_activity():
    planner = FakePlanner([
        tool_response('prepare_robot_runtime', {}, 'prepare'),
        final_response('运行环境已准备。'),
        final_response('继续。', 'informational', []),
    ])
    runtime, _pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '准备运行环境', 'run_id': 'run_1', 'session_id': 'voice_1'})
    runtime.resume_turn(complete(manager, waiting))
    lease = runtime.runtime_lease
    runtime.run_turn({'text': '继续', 'run_id': 'run_2', 'session_id': 'voice_1'})
    visible = {item['function']['name'] for item in planner.requests[-1]['tools']}

    assert lease is runtime.runtime_lease
    assert lease.state == 'active'
    assert 'prepare_robot_runtime' not in visible


def test_idle_lease_reclaim_waits_for_patrol_and_does_not_overwrite_main_result():
    now = [10.0]
    runtime, pubs = make_runtime(clock=lambda: now[0])
    manager = AgentOperationManager(clock=lambda: now[0])
    runtime.tools.operation_manager = manager
    runtime.runtime_lease = RuntimeLease('lease_1', 'voice', 0.0, 5.0)
    runtime.state.latest_result = {'status': 'succeeded', 'message': 'main'}
    runtime.state.patrol_status = {'state': 'running'}

    assert runtime.reap_runtime_lease() is None
    assert pubs.system.messages == []
    runtime.state.patrol_status = {'state': 'idle'}
    now[0] = runtime.runtime_lease.expires_at + 1.0
    runtime.reap_runtime_lease()

    assert pubs.system.messages[-1]['command'] == 'release_robot_runtime'
    assert runtime.state.latest_result == {'status': 'succeeded', 'message': 'main'}


def test_more_than_six_planner_steps_are_allowed():
    responses = [
        tool_response('get_system_status', {}, f'status_{index}') for index in range(8)
    ] + [final_response('查询完成。')]
    planner = FakePlanner(responses)
    runtime, _pubs = make_runtime(planner)

    result = runtime.run_turn({'text': '持续查询'})

    assert planner.calls == 9
    assert result['assistant_text'] == '查询完成。'


def test_final_response_protocol_error_is_retried_once():
    planner = FakePlanner([
        {'message': {'role': 'assistant', 'content': '普通文本'}, 'content': '普通文本'},
        final_response('请补充信息。', 'needs_input', []),
    ])
    runtime, _pubs = make_runtime(planner)

    result = runtime.run_turn({'text': '不明确请求'})

    assert planner.calls == 2
    assert result['assistant_text'] == '请补充信息。'
    assert any(
        str(item.get('content') or '').startswith('FINAL_RESPONSE_PROTOCOL_ERROR')
        for item in planner.requests[-1]['messages']
    )


def test_plain_text_after_protocol_retry_is_rejected_not_synthesized():
    planner = FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move'),
        {'message': {'role': 'assistant', 'content': '已向前移动。'}},
        {'message': {'role': 'assistant', 'content': '机器人已向前移动。'}},
    ])
    runtime, _pubs = make_runtime(planner)
    manager = AgentOperationManager(clock=lambda: 10.0)
    runtime.tools.operation_manager = manager

    waiting = runtime.run_turn({'text': '向前一点', 'run_id': 'run_1'})
    result = runtime.resume_turn(complete(manager, waiting))

    assert result['role'] == 'tool'
    assert result['result']['error_code'] == 'agent_error'
    assert result['assistant_text'] == '智能助手回复格式异常，请重试。'
    assert result['observations'][0]['state'] == 'succeeded'
    assert 'task_state' not in result['result'].get('data', {})


def test_unrecoverable_final_protocol_error_is_user_facing_chinese():
    empty = {'message': {'role': 'assistant', 'content': ''}}
    runtime, _pubs = make_runtime(FakePlanner([empty, empty]))

    result = runtime.run_turn({'text': '普通问题'})

    assert result['result']['error_code'] == 'agent_error'
    assert result['assistant_text'] == '智能助手回复格式异常，请重试。'
    assert result['speech_text'] == '智能助手回复格式异常，请重试。'
    assert 'SchemaError' not in result['assistant_text']


def test_third_identical_no_progress_call_is_not_executed():
    planner = FakePlanner([
        tool_response('move_relative', {'distance_m': 0.1}, 'move_1'),
        tool_response('move_relative', {'distance_m': 0.1}, 'move_2'),
        tool_response('move_relative', {'distance_m': 0.1}, 'move_3'),
        final_response('无法执行。'),
    ])
    runtime, pubs = make_runtime(planner)
    runtime.tool_schemas['move_relative']['preconditions'] = ['robot_ready']

    result = runtime.run_turn({'text': '向前一点'})

    assert [item['error_code'] for item in result['observations']] == [
        'precondition_failed', 'precondition_failed', 'repeated_tool_call',
    ]
    assert pubs.base.messages == []


@pytest.mark.parametrize(
    ('result_status', 'expected'),
    [('submitted', 'submitted'), ('running', 'running'), ('canceled', 'canceled')],
)
def test_operation_business_state_is_not_promoted_to_succeeded(result_status, expected):
    result = InspectionAgentRuntime._operation_result({
        'operation_id': 'op_1', 'tool_name': 'upload_3d_model',
        'state': 'succeeded', 'arguments': {'session_id': 's1'},
        'idempotency_key': 'run:call',
        'result': {'result_status': result_status, 'message': expected},
    })

    assert result['status'] == expected
    assert result['data']['idempotency_key'] == 'run:call'


def test_no_unconditional_robot_summary_is_injected():
    planner = FakePlanner([final_response('收到。', 'informational', [])])
    runtime, _pubs = make_runtime(planner)

    runtime.run_turn({'text': '任意问题'})

    assert not any(
        str(item.get('content') or '').startswith('INTERNAL_FRESH_ROBOT_SUMMARY')
        for item in planner.requests[0]['messages']
    )
