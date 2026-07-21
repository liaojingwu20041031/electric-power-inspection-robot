import json
from types import SimpleNamespace

from ylhb_llm.agent_policy import authorize
from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator


class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def test_system_tool_arguments_cannot_override_command_name():
    system_pub = FakePub()
    event_pub = FakePub()
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        system_pub,
        FakePub(),
        FakePub(),
        event_pub,
    )
    decision = {
        'tool_call': {
            'name': 'start_patrol_mode',
            'arguments': {'command': 'stop_robot_stack', 'profile': 'inspection'},
        }
    }

    tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert system_pub.messages[0]['command'] == 'start_patrol_mode'
    assert system_pub.messages[0]['profile'] == 'inspection'


def test_motion_tool_publishes_structured_motion_command():
    motion_pub = FakePub()
    event_pub = FakePub()
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1230000000))
    tools = AgentTools(
        SimpleNamespace(get_clock=lambda: clock),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(),
        motion_pub,
        FakePub(),
        event_pub,
    )
    decision = {
        'decision_id': 'd1',
        'tool_call': {'name': 'send_motion_command', 'arguments': {'command': '后退'}},
    }

    tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert motion_pub.messages[0] == {
        'schema_version': '1.0',
        'source': 'inspection_agent',
        'command': '后退',
        'request_id': 'd1',
        'timestamp': 1.23,
    }


def test_start_route_publishes_system_command_with_route_id():
    system_pub = FakePub()
    event_pub = FakePub()
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        system_pub,
        FakePub(),
        FakePub(),
        event_pub,
    )
    decision = {
        'tool_call': {'name': 'start_route', 'arguments': {'route_id': 'route_patrol_001'}}
    }

    tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert system_pub.messages[0]['command'] == 'start_patrol_mode'
    assert system_pub.messages[0]['route_id'] == 'route_patrol_001'


def test_go_to_checkpoint_resolves_target_and_publishes_supervisor_command():
    system_pub = FakePub()
    patrol_pub = FakePub()
    data = {
        'routes': [{'id': 'route_patrol_001', 'name': '路线', 'target_ids': ['target_003']}],
        'targets': [{'id': 'target_003', 'name': '巡检点3', 'pose': {'x': 1, 'y': 2, 'yaw': 0}}],
    }
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        system_pub,
        FakePub(),
        FakePub(),
        FakePub(),
        patrol_pub=patrol_pub,
        route_toolpack=RouteToolPack(RouteCatalog(data)),
    )
    decision = {
        'decision_id': 'd1',
        'tool_call': {'name': 'go_to_checkpoint', 'arguments': {'target_id': '巡检点3'}},
    }

    tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert system_pub.messages[0]['command'] == 'go_to_checkpoint'
    assert system_pub.messages[0]['target_id'] == 'target_003'
    assert patrol_pub.messages == []


def test_robot_summary_tool_uses_chinese_user_facing_message():
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(), FakePub(), FakePub(), FakePub(),
        status_aggregator=SimpleNamespace(summary=lambda: {'health': 'ok'}),
    )
    decision = {'tool_call': {'name': 'get_robot_summary', 'arguments': {}}}

    result = tools.execute(decision, authorize(decision, {}))

    assert result['message'] == '机器人状态摘要'


def test_rotate_relative_publishes_base_skill_not_cmd_vel():
    base_skill_pub = FakePub()
    motion_pub = FakePub()
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(),
        motion_pub,
        FakePub(),
        FakePub(),
        base_skill_pub=base_skill_pub,
    )
    decision = {
        'decision_id': 'd1',
        'tool_call': {'name': 'rotate_relative', 'arguments': {'angle_deg': 180}},
    }

    tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert base_skill_pub.messages[0]['command'] == 'rotate_relative'
    assert base_skill_pub.messages[0]['arguments'] == {'angle_deg': 180}
    assert motion_pub.messages == []


def test_side_effect_tool_creates_operation_and_forwards_correlation_ids():
    base_skill_pub = FakePub()
    manager = AgentOperationManager(clock=lambda: 10.0)
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(), FakePub(), FakePub(), FakePub(),
        base_skill_pub=base_skill_pub,
        operation_manager=manager,
        tool_schemas={'rotate_relative': {'side_effect': 'robot_motion', 'timeout_sec': 12.0}},
    )
    decision = {
        'decision_id': 'decision_1',
        'run_id': 'run_1',
        'tool_call_id': 'call_1',
        'tool_call': {'name': 'rotate_relative', 'arguments': {'angle_deg': 10}},
    }

    result = tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    operation_id = result['data']['operation_id']
    assert result['status'] == 'sent'
    assert manager.get(operation_id, now=10.0)['state'] == 'sent'
    assert base_skill_pub.messages[0]['operation_id'] == operation_id
    assert base_skill_pub.messages[0]['tool_call_id'] == 'call_1'


def test_get_robot_summary_reuses_status_aggregator():
    aggregator = RobotStatusAggregator(clock=lambda: 10.0)
    aggregator.update('system_status', {'mode': 'ready'}, now=10.0)
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(), FakePub(), FakePub(), FakePub(),
        status_aggregator=aggregator,
        tool_schemas={'get_robot_summary': {'risk_level': 'read_only'}},
    )
    decision = {'tool_call': {'name': 'get_robot_summary', 'arguments': {}}}

    result = tools.execute(decision, authorize(decision, {'patrol_state': 'idle'}))

    assert result['ok'] is True
    assert result['data']['robot_mode'] == 'ready'


def test_read_only_operational_tools_use_injected_local_dependencies():
    knowledge = SimpleNamespace(search=lambda query: [{'path': 'README.md', 'title': '使用', 'content': query}])
    diagnostics = SimpleNamespace(
        get_connection_info=lambda target='all': {'target': target},
        run_self_check=lambda scope='all': {'overall': 'ok', 'scope': scope},
    )
    tools = AgentTools(
        SimpleNamespace(), SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(), FakePub(), FakePub(), FakePub(),
        tool_schemas={
            'search_robot_help': {'executor': 'knowledge', 'risk_level': 'read_only'},
            'get_connection_info': {'executor': 'connection', 'risk_level': 'read_only'},
            'run_self_check': {'executor': 'diagnostic', 'risk_level': 'read_only'},
        },
        knowledge_index=knowledge,
        diagnostic_engine=diagnostics,
    )

    help_result = tools.execute(
        {'tool_call': {'name': 'search_robot_help', 'arguments': {'query': 'APP'}}},
        authorize({'tool_call': {'name': 'search_robot_help', 'arguments': {'query': 'APP'}}}, {}, tools.tool_schemas),
    )
    connection = tools.execute(
        {'tool_call': {'name': 'get_connection_info', 'arguments': {'target': 'local_app'}}},
        authorize({'tool_call': {'name': 'get_connection_info', 'arguments': {'target': 'local_app'}}}, {}, tools.tool_schemas),
    )
    diagnostic = tools.execute(
        {'tool_call': {'name': 'run_self_check', 'arguments': {'scope': 'base'}}},
        authorize({'tool_call': {'name': 'run_self_check', 'arguments': {'scope': 'base'}}}, {}, tools.tool_schemas),
    )

    assert help_result['data']['results'][0]['path'] == 'README.md'
    assert connection['data']['target'] == 'local_app'
    assert diagnostic['data']['scope'] == 'base'


def test_recover_component_uses_supervisor_operation_not_direct_process_control():
    system_pub = FakePub()
    manager = AgentOperationManager(clock=lambda: 10.0)
    catalog = SimpleNamespace(names=lambda: ['perception'], get=lambda name: {'process': name})
    diagnostic = SimpleNamespace(last_report={
        'diagnostic_id': 'diag_1', 'generated_at': 10.0,
        'issues': [{'component': 'perception', 'recoverable': True, 'recovery_component': 'perception'}],
    })
    tools = AgentTools(
        SimpleNamespace(), SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        system_pub, FakePub(), FakePub(), FakePub(), operation_manager=manager,
        recovery_catalog=catalog, diagnostic_engine=diagnostic,
        tool_schemas={'recover_component': {
            'executor': 'system', 'command': 'recover_component',
            'side_effect': 'component_recovery', 'timeout_sec': 30.0,
        }},
    )
    decision = {
        'decision_id': 'd1', 'run_id': 'run1', 'tool_call_id': 'call1',
        'tool_call': {'name': 'recover_component', 'arguments': {'component': 'perception'}},
    }

    result = tools.execute(decision, SimpleNamespace(allowed=True))

    assert result['status'] == 'sent'
    assert system_pub.messages[0]['command'] == 'recover_component'
    assert system_pub.messages[0]['component'] == 'perception'
