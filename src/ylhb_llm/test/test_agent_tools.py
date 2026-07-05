import json
from types import SimpleNamespace

from ylhb_llm.agent_policy import authorize
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


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


def test_go_to_checkpoint_resolves_target_and_publishes_patrol_command():
    patrol_pub = FakePub()
    data = {
        'routes': [{'id': 'route_patrol_001', 'name': '路线', 'target_ids': ['target_003']}],
        'targets': [{'id': 'target_003', 'name': '巡检点3', 'pose': {'x': 1, 'y': 2, 'yaw': 0}}],
    }
    tools = AgentTools(
        SimpleNamespace(),
        SimpleNamespace(system_status={}, patrol_status={}, voice_status={}),
        FakePub(),
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

    assert patrol_pub.messages[0]['command'] == 'go_to_target'
    assert patrol_pub.messages[0]['target_id'] == 'target_003'


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
