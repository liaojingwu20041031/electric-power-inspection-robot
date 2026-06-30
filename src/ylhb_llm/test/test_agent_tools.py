import json
from types import SimpleNamespace

from ylhb_llm.agent_policy import authorize
from ylhb_llm.agent_tools import AgentTools


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
