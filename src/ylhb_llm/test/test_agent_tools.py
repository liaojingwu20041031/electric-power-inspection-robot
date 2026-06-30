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
