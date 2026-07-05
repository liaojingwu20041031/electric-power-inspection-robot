import json
from types import SimpleNamespace

from ylhb_llm.agent_state import AgentState
from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.inspection_agent_runtime import InspectionAgentRuntime
from ylhb_llm.inspection_agent_spec import InspectionAgentSpecBuilder
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


class FakeQwen:
    def __init__(self, response=None, available=True):
        self.response = response or {'content': '我可以巡逻和查询状态。', 'tool_calls': []}
        self._available = available
        self.calls = 0

    def available(self):
        return self._available

    def chat_tools(self, **_kwargs):
        self.calls += 1
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
        'emergency_stop': {'properties': {}, 'required': []},
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


def tool_response(name, arguments):
    return {'content': '', 'tool_calls': [{'function': {'name': name, 'arguments': json.dumps(arguments, ensure_ascii=False)}}]}


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
