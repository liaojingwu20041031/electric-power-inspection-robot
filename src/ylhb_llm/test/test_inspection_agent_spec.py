from ylhb_llm.agent_tools import AgentTools
from ylhb_llm.inspection_agent_spec import InspectionAgentSpecBuilder
from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


class FakePub:
    def publish(self, _msg):
        pass


def test_spec_builder_summarizes_tools_routes_and_boundaries():
    route_toolpack = RouteToolPack(RouteCatalog({
        'active_route_id': 'route_1',
        'routes': [{'id': 'route_1', 'name': '路线', 'target_ids': ['target_003']}],
        'targets': [{'id': 'target_003', 'name': '巡检点3', 'inspection_items': ['测温']}],
    }))
    schemas = {
        'rotate_relative': {'properties': {'angle_deg': {'type': 'number'}}, 'required': ['angle_deg'], 'risk_level': 'normal'},
        'send_motion_command': {'properties': {}, 'required': []},
    }
    tools = AgentTools(object(), object(), FakePub(), FakePub(), FakePub(), FakePub(), route_toolpack=route_toolpack, tool_schemas=schemas)

    spec = InspectionAgentSpecBuilder(route_toolpack, schemas, tools.registry).build()
    summary = spec.summary()

    assert any('rotate_relative(angle_deg)' in item for item in summary['capabilities'])
    assert not any('send_motion_command' in item for item in summary['capabilities'])
    assert 'active_route_id=route_1' in summary['project_context']
    assert 'target_003' in summary['project_context']
    assert any('/cmd_vel' in item for item in summary['boundaries'])


def test_system_prompt_requires_evidence_and_feedback_after_actions():
    spec = InspectionAgentSpecBuilder().build()
    prompt = spec.system_prompt()

    for requirement in ('动作前先查询必要状态', '不要把 sent 当成 completed', '工具失败时先查询状态', '没有证据时说“未知”'):
        assert requirement in prompt
