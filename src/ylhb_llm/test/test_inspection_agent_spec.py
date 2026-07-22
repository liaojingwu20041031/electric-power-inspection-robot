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

    for requirement in (
        '优先调用用户要求的业务目标工具',
        '检查目标工具 preconditions',
        'recovery_components=[bringup]',
        '重试同一目标',
        'sent/accepted/running 都不等于任务完成',
        '同一目标取得终态后不得再次执行',
        '组件准备不能作为动作成功证据',
        'Supervisor 内部准备',
        '最终答案只能来自真实 ToolResult',
        '自然简体中文',
        '三维采集启动不代表 SVO 已保存',
        '重建启动不代表 PLY 已生成',
        '上传任务提交不代表平台上传成功',
        'get_recent_inspection_results',
    ):
        assert requirement in prompt
