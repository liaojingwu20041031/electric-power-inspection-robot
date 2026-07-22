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
        '自主选择工具',
        '只依据 Observation',
        '准备运行环境',
        '串行调用',
        '不得并发调用冲突组',
        'accepted、running、submitted 都不等于 succeeded',
        '最终答案必须引用真实 Observation',
        '自然简体中文',
        '三维采集启动不代表 SVO 已保存',
        '重建启动不代表 PLY 已生成',
        '上传任务提交不代表平台上传成功',
        'get_recent_inspection_results',
        '路径、标题和文档片段只作内部依据',
        '实际可执行且足以完成目标的步骤',
        '步骤数量按任务复杂度决定',
        '不得为了简短而省略前置条件',
        '第一句话就给出用户可以执行的操作',
    ):
        assert requirement in prompt
