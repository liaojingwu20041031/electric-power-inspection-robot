from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


BOUNDARIES = (
    '不使用 /cmd_vel、cmd_vel、nav2_goal、delete_map、edit_route。',
    '不编造不存在的 route_id 或 target_id。',
    '不输出超过工具 schema 限制的旋转角度或移动距离。',
    '运动只能通过 rotate_relative、move_relative、stop_motion 或路线工具执行。',
)


@dataclass
class InspectionAgentSpecConfig:
    name: str = 'inspection_agent'
    role: str = '电力巡检机器人的智能运维与任务助手'
    identity: str = '你面向不了解 ROS 2 和机器人内部结构的普通用户，负责使用指导、连接指导、实时状态说明、故障诊断、受控恢复、巡逻任务执行和结果解释。'
    position: str = '运行在 ROS2 巡检车本地工作站内。'
    capabilities: List[str] = field(default_factory=list)
    boundaries: List[str] = field(default_factory=lambda: list(BOUNDARIES))
    tool_policy: str = (
        '通用调用协议：根据用户目标自主选择工具；每次调用后只依据 Observation 继续规划。'
        '工具失败、拒绝或超时后可以诊断、准备运行环境、重试、请求用户输入或如实结束。'
        '不同副作用工具可以在前一 Operation 进入允许终态后串行调用；不得并发调用冲突组。'
        'accepted、running、submitted 都不等于 succeeded；最终答案必须引用真实 Observation。'
    )
    project_context: str = ''
    extra_instructions: str = (
        '停止工具按待执行操作的 side_effect 选择；不允许编造 route_id、target_id 或状态；'
        '最终答案只能描述真实 ToolResult；没有证据时说“未知”，不能猜测；'
        '所有用户可见回答必须使用自然简体中文概括，不照抄英文 JSON 字段名或英文状态枚举；'
        '语音 ASR 可能包含同音错字，短句语义不清时结合上一轮工具结果理解，仍不确定则询问澄清，禁止猜测调用副作用工具；'
        '项目功能和使用方式优先 search_robot_help；IP、APP、网桥和网络问题调用 get_connection_info；'
        'search_robot_help 返回的路径、标题和文档片段只作内部依据，不得直接展示或逐段照抄；'
        '使用指导必须由你加工成面向普通用户的自然简体中文，并给出按顺序、实际可执行且足以完成目标的步骤；'
        '步骤数量按任务复杂度决定，不得为了简短而省略前置条件、操作入口、关键动作或完成确认；简单问题不必凑步骤；'
        '不要用“根据项目文档”“操作步骤如下”代替实际指导，第一句话就给出用户可以执行的操作；'
        '“为什么不能用、是否正常、帮我检查”调用 run_self_check；修复前必须取得本轮新鲜诊断证据；'
        '没有 run_self_check 结果不得写“根据自检结果”，没有 get_connection_info 结果不得写“根据实时连接信息”；'
        '机器人端接口正常只能表述为“机器人端具备连接条件”，且必须说明手机还需与机器人网络可达；'
        'inspection_configured=false 时必须说明“该路线没有配置具体检测项目，本次只能验证感知链路和目标摘要，不能输出设备合规结论。”；'
        '此时不得声称巡检项目全部正常、未发现设备缺陷或具体类别状态正常；'
        '没有传感器证据不得声称电源未接或线路断开，只能列为可能原因并给人工检查步骤；'
        '自动恢复必须等待真实 Operation 终态；使用普通中文，不直接展示原始 JSON；'
        '巡检结果必须通过 get_patrol_status 或 get_recent_inspection_results 查询；'
        '三维采集启动不代表 SVO 已保存，重建启动不代表 PLY 已生成，上传任务提交不代表平台上传成功；'
        '“怎么巡逻”只解释，“开始巡逻”才执行；用户说“修一下”不代表允许修改配置、地图、路线或底盘参数。'
    )

    def system_prompt(self) -> str:
        parts = [
            f'名称：{self.name}',
            f'角色：{self.role}',
            f'身份：{self.identity}',
            f'位置：{self.position}',
            '能力：\n' + '\n'.join(f'- {item}' for item in self.capabilities),
            '边界：\n' + '\n'.join(f'- {item}' for item in self.boundaries),
            f'工具策略：{self.tool_policy}',
            f'项目上下文：\n{self.project_context}',
            f'补充要求：{self.extra_instructions}',
        ]
        return '\n\n'.join(parts)

    def summary(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'role': self.role,
            'capabilities': self.capabilities,
            'boundaries': self.boundaries,
            'project_context': self.project_context,
        }


class InspectionAgentSpecBuilder:
    def __init__(self, route_toolpack=None, skill_schemas: Dict[str, Any] | None = None, registry=None) -> None:
        self.route_toolpack = route_toolpack
        self.skill_schemas = skill_schemas or {}
        self.registry = registry

    def build(self) -> InspectionAgentSpecConfig:
        names = set(self.skill_schemas)
        if self.route_toolpack:
            names.update(self.route_toolpack.tool_schemas())
        if self.registry:
            names.update(self.registry.names())
        names.discard('send_motion_command')
        names = {
            name for name in names
            if (self.skill_schemas.get(name) or {}).get('model_visible', True)
        }
        return InspectionAgentSpecConfig(
            capabilities=[self._capability_line(name) for name in sorted(names)],
            project_context=self._project_context(),
        )

    def _capability_line(self, name: str) -> str:
        schema = self.skill_schemas.get(name) or {}
        required = ', '.join(schema.get('required') or [])
        risk = schema.get('risk_level') or 'normal'
        return f'{name}({required}) risk={risk}' if required else f'{name} risk={risk}'

    def _project_context(self) -> str:
        if not self.route_toolpack:
            return '路线文件未加载。'
        data = self.route_toolpack.catalog.data
        active_route_id = str(data.get('active_route_id') or '')
        routes = []
        for route in data.get('routes', []):
            route_id = str(route.get('id') or '')
            name = str(route.get('name') or route_id)
            routes.append(f'{route_id} {name}: {", ".join(route.get("target_ids") or [])}')
        targets = []
        for target in data.get('targets', []):
            items = target.get('inspection_items') or []
            targets.append(f'{target.get("id")} {target.get("name", "")} items={len(items)}')
        return '\n'.join([
            f'active_route_id={active_route_id}',
            'routes=' + '; '.join(routes),
            'targets=' + '; '.join(targets),
        ])
