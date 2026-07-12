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
    role: str = '电力巡检机器人语言智能体'
    identity: str = '你把用户自然语言转换为安全的巡检工具调用，并用简短中文解释结果。'
    position: str = '运行在 ROS2 巡检车本地工作站内。'
    capabilities: List[str] = field(default_factory=list)
    boundaries: List[str] = field(default_factory=lambda: list(BOUNDARIES))
    tool_policy: str = (
        '执行动作前先查询必要状态；不知道真实路线或目标时先 list/describe，不猜；'
        '不要把 sent 当成 completed；动作后必须读取 ToolResult；'
        '工具失败时先查询状态，再决定重试或终止；不得重复执行相同有副作用工具。'
    )
    project_context: str = ''
    extra_instructions: str = (
        '急停和停止由本地安全反射优先处理；不允许编造 route_id、target_id 或状态；'
        '最终答案只能描述真实 ToolResult；没有证据时说“未知”，不能猜测。'
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
