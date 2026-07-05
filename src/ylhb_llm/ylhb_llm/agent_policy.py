from dataclasses import dataclass
from typing import Any, Dict


ACTIVE_PATROL_STATES = {'running', 'returning_home', 'waiting_loop'}
CANCELABLE_PATROL_STATES = ACTIVE_PATROL_STATES | {'paused', 'canceling'}
SAFE_MOTIONS = {'前进', '后退', '左转', '右转', '停止'}
DANGEROUS_TOOLS = {'/cmd_vel', 'cmd_vel', 'nav2_goal', 'delete_map', 'edit_route'}


@dataclass
class PolicyResult:
    allowed: bool
    reason: str = ''
    priority: int = 5
    interrupt: bool = False
    system_command: str = ''


def authorize(
    decision: Dict[str, Any],
    state: Dict[str, Any],
    tool_schemas: Dict[str, Any] | None = None,
) -> PolicyResult:
    tool_call = decision.get('tool_call') if isinstance(decision, dict) else {}
    tool = str((tool_call or {}).get('name') or '')
    arguments = (tool_call or {}).get('arguments') or {}
    patrol_state = str(state.get('patrol_state') or state.get('state') or 'unknown')

    if tool == 'emergency_stop':
        return PolicyResult(True, priority=10, interrupt=True, system_command='emergency_stop')
    if tool in DANGEROUS_TOOLS:
        return PolicyResult(False, '拒绝危险工具')
    if tool_schemas and tool in tool_schemas:
        rejected = _reject_by_schema(tool, arguments, tool_schemas[tool])
        if rejected:
            return PolicyResult(False, rejected)
    if tool == 'start_patrol_mode':
        return PolicyResult(True, system_command='start_patrol_mode')
    if tool == 'pause_patrol':
        return PolicyResult(
            patrol_state in ACTIVE_PATROL_STATES,
            '当前没有可暂停的巡逻任务',
            system_command='pause_patrol',
        )
    if tool == 'resume_patrol':
        if patrol_state == 'paused':
            return PolicyResult(True, system_command='resume_patrol')
        if patrol_state == 'unknown':
            return PolicyResult(True, '状态未知，等待执行器确认', system_command='resume_patrol')
        return PolicyResult(False, '当前没有已暂停的巡逻任务')
    if tool == 'cancel_patrol':
        return PolicyResult(
            patrol_state in CANCELABLE_PATROL_STATES,
            '当前没有可取消的巡逻任务',
            system_command='cancel_patrol',
        )
    if tool == 'send_motion_command':
        command = str(arguments.get('command') or '')
        return PolicyResult(command in SAFE_MOTIONS, '不支持的运动指令')
    if tool in {
        'get_system_status',
        'get_patrol_status',
        'get_voice_status',
        'generate_local_status_reply',
        'list_routes',
        'describe_route',
        'list_checkpoints',
        'inspect_checkpoint',
        'start_route',
        'go_to_checkpoint',
        'stop_motion',
    }:
        return PolicyResult(True)
    if tool in {'rotate_relative', 'move_relative'}:
        return PolicyResult(True)
    return PolicyResult(False, '未知工具')


def _reject_by_schema(tool: str, arguments: Dict[str, Any], schema: Dict[str, Any]) -> str:
    for field in schema.get('required') or []:
        if field not in arguments:
            return f'{field} 是必填参数'
    for field, rules in (schema.get('properties') or {}).items():
        if field not in arguments:
            continue
        value = arguments[field]
        if rules.get('type') == 'number':
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f'{field} 必须是数字'
            if 'minimum' in rules and float(value) < float(rules['minimum']):
                return f'{field} 超出允许范围'
            if 'maximum' in rules and float(value) > float(rules['maximum']):
                return f'{field} 超出允许范围'
        if rules.get('enum') and value not in rules['enum']:
            return f'{field} 不在允许列表'
    return ''
