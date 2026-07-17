from dataclasses import dataclass, field
from typing import Any, Dict, List


ACTIVE_PATROL_STATES = {'running', 'returning_home', 'waiting_loop'}
CANCELABLE_PATROL_STATES = ACTIVE_PATROL_STATES | {
    'unknown', 'starting', 'command_sent', 'waiting_nav2', 'waiting_localization',
    'paused', 'canceling',
}
SAFE_MOTIONS = {'前进', '后退', '左转', '右转', '停止'}
DANGEROUS_TOOLS = {'/cmd_vel', 'cmd_vel', 'nav2_goal', 'delete_map', 'edit_route'}


@dataclass
class PolicyResult:
    allowed: bool
    reason: str = ''
    priority: int = 5
    interrupt: bool = False
    system_command: str = ''
    error_code: str = ''
    missing_preconditions: List[str] = field(default_factory=list)
    recoverable: bool = False
    recovery_components: List[str] = field(default_factory=list)
    state_summary: Dict[str, Any] = field(default_factory=dict)


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
        schema = tool_schemas[tool]
        rejected = _reject_by_schema(tool, arguments, schema)
        if rejected:
            return PolicyResult(False, rejected)
        preconditions = list(schema.get('preconditions') or [])
        for field, values in (schema.get('preconditions_by_argument') or {}).items():
            preconditions.extend((values or {}).get(arguments.get(field), []))
        missing, summary = _missing_preconditions(preconditions, arguments, state)
        if missing:
            recovery_components = _recovery_components(missing)
            return PolicyResult(
                False,
                '缺少工具前置条件: ' + ', '.join(missing),
                error_code='precondition_failed',
                missing_preconditions=missing,
                recoverable=bool(recovery_components),
                recovery_components=recovery_components,
                state_summary=summary,
            )
    if tool == 'start_patrol_mode':
        return PolicyResult(True, system_command='start_patrol_mode')
    if tool == 'pause_patrol':
        return PolicyResult(
            bool(tool_schemas and tool in tool_schemas) or patrol_state in ACTIVE_PATROL_STATES,
            '当前没有可暂停的巡逻任务',
            system_command=tool,
        )
    if tool == 'resume_patrol':
        return PolicyResult(
            bool(tool_schemas and tool in tool_schemas) or patrol_state in {'paused', 'unknown'},
            '当前没有已暂停的巡逻任务',
            system_command=tool,
        )
    if tool == 'cancel_patrol':
        return PolicyResult(
            bool(tool_schemas and tool in tool_schemas) or patrol_state in CANCELABLE_PATROL_STATES,
            '当前没有可取消的巡逻任务',
            system_command=tool,
        )
    if tool == 'send_motion_command':
        command = str(arguments.get('command') or '')
        return PolicyResult(command in SAFE_MOTIONS, '不支持的运动指令')
    if tool in {
        'get_system_status',
        'get_robot_summary',
        'get_patrol_status',
        'get_voice_status',
        'generate_local_status_reply',
        'list_routes',
        'describe_route',
        'list_checkpoints',
        'inspect_checkpoint',
        'start_route',
        'start_component',
        'stop_component',
        'end_voice_conversation',
        'close_voice_mode',
        'go_to_checkpoint',
        'stop_motion',
    }:
        return PolicyResult(True)
    if tool in {'rotate_relative', 'move_relative'}:
        return PolicyResult(True)
    return PolicyResult(False, '未知工具')


def _missing_preconditions(
    preconditions: List[str],
    arguments: Dict[str, Any],
    state: Dict[str, Any],
) -> tuple[List[str], Dict[str, Any]]:
    system = state.get('system_status') or {}
    robot = state.get('robot_summary') or {}
    components = robot.get('components') or {
        name: system.get(name, 'unknown')
        for name in ('bringup', 'navigation', 'perception', 'patrol_executor')
    }
    patrol_state = str(state.get('patrol_state') or state.get('state') or 'unknown')
    chassis = robot.get('chassis') or {}
    sensors = robot.get('sensors') or {}
    started = set(state.get('started_components') or [])
    component = str(arguments.get('component') or '')
    predicates = {
        'robot_ready': components.get('bringup') in {'running', 'embedded'},
        'bringup_running': components.get('bringup') in {'running', 'embedded'},
        'navigation_running': components.get('navigation') in {'running', 'embedded'},
        'perception_running': components.get('perception') in {'running', 'embedded'},
        'patrol_executor_running': components.get('patrol_executor') in {'running', 'embedded'},
        'chassis_online': (
            chassis.get('state') == 'online'
            and chassis.get('fresh') is not False
        ),
        'sensor_fresh': sensors.get('lidar') == 'ok' and sensors.get('odom') == 'ok',
        'no_active_patrol': patrol_state not in ACTIVE_PATROL_STATES | {'starting', 'command_sent', 'paused'},
        'active_patrol': patrol_state in ACTIVE_PATROL_STATES,
        'paused_patrol': patrol_state == 'paused',
        'started_this_turn': bool(component and component in started),
    }
    missing = [name for name in preconditions if not predicates.get(name, False)]
    return missing, {
        'components': dict(components),
        'patrol_state': patrol_state,
        'chassis_online': predicates['chassis_online'],
        'sensor_fresh': predicates['sensor_fresh'],
        'started_components': sorted(started),
    }


def _recovery_components(missing: List[str]) -> List[str]:
    if 'robot_ready' in missing or 'bringup_running' in missing:
        return ['bringup']
    return []


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
