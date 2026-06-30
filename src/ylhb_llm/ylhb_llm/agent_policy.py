from dataclasses import dataclass
from typing import Any, Dict


ACTIVE_PATROL_STATES = {'running', 'returning_home', 'waiting_loop'}
CANCELABLE_PATROL_STATES = ACTIVE_PATROL_STATES | {'paused', 'canceling'}
SAFE_MOTIONS = {'前进', '后退', '左转', '右转', '停止'}


@dataclass
class PolicyResult:
    allowed: bool
    reason: str = ''
    priority: int = 5
    interrupt: bool = False
    system_command: str = ''


def authorize(decision: Dict[str, Any], state: Dict[str, Any]) -> PolicyResult:
    tool_call = decision.get('tool_call') if isinstance(decision, dict) else {}
    tool = str((tool_call or {}).get('name') or '')
    arguments = (tool_call or {}).get('arguments') or {}
    patrol_state = str(state.get('patrol_state') or state.get('state') or 'unknown')

    if tool == 'emergency_stop':
        return PolicyResult(True, priority=10, interrupt=True, system_command='emergency_stop')
    if tool in {'/cmd_vel', 'cmd_vel', 'nav2_goal', 'delete_map', 'edit_route'}:
        return PolicyResult(False, '拒绝危险工具')
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
    if tool in {'get_system_status', 'get_patrol_status', 'get_voice_status', 'generate_local_status_reply'}:
        return PolicyResult(True)
    return PolicyResult(False, '未知工具')
