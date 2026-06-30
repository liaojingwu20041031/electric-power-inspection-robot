import json
import time
from typing import Any, Dict


ALLOWED_TOOLS = {
    'get_system_status',
    'get_patrol_status',
    'get_voice_status',
    'start_patrol_mode',
    'pause_patrol',
    'resume_patrol',
    'cancel_patrol',
    'emergency_stop',
    'send_motion_command',
    'generate_local_status_reply',
}

MOTION_COMMANDS = {'前进', '后退', '左转', '右转', '停止'}
RESPONSE_TYPES = {'tool_call', 'final_answer', 'status_reply', 'reject', 'ignore'}
SAFETY_LEVELS = {'emergency', 'normal', 'requires_confirm', 'blocked'}
RESULT_STATUSES = {'sent', 'done', 'rejected', 'failed', 'timeout', 'ok'}
LEGACY_RESPONSE_TYPES = {'tool': 'tool_call', 'final': 'final_answer'}
LEGACY_SAFETY_LEVELS = {'critical': 'emergency', 'safe': 'normal'}
REQUIRED_DECISION_FIELDS = {
    'schema_version',
    'decision_id',
    'response_type',
    'intent',
    'safety_level',
    'tool_call',
    'speak',
    'final_answer',
    'need_confirm',
    'reason_cn',
}


class SchemaError(ValueError):
    pass


def validate_decision(value: Any) -> Dict[str, Any]:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as exc:
            raise SchemaError(f'invalid json: {exc}') from exc
    if not isinstance(value, dict):
        raise SchemaError('AgentDecision must be an object')
    value = dict(value)
    if 'tool' in value and 'tool_call' not in value:
        value['tool_call'] = value.pop('tool')
    if 'final' in value and 'final_answer' not in value:
        value['final_answer'] = value.pop('final')
    value['response_type'] = LEGACY_RESPONSE_TYPES.get(
        str(value.get('response_type') or ''),
        value.get('response_type'),
    )
    value['safety_level'] = LEGACY_SAFETY_LEVELS.get(
        str(value.get('safety_level') or ''),
        value.get('safety_level'),
    )
    if isinstance(value.get('speak'), str):
        value['speak'] = {
            'reply_key': '',
            'text': value['speak'],
            'priority': 5,
            'interrupt': False,
        }

    missing = REQUIRED_DECISION_FIELDS - set(value)
    if missing:
        raise SchemaError('missing fields: ' + ', '.join(sorted(missing)))
    if str(value['schema_version']) != '1.0':
        raise SchemaError('unsupported schema_version')
    if str(value['response_type']) not in RESPONSE_TYPES:
        raise SchemaError('invalid response_type')
    if str(value['safety_level']) not in SAFETY_LEVELS:
        raise SchemaError('invalid safety_level')
    if not isinstance(value['need_confirm'], bool):
        raise SchemaError('need_confirm must be bool')

    tool_call = value['tool_call']
    if not isinstance(tool_call, dict):
        raise SchemaError('tool_call must be object')
    name = str(tool_call.get('name') or '')
    if name not in ALLOWED_TOOLS:
        raise SchemaError(f'unknown tool: {name}')
    arguments = tool_call.get('arguments', {})
    if arguments is None:
        arguments = {}
        tool_call['arguments'] = arguments
    if not isinstance(arguments, dict):
        raise SchemaError('tool_call.arguments must be object')
    if name == 'send_motion_command' and str(arguments.get('command') or '') not in MOTION_COMMANDS:
        raise SchemaError('unsupported motion command')

    speak = value['speak']
    if not isinstance(speak, dict):
        raise SchemaError('speak must be object')
    speak['reply_key'] = str(speak.get('reply_key') or '')
    speak['text'] = str(speak.get('text') or '')
    speak['priority'] = int(speak.get('priority') or 5)
    speak['interrupt'] = bool(speak.get('interrupt'))

    for field in ('decision_id', 'intent', 'final_answer', 'reason_cn'):
        value[field] = str(value[field])
    return value


def tool_result(
    tool_name: str,
    ok: bool,
    status: str,
    message: str,
    data: Dict[str, Any] | None = None,
    error_code: str = '',
) -> Dict[str, Any]:
    if str(status) not in RESULT_STATUSES:
        raise SchemaError('invalid tool result status')
    return {
        'schema_version': '1.0',
        'tool_name': str(tool_name),
        'ok': bool(ok),
        'status': str(status),
        'message': str(message),
        'data': data or {},
        'error_code': str(error_code),
        'timestamp': time.time(),
    }
