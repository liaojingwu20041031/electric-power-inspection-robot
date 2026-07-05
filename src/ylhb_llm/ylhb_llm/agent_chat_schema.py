import time
from typing import Any, Dict


ROLES = {'user', 'assistant', 'tool', 'safety', 'system'}
FIELDS = (
    'schema_version',
    'turn_id',
    'client_msg_id',
    'role',
    'text',
    'intent',
    'tool_name',
    'status',
    'timestamp',
    'source',
    'raw',
)


def make_agent_chat(
    role: str,
    text: str,
    turn_id: str = '',
    client_msg_id: str = '',
    intent: str = '',
    tool_name: str = '',
    status: str = '',
    source: str = 'inspection_agent',
    raw: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    if role not in ROLES:
        raise ValueError(f'invalid agent chat role: {role}')
    return {
        'schema_version': '1.0',
        'turn_id': str(turn_id),
        'client_msg_id': str(client_msg_id),
        'role': role,
        'text': str(text),
        'intent': str(intent),
        'tool_name': str(tool_name),
        'status': str(status),
        'timestamp': time.time(),
        'source': str(source),
        'raw': raw or {},
    }


def dedupe_key(message: Dict[str, Any]) -> str:
    client_msg_id = str(message.get('client_msg_id') or '')
    if client_msg_id and str(message.get('role') or '') == 'user':
        return f'user:{client_msg_id}'
    return ':'.join(
        str(message.get(field) or '')
        for field in ('turn_id', 'role', 'tool_name', 'status', 'text')
    )
