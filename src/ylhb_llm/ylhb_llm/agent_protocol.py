"""Shared Agent operation/result state vocabulary."""

OPERATION_STATES = {
    'created', 'sent', 'accepted', 'running', 'succeeded', 'failed', 'canceled', 'timeout',
}
TERMINAL_OPERATION_STATES = {'succeeded', 'failed', 'canceled', 'timeout'}
TOOL_RESULT_STATUSES = {
    'sent', 'accepted', 'running', 'submitted', 'succeeded', 'failed', 'canceled', 'timeout',
    'ok', 'warning', 'rejected',
}
LEGACY_OPERATION_STATES = {'done': 'succeeded', 'cancelled': 'canceled'}

OPERATION_TRANSITIONS = {
    'created': {'sent', 'failed'},
    'sent': {'accepted', 'running', 'succeeded', 'failed', 'canceled', 'timeout'},
    'accepted': {'running', 'succeeded', 'failed', 'canceled', 'timeout'},
    'running': {'succeeded', 'failed', 'canceled', 'timeout'},
}
