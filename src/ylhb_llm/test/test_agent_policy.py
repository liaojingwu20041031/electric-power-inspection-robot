from ylhb_llm.agent_policy import authorize


def decision(tool, **arguments):
    return {
        'schema_version': '1.0',
        'decision_id': 'd1',
        'response_type': 'tool',
        'intent': tool,
        'safety_level': 'normal',
        'tool_call': {'name': tool, 'arguments': arguments},
        'speak': '',
        'final_answer': '',
        'need_confirm': False,
        'reason_cn': '',
    }


def test_emergency_stop_is_always_allowed():
    result = authorize(decision('emergency_stop'), {'patrol_state': 'unknown'})

    assert result.allowed is True
    assert result.priority == 10
    assert result.interrupt is True


def test_pause_patrol_requires_active_state():
    result = authorize(decision('pause_patrol'), {'patrol_state': 'idle'})

    assert result.allowed is False
    assert result.reason


def test_cancel_patrol_allowed_when_paused():
    result = authorize(decision('cancel_patrol'), {'patrol_state': 'paused'})

    assert result.allowed is True


def test_motion_stop_is_not_patrol_cancel():
    result = authorize(decision('send_motion_command', command='停止'), {'patrol_state': 'running'})

    assert result.allowed is True
    assert result.system_command == ''


def test_dangerous_tool_is_rejected_even_if_schema_was_bypassed():
    result = authorize(decision('/cmd_vel'), {'patrol_state': 'running'})

    assert result.allowed is False
