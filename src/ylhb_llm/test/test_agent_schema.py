import json

import pytest

from ylhb_llm.agent_schema import SchemaError, tool_result, validate_decision


def valid_decision(**overrides):
    data = {
        'schema_version': '1.0',
        'decision_id': 'd1',
        'response_type': 'tool_call',
        'intent': 'start_patrol',
        'safety_level': 'normal',
        'tool_call': {'name': 'start_patrol_mode', 'arguments': {}},
        'speak': {'reply_key': 'command.patrol_start', 'text': '准备开始巡逻。', 'priority': 5, 'interrupt': False},
        'final_answer': '',
        'need_confirm': False,
        'reason_cn': '用户要求开始巡逻',
    }
    data.update(overrides)
    return data


def test_validate_decision_rejects_missing_required_field():
    data = valid_decision()
    del data['decision_id']

    with pytest.raises(SchemaError):
        validate_decision(data)


def test_validate_decision_rejects_unknown_tool():
    data = valid_decision(tool_call={'name': 'cmd_vel', 'arguments': {}})

    with pytest.raises(SchemaError):
        validate_decision(data)


def test_validate_decision_accepts_dynamic_tool_schema():
    data = valid_decision(
        intent='rotate',
        tool_call={'name': 'rotate_relative', 'arguments': {'angle_deg': 90}},
    )

    result = validate_decision(
        data,
        {'rotate_relative'},
        {
            'rotate_relative': {
                'required': ['angle_deg'],
                'properties': {'angle_deg': {'type': 'number', 'minimum': -180, 'maximum': 180}},
            }
        },
    )

    assert result['tool_call']['name'] == 'rotate_relative'


def test_validate_decision_rejects_extra_closed_schema_argument():
    data = valid_decision(
        intent='rotate',
        tool_call={'name': 'rotate_relative', 'arguments': {'angle_deg': 90, 'unsafe': True}},
    )

    with pytest.raises(SchemaError, match='unexpected argument'):
        validate_decision(
            data,
            {'rotate_relative'},
            {
                'rotate_relative': {
                    'required': ['angle_deg'],
                    'properties': {'angle_deg': {'type': 'number'}},
                    'additionalProperties': False,
                }
            },
        )


def test_validate_decision_rejects_dangerous_tool_even_when_allowed():
    data = valid_decision(tool_call={'name': '/cmd_vel', 'arguments': {}})

    with pytest.raises(SchemaError):
        validate_decision(data, {'/cmd_vel'}, {})


def test_validate_decision_rejects_bad_motion_command():
    data = valid_decision(
        intent='motion',
        tool_call={'name': 'send_motion_command', 'arguments': {'command': '旋转'}},
    )

    with pytest.raises(SchemaError):
        validate_decision(data)


def test_validate_decision_accepts_json_string():
    result = validate_decision(json.dumps(valid_decision(), ensure_ascii=False))

    assert result['tool_call']['name'] == 'start_patrol_mode'


def test_validate_decision_converts_legacy_schema():
    result = validate_decision(valid_decision(response_type='tool', safety_level='critical', speak='已急停。'))

    assert result['response_type'] == 'tool_call'
    assert result['safety_level'] == 'emergency'
    assert result['speak']['text'] == '已急停。'


def test_tool_result_shape_is_stable():
    result = tool_result('pause_patrol', True, 'sent', '已发送')

    assert set(result) == {
        'schema_version',
        'tool_name',
        'ok',
        'status',
        'message',
        'data',
        'error_code',
        'timestamp',
    }
    assert result['tool_name'] == 'pause_patrol'


def test_tool_result_rejects_old_status():
    with pytest.raises(SchemaError):
        tool_result('pause_patrol', True, 'executed', '已发送')
