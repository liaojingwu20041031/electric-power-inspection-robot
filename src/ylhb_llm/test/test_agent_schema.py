import json

import pytest

from ylhb_llm.agent_schema import SchemaError, tool_result, validate_decision


def valid_decision(**overrides):
    data = {
        'schema_version': '1.0',
        'decision_id': 'd1',
        'response_type': 'tool',
        'intent': 'start_patrol',
        'safety_level': 'normal',
        'tool_call': {'name': 'start_patrol_mode', 'arguments': {}},
        'speak': '准备开始巡逻。',
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


def test_validate_decision_rejects_bad_motion_command():
    data = valid_decision(
        intent='motion',
        tool_call={'name': 'send_text_motion', 'arguments': {'command': '旋转'}},
    )

    with pytest.raises(SchemaError):
        validate_decision(data)


def test_validate_decision_accepts_json_string():
    result = validate_decision(json.dumps(valid_decision(), ensure_ascii=False))

    assert result['tool_call']['name'] == 'start_patrol_mode'


def test_tool_result_shape_is_stable():
    result = tool_result('pause_patrol', True, 'executed', '已发送')

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
