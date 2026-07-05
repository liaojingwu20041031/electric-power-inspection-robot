import pytest

from ylhb_llm.agent_chat_schema import FIELDS, dedupe_key, make_agent_chat


def test_agent_chat_schema_fields_and_role():
    message = make_agent_chat('assistant', '收到', turn_id='t1', client_msg_id='c1')

    assert tuple(message) == FIELDS
    assert message['role'] == 'assistant'
    assert message['text'] == '收到'


def test_agent_chat_rejects_unknown_role():
    with pytest.raises(ValueError):
        make_agent_chat('bad', 'x')


def test_user_dedupe_key_prefers_client_msg_id():
    a = make_agent_chat('user', '开始', turn_id='t1', client_msg_id='c1')
    b = make_agent_chat('user', '开始巡逻', turn_id='t2', client_msg_id='c1')

    assert dedupe_key(a) == dedupe_key(b) == 'user:c1'
