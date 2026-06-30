import json

from ylhb_llm.agent_schema import validate_decision
from ylhb_llm.inspection_agent_node import decide_local


def test_start_patrol_maps_to_agent_tool():
    decision = validate_decision(decide_local({'text': '开始巡逻'}, {}))

    assert decision['tool_call']['name'] == 'start_patrol_mode'
    assert decision['tool_call']['arguments']['profile'] == 'navigation'


def test_voice_payload_start_patrol_maps_to_agent_tool():
    decision = validate_decision(decide_local({'source': 'voice', 'text': '开始巡检'}, {}))

    assert decision['tool_call']['name'] == 'start_patrol_mode'


def test_patrol_control_words_map_to_patrol_tools():
    assert validate_decision(decide_local({'text': '暂停巡逻'}, {'patrol_state': 'running'}))['tool_call']['name'] == 'pause_patrol'
    assert validate_decision(decide_local({'text': '继续巡检'}, {'patrol_state': 'paused'}))['tool_call']['name'] == 'resume_patrol'
    assert validate_decision(decide_local({'text': '停止巡逻'}, {'patrol_state': 'running'}))['tool_call']['name'] == 'cancel_patrol'


def test_plain_stop_maps_to_motion_stop_not_cancel():
    decision = validate_decision(decide_local({'text': '停止'}, {'patrol_state': 'running'}))

    assert decision['tool_call'] == {'name': 'send_text_motion', 'arguments': {'command': '停止'}}


def test_status_query_is_local_final_answer():
    decision = validate_decision(decide_local({'text': '当前状态怎么样'}, {'patrol_state': 'running'}))

    assert decision['response_type'] == 'status_reply'
    assert decision['tool_call']['name'] == 'generate_local_status_reply'


def test_emergency_stop_is_interrupting_tool():
    decision = validate_decision(decide_local({'text': '急停'}, {}))

    assert decision['tool_call']['name'] == 'emergency_stop'
    assert decision['safety_level'] == 'emergency'
    assert decision['speak']['interrupt'] is True


def test_complex_question_requests_llm_fallback():
    assert decide_local({'text': '电力巡检报告应该怎么写'}, {}) is None
