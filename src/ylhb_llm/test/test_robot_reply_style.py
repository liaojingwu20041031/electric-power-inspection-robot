from ylhb_llm.robot_reply_style import BAD_TONE_WORDS, REPLIES, prepare_speech_text, speak


def test_control_replies_are_short():
    for intent in ('patrol_start', 'patrol_pause', 'patrol_resume', 'patrol_cancel', 'motion_stop'):
        text = speak(intent)['text']
        assert text.count('。') <= 2


def test_emergency_reply_interrupts():
    reply = speak('emergency_stop')

    assert reply['text'] == '收到，已急停。'
    assert reply['priority'] == 10
    assert reply['interrupt'] is True


def test_fallback_reply_is_not_empty():
    assert speak('unknown_local')['text']


def test_replies_do_not_use_bad_tone_words():
    all_text = ''.join(reply[0] for reply in REPLIES.values())

    assert not any(word in all_text for word in BAD_TONE_WORDS)


def test_prepare_speech_text_keeps_display_content_out_of_short_tts():
    answer = """## 检查结果

变压器温度正常，未发现告警。第二段包含详细测量数据和处理建议。

```json
{"tool_name": "get_system_status"}
```

详情见 https://example.com/report
"""

    speech = prepare_speech_text(answer)

    assert len(speech) <= 60
    assert sum(speech.count(mark) for mark in '。！？') <= 2
    assert 'http' not in speech
    assert 'tool_name' not in speech
    assert '```' not in speech
    assert speech.endswith('详细内容请看屏幕。')
