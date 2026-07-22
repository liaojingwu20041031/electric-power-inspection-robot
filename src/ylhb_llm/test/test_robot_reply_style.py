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
    assert '请看屏幕' not in speech


def test_prepare_speech_text_keeps_actionable_prefix_when_first_sentence_is_long():
    answer = '请先打开机器人热点并确认手机与机器人处于同一网络，然后在浏览器中输入页面显示的当前地址完成连接，并继续检查移动端页面是否能正常读取机器人状态。后续还有很多说明。'

    speech = prepare_speech_text(answer)

    assert speech.startswith('请先打开机器人热点')
    assert '结果较长' not in speech
    assert '请看屏幕' not in speech
    assert sum(speech.count(mark) for mark in '。！？') <= 2


def test_prepare_speech_text_uses_lead_sentence_before_markdown_status_list():
    answer = '''## 连接检查

根据实时连接信息，机器人端具备连接条件：

- 移动桥接服务正常运行（ok）
- TCP 端口可达
- 手机还需与机器人网络可达
'''

    speech = prepare_speech_text(answer)

    assert speech.startswith('根据实时连接信息，机器人端具备连接条件。')
    assert '请看屏幕' not in speech
    assert 'ok' not in speech
    assert 'TCP' not in speech


def test_prepare_speech_text_skips_document_preamble_and_speaks_first_step():
    answer = '''根据项目文档，使用 APP 连接设备的操作步骤如下：

1. 打开机器人现场控制界面，查看当前 APP 地址。
2. 确认手机与机器人处于可达网络。
3. 在手机 APP 中输入页面显示的地址。
'''

    speech = prepare_speech_text(answer)

    assert speech.startswith('打开机器人现场控制界面，查看当前 APP 地址。')
    assert '确认手机与机器人处于可达网络。' in speech
    assert '请看屏幕' not in speech
    assert '请按以下步骤操作' not in speech
    assert sum(speech.count(mark) for mark in '。！？') <= 2
    assert '根据项目文档' not in speech
    assert '步骤如下' not in speech


def test_prepare_speech_text_skips_please_follow_steps_preamble():
    answer = '''要用手机 APP 连接到机器人，请按以下步骤操作。
1. 打开机器人现场控制界面，查看当前 APP 地址。
2. 确认手机与机器人处于可达网络。
'''

    speech = prepare_speech_text(answer)

    assert speech == '打开机器人现场控制界面，查看当前 APP 地址。确认手机与机器人处于可达网络。'


def test_prepare_speech_text_removes_internal_component_enums_before_truncation():
    answer = '根据自检结果，机器人当前状态正常（mode=ready），但底层组件（bringup、navigation、perception）尚未启动。'

    speech = prepare_speech_text(answer)

    assert 'ready' not in speech
    assert 'bringup' not in speech
    assert 'navigation' not in speech
    assert 'perception' not in speech
