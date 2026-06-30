from ylhb_llm.robot_reply_style import BAD_TONE_WORDS, REPLIES, speak


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
