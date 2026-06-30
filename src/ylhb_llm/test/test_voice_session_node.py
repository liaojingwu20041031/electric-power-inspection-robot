import threading
import time
from types import SimpleNamespace

from ylhb_llm.voice_session_node import VoiceSessionNode


class FakeQwen:
    def __init__(self, available=True):
        self._available = available

    def available(self):
        return self._available


def response():
    return SimpleNamespace(success=False, message='')


def make_node(api_key=True):
    node = VoiceSessionNode.__new__(VoiceSessionNode)
    node.enabled = True
    node.qwen = FakeQwen(api_key)
    node.lock = threading.Lock()
    node.session_enabled = False
    node.awakened = False
    node.session_id = ''
    node.utterance_seq = 0
    node.asr_fail_count = 0
    node.last_error = ''
    node.last_active_at = 0.0
    node.pause_listen_until = 0.0
    node.last_event_published_at = 0.0
    node.context_followup_until = 0.0
    node.in_context_followup = False
    node.wake_phrase = '小零小零'
    node.start_prompt_cooldown_sec = 8.0
    node.repeat_start_feedback = False
    node.min_voice_sec = 0.5
    node.vad_silence_sec = 0.85
    node.command_min_voice_sec = 0.8
    node.command_vad_silence_sec = 1.25
    node.tts_tail_pause_sec = 0.9
    node.is_tts_playing = False
    node.last_tts_speaking = False
    node.last_start_prompt_at = 0.0
    node.say_messages = []
    node.say = lambda task_id, text, priority=5, interrupt=False: node.say_messages.append((task_id, text, priority, interrupt))
    node.publish_count = 0
    node.publish_status = lambda: setattr(node, 'publish_count', node.publish_count + 1)
    node.set_state = lambda state: setattr(node, 'state', state)
    return node


def test_start_when_already_enabled_keeps_session_and_does_not_speak_by_default():
    node = make_node()
    node.session_enabled = True
    node.session_id = 'voice_old'
    node.utterance_seq = 7
    node.asr_fail_count = 2

    res = VoiceSessionNode.start_callback(node, None, response())

    assert res.success is True
    assert node.session_id == 'voice_old'
    assert node.utterance_seq == 7
    assert node.asr_fail_count == 2
    assert node.say_messages == []


def test_repeat_start_feedback_obeys_cooldown():
    node = make_node()
    node.session_enabled = True
    node.repeat_start_feedback = True
    node.last_start_prompt_at = 0.0

    VoiceSessionNode.start_callback(node, None, response())

    assert node.say_messages[-1][1] == '语音模式已经开启。'


def test_stop_then_start_creates_new_session():
    node = make_node()
    node.session_enabled = True
    node.session_id = 'voice_old'

    VoiceSessionNode.stop_session(node, 'off', say=False)
    VoiceSessionNode.start_callback(node, None, response())

    assert node.session_enabled is True
    assert node.session_id != 'voice_old'
    assert node.utterance_seq == 0


def test_missing_api_key_rejects_start():
    node = make_node(api_key=False)

    res = VoiceSessionNode.start_callback(node, None, response())

    assert res.success is False
    assert 'DASHSCOPE_API_KEY' in res.message


def test_command_listening_uses_longer_vad_window():
    node = make_node()

    assert VoiceSessionNode.effective_vad_timing(node, False) == (0.5, 0.85)
    assert VoiceSessionNode.effective_vad_timing(node, True) == (0.8, 1.25)


def test_tts_finished_keeps_listener_paused_for_tail_audio():
    node = make_node()
    node.last_tts_speaking = True
    before = time.monotonic()

    VoiceSessionNode.voice_status_callback(node, SimpleNamespace(speaking=False))

    assert node.pause_listen_until >= before + 0.8


def test_debug_audio_prune_keeps_latest_files(tmp_path):
    node = make_node()
    node.debug_audio_dir = str(tmp_path)
    node.debug_audio_keep = 2
    files = []
    for index in range(3):
        path = tmp_path / f'{index}.wav'
        path.write_bytes(b'x')
        files.append(path)

    VoiceSessionNode.prune_debug_audio(node)

    assert len(list(tmp_path.glob('*.wav'))) == 2
