import threading
import time
import json
import subprocess
from types import SimpleNamespace

from ylhb_llm.voice_session_node import VoiceSessionNode


class FakeQwen:
    def __init__(self, available=True):
        self._available = available

    def available(self):
        return self._available


class FakeKws:
    def __init__(self, ready=True, detected=True, error=''):
        self.ready = ready
        self.detected = detected
        self.error = error
        self.listen_calls = 0
        self.stop_calls = 0

    def listen(self, _should_stop):
        self.listen_calls += 1
        return self.detected

    def stop(self):
        self.stop_calls += 1


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


def response():
    return SimpleNamespace(success=False, message='')


def make_node(api_key=True):
    node = VoiceSessionNode.__new__(VoiceSessionNode)
    node.enabled = True
    node.qwen = FakeQwen(api_key)
    node.local_kws_enabled = True
    node.local_kws = FakeKws()
    node.local_kws_error = ''
    node.silent_start = True
    node.lock = threading.Lock()
    node.session_enabled = False
    node.awakened = False
    node.session_id = ''
    node.utterance_seq = 0
    node.asr_fail_count = 0
    node.last_error = ''
    node.last_active_at = 0.0
    node.pause_listen_until = 0.0
    node.last_request_published_at = 0.0
    node.awaiting_turn_id = ''
    node.awaiting_response_deadline = 0.0
    node.awaiting_agent_response_received = False
    node.awaiting_answer_received = False
    node.answer_received_at = 0.0
    node.response_tts_started = False
    node.resume_after_tts_at = 0.0
    node.agent_response_timeout_sec = 30.0
    node.post_event_listen_pause_sec = 3.0
    node.context_followup_until = 0.0
    node.in_context_followup = False
    node.context_followup_timeout_sec = 12.0
    node.wake_phrase = '小零小零'
    node.wake_aliases = []
    node.wake_match_threshold = 0.55
    node.start_prompt_cooldown_sec = 8.0
    node.repeat_start_feedback = False
    node.min_voice_sec = 0.5
    node.vad_silence_sec = 0.85
    node.command_min_voice_sec = 0.8
    node.command_vad_silence_sec = 1.25
    node.tts_tail_pause_sec = 0.9
    node.is_tts_playing = False
    node.is_recording = False
    node.last_tts_speaking = False
    node.last_start_prompt_at = 0.0
    node.debug_state_transitions = False
    node.voice_cue_enabled = True
    node.audio_output_device = 'plughw:0,0'
    node.wake_cue_path = '/tmp/wake.wav'
    node.standby_cue_path = '/tmp/standby.wav'
    node.cue_calls = []
    node.play_local_cue = lambda path: node.cue_calls.append(path)
    node.current_recording_proc = None
    node.say_messages = []
    node.say = lambda task_id, text, priority=5, interrupt=False: node.say_messages.append((task_id, text, priority, interrupt))
    node.publish_count = 0
    node.publish_status = lambda force=False: setattr(node, 'publish_count', node.publish_count + 1)
    node.set_state = lambda state: setattr(node, 'state', state)
    node.agent_request_pub = FakePublisher()
    node.get_logger = lambda: SimpleNamespace(info=lambda _msg: None, warn=lambda _msg: None)
    return node


def make_status_node():
    node = make_node()
    node.state = 'WAIT_WAKE'
    node.last_asr_text = ''
    node.last_published_text = ''
    node.last_status_payload_json = ''
    node.status_pub = FakePublisher()
    return node


class FakeLogger:
    def __init__(self):
        self.infos = []

    def info(self, message):
        self.infos.append(message)


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


def test_silent_start_does_not_repeat_feedback():
    node = make_node()
    node.session_enabled = True
    node.repeat_start_feedback = True
    node.last_start_prompt_at = 0.0

    VoiceSessionNode.start_callback(node, None, response())

    assert node.say_messages == []


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


def test_local_kws_failure_rejects_start_without_cloud_fallback():
    node = make_node()
    node.local_kws = FakeKws(ready=False, error='tokens.txt 不存在')

    res = VoiceSessionNode.start_callback(node, None, response())

    assert res.success is False
    assert '本地唤醒不可用' in res.message
    assert 'tokens.txt 不存在' in res.message


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


def test_publish_status_skips_identical_payload():
    node = make_status_node()

    VoiceSessionNode.publish_status(node)
    VoiceSessionNode.publish_status(node)

    assert len(node.status_pub.messages) == 1


def test_publish_status_sends_changed_payload():
    node = make_status_node()

    VoiceSessionNode.publish_status(node)
    node.state = 'RECORDING'
    VoiceSessionNode.publish_status(node)

    assert len(node.status_pub.messages) == 2
    assert json.loads(node.status_pub.messages[-1])['state'] == 'RECORDING'
    assert json.loads(node.status_pub.messages[-1])['agent_voice_state'] == 'recording'


def test_publish_status_force_sends_even_when_unchanged():
    node = make_status_node()

    VoiceSessionNode.publish_status(node)
    VoiceSessionNode.publish_status(node, force=True)

    assert len(node.status_pub.messages) == 2


def test_agent_voice_state_mapping_covers_main_states():
    node = make_status_node()

    cases = {
        'WAIT_WAKE': ('waiting_wake', '待唤醒'),
        'RECORDING': ('recording', '录音中'),
        'ASR_PROCESSING': ('recognizing', '识别中'),
        'WAITING_RESPONSE': ('responding', '等待响应'),
    }
    for state, expected in cases.items():
        node.state = state
        assert VoiceSessionNode.agent_voice_state(node) == expected
    node.last_error = 'arecord failed'
    assert VoiceSessionNode.agent_voice_state(node) == ('error', '异常')


def test_default_state_logs_skip_high_frequency_states():
    node = make_status_node()
    node.last_logged_state = ''
    node.debug_state_transitions = False
    node.publish_status = lambda force=False: None
    logger = FakeLogger()
    node.get_logger = lambda: logger

    VoiceSessionNode.set_state(node, 'LISTENING')
    VoiceSessionNode.set_state(node, 'WAITING_RESPONSE')
    VoiceSessionNode.set_state(node, 'RECORDING')

    assert logger.infos == ['连续语音状态切换：RECORDING']


def test_wait_wake_uses_local_kws_without_cloud_asr():
    node = make_node()
    node.session_enabled = True
    node.state = 'WAIT_WAKE'
    node.transcribe_pcm = lambda _audio: (_ for _ in ()).throw(
        AssertionError('WAIT_WAKE must not use ASR'))

    detected = VoiceSessionNode.wait_for_local_wake(node)

    assert detected is True
    assert node.local_kws.listen_calls == 1
    assert node.awakened is True
    assert node.state == 'LISTENING'
    assert node.pause_listen_until >= time.monotonic() + 0.08
    assert node.cue_calls == ['/tmp/wake.wav']
    assert node.say_messages == []


def test_command_after_local_wake_publishes_once_and_waits_for_agent():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.session_id = 'voice_1'

    VoiceSessionNode.handle_asr_text(node, '查询机器人状态')

    payload = json.loads(node.agent_request_pub.messages[0])
    assert payload['text'] == '查询机器人状态'
    assert payload['source'] == 'voice'
    assert payload['request_id'] == 'utt_0001'
    assert node.awaiting_turn_id == 'utt_0001'
    assert node.state == 'WAITING_RESPONSE'


def test_waiting_for_agent_rejects_second_command():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.awaiting_turn_id = 'utt_0001'
    node.state = 'WAITING_RESPONSE'

    VoiceSessionNode.handle_asr_text(node, '第二个问题')

    assert node.agent_request_pub.messages == []


def test_matching_agent_answer_waits_until_tts_and_tail_finish():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.awaiting_turn_id = 'utt_0001'
    node.state = 'WAITING_RESPONSE'

    VoiceSessionNode.agent_chat_callback(node, SimpleNamespace(data=json.dumps({
        'turn_id': 'utt_0001', 'role': 'assistant', 'text': '完整回答',
    })))
    VoiceSessionNode.voice_status_callback(node, SimpleNamespace(speaking=True))
    VoiceSessionNode.voice_status_callback(node, SimpleNamespace(speaking=False))

    assert node.awaiting_answer_received is True
    assert node.response_tts_started is True
    assert node.state == 'WAITING_RESPONSE'
    node.resume_after_tts_at = time.monotonic() - 0.1
    VoiceSessionNode.update_waiting_response(node)
    assert node.awaiting_turn_id == ''
    assert node.awakened is True
    assert node.in_context_followup is True
    assert node.context_followup_until >= time.monotonic() + 11.0
    assert node.state == 'LISTENING'


def test_progress_response_clears_first_response_timeout_but_keeps_waiting_for_terminal_result():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.awaiting_turn_id = 'utt_0001'
    node.awaiting_response_deadline = time.monotonic() - 1.0
    node.state = 'WAITING_RESPONSE'

    for status in ('waiting_feedback', 'sent', 'accepted', 'running'):
        VoiceSessionNode.agent_chat_callback(node, SimpleNamespace(data=json.dumps({
            'turn_id': 'utt_0001',
            'role': 'assistant',
            'status': status,
            'text': '正在等待真实反馈',
        })))

    VoiceSessionNode.update_waiting_response(node)

    assert node.awaiting_agent_response_received is True
    assert node.awaiting_answer_received is False
    assert node.awaiting_turn_id == 'utt_0001'
    assert '超时' not in node.last_error


def test_pure_wake_alias_after_kws_is_ignored_and_keeps_listening():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.wake_aliases = ['小林小林', '小玲小玲']

    VoiceSessionNode.handle_asr_text(node, '小玲小玲')

    assert node.agent_request_pub.messages == []
    assert node.awakened is True
    assert node.state == 'LISTENING'


def test_repeated_wake_prefix_is_stripped_but_real_followup_is_kept():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.in_context_followup = True
    node.session_id = 'voice_1'

    VoiceSessionNode.handle_asr_text(node, '小零小零小零小零旋转回刚刚的位置')

    payload = json.loads(node.agent_request_pub.messages[-1])
    assert payload['text'] == '旋转回刚刚的位置'
    assert payload['contains_wake_phrase'] is True
    assert payload['interaction_phase'] == 'context_followup'


def test_end_phrases_are_forwarded_to_agent_instead_of_hard_coded():
    for phrase in ('关闭这个聊天', '关闭这个语音对话', '那不聊了', '关闭语音模式'):
        node = make_node()
        node.session_enabled = True
        node.awakened = True
        node.session_id = 'voice_1'

        VoiceSessionNode.handle_asr_text(node, phrase)

        assert json.loads(node.agent_request_pub.messages[-1])['text'] == phrase
        assert node.state == 'WAITING_RESPONSE'
        assert node.cue_calls == []


def test_voice_session_tool_commands_apply_confirmed_end_and_close():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    feedback = []
    node.publish_status = lambda force=False: feedback.append(dict(getattr(node, 'voice_operation_feedback', {})))

    VoiceSessionNode.voice_session_command_callback(node, SimpleNamespace(data=json.dumps({
        'command': 'end_voice_conversation', 'operation_id': 'op_end',
    })))

    assert node.session_enabled is True
    assert node.state == 'WAIT_WAKE'
    assert node.cue_calls == ['/tmp/standby.wav']
    assert feedback[-1]['operation_id'] == 'op_end'
    assert feedback[-1]['state'] == 'succeeded'
    assert node.voice_operation_feedback['operation_id'] == 'op_end'

    node.awakened = True
    VoiceSessionNode.voice_session_command_callback(node, SimpleNamespace(data=json.dumps({
        'command': 'close_voice_mode', 'operation_id': 'op_close',
    })))

    assert node.session_enabled is False
    assert node.state == 'OFF'
    assert node.cue_calls == ['/tmp/standby.wav', '/tmp/standby.wav']
    assert feedback[-1]['operation_id'] == 'op_close'
    assert node.voice_operation_feedback['operation_id'] == 'op_close'


def test_followup_timeout_plays_standby_once():
    node = make_node()
    node.session_enabled = True
    node.awakened = True
    node.in_context_followup = True
    node.context_followup_until = time.monotonic() - 0.1

    VoiceSessionNode.update_followup_window(node)

    assert node.state == 'WAIT_WAKE'
    assert node.cue_calls == ['/tmp/standby.wav']


def test_local_cue_uses_output_device_and_bounded_sync_playback(tmp_path, monkeypatch):
    node = make_node()
    cue = tmp_path / 'cue.wav'
    cue.write_bytes(b'RIFF')
    calls = []
    node.play_local_cue = VoiceSessionNode.play_local_cue.__get__(node)
    monkeypatch.setattr(subprocess, 'run', lambda argv, **kwargs: calls.append((argv, kwargs)) or SimpleNamespace(returncode=0))

    node.play_local_cue(str(cue))

    assert calls == [([
        'aplay', '-q', '-D', 'plughw:0,0', str(cue),
    ], {
        'stdout': subprocess.DEVNULL,
        'stderr': subprocess.PIPE,
        'timeout': 1.0,
        'check': False,
        'text': True,
    })]


def test_stop_immediately_releases_kws_and_vad_process():
    node = make_node()
    node.session_enabled = True

    class Proc:
        def __init__(self):
            self.terminated = 0

        def poll(self):
            return None

        def terminate(self):
            self.terminated += 1

    proc = Proc()
    node.current_recording_proc = proc

    VoiceSessionNode.stop_session(node, 'off', say=False)

    assert node.local_kws.stop_calls == 1
    assert proc.terminated == 1
    assert node.state == 'OFF'
