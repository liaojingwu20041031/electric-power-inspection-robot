import itertools
import os
import queue
import sys
import tempfile
import threading
import types
import unittest
import wave
from types import SimpleNamespace
from unittest.mock import Mock, patch

if 'ylhb_interfaces.msg' not in sys.modules:
    fake_interfaces = types.ModuleType('ylhb_interfaces')
    fake_msg = types.ModuleType('ylhb_interfaces.msg')
    fake_msg.SayText = type('SayText', (), {})
    fake_msg.TaskEvent = type('TaskEvent', (), {})
    fake_msg.TaskStatus = type('TaskStatus', (), {})
    fake_msg.VoiceStatus = type('VoiceStatus', (), {})
    sys.modules['ylhb_interfaces'] = fake_interfaces
    sys.modules['ylhb_interfaces.msg'] = fake_msg

from ylhb_llm.inspection_task_node import InspectionTaskNode
from ylhb_llm.voice_output_node import VoiceOutputNode
from ylhb_llm.patrol_voice import VoiceRequest
from ylhb_llm.voice_stability import (
    normalize_voice_text,
    safe_wav_duration_sec,
)


class VoiceStabilityTest(unittest.TestCase):
    def test_voice_text_normalization_only_cleans_asr_noise(self):
        self.assertEqual(normalize_voice_text(' 嗯，小零小零，后退！'), '小零小零后退')

    def test_invalid_wav_duration_uses_file_size_estimate(self):
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            path = tmp.name
        try:
            with wave.open(path, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(16000)
                wav.writeframes(b'\x00\x00' * 16000)
            self.assertTrue(0.9 <= safe_wav_duration_sec(path) <= 1.1)
        finally:
            os.unlink(path)

    def test_inspection_task_suppresses_repeated_emergency_stop_reply(self):
        node = InspectionTaskNode.__new__(InspectionTaskNode)
        node._last_emergency_stop_reply_time = 100.0
        node.emergency_stop_reply_cooldown_sec = 2.0

        self.assertFalse(
            node.should_publish_reply(
                'emergency_stop',
                '已收到停止指令。',
                now=101.0,
            )
        )
        self.assertTrue(
            node.should_publish_reply(
                'emergency_stop',
                '已收到停止指令。',
                now=102.1,
            )
        )

    def test_voice_output_deduplicates_queued_same_task_and_text(self):
        node = VoiceOutputNode.__new__(VoiceOutputNode)
        node._queued_say_keys = set()
        node._queue_sequence = itertools.count()
        node.queue = Mock()

        msg = SimpleNamespace(task_id='emergency_stop_1', text='已收到停止指令。', priority=5, interrupt=False)
        node.say_callback(msg)
        node.say_callback(msg)

        node.queue.put.assert_called_once()

    def test_assistant_chat_reply_is_never_split(self):
        node = VoiceOutputNode.__new__(VoiceOutputNode)
        node.split_long_tts = True
        node.preserve_long_task_tts_single_request = False
        node.tts_segment_max_chars = 10

        self.assertFalse(node.should_split_tts('assistant_chat_123', '很长的回答。' * 20))

    def test_tts_status_is_busy_during_synthesis(self):
        node = VoiceOutputNode.__new__(VoiceOutputNode)
        node.current_task_id = ''
        node.tts_model = 'tts'
        node.tts_voice = 'Serena'
        node.tts_language_type = 'Chinese'
        node.request_timeout_sec = 1.0
        node.enable_tts_cache = False
        node.tts_cache = {}
        node.playback_lock = threading.Lock()
        node.current_playback = None
        node.playback_generation = 0
        node.stop_event = threading.Event()
        node.statuses = []
        node.publish_status_once = lambda: node.statuses.append(node.current_task_id)
        node.get_logger = lambda: SimpleNamespace(info=lambda _msg: None, warn=lambda _msg: None)

        class FailingQwen:
            def available(self):
                return True

            def synthesize_speech_bytes(self, **_kwargs):
                self.busy_task = node.current_task_id
                raise RuntimeError('stop after checking busy state')

        node.qwen = FailingQwen()

        node.speak('短回答', 'assistant_chat_1')

        self.assertEqual(node.qwen.busy_task, 'assistant_chat_1')
        self.assertEqual(node.current_task_id, '')
        self.assertEqual(node.statuses[0], 'assistant_chat_1')
        self.assertEqual(node.statuses[-1], '')

    def _playback_node(self):
        node = VoiceOutputNode.__new__(VoiceOutputNode)
        node.enabled = True
        node.tts_enabled = True
        node.split_long_tts = True
        node.preserve_long_task_tts_single_request = True
        node.tts_segment_max_chars = 70
        node.audio_device = 'default'
        node.stop_event = threading.Event()
        node.playback_generation = 0
        node._queue_sequence = itertools.count()
        node.current_task_id = ''
        node.playback_lock = threading.Lock()
        node.current_playback = None
        node.statuses = []
        node.publish_status_once = lambda: node.statuses.append(node.current_task_id)
        node.get_logger = lambda: SimpleNamespace(info=lambda _msg: None, warn=lambda _msg: None)
        return node

    def test_interrupted_tts_result_is_not_saved_or_played(self):
        node = self._playback_node()
        node.tts_model = 'tts'
        node.tts_voice = 'Serena'
        node.tts_language_type = 'Chinese'
        node.request_timeout_sec = 1.0
        node.enable_tts_cache = False
        node.tts_cache = {}
        node.play_audio_file = Mock()

        def synthesize(**_kwargs):
            node.playback_generation += 1
            return b'old generated wav'

        node.qwen = SimpleNamespace(available=lambda: True, synthesize_speech_bytes=synthesize)
        with tempfile.TemporaryDirectory() as directory:
            inventory_path = os.path.join(directory, 'patrol.wav')
            node.play_request(VoiceRequest(
                'patrol:old', '旧播报', 40, False,
                '/missing.wav', inventory_path))

            self.assertFalse(os.path.exists(inventory_path))
            node.play_audio_file.assert_not_called()

    def test_expired_checkpoint_tts_is_not_played(self):
        node = self._playback_node()
        node.tts_model = 'tts'
        node.tts_voice = 'Serena'
        node.tts_language_type = 'Chinese'
        node.request_timeout_sec = 1.0
        node.enable_tts_cache = False
        node.tts_cache = {}
        node.play_audio_file = Mock()
        clock = [100.0]

        def synthesize(**_kwargs):
            clock[0] = 106.0
            return b'generated wav'

        node.qwen = SimpleNamespace(available=lambda: True, synthesize_speech_bytes=synthesize)

        with patch('ylhb_llm.voice_output_node.time.monotonic', side_effect=lambda: clock[0]):
            node.play_request(
                VoiceRequest(
                    'patrol:checkpoint', '已到达检查点。', 40, False,
                    max_delay_sec=5.0),
                enqueued_at=100.0,
            )

        node.play_audio_file.assert_not_called()

    def test_same_priority_requests_are_queued_fifo(self):
        node = self._playback_node()
        node._queued_say_keys = set()
        node.queue = queue.PriorityQueue()

        first = VoiceRequest('patrol:first', '第一条', 40, False)
        second = VoiceRequest('patrol:second', '第二条', 40, False)
        node.enqueue_request(first)
        node.enqueue_request(second)

        self.assertIs(node.queue.get_nowait()[2], first)
        self.assertIs(node.queue.get_nowait()[2], second)

    def test_existing_local_wav_does_not_call_qwen(self):
        node = self._playback_node()
        node.qwen = Mock()
        node.play_audio_file = Mock()
        with tempfile.NamedTemporaryFile(suffix='.wav') as wav:
            request = VoiceRequest('patrol:1', '固定播报', 40, False, wav.name)
            node.play_request(request)

        node.qwen.assert_not_called()
        node.play_audio_file.assert_called_once_with(
            wav.name, 'patrol:1', False, 0, 0.0)

    def test_missing_local_wav_falls_back_to_tts(self):
        node = self._playback_node()
        node.speak = Mock()

        node.play_request(VoiceRequest(
            'patrol:2', '回退播报', 40, False,
            '/missing.wav', '/inventory/patrol.wav'))

        node.speak.assert_called_once_with(
            '回退播报', 'patrol:2', '/inventory/patrol.wav', 0, 0.0)

    def test_missing_fixed_wav_is_synthesized_into_inventory(self):
        node = self._playback_node()
        node.tts_model = 'tts'
        node.tts_voice = 'Serena'
        node.tts_language_type = 'Chinese'
        node.request_timeout_sec = 1.0
        node.enable_tts_cache = False
        node.tts_cache = {}
        node.play_audio_file = Mock()
        node.qwen = SimpleNamespace(
            available=lambda: True,
            synthesize_speech_bytes=lambda **_kwargs: b'generated wav',
        )
        with tempfile.TemporaryDirectory() as directory:
            inventory_path = os.path.join(directory, 'patrol_route_started.wav')

            node.speak('固定播报', 'patrol:fixed', inventory_path)

            with open(inventory_path, 'rb') as stream:
                self.assertEqual(stream.read(), b'generated wav')
            node.play_audio_file.assert_called_once_with(
                inventory_path, 'patrol:fixed', False, 0, 0.0)

    def test_audio_file_delete_policy_and_busy_status(self):
        node = self._playback_node()
        proc = Mock()
        proc.wait.return_value = 0
        proc.poll.return_value = 0
        node._start_audio_process = Mock(return_value=(proc, 8.0, 1.0))
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as wav:
            local_path = wav.name
        node.play_audio_file(local_path, 'patrol:3', False)
        self.assertTrue(os.path.exists(local_path))
        self.assertEqual(node.statuses[0], 'patrol:3')
        self.assertEqual(node.statuses[-1], '')
        os.unlink(local_path)

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as wav:
            temporary_path = wav.name
        node.play_audio_file(temporary_path, 'tts:1', True)
        self.assertFalse(os.path.exists(temporary_path))

    def test_interrupt_terminates_current_playback(self):
        node = self._playback_node()
        node.interrupt_current_playback = True
        node.playback_generation = 0
        node.terminate_current_playback = Mock()

        node.interrupt_playback()

        self.assertEqual(node.playback_generation, 1)
        node.terminate_current_playback.assert_called_once()


if __name__ == '__main__':
    unittest.main()
