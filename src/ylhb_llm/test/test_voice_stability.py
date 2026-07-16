import os
import sys
import tempfile
import types
import unittest
import wave
import threading
from types import SimpleNamespace
from unittest.mock import Mock

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


if __name__ == '__main__':
    unittest.main()
