import os
import sys
import tempfile
import types
import unittest
import wave
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


if __name__ == '__main__':
    unittest.main()
