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
    VoiceRoutingPolicy,
    classify_voice_intent,
    is_context_followup_text,
    safe_wav_duration_sec,
)


class VoiceStabilityTest(unittest.TestCase):
    def setUp(self):
        motion_aliases = (
            ('向前进', '前进'),
            ('左旋转', '左转'),
            ('前进', '前进'),
            ('停止', '停止'),
        )
        self.policy = VoiceRoutingPolicy(
            system_commands={
                '停止巡检': ('stop_robot_stack', '已发送停止巡检节点命令。'),
            },
            voice_close_words=('关闭语音模式',),
            safety_words=('停止', '停下', '刹车'),
            cancel_words=('取消任务', '不要了'),
            general_qa_words=('你能做什么', '有什么功能'),
            inspection_words=('开始巡检', '检查点', '检测', '漏油', '安全帽'),
            background_words=('AI不出来', '差不多'),
            followup_words=('确认', '继续', '暂停', '取消'),
            motion_aliases=motion_aliases,
            incomplete_motion_words=('旋转',),
        )

    def test_motion_aliases_are_normalized(self):
        self.assertEqual(classify_voice_intent('向前进', self.policy).route, 'motion')
        self.assertEqual(classify_voice_intent('向前进', self.policy).text, '前进')
        self.assertEqual(classify_voice_intent('左旋转', self.policy).text, '左转')

    def test_incomplete_rotation_is_not_sent_to_task_layer(self):
        result = classify_voice_intent('旋转', self.policy)
        self.assertEqual(result.route, 'unsupported_motion')
        self.assertEqual(result.feedback, '请说左转或右转。')

    def test_background_debug_talk_is_ignored(self):
        result = classify_voice_intent('真的是AI不出来那个差不多', self.policy)
        self.assertEqual(result.route, 'ignore')

    def test_inspection_and_general_qa_are_allowed(self):
        self.assertEqual(classify_voice_intent('开始巡检', self.policy).route, 'inspection_command')
        self.assertEqual(classify_voice_intent('检查点一开始检测', self.policy).route, 'inspection_command')
        self.assertEqual(classify_voice_intent('你能做什么', self.policy).route, 'general_qa')

    def test_stop_inspection_has_priority_over_motion_stop(self):
        result = classify_voice_intent('停止巡检', self.policy)
        self.assertEqual(result.route, 'system_command')
        self.assertEqual(result.system_command, 'stop_robot_stack')

    def test_context_followup_is_restricted(self):
        self.assertTrue(is_context_followup_text('继续', self.policy))
        self.assertTrue(is_context_followup_text('确认', self.policy))
        self.assertTrue(is_context_followup_text('取消', self.policy))
        self.assertFalse(is_context_followup_text('今天天气不错', self.policy))

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
