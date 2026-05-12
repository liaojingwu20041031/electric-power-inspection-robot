import json
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ylhb_interfaces.msg import SayText, TaskStatus


SAFETY_WORDS = ('急停', '停止', '停下', '别动', '刹车')
VOICE_CLOSE_WORDS = (
    '关闭语音模式',
    '退出语音模式',
    '停止语音模式',
    '关闭语音',
    '退出语音',
    '关掉语音',
    '结束语音',
    '关机',
)
CANCEL_WORDS = ('取消任务', '取消当前任务', '不要了')
CHECKOUT_WORDS = ('多少钱', '结算', '总价', '一共', '付款')
MOTION_WORDS = ('前进', '后退', '左转', '右转')
SYSTEM_FEEDBACK_WORDS = (
    '怎么不说话',
    '你怎么不说话',
    '没声音',
    '听不到',
    '没有声音',
    '你没说话',
    '语音坏了',
)


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class VoiceCommandRouterNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_command_router_node')
        self.declare_parameter('voice_command_event_topic', '/retail_ai/voice_command_event')
        self.declare_parameter('text_command_topic', '/retail_ai/text_command')
        self.declare_parameter('sales_dialogue_status_topic', '/retail_ai/sales_dialogue_status')
        self.declare_parameter('system_mode_topic', '/retail_ai/system_mode')
        self.declare_parameter('task_status_topic', '/retail_ai/task_status')
        self.declare_parameter('say_text_topic', '/retail_ai/say_text')

        self.system_mode = 'ready'
        self.sales_status: Dict[str, Any] = {}
        self.recent_utterances = set()
        self.recent_utterance_order = []

        self.text_pub = self.create_publisher(String, self.get_parameter('text_command_topic').value, 10)
        self.say_pub = self.create_publisher(SayText, self.get_parameter('say_text_topic').value, 10)
        self.create_subscription(
            String,
            self.get_parameter('voice_command_event_topic').value,
            self.voice_event_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('sales_dialogue_status_topic').value,
            self.sales_status_callback,
            transient_qos(),
        )
        self.create_subscription(
            String,
            self.get_parameter('system_mode_topic').value,
            self.system_mode_callback,
            transient_qos(),
        )
        self.create_subscription(
            TaskStatus,
            self.get_parameter('task_status_topic').value,
            self.task_status_callback,
            10,
        )
        self.get_logger().info('语音命令路由节点已启动。')

    def voice_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid voice command event JSON: {exc}')
            return
        text = self.normalize_text(str(event.get('text') or ''))
        if not text:
            return
        utterance_id = str(event.get('utterance_id') or '')
        session_id = str(event.get('session_id') or '')
        dedupe_key = f'{session_id}:{utterance_id}'
        if dedupe_key in self.recent_utterances:
            self.get_logger().info(f'Ignoring duplicate voice utterance: {dedupe_key}')
            return
        self.remember_utterance(dedupe_key)

        if self.is_close_voice_session(text):
            self.publish_text_command(text, event, 'voice_close')
            self.say('voice_router', '已关闭语音模式。', priority=7)
            return
        if self.contains_any(text, SAFETY_WORDS):
            self.publish_text_command(text, event, 'global_safety')
            self.say('voice_router', '已停止。', priority=8)
            return
        if self.contains_any(text, CANCEL_WORDS):
            self.publish_text_command(text, event, 'global_cancel')
            self.say('voice_router', '已取消当前任务。', priority=7)
            return
        if self.contains_any(text, CHECKOUT_WORDS):
            self.publish_text_command(text, event, 'checkout')
            return
        if self.contains_any(text, MOTION_WORDS):
            self.publish_text_command(text, event, 'task_a_motion')
            return
        if self.contains_any(text, SYSTEM_FEEDBACK_WORDS):
            self.publish_text_command(text, event, 'system_feedback')
            return

        if self.system_mode == 'running':
            self.say('voice_router', '当前正在执行任务，请等待完成，或说取消任务。', priority=6)
            return

        self.publish_text_command(text, event, 'b2_dialogue')

    def publish_text_command(self, text: str, event: Dict[str, Any], route: str) -> None:
        command = {
            'schema_version': '1.0',
            'source': 'voice',
            'route': route,
            'session_id': str(event.get('session_id') or ''),
            'utterance_id': str(event.get('utterance_id') or ''),
            'text': text,
            'raw_asr_text': str(event.get('raw_asr_text') or text),
            'awakened': bool(event.get('awakened')),
            'contains_wake_phrase': bool(event.get('contains_wake_phrase')),
            'confidence': float(event.get('confidence') or 0.0),
            'timestamp': float(event.get('timestamp') or time.time()),
        }
        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.text_pub.publish(msg)
        self.get_logger().info(f'语音命令已路由到文本指令：{msg.data}')

    def sales_status_callback(self, msg: String) -> None:
        try:
            self.sales_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.sales_status = {}

    def system_mode_callback(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode:
            self.system_mode = mode

    def task_status_callback(self, msg: TaskStatus) -> None:
        if msg.status in ('completed', 'failed', 'rejected'):
            return

    def remember_utterance(self, key: str) -> None:
        self.recent_utterances.add(key)
        self.recent_utterance_order.append(key)
        while len(self.recent_utterance_order) > 100:
            old = self.recent_utterance_order.pop(0)
            self.recent_utterances.discard(old)

    def normalize_text(self, text: str) -> str:
        table = str.maketrans('', '', ' ，。！？!?、,. ')
        cleaned = text.strip().translate(table)
        for filler in ('呃', '嗯', '啊'):
            cleaned = cleaned.replace(filler, '')
        return cleaned

    def contains_any(self, text: str, words: tuple) -> bool:
        return any(word in text for word in words)

    def is_close_voice_session(self, text: str) -> bool:
        return any(word == text or word in text for word in VOICE_CLOSE_WORDS)

    def say(self, task_id: str, text: str, priority: int = 5) -> None:
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.priority = int(priority)
        msg.text = text
        self.say_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceCommandRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
