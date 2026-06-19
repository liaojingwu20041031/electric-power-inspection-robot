import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ylhb_interfaces.msg import SayText, TaskEvent, TaskStatus

from .qwen_client import QwenClient, QwenClientError


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class InspectionTaskNode(Node):
    """Generic LLM task scaffold for electric power inspection workflows."""

    def __init__(self) -> None:
        super().__init__('inspection_task_node')
        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('task_event_topic', '/inspection_ai/task_event')
        self.declare_parameter('task_status_topic', '/inspection_ai/task_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('task_context_status_topic', '/inspection_ai/task_context_status')
        self.declare_parameter('localized_objects_topic', '/perception/localized_objects')
        self.declare_parameter('system_mode_topic', '/inspection_ai/system_mode')
        self.declare_parameter('start_demo_service_name', '/inspection_ai/start_demo_task')
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('chat_model', 'qwen3.6-plus')
        self.declare_parameter('request_timeout_sec', 12.0)
        self.declare_parameter('enable_llm_parse', False)
        self.declare_parameter('publish_raw_json', True)

        self.system_mode = 'ready'
        self.latest_detection_json = ''
        self.latest_detection_time = 0.0
        self.task_counter = 0
        self.active_task_id = ''
        self.last_context: Dict[str, Any] = {
            'schema_version': '1.0',
            'state': 'idle',
            'active_task_id': '',
            'last_intent': '',
            'last_text': '',
            'last_message': 'inspection task layer ready',
            'timestamp': time.time(),
        }

        self.qwen = QwenClient(str(self.get_parameter('dashscope_base_url').value))
        self.chat_model = str(self.get_parameter('chat_model').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.enable_llm_parse = bool(self.get_parameter('enable_llm_parse').value)
        self.publish_raw_json = bool(self.get_parameter('publish_raw_json').value)

        self.task_event_pub = self.create_publisher(TaskEvent, self.get_parameter('task_event_topic').value, 10)
        self.say_pub = self.create_publisher(SayText, self.get_parameter('say_text_topic').value, 10)
        self.context_pub = self.create_publisher(String, self.get_parameter('task_context_status_topic').value, latched_qos())
        self.create_subscription(String, self.get_parameter('text_command_topic').value, self.text_command_callback, 10)
        self.create_subscription(TaskStatus, self.get_parameter('task_status_topic').value, self.task_status_callback, 10)
        self.create_subscription(String, self.get_parameter('localized_objects_topic').value, self.localized_objects_callback, 10)
        self.create_subscription(String, self.get_parameter('system_mode_topic').value, self.system_mode_callback, latched_qos())
        self.create_service(Trigger, self.get_parameter('start_demo_service_name').value, self.start_demo_callback)
        self.publish_context()
        self.get_logger().info('Inspection task scaffold started.')

    def text_command_callback(self, msg: String) -> None:
        payload = self.parse_command_payload(msg.data)
        text = str(payload.get('text') or payload.get('command') or msg.data).strip()
        if not text:
            return
        decision = self.decide_intent(text, payload)
        task_id = str(payload.get('task_id') or self.next_task_id(decision['intent']))
        self.active_task_id = task_id
        self.publish_task_event(task_id, decision, payload)
        self.say(task_id, decision['reply_cn'], priority=decision.get('priority', 5))
        self.update_context(
            state='task_event_published',
            active_task_id=task_id,
            last_intent=decision['intent'],
            last_text=text,
            last_message=decision['reply_cn'],
            decision=decision,
        )

    def parse_command_payload(self, raw: str) -> Dict[str, Any]:
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass
        return {'schema_version': '1.0', 'source': 'text', 'text': raw}

    def decide_intent(self, text: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        if self.enable_llm_parse and self.qwen.available():
            try:
                decision = self.qwen.parse_inspection_command(text, self.chat_model, self.request_timeout_sec)
                if isinstance(decision, dict) and decision.get('intent'):
                    return self.normalize_decision(text, decision)
            except QwenClientError as exc:
                self.get_logger().warn(f'LLM parse failed, fallback to rules: {exc}')

        normalized = text.replace(' ', '')
        if any(word in normalized for word in ('急停', '停止', '停下', '刹车')):
            return self.normalize_decision(text, {'intent': 'emergency_stop', 'reply_cn': '已收到停止指令。', 'confidence': 1.0, 'requires_ack': False})
        if any(word in normalized for word in ('取消', '终止')):
            return self.normalize_decision(text, {'intent': 'cancel_inspection', 'reply_cn': '已收到取消巡检任务指令。', 'confidence': 0.95})
        if '暂停' in normalized:
            return self.normalize_decision(text, {'intent': 'pause_inspection', 'reply_cn': '已收到暂停巡检任务指令。', 'confidence': 0.95})
        if any(word in normalized for word in ('恢复', '继续')):
            return self.normalize_decision(text, {'intent': 'resume_inspection', 'reply_cn': '已收到恢复巡检任务指令。', 'confidence': 0.95})
        if any(word in normalized for word in ('接管', '人工')):
            return self.normalize_decision(text, {'intent': 'manual_takeover', 'reply_cn': '已切换为人工接管请求。', 'confidence': 0.9})
        if any(word in normalized for word in ('开始巡检', '启动巡检', '执行巡检', '巡检任务')):
            return self.normalize_decision(text, {'intent': 'start_inspection', 'destination': 'route', 'target_name': str(payload.get('route_name') or '默认巡检路线'), 'reply_cn': '已生成巡检任务事件，等待执行层接管。', 'confidence': 0.9})
        if any(word in normalized for word in ('检查点', '到点', '开始检查', '检测')):
            return self.normalize_decision(text, {'intent': 'inspect_checkpoint', 'destination': 'checkpoint', 'target_name': str(payload.get('checkpoint_name') or '待配置检查点'), 'reply_cn': '已生成检查点检测事件。', 'confidence': 0.85})
        return self.normalize_decision(text, {'intent': 'inspection_query', 'reply_cn': '已收到巡检相关指令，当前框架仅发布通用任务事件，具体业务请在后续状态机中实现。', 'confidence': 0.55, 'requires_ack': False})

    def normalize_decision(self, text: str, decision: Dict[str, Any]) -> Dict[str, Any]:
        intent = str(decision.get('intent') or 'inspection_query').strip() or 'inspection_query'
        return {
            'schema_version': '1.0',
            'task_type': 'inspection',
            'intent': intent,
            'target_id': str(decision.get('target_id') or decision.get('checkpoint_id') or decision.get('route_id') or ''),
            'target_name': str(decision.get('target_name') or decision.get('checkpoint_name') or decision.get('route_name') or ''),
            'destination': str(decision.get('destination') or ''),
            'reply_cn': str(decision.get('reply_cn') or '已收到巡检任务指令。'),
            'confidence': float(decision.get('confidence') or 0.5),
            'requires_ack': bool(decision.get('requires_ack', intent not in ('inspection_query', 'emergency_stop'))),
            'raw_text': text,
            'timestamp': time.time(),
        }

    def publish_task_event(self, task_id: str, decision: Dict[str, Any], source_payload: Dict[str, Any]) -> None:
        msg = TaskEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.intent = decision['intent']
        msg.item_id = decision.get('target_id', '')
        msg.item_name = decision.get('target_name', '')
        msg.destination = decision.get('destination', '')
        msg.confidence = float(decision.get('confidence', 0.0))
        msg.source = str(source_payload.get('source') or 'text')
        msg.requires_ack = bool(decision.get('requires_ack', False))
        raw = {
            'decision': decision,
            'source_payload': source_payload,
            'latest_detection_age_sec': time.monotonic() - self.latest_detection_time if self.latest_detection_time else None,
            'latest_detection_json': self.latest_detection_json[:2000],
        }
        msg.raw_json = json.dumps(raw, ensure_ascii=False) if self.publish_raw_json else ''
        self.task_event_pub.publish(msg)

    def task_status_callback(self, msg: TaskStatus) -> None:
        self.update_context(state='task_status_received', active_task_id=msg.task_id, last_message=f'{msg.stage}: {msg.status} {msg.reason}'.strip())

    def localized_objects_callback(self, msg: String) -> None:
        self.latest_detection_json = msg.data
        self.latest_detection_time = time.monotonic()

    def system_mode_callback(self, msg: String) -> None:
        if msg.data.strip():
            self.system_mode = msg.data.strip()
            self.update_context(state=self.system_mode)

    def start_demo_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        payload = {'schema_version': '1.0', 'source': 'service', 'text': '开始巡检任务', 'route_name': '默认巡检路线'}
        decision = self.decide_intent(payload['text'], payload)
        task_id = self.next_task_id(decision['intent'])
        self.active_task_id = task_id
        self.publish_task_event(task_id, decision, payload)
        self.say(task_id, '已创建演示巡检任务事件。', priority=6)
        self.update_context(state='demo_task_started', active_task_id=task_id, last_intent=decision['intent'], last_text=payload['text'], last_message='已创建演示巡检任务事件。', decision=decision)
        response.success = True
        response.message = json.dumps({'task_id': task_id, 'decision': decision}, ensure_ascii=False)
        return response

    def say(self, task_id: str, text: str, priority: int = 5, interrupt: bool = False) -> None:
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.text = text
        msg.priority = int(priority)
        msg.interrupt = bool(interrupt)
        self.say_pub.publish(msg)

    def next_task_id(self, prefix: str) -> str:
        self.task_counter += 1
        safe_prefix = ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in prefix)[:32]
        return f'{safe_prefix}_{int(time.time())}_{self.task_counter:03d}'

    def update_context(self, **kwargs: Any) -> None:
        self.last_context.update(kwargs)
        self.last_context['schema_version'] = '1.0'
        self.last_context['system_mode'] = self.system_mode
        self.last_context['timestamp'] = time.time()
        self.publish_context()

    def publish_context(self) -> None:
        msg = String()
        msg.data = json.dumps(self.last_context, ensure_ascii=False)
        self.context_pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = InspectionTaskNode()
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
