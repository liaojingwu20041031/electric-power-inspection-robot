import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .agent_policy import authorize
from .agent_schema import SchemaError, tool_result, validate_decision
from .agent_state import AgentState
from .agent_tools import AgentTools
from .qwen_client import QwenClient, QwenClientError, parse_json_object


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_decision(
    intent: str,
    tool: str,
    text: str,
    speak: str = '',
    final_answer: str = '',
    response_type: str = 'tool_call',
    safety_level: str = 'normal',
    arguments: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        'schema_version': '1.0',
        'decision_id': f'{intent}_{int(time.time() * 1000)}',
        'response_type': response_type,
        'intent': intent,
        'safety_level': safety_level,
        'tool_call': {'name': tool, 'arguments': arguments or {}},
        'speak': {'reply_key': f'command.{intent}', 'text': speak, 'priority': 5, 'interrupt': safety_level == 'emergency'},
        'final_answer': final_answer,
        'need_confirm': False,
        'reason_cn': text,
    }


def decide_local(request: Dict[str, Any], state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    text = str(request.get('text') or request.get('command') or '').strip()
    normalized = text.replace(' ', '')
    if not normalized:
        return make_decision('empty', 'generate_local_status_reply', text, response_type='ignore')
    if any(word in normalized for word in ('急停', '紧急停止', '别动', '刹车')):
        return make_decision('emergency_stop', 'emergency_stop', text, '已发送急停。', safety_level='emergency')
    if normalized in {'停止', '停下', '停下来', '停'}:
        return make_decision('motion_stop', 'send_text_motion', text, '已停止运动。', arguments={'command': '停止'})
    if any(word in normalized for word in ('开始巡逻', '开始巡检', '启动巡逻', '启动巡检', '执行巡逻', '执行巡检')):
        return make_decision('patrol_start', 'start_patrol_mode', text, '已发送开始巡逻命令。', arguments={'profile': 'navigation'})
    if '暂停' in normalized and ('巡逻' in normalized or '巡检' in normalized):
        return make_decision('patrol_pause', 'pause_patrol', text, '已发送暂停巡逻命令。')
    if any(word in normalized for word in ('继续巡逻', '继续巡检', '恢复巡逻', '恢复巡检')):
        return make_decision('patrol_resume', 'resume_patrol', text, '已发送继续巡逻命令。')
    if any(word in normalized for word in ('取消巡逻', '取消巡检', '终止巡逻', '终止巡检', '停止巡逻', '停止巡检', '结束巡逻', '结束巡检')):
        return make_decision('patrol_cancel', 'cancel_patrol', text, '已发送取消巡逻命令。')
    for command in ('前进', '后退', '左转', '右转'):
        if command in normalized:
            return make_decision(f'motion_{command}', 'send_text_motion', text, f'执行{command}。', arguments={'command': command})
    if any(word in normalized for word in ('状态', '怎么样', '情况', '进度')):
        patrol_state = str(state.get('patrol_state') or 'unknown')
        answer = f'当前巡逻状态是 {patrol_state}。'
        return make_decision('status_query', 'generate_local_status_reply', text, answer, answer, 'status_reply')
    if any(word in normalized for word in ('你能做什么', '有什么功能', '功能', '怎么用')):
        answer = '我可以协助开始、暂停、继续、取消巡逻，执行短时运动，查询当前状态。'
        return make_decision('capability_query', 'generate_local_status_reply', text, answer, answer, 'final_answer')
    return None


class InspectionAgentNode(Node):
    def __init__(self) -> None:
        super().__init__('inspection_agent_node')
        self.declare_parameter('agent_request_topic', '/inspection_ai/agent_request')
        self.declare_parameter('agent_status_topic', '/inspection_ai/agent_status')
        self.declare_parameter('agent_event_topic', '/inspection_ai/agent_event')
        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('patrol_status_topic', '/patrol/status')
        self.declare_parameter('voice_session_status_topic', '/inspection_ai/voice_session_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('chat_model', 'qwen3.6-plus')
        self.declare_parameter('request_timeout_sec', 12.0)
        self.declare_parameter('enable_llm_fallback', False)
        self.declare_parameter('llm_json_retry_count', 1)

        self.state = AgentState()
        self.qwen = QwenClient(str(self.get_parameter('dashscope_base_url').value))
        self.chat_model = str(self.get_parameter('chat_model').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.enable_llm_fallback = bool(self.get_parameter('enable_llm_fallback').value)
        self.llm_json_retry_count = int(self.get_parameter('llm_json_retry_count').value)

        self.status_pub = self.create_publisher(String, self.get_parameter('agent_status_topic').value, latched_qos())
        self.event_pub = self.create_publisher(String, self.get_parameter('agent_event_topic').value, 10)
        self.system_pub = self.create_publisher(String, self.get_parameter('system_command_topic').value, 10)
        self.text_pub = self.create_publisher(String, self.get_parameter('text_command_topic').value, 10)
        self.say_pub = self.create_publisher(__import__('ylhb_interfaces.msg').msg.SayText, self.get_parameter('say_text_topic').value, 10)
        self.tools = AgentTools(self, self.state, self.system_pub, self.text_pub, self.say_pub, self.event_pub)

        self.create_subscription(String, self.get_parameter('agent_request_topic').value, self.request_callback, 10)
        self.create_subscription(String, self.get_parameter('system_status_topic').value, self.system_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('patrol_status_topic').value, self.patrol_status_callback, 10)
        self.create_subscription(String, self.get_parameter('voice_session_status_topic').value, self.voice_status_callback, latched_qos())
        self.publish_status()
        self.get_logger().info('inspection agent node started')

    def request_callback(self, msg: String) -> None:
        request = self.parse_payload(msg.data)
        self.state.latest_request = request
        try:
            decision = decide_local(request, self.state.policy_context()) or self.decide_llm(request)
            decision = validate_decision(decision)
            policy = authorize(decision, self.state.policy_context())
            if not policy.allowed:
                result = tool_result(decision['tool_call']['name'], False, 'rejected', policy.reason, error_code='policy_rejected')
                self.tools.publish_event(result)
                decision = dict(decision)
                decision['speak'] = {
                    'reply_key': 'policy.rejected',
                    'text': policy.reason or '当前指令已被安全策略拒绝。',
                    'priority': policy.priority,
                    'interrupt': policy.interrupt,
                }
            else:
                result = self.tools.execute(decision, policy)
            self.state.latest_decision = decision
            self.state.latest_result = result
            self.tools.say(decision, priority=policy.priority, interrupt=policy.interrupt)
        except (SchemaError, QwenClientError, ValueError) as exc:
            self.state.last_error = str(exc)
            self.tools.publish_event(tool_result('inspection_agent', False, 'rejected', str(exc), error_code='agent_error'))
        self.publish_status()

    def decide_llm(self, request: Dict[str, Any]) -> Dict[str, Any]:
        text = str(request.get('text') or '')
        if not self.enable_llm_fallback or not self.qwen.available():
            answer = '这个问题需要联网大模型回答，当前已关闭 LLM fallback。'
            return make_decision('complex_qa', 'generate_local_status_reply', text, answer, answer, 'final_answer')
        prompt = (
            '只输出 schema_version=1.0 的新版 AgentDecision JSON。不要 Markdown。'
            'response_type 只能用 tool_call/final_answer/status_reply/need_confirm/reject/ignore。'
            'safety_level 只能用 emergency/normal/requires_confirm/blocked。'
            'speak 必须是对象，包含 reply_key/text/priority/interrupt。'
            '只能 final_answer 回答复杂巡检知识问题，不要调用 ROS topic、cmd_vel 或 Nav2 goal。'
            f'用户问题：{text}'
        )
        last_error = ''
        for _ in range(max(1, self.llm_json_retry_count + 1)):
            raw = self.qwen.chat_completion(
                model=self.chat_model,
                messages=[{'role': 'user', 'content': prompt}],
                timeout_sec=self.request_timeout_sec,
                temperature=0.0,
                extra_body={'enable_thinking': False},
            )
            try:
                return validate_decision(parse_json_object(raw))
            except (SchemaError, ValueError) as exc:
                last_error = str(exc)
                prompt = '上次输出不是合法 AgentDecision JSON，请只输出合法 JSON。'
        raise SchemaError(last_error or 'LLM JSON invalid')

    @staticmethod
    def parse_payload(raw: str) -> Dict[str, Any]:
        try:
            data = json.loads(raw)
            if isinstance(data, dict):
                return data
        except json.JSONDecodeError:
            pass
        return {'schema_version': '1.0', 'source': 'text', 'text': raw}

    def system_status_callback(self, msg: String) -> None:
        self.state.system_status = self.parse_payload(msg.data)
        self.publish_status()

    def patrol_status_callback(self, msg: String) -> None:
        self.state.patrol_status = self.parse_payload(msg.data)
        self.publish_status()

    def voice_status_callback(self, msg: String) -> None:
        self.state.voice_status = self.parse_payload(msg.data)
        self.publish_status()

    def publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.state.snapshot(), ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionAgentNode()
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
