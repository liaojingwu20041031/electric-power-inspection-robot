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
from .robot_reply_style import speak as styled_speak


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
        'speak': styled_speak(intent, speak),
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
        return make_decision('emergency_stop', 'emergency_stop', text, safety_level='emergency')
    return None


class InspectionAgentNode(Node):
    def __init__(self) -> None:
        super().__init__('inspection_agent_node')
        self.declare_parameter('agent_request_topic', '/inspection_ai/agent_request')
        self.declare_parameter('agent_status_topic', '/inspection_ai/agent_status')
        self.declare_parameter('agent_event_topic', '/inspection_ai/agent_event')
        self.declare_parameter('motion_command_topic', '/inspection_ai/motion_command')
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
        self.motion_pub = self.create_publisher(String, self.get_parameter('motion_command_topic').value, 10)
        self.say_pub = self.create_publisher(__import__('ylhb_interfaces.msg').msg.SayText, self.get_parameter('say_text_topic').value, 10)
        self.tools = AgentTools(self, self.state, self.system_pub, self.motion_pub, self.say_pub, self.event_pub)

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
            result = tool_result('inspection_agent', False, 'failed', str(exc), error_code='agent_error')
            self.state.latest_result = result
            self.tools.publish_event(result)
            self.tools.say(self.error_decision(str(exc)), priority=7, interrupt=False)
        finally:
            decision = self.state.latest_decision or {}
            result = self.state.latest_result or {}
            self.get_logger().info(
                'agent turn: text="%s", response_type=%s, tool=%s, result=%s'
                % (
                    str(request.get('text') or ''),
                    str(decision.get('response_type') or ''),
                    str((decision.get('tool_call') or {}).get('name') or ''),
                    str(result.get('status') or ''),
                )
            )
        self.publish_status()

    def decide_llm(self, request: Dict[str, Any]) -> Dict[str, Any]:
        text = str(request.get('text') or '')
        if not self.enable_llm_fallback or not self.qwen.available():
            raise QwenClientError('LLM unavailable')
        prompt = (
            '你是电力巡检机器人语言智能体。只输出 schema_version=1.0 的 AgentDecision JSON，不要 Markdown。'
            'response_type 只能用 tool_call/final_answer/status_reply/reject/ignore。'
            'safety_level 只能用 emergency/normal/requires_confirm/blocked。'
            'tool_call.name 只能是 get_system_status/get_patrol_status/get_voice_status/'
            'start_patrol_mode/pause_patrol/resume_patrol/cancel_patrol/emergency_stop/'
            'send_motion_command/generate_local_status_reply。'
            '短时运动 command 只能是 前进/后退/左转/右转/停止。'
            '支持巡逻开始/暂停/继续/取消、急停、短时运动、系统/巡逻/语音状态查询、最终回答。'
            '不支持旋转角度、任意导航点、地图或路线修改、直接 /cmd_vel。'
            '遇到“转个一百八十度”“后退旋转”等枚举外运动，输出 reject 或 final_answer 澄清，不能改写成后退。'
            'final_answer/status_reply/reject 使用 generate_local_status_reply 工具。'
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
    def error_decision(reason: str) -> Dict[str, Any]:
        text = '语言模型暂不可用，未执行动作。'
        return {
            'schema_version': '1.0',
            'decision_id': f'agent_error_{int(time.time() * 1000)}',
            'response_type': 'reject',
            'intent': 'agent_error',
            'safety_level': 'blocked',
            'tool_call': {'name': 'generate_local_status_reply', 'arguments': {}},
            'speak': {'reply_key': 'agent.error', 'text': text, 'priority': 7, 'interrupt': False},
            'final_answer': text,
            'need_confirm': False,
            'reason_cn': reason,
        }

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
