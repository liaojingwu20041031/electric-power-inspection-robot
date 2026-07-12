import json
import queue
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict

import rclpy
from ament_index_python.packages import get_package_share_path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .agent_chat_schema import make_agent_chat
from .agent_schema import SchemaError, tool_result
from .agent_state import AgentState
from .agent_tools import AgentTools
from .inspection_agent_runtime import InspectionAgentRuntime, decide_local
from .inspection_agent_spec import InspectionAgentSpecBuilder
from .openai_tool_client import OpenAIToolClient, OpenAIToolClientError
from .route_toolpack import RouteCatalog, RouteToolPack
from .skill_toolpack import SkillToolPack
from ylhb_mobile_bridge.patrol_route_store import default_workspace_dir, resolve_route_file_path


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class InspectionAgentNode(Node):
    def __init__(self) -> None:
        super().__init__('inspection_agent_node')
        self.declare_parameter('agent_request_topic', '/inspection_ai/agent_request')
        self.declare_parameter('agent_status_topic', '/inspection_ai/agent_status')
        self.declare_parameter('agent_event_topic', '/inspection_ai/agent_event')
        self.declare_parameter('agent_chat_topic', '/inspection_ai/agent_chat')
        self.declare_parameter('motion_command_topic', '/inspection_ai/motion_command')
        self.declare_parameter('base_skill_command_topic', '/inspection_ai/base_skill_command')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('patrol_command_topic', '/patrol/command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('patrol_status_topic', '/patrol/status')
        self.declare_parameter('voice_session_status_topic', '/inspection_ai/voice_session_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('planner_provider_name', 'dashscope')
        self.declare_parameter('planner_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('planner_model', 'qwen3.7-plus')
        self.declare_parameter('planner_api_key_env', 'DASHSCOPE_API_KEY')
        self.declare_parameter('planner_api_key_required', True)
        self.declare_parameter('planner_chat_path', '/chat/completions')
        self.declare_parameter('planner_models_path', '/models')
        self.declare_parameter('planner_extra_body_json', '{"enable_thinking": false}')
        self.declare_parameter('request_timeout_sec', 12.0)
        self.declare_parameter('enable_llm_planner', False)
        self.declare_parameter('offline_safe_mode', True)
        self.declare_parameter('workspace_dir', '')
        self.declare_parameter('route_directory', '')
        self.declare_parameter('patrol_route_path', 'auto')
        self.declare_parameter('route_file_path', 'auto')  # deprecated alias
        self.declare_parameter('robot_capabilities_file', '')
        self.declare_parameter('max_agent_steps', 8)
        self.declare_parameter('max_side_effect_tools_per_turn', 4)
        self.declare_parameter('max_identical_tool_calls', 2)

        self.state = AgentState()
        self.planner = OpenAIToolClient(
            provider_name=str(self.get_parameter('planner_provider_name').value),
            base_url=str(self.get_parameter('planner_base_url').value),
            api_key_env=str(self.get_parameter('planner_api_key_env').value),
            api_key_required=bool(self.get_parameter('planner_api_key_required').value),
            chat_path=str(self.get_parameter('planner_chat_path').value),
            models_path=str(self.get_parameter('planner_models_path').value),
            extra_body_json=str(self.get_parameter('planner_extra_body_json').value),
        )
        self.planner_model = str(self.get_parameter('planner_model').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.enable_llm_planner = bool(self.get_parameter('enable_llm_planner').value)
        self.offline_safe_mode = bool(self.get_parameter('offline_safe_mode').value)
        workspace_dir = str(self.get_parameter('workspace_dir').value).strip()
        self.workspace_dir = Path(workspace_dir).expanduser() if workspace_dir else default_workspace_dir()
        route_directory = str(self.get_parameter('route_directory').value).strip()
        self.route_directory = Path(route_directory).expanduser() if route_directory else self.workspace_dir / 'maps'
        patrol_route_path = str(self.get_parameter('patrol_route_path').value).strip()
        route_file_path = str(self.get_parameter('route_file_path').value).strip()
        self.resolved_route_file = str(resolve_route_file_path(
            patrol_route_path or route_file_path or 'auto', self.route_directory,
        ))
        self.seen_request_ids: set[str] = set()
        self.seen_request_order: deque[str] = deque(maxlen=256)
        self.request_queue: queue.Queue = queue.Queue()
        self._worker_stop = threading.Event()
        self.last_error_tts: Dict[str, float] = {}

        self.status_pub = self.create_publisher(String, self.get_parameter('agent_status_topic').value, latched_qos())
        self.event_pub = self.create_publisher(String, self.get_parameter('agent_event_topic').value, 10)
        self.chat_pub = self.create_publisher(String, self.get_parameter('agent_chat_topic').value, 10)
        self.system_pub = self.create_publisher(String, self.get_parameter('system_command_topic').value, 10)
        self.motion_pub = self.create_publisher(String, self.get_parameter('motion_command_topic').value, 10)
        self.base_skill_pub = self.create_publisher(String, self.get_parameter('base_skill_command_topic').value, 10)
        self.patrol_pub = self.create_publisher(String, self.get_parameter('patrol_command_topic').value, 10)
        self.say_pub = self.create_publisher(__import__('ylhb_interfaces.msg').msg.SayText, self.get_parameter('say_text_topic').value, 10)
        self.capabilities_file = ''
        self.route_toolpack, self.tool_schemas = self.load_toolpacks()
        self.tools = AgentTools(
            self,
            self.state,
            self.system_pub,
            self.motion_pub,
            self.say_pub,
            self.event_pub,
            patrol_pub=self.patrol_pub,
            base_skill_pub=self.base_skill_pub,
            route_toolpack=self.route_toolpack,
            tool_schemas=self.tool_schemas,
        )
        self.agent_spec = InspectionAgentSpecBuilder(
            self.route_toolpack,
            self.tool_schemas,
            self.tools.registry,
        ).build()
        self.agent_runtime = InspectionAgentRuntime(
            self.planner,
            self.tools,
            self.state,
            self.agent_spec,
            self.tool_schemas,
            route_toolpack=self.route_toolpack,
            model=self.planner_model,
            timeout_sec=self.request_timeout_sec,
            enabled=self.enable_llm_planner,
            max_steps=int(self.get_parameter('max_agent_steps').value),
            max_side_effect_tools_per_turn=int(self.get_parameter('max_side_effect_tools_per_turn').value),
            max_identical_tool_calls=int(self.get_parameter('max_identical_tool_calls').value),
        )

        self._worker = threading.Thread(target=self.agent_worker, name='inspection-agent-worker', daemon=True)
        self._worker.start()

        self.create_subscription(String, self.get_parameter('agent_request_topic').value, self.request_callback, 10)
        self.create_subscription(String, self.get_parameter('system_status_topic').value, self.system_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('patrol_status_topic').value, self.patrol_status_callback, 10)
        self.create_subscription(String, self.get_parameter('voice_session_status_topic').value, self.voice_status_callback, latched_qos())
        self.publish_status()
        self.get_logger().info(
            'inspection agent node started: '
            f'planner provider={self.planner.provider_name} '
            f'model={self.planner_model} base_url={self.planner.base_url} '
            f'route_file={self.resolved_route_file} capabilities_file={self.capabilities_file}'
        )

    def request_callback(self, msg: String) -> None:
        request = self.parse_payload(msg.data)
        request_id = self.request_key(request)
        if request_id and request_id in self.seen_request_ids:
            return
        if request_id:
            if len(self.seen_request_order) == self.seen_request_order.maxlen:
                self.seen_request_ids.discard(self.seen_request_order[0])
            self.seen_request_order.append(request_id)
            self.seen_request_ids.add(request_id)
        self.state.latest_request = request
        turn_id = request_id or f'turn_{int(time.time() * 1000)}'
        client_msg_id = str(request.get('client_msg_id') or '')
        text = str(request.get('text') or request.get('command') or '')
        self.publish_chat(make_agent_chat('user', text, turn_id, client_msg_id, source=str(request.get('source') or 'user'), raw=request))
        self.request_queue.put((request, turn_id, client_msg_id))

    def agent_worker(self) -> None:
        while not self._worker_stop.is_set():
            item = self.request_queue.get()
            if item is None:
                return
            self.process_request(*item)

    def process_next_request(self) -> bool:
        try:
            item = self.request_queue.get_nowait()
        except queue.Empty:
            return False
        self.process_request(*item)
        return True

    def process_request(self, request: Dict[str, Any], turn_id: str, client_msg_id: str) -> None:
        try:
            turn = self.agent_runtime.run_turn(request)
            decision = turn.get('decision') or {}
            result = turn.get('result') or {}
            role = str(turn.get('role') or 'assistant')
            if role == 'system':
                self.publish_chat(make_agent_chat('system', str(turn.get('assistant_text') or ''), turn_id, client_msg_id, status=str(result.get('status') or ''), raw=result))
            else:
                self.publish_chat(make_agent_chat('assistant', str(turn.get('assistant_text') or result.get('message') or ''), turn_id, client_msg_id, intent=str(decision.get('intent') or ''), tool_name=str((decision.get('tool_call') or {}).get('name') or ''), status=str(result.get('status') or ''), raw=result))
            speak = (decision.get('speak') or {}).get('text')
            if speak:
                self.tools.say(decision)
        except (SchemaError, OpenAIToolClientError, ValueError, RuntimeError) as exc:
            error_text = self.format_exception(exc)
            self.state.last_error = error_text
            result = tool_result('inspection_agent', False, 'failed', error_text, error_code='agent_error')
            self.state.latest_result = result
            self.tools.publish_event(result)
            self.publish_chat(make_agent_chat('system', f'Planner 调用失败：{error_text}', turn_id, client_msg_id, status='failed', raw=result))
            logger = self.get_logger()
            if hasattr(logger, 'error'):
                logger.error(f'agent planner error: {error_text}')
            else:
                logger.info(f'agent planner error: {error_text}')
            self.say_error_throttled(error_text)
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

    def shutdown_worker(self) -> None:
        self._worker_stop.set()
        self.request_queue.put(None)
        self._worker.join(timeout=1.0)

    def load_toolpacks(self):
        route_toolpack = None
        route_schemas: Dict[str, Dict[str, Any]] = {}
        try:
            route_catalog = RouteCatalog.from_file(self.resolved_route_file, self.route_directory)
            route_toolpack = RouteToolPack(route_catalog)
            route_schemas = route_toolpack.tool_schemas()
            self.get_logger().info(f'agent route catalog: resolved_file={route_catalog.route_file_path}')
        except Exception as exc:
            self.get_logger().warning(f'route toolpack unavailable: {exc}')
        try:
            package_config = get_package_share_path('ylhb_llm') / 'config'
            configured = str(self.get_parameter('robot_capabilities_file').value).strip()
            capability_path = Path(configured).expanduser() if configured else package_config / 'robot_capabilities.yaml'
            if not capability_path.is_absolute():
                capability_path = package_config / capability_path
            schemas = SkillToolPack.from_file(str(capability_path), route_schemas).tool_schemas()
            self.capabilities_file = str(capability_path)
            self.get_logger().info(f'agent capabilities file: {capability_path}')
        except Exception as exc:
            self.get_logger().warning(f'skill toolpack unavailable: {exc}')
            schemas = {}
        return route_toolpack, schemas

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
    def request_key(request: Dict[str, Any]) -> str:
        return str(request.get('client_msg_id') or request.get('request_id') or request.get('utterance_id') or '')

    @staticmethod
    def format_exception(exc: Exception) -> str:
        text = str(exc).strip() or repr(exc)
        return f'{exc.__class__.__name__}: {text}'

    def say_error_throttled(self, reason: str) -> None:
        now = time.monotonic()
        previous = self.last_error_tts.get(reason, 0.0)
        if now - previous < 3.0:
            return
        self.last_error_tts[reason] = now
        self.tools.say(self.error_decision(reason), priority=7, interrupt=False)

    def publish_chat(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.chat_pub.publish(msg)

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
        payload = self.state.snapshot()
        payload['agent_spec_summary'] = self.agent_spec.summary() if hasattr(self, 'agent_spec') else {}
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionAgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_worker()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
