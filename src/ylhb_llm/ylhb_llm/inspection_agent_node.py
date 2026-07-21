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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import yaml

from .agent_chat_schema import make_agent_chat
from .agent_schema import SchemaError, tool_result
from .agent_state import AgentState
from .agent_tools import AgentTools
from .agent_operation_manager import AgentOperationManager
from .agent_protocol import TERMINAL_OPERATION_STATES
from .inspection_agent_runtime import InspectionAgentRuntime, decide_local
from .inspection_agent_spec import InspectionAgentSpecBuilder
from .openai_tool_client import OpenAIToolClient, OpenAIToolClientError
from .route_toolpack import RouteCatalog, RouteToolPack
from .skill_toolpack import SkillToolPack
from .robot_status_aggregator import RobotStatusAggregator
from .robot_knowledge import RobotKnowledgeIndex
from .robot_diagnostics import RobotDiagnosticEngine
from .robot_recovery import RecoveryCatalog
from .ros_params import declare_string_array_parameter
from ylhb_mobile_bridge.network_status import NetworkStatusProvider
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
        self.declare_parameter('voice_session_command_topic', '/inspection_ai/voice_session_command')
        self.declare_parameter('motion_command_topic', '/inspection_ai/motion_command')
        self.declare_parameter('base_skill_command_topic', '/inspection_ai/base_skill_command')
        self.declare_parameter('system_command_topic', '/inspection_ai/system_command')
        self.declare_parameter('patrol_command_topic', '/patrol/command')
        self.declare_parameter('system_status_topic', '/inspection_ai/system_status')
        self.declare_parameter('patrol_status_topic', '/patrol/status')
        self.declare_parameter('voice_session_status_topic', '/inspection_ai/voice_session_status')
        self.declare_parameter('base_skill_status_topic', '/inspection_ai/base_skill_status')
        self.declare_parameter('zlac_status_topic', '/zlac8015d/status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('localized_objects_topic', '/perception/localized_objects')
        self.declare_parameter('task_status_topic', '/inspection_ai/task_status')
        self.declare_parameter('status_default_max_age_sec', 2.5)
        self.declare_parameter('operation_history_limit', 128)
        self.declare_parameter('operation_poll_sec', 0.1)
        self.declare_parameter('operation_ack_timeout_sec', 8.0)
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
        declare_string_array_parameter(self, 'knowledge_globs')
        self.declare_parameter('diagnostics_config_file', '')
        self.declare_parameter('recovery_config_file', '')
        self.declare_parameter('mobile_bridge_port', 8000)
        self.declare_parameter('enable_agent_health_monitor', True)
        self.declare_parameter('enable_agent_auto_recovery', False)
        self.declare_parameter('health_monitor_interval_sec', 2.0)
        self.declare_parameter('health_issue_debounce_sec', 5.0)
        self.declare_parameter('health_recovery_cooldown_sec', 60.0)
        self.declare_parameter('zlac_fault_topic', '/zlac8015d/fault')
        self.declare_parameter('local_app_status_topic', '/mobile_bridge/local_app_status')
        self.declare_parameter('cloud_status_topic', '/mobile_bridge/cloud_status')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('max_agent_steps', 12)
        self.declare_parameter('max_side_effect_tools_per_turn', 4)
        self.declare_parameter('max_identical_tool_calls', 2)

        self.state = AgentState()
        self.status_default_max_age_sec = float(self.get_parameter('status_default_max_age_sec').value)
        self.operation_manager = AgentOperationManager(
            max_operations=int(self.get_parameter('operation_history_limit').value),
        )
        self.status_aggregator = RobotStatusAggregator(self.status_default_max_age_sec)
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
        self.enable_agent_health_monitor = bool(self.get_parameter('enable_agent_health_monitor').value)
        self.enable_agent_auto_recovery = bool(self.get_parameter('enable_agent_auto_recovery').value)
        self.health_issue_debounce_sec = float(self.get_parameter('health_issue_debounce_sec').value)
        self.health_recovery_cooldown_sec = float(self.get_parameter('health_recovery_cooldown_sec').value)
        self.health_clock = time.monotonic
        self.health_issue_first_seen: Dict[str, float] = {}
        self.health_published_incidents: set[str] = set()
        self.health_auto_recovered_incidents: set[str] = set()
        self.health_last_recovery_at: Dict[str, float] = {}
        workspace_dir = str(self.get_parameter('workspace_dir').value).strip()
        self.workspace_dir = Path(workspace_dir).expanduser() if workspace_dir else default_workspace_dir()
        if not self.workspace_dir.exists():
            self.workspace_dir = self._git_workspace_root(Path(__file__)) or self.workspace_dir
        route_directory = str(self.get_parameter('route_directory').value).strip()
        self.route_directory = Path(route_directory).expanduser() if route_directory else self.workspace_dir / 'maps'
        patrol_route_path = str(self.get_parameter('patrol_route_path').value).strip()
        route_file_path = str(self.get_parameter('route_file_path').value).strip()
        resolved_route_file = resolve_route_file_path(
            patrol_route_path or route_file_path or 'auto',
            self.route_directory,
            required=False,
        )
        self.resolved_route_file = str(resolved_route_file or '')
        self.seen_request_ids: set[str] = set()
        self.seen_request_order: deque[str] = deque(maxlen=256)
        self.request_queue: queue.Queue = queue.Queue()
        self.pending_turn_context: Dict[str, Any] | None = None
        self.pending_turn_lock = threading.Lock()
        self._worker_stop = threading.Event()
        self.last_error_tts: Dict[str, float] = {}

        self.status_pub = self.create_publisher(String, self.get_parameter('agent_status_topic').value, latched_qos())
        self.event_pub = self.create_publisher(String, self.get_parameter('agent_event_topic').value, 10)
        self.health_status_pub = self.create_publisher(String, '/inspection_ai/health_status', latched_qos())
        self.diagnostic_event_pub = self.create_publisher(String, '/inspection_ai/diagnostic_event', 10)
        self.chat_pub = self.create_publisher(String, self.get_parameter('agent_chat_topic').value, 10)
        self.system_pub = self.create_publisher(String, self.get_parameter('system_command_topic').value, 10)
        self.voice_session_pub = self.create_publisher(
            String, self.get_parameter('voice_session_command_topic').value, 10)
        self.motion_pub = self.create_publisher(String, self.get_parameter('motion_command_topic').value, 10)
        self.base_skill_pub = self.create_publisher(String, self.get_parameter('base_skill_command_topic').value, 10)
        self.patrol_pub = self.create_publisher(String, self.get_parameter('patrol_command_topic').value, 10)
        self.say_pub = self.create_publisher(__import__('ylhb_interfaces.msg').msg.SayText, self.get_parameter('say_text_topic').value, 10)
        self.capabilities_file = ''
        package_config = get_package_share_path('ylhb_llm') / 'config'
        diagnostics_path = self._config_path(
            str(self.get_parameter('diagnostics_config_file').value), package_config,
            'agent_diagnostics.yaml')
        recovery_path = self._config_path(
            str(self.get_parameter('recovery_config_file').value), package_config,
            'agent_recovery.yaml')
        diagnostics_config = self._load_yaml(diagnostics_path)
        diagnostics_config['mobile_bridge_port'] = int(self.get_parameter('mobile_bridge_port').value)
        self.status_freshness = dict(diagnostics_config.get('freshness') or {})
        self.status_aggregator.configure_expected_components(
            diagnostics_config.get('expected_components') or {})
        knowledge_globs = list(self.get_parameter('knowledge_globs').value or [])
        self.knowledge_index = RobotKnowledgeIndex(self.workspace_dir, knowledge_globs)
        self.network_status_provider = NetworkStatusProvider()
        self.diagnostic_engine = RobotDiagnosticEngine(
            self.status_aggregator, self.network_status_provider, diagnostics_config)
        self.recovery_catalog = RecoveryCatalog.from_file(recovery_path)
        self.route_toolpack, self.tool_schemas = self.load_toolpacks()
        recovery_schema = self.tool_schemas.get('recover_component')
        if recovery_schema is not None:
            recovery_schema.setdefault('properties', {}).setdefault('component', {})['enum'] = self.recovery_catalog.names()
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
            operation_manager=self.operation_manager,
            status_aggregator=self.status_aggregator,
            voice_session_pub=self.voice_session_pub,
            knowledge_index=self.knowledge_index,
            diagnostic_engine=self.diagnostic_engine,
            recovery_catalog=self.recovery_catalog,
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
        self.create_timer(
            max(0.05, float(self.get_parameter('operation_poll_sec').value)),
            self.poll_pending_operation,
        )
        if self.enable_agent_health_monitor:
            self.create_timer(
                max(0.5, float(self.get_parameter('health_monitor_interval_sec').value)),
                self.health_monitor_tick,
            )

        self.create_subscription(String, self.get_parameter('agent_request_topic').value, self.request_callback, 10)
        self.create_subscription(String, self.get_parameter('system_status_topic').value, self.system_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('patrol_status_topic').value, self.patrol_status_callback, 10)
        self.create_subscription(String, self.get_parameter('voice_session_status_topic').value, self.voice_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('base_skill_status_topic').value, self.base_skill_status_callback, 10)
        self.create_subscription(String, self.get_parameter('zlac_status_topic').value, self.chassis_status_callback, 10)
        self.create_subscription(String, self.get_parameter('zlac_fault_topic').value, self.chassis_fault_callback, 10)
        self.create_subscription(String, self.get_parameter('local_app_status_topic').value, self.local_app_status_callback, latched_qos())
        self.create_subscription(String, self.get_parameter('cloud_status_topic').value, self.cloud_status_callback, latched_qos())
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('amcl_pose_topic').value, self.amcl_pose_callback, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)
        self.create_subscription(String, self.get_parameter('localized_objects_topic').value, self.perception_callback, 10)
        self.create_subscription(String, self.get_parameter('task_status_topic').value, self.task_status_callback, 10)
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
        if decide_local(request, self.state.policy_context()) is not None:
            self.process_request(request, turn_id, client_msg_id)
            return
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

    def process_request(
        self,
        request: Dict[str, Any],
        turn_id: str,
        client_msg_id: str,
        operation: Dict[str, Any] | None = None,
    ) -> None:
        try:
            turn = (
                self.agent_runtime.resume_turn(operation)
                if operation is not None
                else self.agent_runtime.run_turn(request)
            )
            decision = turn.get('decision') or {}
            result = turn.get('result') or {}
            if turn.get('state') == 'waiting_feedback':
                operation_id = str(turn.get('pending_operation_id') or '')
                with self.pending_turn_lock:
                    if not self.pending_turn_context or self.pending_turn_context.get('operation_id') != operation_id:
                        self.pending_turn_context = {
                            'operation_id': operation_id,
                            'request': request,
                            'turn_id': turn_id,
                            'client_msg_id': client_msg_id,
                        }
            elif operation is not None:
                with self.pending_turn_lock:
                    self.pending_turn_context = None
            role = str(turn.get('role') or 'assistant')
            display_text = str(
                turn.get('display_text') or turn.get('assistant_text') or result.get('message') or '')
            if role == 'system':
                self.publish_chat(make_agent_chat('system', display_text, turn_id, client_msg_id, status=str(result.get('status') or ''), raw=result))
            else:
                self.publish_chat(make_agent_chat('assistant', display_text, turn_id, client_msg_id, intent=str(decision.get('intent') or ''), tool_name=str((decision.get('tool_call') or {}).get('name') or ''), status=str(result.get('status') or ''), raw=result))
            speak = (decision.get('speak') or {}).get('text')
            if speak and not self.voice_request_stopped(request):
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
            if not self.voice_request_stopped(request):
                self.say_error_throttled(error_text)
        finally:
            decision = self.state.latest_decision or {}
            result = self.state.latest_result or {}
            self.get_logger().info(
                'agent turn: text="%s", response_type=%s, tool=%s, arguments=%s, result=%s, error=%s'
                % (
                    str(request.get('text') or ''),
                    str(decision.get('response_type') or ''),
                    str((decision.get('tool_call') or {}).get('name') or ''),
                    json.dumps((decision.get('tool_call') or {}).get('arguments') or {}, ensure_ascii=False),
                    str(result.get('status') or ''),
                    str(result.get('error_code') or ''),
                )
            )
        self.publish_status()

    def voice_request_stopped(self, request: Dict[str, Any]) -> bool:
        return (
            str(request.get('source') or '') == 'voice'
            and (self.state.voice_status or {}).get('enabled') is False
        )

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
    def _config_path(configured: str, package_config: Path, default_name: str) -> Path:
        path = Path(str(configured or default_name)).expanduser()
        return path if path.is_absolute() else package_config / path

    @staticmethod
    def _load_yaml(path: Path) -> Dict[str, Any]:
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            return data if isinstance(data, dict) else {}
        except (OSError, yaml.YAMLError):
            return {}

    @staticmethod
    def _git_workspace_root(start: Path) -> Path | None:
        for parent in start.resolve().parents:
            if (parent / '.git').exists():
                return parent
        return None

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
        try:
            self.state.system_status['topic_publishers'] = {
                'scan': self.count_publishers(str(self.get_parameter('scan_topic').value)),
                'imu': self.count_publishers(str(self.get_parameter('imu_topic').value)),
                'odom': self.count_publishers(str(self.get_parameter('odom_topic').value)),
            }
        except (AttributeError, RuntimeError):
            pass
        self.status_aggregator.update('system_status', self.state.system_status, max_age_sec=self._max_age('system_status'))
        self.update_operation_from_feedback(self.state.system_status)
        self.publish_status()

    def patrol_status_callback(self, msg: String) -> None:
        self.state.patrol_status = self.parse_payload(msg.data)
        self.status_aggregator.update('patrol_status', self.state.patrol_status, max_age_sec=self._max_age('system_status'))
        self.update_operation_from_feedback(self.state.patrol_status)
        self.publish_status()

    def voice_status_callback(self, msg: String) -> None:
        self.state.voice_status = self.parse_payload(msg.data)
        self.status_aggregator.update('voice_status', self.state.voice_status, max_age_sec=self._max_age('voice'))
        feedback = self.state.voice_status.get('agent_operation_feedback') or {}
        if isinstance(feedback, dict):
            self.update_operation_from_feedback(feedback)
        self.publish_status()

    def base_skill_status_callback(self, msg: String) -> None:
        payload = self.parse_payload(msg.data)
        self.status_aggregator.update('base_skill_status', payload, max_age_sec=self._max_age('chassis'))
        self.update_operation_from_feedback(payload)

    def chassis_status_callback(self, msg: String) -> None:
        payload = self.parse_payload(msg.data)
        if 'state' not in payload:
            payload['state'] = str(payload.get('text') or '').split(maxsplit=1)[0] or 'unknown'
        self.status_aggregator.update('chassis_status', payload, max_age_sec=self._max_age('chassis'))

    def chassis_fault_callback(self, msg: String) -> None:
        self.status_aggregator.update('chassis_fault', self.parse_payload(msg.data))

    def local_app_status_callback(self, msg: String) -> None:
        self.status_aggregator.update('local_app_status', self.parse_payload(msg.data))

    def cloud_status_callback(self, msg: String) -> None:
        self.status_aggregator.update('cloud_status', self.parse_payload(msg.data))

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.status_aggregator.update('odom', {'x': pose.position.x, 'y': pose.position.y}, max_age_sec=self._max_age('odom'))

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.status_aggregator.update('amcl_pose', {'x': pose.position.x, 'y': pose.position.y}, max_age_sec=self._max_age('localization'))

    def scan_callback(self, _msg: LaserScan) -> None:
        self.status_aggregator.update('scan', {'state': 'ok'}, max_age_sec=self._max_age('scan'))

    def imu_callback(self, _msg: Imu) -> None:
        self.status_aggregator.update('imu', {'state': 'ok'}, max_age_sec=self._max_age('imu'))

    def perception_callback(self, msg: String) -> None:
        self.status_aggregator.update('perception', self.parse_payload(msg.data), max_age_sec=self._max_age('perception'))

    def task_status_callback(self, msg: String) -> None:
        self.status_aggregator.update('task_status', self.parse_payload(msg.data))

    def _max_age(self, key: str) -> float:
        freshness = getattr(self, 'status_freshness', {})
        return float(freshness.get(f'{key}_sec', freshness.get(key, self.status_default_max_age_sec if hasattr(self, 'status_default_max_age_sec') else 2.5)))

    def update_operation_from_feedback(self, payload: Dict[str, Any]) -> None:
        operation_id = str(payload.get('operation_id') or '')
        raw_state = str(payload.get('state') or payload.get('status') or '')
        if not operation_id or not raw_state:
            return
        try:
            current = self.operation_manager.get(operation_id)
        except KeyError:
            return
        tool_name = str(current.get('tool_name') or '')
        state = {
            'done': 'succeeded',
            'completed': 'succeeded',
            'cancelled': 'canceled',
            'rejected': 'failed',
            'paused': 'succeeded' if tool_name == 'pause_patrol' else 'running',
            'running': 'succeeded' if tool_name in {'resume_patrol', 'start_route'} else 'running',
            'canceled': 'succeeded' if tool_name == 'cancel_patrol' else 'canceled',
        }.get(raw_state, raw_state)
        try:
            operation = self.operation_manager.update(operation_id, state, payload)
        except (KeyError, ValueError):
            return
        if operation.state in TERMINAL_OPERATION_STATES:
            if operation.tool_name == 'recover_component' and hasattr(self, 'diagnostic_event_pub'):
                AgentTools.publish_json(self.diagnostic_event_pub, {
                    'schema_version': '1.0',
                    'event': 'auto_recovery_finished',
                    'component': str(operation.arguments.get('component') or ''),
                    'operation_id': operation.operation_id,
                    'status': operation.state,
                    'message': str(payload.get('message') or ''),
                    'timestamp': time.time(),
                })
            self.enqueue_operation_feedback(self.operation_manager.get(operation_id))
            target_id = str(operation.arguments.get('target_operation_id') or '')
            if target_id and operation.tool_name in {'emergency_stop', 'stop_motion', 'cancel_patrol'}:
                try:
                    target = self.operation_manager.update(
                        target_id,
                        'canceled',
                        {
                            'ok': False,
                            'status': 'canceled',
                            'message': f'操作被 {operation.tool_name} 中断',
                            'operation_id': target_id,
                        },
                    )
                except (KeyError, ValueError):
                    return
                self.enqueue_operation_feedback(self.operation_manager.get(target.operation_id))

    def poll_pending_operation(self) -> None:
        with self.pending_turn_lock:
            pending = dict(self.pending_turn_context or {})
        if not pending:
            return
        try:
            operation = self.operation_manager.get(str(pending['operation_id']))
        except KeyError:
            return
        if operation['state'] in TERMINAL_OPERATION_STATES:
            if operation['state'] == 'timeout':
                self.cancel_timed_out_operation(operation)
            self.enqueue_operation_feedback(operation)

    def cancel_timed_out_operation(self, operation: Dict[str, Any]) -> None:
        correlation = {
            'schema_version': '1.0',
            'source': 'inspection_agent_timeout_guard',
            'request_id': str(operation.get('operation_id') or ''),
            'target_operation_id': str(operation.get('operation_id') or ''),
        }
        tool_name = str(operation.get('tool_name') or '')
        if tool_name in {'rotate_relative', 'move_relative'}:
            AgentTools.publish_json(
                self.base_skill_pub, {**correlation, 'command': 'stop_motion', 'arguments': {}})
        elif tool_name in {'go_to_checkpoint', 'start_route', 'start_patrol_mode'}:
            AgentTools.publish_json(self.system_pub, {**correlation, 'command': 'cancel_patrol'})

    def enqueue_operation_feedback(self, operation: Dict[str, Any]) -> None:
        with self.pending_turn_lock:
            pending = self.pending_turn_context
            if (
                not pending
                or str(operation.get('operation_id') or '') != str(pending.get('operation_id') or '')
            ):
                return
            self.pending_turn_context = None
        self.request_queue.put((
            pending['request'], pending['turn_id'], pending['client_msg_id'], operation,
        ))

    def publish_status(self) -> None:
        msg = String()
        payload = self.state.snapshot()
        payload['robot_summary'] = self.status_aggregator.mode_aware_summary()
        payload['active_operations'] = self.operation_manager.list_active()
        payload['agent_spec_summary'] = self.agent_spec.summary() if hasattr(self, 'agent_spec') else {}
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def health_monitor_tick(self) -> None:
        if not getattr(self, 'enable_agent_health_monitor', False):
            return
        report = self.diagnostic_engine.run_self_check('all')
        AgentTools.publish_json(self.health_status_pub, report)
        now = self.health_clock()
        active_keys = set()
        for issue in report.get('issues') or []:
            key = f"{issue.get('code')}:{issue.get('component')}"
            active_keys.add(key)
            first_seen = self.health_issue_first_seen.get(key)
            if first_seen is None:
                self.health_issue_first_seen[key] = now
                continue
            if now - first_seen < self.health_issue_debounce_sec:
                continue
            if key not in self.health_published_incidents:
                AgentTools.publish_json(self.diagnostic_event_pub, {
                    'schema_version': '1.0', 'event': 'diagnostic_issue',
                    'diagnostic_id': report.get('diagnostic_id'),
                    'issue': issue, 'timestamp': time.time(),
                })
                self.health_published_incidents.add(key)
            if getattr(self, 'enable_agent_auto_recovery', False):
                self.try_auto_recover(key, issue, report, now)
        for key in set(self.health_issue_first_seen) - active_keys:
            self.health_issue_first_seen.pop(key, None)
            self.health_published_incidents.discard(key)
            self.health_auto_recovered_incidents.discard(key)

    def try_auto_recover(self, incident_key: str, issue: Dict[str, Any], report: Dict[str, Any], now: float) -> None:
        component = str(issue.get('recovery_component') or '')
        if not component or issue.get('recoverable') is not True:
            return
        catalog = getattr(self, 'recovery_catalog', None)
        if catalog is None or component not in catalog.names():
            return
        recovery = catalog.get(component)
        if not recovery.get('auto_allowed') or incident_key in self.health_auto_recovered_incidents:
            return
        if recovery.get('requires_no_active_patrol') and self.state.patrol_state() in {
            'starting', 'command_sent', 'running', 'paused', 'returning_home',
        }:
            return
        system = self.status_aggregator.raw('system_status')
        if recovery.get('requires_supervisor_ownership') and system.get('mobile_bridge_owner') != 'supervisor':
            return
        cooldown = max(
            self.health_recovery_cooldown_sec,
            float(recovery.get('cooldown_sec') or 0.0),
        )
        if now - self.health_last_recovery_at.get(component, 0.0) < cooldown:
            return
        operation = self.operation_manager.create(
            f"health_{report.get('diagnostic_id')}",
            f"health_{component}",
            'recover_component',
            {'component': component, 'diagnostic_id': report.get('diagnostic_id')},
            float(recovery.get('timeout_sec') or 30.0),
        )
        self.operation_manager.mark_sent(operation.operation_id)
        self.health_auto_recovered_incidents.add(incident_key)
        self.health_last_recovery_at[component] = now
        AgentTools.publish_json(self.diagnostic_event_pub, {
            'schema_version': '1.0', 'event': 'auto_recovery_started',
            'component': component, 'diagnostic_id': report.get('diagnostic_id'),
            'operation_id': operation.operation_id, 'timestamp': time.time(),
        })
        AgentTools.publish_json(self.system_pub, {
            'schema_version': '1.0', 'source': 'inspection_agent_health_monitor',
            'command': 'recover_component', 'component': component,
            'diagnostic_id': report.get('diagnostic_id'),
            'run_id': operation.run_id, 'operation_id': operation.operation_id,
            'tool_call_id': operation.tool_call_id,
        })


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
