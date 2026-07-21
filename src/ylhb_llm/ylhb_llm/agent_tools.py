import json
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from std_msgs.msg import String

from .agent_schema import tool_result
from .agent_policy import ACTIVE_PATROL_STATES, FORBIDDEN_RECOVERY_COMPONENTS

SYSTEM_TOOLS = {
    'start_patrol_mode', 'start_route', 'go_to_checkpoint',
    'pause_patrol', 'resume_patrol', 'cancel_patrol', 'emergency_stop',
}
BASE_SKILL_TOOLS = {'rotate_relative', 'move_relative', 'stop_motion'}
COMPONENT_TOOLS = {'start_component', 'stop_component'}
VOICE_SESSION_TOOLS = {'end_voice_conversation', 'close_voice_mode'}

@dataclass(frozen=True)
class ToolDefinition:
    name: str
    schema: Dict[str, Any]
    handler: Callable[[Dict[str, Any], Dict[str, Any]], Dict[str, Any]]


class ToolRegistry:
    def __init__(self) -> None:
        self._tools: Dict[str, ToolDefinition] = {}

    def register(self, definition: ToolDefinition) -> None:
        self._tools[definition.name] = definition

    def get(self, name: str) -> Optional[ToolDefinition]:
        return self._tools.get(name)

    def names(self) -> set[str]:
        return set(self._tools)

    def schemas(self) -> Dict[str, Dict[str, Any]]:
        return {name: definition.schema for name, definition in self._tools.items()}


class AgentTools:
    def __init__(
        self,
        node,
        state,
        system_pub,
        motion_pub,
        say_pub,
        event_pub,
        patrol_pub=None,
        base_skill_pub=None,
        route_toolpack=None,
        tool_schemas: Dict[str, Any] | None = None,
        operation_manager=None,
        status_aggregator=None,
        voice_session_pub=None,
        knowledge_index=None,
        diagnostic_engine=None,
        recovery_catalog=None,
    ) -> None:
        self.node = node
        self.state = state
        self.system_pub = system_pub
        self.motion_pub = motion_pub
        self.say_pub = say_pub
        self.event_pub = event_pub
        self.patrol_pub = patrol_pub
        self.base_skill_pub = base_skill_pub
        self.route_toolpack = route_toolpack
        self.operation_manager = operation_manager
        self.status_aggregator = status_aggregator
        self.voice_session_pub = voice_session_pub
        self.knowledge_index = knowledge_index
        self.diagnostic_engine = diagnostic_engine
        self.recovery_catalog = recovery_catalog
        self.tool_schemas = dict(tool_schemas or {})
        self.registry = ToolRegistry()
        self._register_tools(self.tool_schemas)

    def _register_tools(self, tool_schemas: Dict[str, Any]) -> None:
        handlers = {
            'system': self._execute_system,
            'base_skill': self._execute_base_skill,
            'voice_session': self._execute_voice_session,
            'knowledge': self._execute_knowledge,
            'connection': self._execute_connection,
            'diagnostic': self._execute_diagnostic,
            'local': self._execute_local,
            'route_read': self._execute_local,
        }
        for name, schema in tool_schemas.items():
            handler = handlers.get(str(schema.get('executor') or ''))
            if handler is not None:
                self.registry.register(ToolDefinition(name, schema, handler))
        for name in SYSTEM_TOOLS | COMPONENT_TOOLS:
            if self.registry.get(name) is None:
                self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_system))
        for name in BASE_SKILL_TOOLS:
            if self.registry.get(name) is None:
                self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_base_skill))
        for name in VOICE_SESSION_TOOLS:
            if self.registry.get(name) is None:
                self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_voice_session))
        for name in {'get_robot_summary', 'get_system_status', 'get_patrol_status', 'get_voice_status', 'generate_local_status_reply', 'list_routes', 'describe_route', 'list_checkpoints', 'inspect_checkpoint'}:
            if self.registry.get(name) is None:
                self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_local))

    def execute(self, decision: Dict[str, Any], policy) -> Dict[str, Any]:
        tool_call = decision['tool_call']
        name = str(tool_call['name'])
        args = tool_call.get('arguments') or {}
        if not policy.allowed:
            result = tool_result(name, False, 'rejected', policy.reason, error_code='policy_rejected')
            self.publish_event(result)
            return result

        schema = self.tool_schemas.get(name) or {}
        if schema.get('side_effect', 'none') != 'none' and self.operation_manager is None:
            result = tool_result(
                name, False, 'failed', 'operation manager unavailable',
                error_code='operation_manager_unavailable',
            )
            self.publish_event(result)
            return result

        operation = self._create_operation(decision, args)
        if operation is not None:
            decision = {**decision, 'operation_id': operation.operation_id}
        definition = self.registry.get(name)
        if definition:
            result = definition.handler(decision, args)
        elif name == 'send_motion_command':
            command = str(args.get('command') or '')
            self.publish_json(self.motion_pub, {'schema_version': '1.0', 'source': 'inspection_agent', 'command': command, 'request_id': str(decision.get('decision_id') or ''), 'timestamp': self.node.get_clock().now().nanoseconds / 1e9})
            result = tool_result(name, True, 'sent', '已发送运动命令', {'command': command})
        elif name == 'get_system_status':
            result = tool_result(name, True, 'ok', 'system status', self.state.system_status)
        elif name == 'get_patrol_status':
            result = tool_result(name, True, 'ok', 'patrol status', self.state.patrol_status)
        elif name == 'get_voice_status':
            result = tool_result(name, True, 'ok', 'voice status', self.state.voice_status)
        else:
            speak = decision.get('speak') or {}
            answer = str(decision.get('final_answer') or speak.get('text') or '当前状态未知。')
            result = tool_result(name, True, 'ok', answer, {'answer': answer})

        if operation is not None:
            if result.get('ok'):
                operation = self.operation_manager.mark_sent(operation.operation_id)
            else:
                operation = self.operation_manager.update(operation.operation_id, 'failed', result)
            result = {
                **result,
                'data': {**(result.get('data') or {}), 'operation_id': operation.operation_id, 'operation_state': operation.state},
            }
        self.publish_event(result)
        return result

    def _create_operation(self, decision: Dict[str, Any], arguments: Dict[str, Any]):
        if self.operation_manager is None or decision.get('operation_id'):
            return None
        name = str((decision.get('tool_call') or {}).get('name') or '')
        schema = self.tool_schemas.get(name) or {}
        if schema.get('side_effect', 'none') == 'none' and name not in SYSTEM_TOOLS | BASE_SKILL_TOOLS:
            return None
        return self.operation_manager.create(
            str(decision.get('run_id') or decision.get('decision_id') or ''),
            str(decision.get('tool_call_id') or ''),
            name,
            {
                **arguments,
                **({'target_operation_id': str(decision['target_operation_id'])}
                   if decision.get('target_operation_id') else {}),
            },
            float(schema.get('timeout_sec') or 15.0),
        )

    @staticmethod
    def _correlation_fields(decision: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'request_id': str(decision.get('request_id') or decision.get('decision_id') or ''),
            'run_id': str(decision.get('run_id') or ''),
            'operation_id': str(decision.get('operation_id') or ''),
            'tool_call_id': str(decision.get('tool_call_id') or ''),
            'target_operation_id': str(decision.get('target_operation_id') or ''),
        }

    def _execute_system(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if name == 'recover_component':
            component = str(args.get('component') or '')
            if component.lower() in FORBIDDEN_RECOVERY_COMPONENTS:
                return tool_result(name, False, 'rejected', '该组件禁止自动恢复', error_code='recovery_forbidden')
            if self.recovery_catalog is None or component not in self.recovery_catalog.names():
                return tool_result(name, False, 'rejected', '组件不在恢复白名单', error_code='recovery_not_allowed')
            recovery = self.recovery_catalog.get(component)
            patrol_state = str(
                getattr(self.state, 'patrol_state', lambda: 'unknown')()
                if callable(getattr(self.state, 'patrol_state', None))
                else getattr(self.state, 'patrol_status', {}).get('state', 'unknown')
            )
            if recovery.get('requires_no_active_patrol') and patrol_state in ACTIVE_PATROL_STATES | {'starting', 'command_sent', 'paused'}:
                return tool_result(name, False, 'rejected', '活动巡逻期间禁止恢复该组件', error_code='active_patrol')
            if self.operation_manager is not None and any(
                item.get('tool_name') in {'rotate_relative', 'move_relative', 'recover_component'}
                and item.get('operation_id') != decision.get('operation_id')
                for item in self.operation_manager.list_active()
            ):
                return tool_result(name, False, 'rejected', '存在运动或恢复操作，拒绝并发恢复', error_code='operation_conflict')
            report = getattr(self.diagnostic_engine, 'last_report', {}) if self.diagnostic_engine else {}
            engine_clock = getattr(self.diagnostic_engine, 'clock', None)
            now = engine_clock() if callable(engine_clock) else float(report.get('generated_at') or time.time())
            if now - float(report.get('generated_at') or 0.0) > float(
                getattr(self.diagnostic_engine, 'diagnostic_freshness_sec', 5.0)):
                return tool_result(name, False, 'rejected', '最近一次诊断已过期', error_code='fresh_diagnostic_required')
            matching = any(
                issue.get('recoverable') is True
                and str(issue.get('recovery_component') or issue.get('component') or '') == component
                for issue in report.get('issues') or []
            )
            if not matching:
                return tool_result(name, False, 'rejected', '缺少本轮可恢复诊断证据', error_code='fresh_diagnostic_required')
            args = {**args, 'diagnostic_id': str(report.get('diagnostic_id') or '')}
        if name in COMPONENT_TOOLS:
            component = str(args.get('component') or '')
            command = f'{"start" if name == "start_component" else "stop"}_{component}'
        else:
            command = str(
                (self.tool_schemas.get(name) or {}).get('command')
                or ('start_patrol_mode' if name == 'start_route' else name)
            )
        if name == 'go_to_checkpoint' and self.route_toolpack is not None:
            args = {
                **args,
                'target_id': self.route_toolpack.catalog.resolve_target_id(
                    str(args.get('target_id') or '')),
            }
        payload = {'schema_version': '1.0', 'source': 'inspection_agent', **self._correlation_fields(decision)}
        payload.update({
            key: value for key, value in args.items()
            if key != 'command' and (key != 'component' or name == 'recover_component')
        })
        payload['command'] = command
        self.publish_json(self.system_pub, payload)
        return tool_result(name, True, 'sent', f'已发送系统命令: {command}', {'command': command})

    def _execute_knowledge(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if self.knowledge_index is None:
            return tool_result(name, False, 'failed', '项目文档索引不可用')
        results = self.knowledge_index.search(str(args.get('query') or ''))
        return tool_result(name, True, 'ok', '已查询项目文档', {'results': results})

    def _execute_connection(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if self.diagnostic_engine is None:
            return tool_result(name, False, 'failed', '连接状态提供器不可用')
        data = self.diagnostic_engine.get_connection_info(str(args.get('target') or 'all'))
        return tool_result(name, True, 'ok', '已读取实时连接信息', data)

    def _execute_diagnostic(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if self.diagnostic_engine is None:
            return tool_result(name, False, 'failed', '机器人诊断引擎不可用')
        data = self.diagnostic_engine.run_self_check(str(args.get('scope') or 'all'))
        status = str(data.get('overall') or 'failed')
        return tool_result(name, status != 'failed', status, '只读自检完成', data)

    def _execute_base_skill(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        if self.base_skill_pub is None:
            return tool_result(decision['tool_call']['name'], False, 'failed', 'base skill publisher unavailable')
        name = str(decision['tool_call']['name'])
        payload = {
            'schema_version': '1.0',
            'source': 'inspection_agent',
            'command': str((self.tool_schemas.get(name) or {}).get('command') or name),
            'arguments': args,
            **self._correlation_fields(decision),
        }
        self.publish_json(self.base_skill_pub, payload)
        return tool_result(name, True, 'sent', '已发送基础运动技能命令', {'command': name, **args})

    def _execute_voice_session(self, decision: Dict[str, Any], _args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if self.voice_session_pub is None:
            return tool_result(name, False, 'failed', 'voice session publisher unavailable')
        self.publish_json(self.voice_session_pub, {
            'schema_version': '1.0',
            'source': 'inspection_agent',
            'command': name,
            **self._correlation_fields(decision),
        })
        return tool_result(name, True, 'sent', f'已发送语音会话命令: {name}', {'command': name})

    def _execute_local(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if name == 'get_system_status':
            return tool_result(name, True, 'ok', 'system status', self.state.system_status)
        if name == 'get_patrol_status':
            return tool_result(name, True, 'ok', 'patrol status', self.state.patrol_status)
        if name == 'get_voice_status':
            return tool_result(name, True, 'ok', 'voice status', self.state.voice_status)
        if name == 'get_robot_summary':
            if self.status_aggregator is None:
                return tool_result(name, False, 'failed', 'robot status aggregator unavailable')
            summarize = getattr(
                self.status_aggregator, 'mode_aware_summary', self.status_aggregator.summary)
            return tool_result(name, True, 'ok', '机器人状态摘要', summarize())
        if self.route_toolpack and name == 'list_routes':
            return tool_result(name, True, 'ok', 'routes', {'routes': self.route_toolpack.list_routes()})
        if self.route_toolpack and name == 'describe_route':
            return tool_result(name, True, 'ok', 'route', self.route_toolpack.describe_route(str(args.get('route_id') or '')))
        if self.route_toolpack and name == 'list_checkpoints':
            return tool_result(name, True, 'ok', 'checkpoints', {'targets': self.route_toolpack.route_targets(str(args.get('route_id') or ''))})
        if self.route_toolpack and name == 'inspect_checkpoint':
            target_id = self.route_toolpack.catalog.resolve_target_id(str(args.get('target_id') or ''))
            return tool_result(name, True, 'ok', 'inspection items', self.route_toolpack.inspect_checkpoint(target_id))
        speak = decision.get('speak') or {}
        answer = str(decision.get('final_answer') or speak.get('text') or '当前状态未知。')
        return tool_result(name, True, 'ok', answer, {'answer': answer})

    def say(self, decision: Dict[str, Any], priority: int = 5, interrupt: bool = False) -> None:
        speak = decision.get('speak') or {}
        text = str(speak.get('text') or decision.get('final_answer') or '')
        if not text:
            return
        from ylhb_interfaces.msg import SayText

        msg = SayText()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.task_id = str(decision.get('decision_id') or 'inspection_agent')
        msg.priority = int(speak.get('priority') or priority)
        msg.interrupt = bool(speak.get('interrupt') or interrupt)
        msg.text = text
        self.say_pub.publish(msg)

    def publish_event(self, payload: Dict[str, Any]) -> None:
        self.publish_json(self.event_pub, payload)

    @staticmethod
    def publish_json(pub, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pub.publish(msg)
