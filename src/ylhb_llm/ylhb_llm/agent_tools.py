import json
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from std_msgs.msg import String

from .agent_schema import tool_result


SYSTEM_TOOLS = {
    'start_patrol_mode',
    'pause_patrol',
    'resume_patrol',
    'cancel_patrol',
    'emergency_stop',
}
BASE_SKILL_TOOLS = {'rotate_relative', 'move_relative', 'stop_motion'}


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
        self.tool_schemas = dict(tool_schemas or {})
        self.registry = ToolRegistry()
        self._register_tools(self.tool_schemas)

    def _register_tools(self, tool_schemas: Dict[str, Any]) -> None:
        for name in SYSTEM_TOOLS | {'start_route'}:
            self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_system))
        for name in BASE_SKILL_TOOLS:
            self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_base_skill))
        for name in {'go_to_checkpoint'}:
            self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_patrol))
        for name in {
            'get_system_status',
            'get_patrol_status',
            'get_voice_status',
            'generate_local_status_reply',
            'list_routes',
            'describe_route',
            'list_checkpoints',
            'inspect_checkpoint',
        }:
            self.registry.register(ToolDefinition(name, tool_schemas.get(name, {}), self._execute_local))

    def execute(self, decision: Dict[str, Any], policy) -> Dict[str, Any]:
        tool_call = decision['tool_call']
        name = str(tool_call['name'])
        args = tool_call.get('arguments') or {}
        if not policy.allowed:
            result = tool_result(name, False, 'rejected', policy.reason, error_code='policy_rejected')
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
            self.publish_json(self.motion_pub, {
                'schema_version': '1.0',
                'source': 'inspection_agent',
                'command': command,
                'request_id': str(decision.get('decision_id') or ''),
                'timestamp': self.node.get_clock().now().nanoseconds / 1e9,
            })
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
        name = str((decision.get('tool_call') or {}).get('name') or '')
        if self.operation_manager is None or name not in SYSTEM_TOOLS | BASE_SKILL_TOOLS | {'start_route', 'go_to_checkpoint'}:
            return None
        schema = self.tool_schemas.get(name) or {}
        return self.operation_manager.create(
            str(decision.get('run_id') or decision.get('decision_id') or ''),
            str(decision.get('tool_call_id') or ''),
            name,
            arguments,
            float(schema.get('timeout_sec') or 15.0),
        )

    @staticmethod
    def _correlation_fields(decision: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'request_id': str(decision.get('request_id') or decision.get('decision_id') or ''),
            'run_id': str(decision.get('run_id') or ''),
            'operation_id': str(decision.get('operation_id') or ''),
            'tool_call_id': str(decision.get('tool_call_id') or ''),
        }

    def _execute_system(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        command = 'start_patrol_mode' if name == 'start_route' else name
        payload = {'schema_version': '1.0', 'source': 'inspection_agent', **self._correlation_fields(decision)}
        payload.update({key: value for key, value in args.items() if key != 'command'})
        payload['command'] = command
        self.publish_json(self.system_pub, payload)
        return tool_result(name, True, 'sent', f'已发送系统命令: {command}', {'command': command})

    def _execute_patrol(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        if self.patrol_pub is None:
            return tool_result(decision['tool_call']['name'], False, 'failed', 'patrol publisher unavailable')
        target_id = str(args.get('target_id') or '')
        if self.route_toolpack is not None:
            target_id = self.route_toolpack.catalog.resolve_target_id(target_id)
        payload = {
            'schema_version': '1.0',
            'source': 'inspection_agent',
            'command': 'go_to_target',
            'target_id': target_id,
            **self._correlation_fields(decision),
        }
        self.publish_json(self.patrol_pub, payload)
        return tool_result('go_to_checkpoint', True, 'sent', '已发送目标点导航命令', {'target_id': target_id})

    def _execute_base_skill(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        if self.base_skill_pub is None:
            return tool_result(decision['tool_call']['name'], False, 'failed', 'base skill publisher unavailable')
        name = str(decision['tool_call']['name'])
        payload = {
            'schema_version': '1.0',
            'source': 'inspection_agent',
            'command': name,
            'arguments': args,
            **self._correlation_fields(decision),
        }
        self.publish_json(self.base_skill_pub, payload)
        return tool_result(name, True, 'sent', '已发送基础运动技能命令', {'command': name, **args})

    def _execute_local(self, decision: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
        name = str(decision['tool_call']['name'])
        if name == 'get_system_status':
            return tool_result(name, True, 'ok', 'system status', self.state.system_status)
        if name == 'get_patrol_status':
            return tool_result(name, True, 'ok', 'patrol status', self.state.patrol_status)
        if name == 'get_voice_status':
            return tool_result(name, True, 'ok', 'voice status', self.state.voice_status)
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
