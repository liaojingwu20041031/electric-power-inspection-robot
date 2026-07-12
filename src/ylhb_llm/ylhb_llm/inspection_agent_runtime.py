from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List

from .agent_policy import authorize
from .agent_schema import SchemaError, tool_result, validate_decision
from .robot_reply_style import speak as styled_speak


def make_decision(
    intent: str,
    tool: str,
    text: str,
    speak: str = '',
    final_answer: str = '',
    response_type: str = 'tool_call',
    safety_level: str = 'normal',
    arguments: Dict[str, Any] | None = None,
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


def decide_local(request: Dict[str, Any], _state: Dict[str, Any]) -> Dict[str, Any] | None:
    text = str(request.get('text') or request.get('command') or '').strip()
    normalized = text.replace(' ', '')
    if not normalized:
        return make_decision('empty', 'generate_local_status_reply', text, response_type='ignore')
    if any(word in normalized for word in ('急停', '紧急停止', '别动', '刹车')):
        return make_decision('emergency_stop', 'emergency_stop', text, safety_level='emergency')
    if normalized in ('停止', '停下', '停车', '不要动'):
        return make_decision('stop_motion', 'stop_motion', text, speak='已停止短时运动。')
    return None


def _wants_local_status(text: str) -> bool:
    normalized = text.replace(' ', '')
    return any(word in normalized for word in (
        '自我介绍', '介绍一下', '你是谁', '能做什么', '能够做什么', '有什么功能',
        '现在有什么问题', '哪里有问题', '当前状态',
    ))


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: Dict[str, Any]


@dataclass
class ToolResult:
    tool_call_id: str
    name: str
    result: Dict[str, Any]


class InspectionAgentRuntime:
    def __init__(
        self,
        planner,
        tools,
        state,
        spec,
        tool_schemas: Dict[str, Any],
        route_toolpack=None,
        model: str = '',
        timeout_sec: float = 12.0,
        enabled: bool = True,
        max_steps: int = 8,
        max_side_effect_tools_per_turn: int = 4,
        max_identical_tool_calls: int = 2,
    ) -> None:
        self.planner = planner
        self.tools = tools
        self.state = state
        self.spec = spec
        self.tool_schemas = dict(tool_schemas)
        self.route_toolpack = route_toolpack
        self.model = model
        self.timeout_sec = timeout_sec
        self.enabled = enabled
        self.max_steps = max_steps
        self.max_side_effect_tools_per_turn = max_side_effect_tools_per_turn
        self.max_identical_tool_calls = max_identical_tool_calls
        self.messages: List[Dict[str, Any]] = []
        self.allowed_tool_names = set(self.tool_schemas) | self.tools.registry.names()
        self.allowed_tool_names.discard('send_motion_command')

    def run_turn(self, request: Dict[str, Any]) -> Dict[str, Any]:
        local = decide_local(request, self.state.policy_context())
        if local:
            return self._execute_decision(local)
        text = str(request.get('text') or request.get('command') or '').strip()
        if not self.enabled or not self.planner.available():
            return self._planner_unavailable(text)

        run_id = str(request.get('run_id') or f'run_{int(time.time() * 1000)}')
        request_id = str(request.get('request_id') or request.get('client_msg_id') or '')
        self.messages.append({'role': 'user', 'content': text})
        tool_results: List[ToolResult] = []
        decision: Dict[str, Any] = {}
        side_effect_tools = 0
        previous_call_key = ''
        identical_call_count = 0
        for _ in range(max(1, self.max_steps)):
            response = self.planner.chat_tools(
                model=self.model,
                system_prompt=self.spec.system_prompt(),
                messages=self.messages,
                tools=self.openai_tools(),
                timeout_sec=self.timeout_sec,
                temperature=0.0,
            )
            assistant_message = response.get('message')
            if not isinstance(assistant_message, dict):
                raise SchemaError('tool response is missing assistant message')
            self.messages.append(assistant_message)
            assistant_text = str(assistant_message.get('content') or response.get('content') or '')
            raw_calls = assistant_message.get('tool_calls') or []
            if not isinstance(raw_calls, list):
                raise SchemaError('assistant tool_calls must be an array')
            if not raw_calls:
                decision = make_decision(
                    'assistant_chat', 'generate_local_status_reply', assistant_text,
                    speak=assistant_text, final_answer=assistant_text, response_type='final_answer',
                )
                result = tool_result('assistant_chat', True, 'ok', assistant_text, {'answer': assistant_text})
                self.state.latest_decision = decision
                self.state.latest_result = result
                return {
                    'decision': decision, 'result': result, 'assistant_text': assistant_text,
                    'role': 'assistant', 'tool_results': tool_results,
                }

            for raw_call in raw_calls:
                call = self._parse_tool_call(raw_call)
                decision = self._decision_from_tool_call(call, assistant_text, run_id, request_id)
                call_key = self._tool_call_key(call)
                if call_key == previous_call_key:
                    identical_call_count += 1
                else:
                    previous_call_key = call_key
                    identical_call_count = 1
                if identical_call_count > self.max_identical_tool_calls:
                    result = tool_result(
                        call.name, False, 'failed', '相同工具调用重复，已停止任务',
                        error_code='repeated_tool_call',
                    )
                    return self._append_and_return(call, decision, result, tool_results)

                side_effect = self._is_side_effect_tool(call.name)
                if side_effect and side_effect_tools >= self.max_side_effect_tools_per_turn:
                    result = tool_result(
                        call.name, False, 'failed', '已达到本轮副作用工具上限，已停止任务',
                        error_code='max_side_effect_tools',
                    )
                    return self._append_and_return(call, decision, result, tool_results)

                result = self._execute_decision(decision)['result']
                self._append_tool_result(call, result)
                tool_results.append(ToolResult(call.id, call.name, result))
                if side_effect:
                    side_effect_tools += 1
                if result.get('error_code') == 'policy_rejected':
                    return self._tool_turn(decision, result, tool_results)
                # Re-evaluate after any side effect instead of trusting later parallel calls.
                if side_effect:
                    break

        result = tool_result(
            'inspection_agent', False, 'failed',
            f'已达到最大步骤数 {self.max_steps}，停止继续调用工具',
            {'completed_tool_calls': len(tool_results)}, error_code='max_agent_steps',
        )
        self.state.latest_result = result
        return self._tool_turn(decision, result, tool_results)

    def _planner_unavailable(self, text: str) -> Dict[str, Any]:
        answer = 'LLM Planner 不可用，未执行动作。'
        if _wants_local_status(text):
            context = self.state.policy_context()
            patrol_state = str(context.get('patrol_state') or 'unknown')
            voice_state = str((context.get('voice_status') or {}).get('state') or '')
            answer = f'我是巡检机器人语言 Agent。当前巡逻状态：{patrol_state}；语音状态：{voice_state or "unknown"}。LLM Planner 不可用，未执行动作。'
        decision = make_decision('planner_unavailable', 'generate_local_status_reply', answer, final_answer=answer, response_type='reject')
        result = tool_result('inspection_agent', False, 'failed', answer, error_code='planner_unavailable')
        self.state.latest_decision = decision
        self.state.latest_result = result
        self.state.last_error = 'planner_unavailable'
        return {'decision': decision, 'result': result, 'assistant_text': answer, 'role': 'system'}

    @staticmethod
    def _tool_turn(decision: Dict[str, Any], result: Dict[str, Any], tool_results: List[ToolResult]) -> Dict[str, Any]:
        return {
            'decision': decision,
            'result': result,
            'assistant_text': str(result.get('message') or '工具执行未完成。'),
            'role': 'tool',
            'tool_results': tool_results,
        }

    def _append_and_return(
        self,
        call: ToolCall,
        decision: Dict[str, Any],
        result: Dict[str, Any],
        tool_results: List[ToolResult],
    ) -> Dict[str, Any]:
        self._append_tool_result(call, result)
        tool_results.append(ToolResult(call.id, call.name, result))
        self.state.latest_decision = decision
        self.state.latest_result = result
        return self._tool_turn(decision, result, tool_results)

    def _append_tool_result(self, call: ToolCall, result: Dict[str, Any]) -> None:
        self.messages.append({
            'role': 'tool',
            'tool_call_id': call.id,
            'name': call.name,
            'content': json.dumps(result, ensure_ascii=False),
        })

    def _is_side_effect_tool(self, name: str) -> bool:
        schema = self.tool_schemas.get(name) or {}
        if schema.get('risk_level') == 'read_only':
            return False
        return name not in {
            'generate_local_status_reply', 'get_system_status', 'get_patrol_status',
            'get_voice_status', 'get_robot_summary', 'get_navigation_status',
            'get_localization_status', 'get_sensor_status', 'get_base_status',
            'get_perception_status', 'get_route_context', 'get_recent_faults',
            'get_operation_status', 'list_active_operations', 'wait_operation',
            'list_routes', 'describe_route', 'list_checkpoints', 'inspect_checkpoint',
        }

    @staticmethod
    def _tool_call_key(call: ToolCall) -> str:
        arguments = json.dumps(call.arguments, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        return f'{call.name}:{arguments}'

    def openai_tools(self) -> List[Dict[str, Any]]:
        tools = []
        for name in sorted(self.allowed_tool_names):
            schema = self.tool_schemas.get(name) or self.tools.registry.schemas().get(name) or {}
            tools.append({
                'type': 'function',
                'function': {
                    'name': name,
                    'description': str(schema.get('description') or f'Inspection robot tool: {name}'),
                    'parameters': {
                        'type': 'object',
                        'properties': schema.get('properties') or {},
                        'required': schema.get('required') or [],
                        'additionalProperties': False,
                    },
                },
            })
        return tools

    def _parse_tool_call(self, raw: Dict[str, Any]) -> ToolCall:
        if not isinstance(raw, dict):
            raise SchemaError('tool call must be object')
        call_id = str(raw.get('id') or '')
        if not call_id:
            raise SchemaError('tool call id is required')
        function = raw.get('function') or raw
        name = str(function.get('name') or raw.get('name') or '')
        arguments = function.get('arguments') or raw.get('arguments') or {}
        if isinstance(arguments, str):
            arguments = json.loads(arguments or '{}')
        if not isinstance(arguments, dict):
            raise SchemaError('tool arguments must be object')
        if self.route_toolpack and name in ('go_to_checkpoint', 'inspect_checkpoint') and 'target_id' in arguments:
            arguments = dict(arguments)
            arguments['target_id'] = self.route_toolpack.catalog.resolve_target_id(str(arguments['target_id']))
        if self.route_toolpack and name in ('start_route', 'describe_route', 'list_checkpoints') and 'route_id' in arguments:
            arguments = dict(arguments)
            arguments['route_id'] = self.route_toolpack.catalog.route(str(arguments['route_id']))['id']
        return ToolCall(call_id, name, arguments)

    def _decision_from_tool_call(
        self,
        call: ToolCall,
        assistant_text: str = '',
        run_id: str = '',
        request_id: str = '',
    ) -> Dict[str, Any]:
        decision = make_decision(
            call.name, call.name, assistant_text or call.name, speak='',
            final_answer=assistant_text, arguments=call.arguments,
        )
        decision = validate_decision(decision, self.allowed_tool_names, self.tool_schemas)
        decision['run_id'] = run_id
        decision['request_id'] = request_id
        decision['tool_call_id'] = call.id
        return decision

    def _execute_decision(self, decision: Dict[str, Any]) -> Dict[str, Any]:
        decision = validate_decision(
            decision,
            self.allowed_tool_names | {'emergency_stop', 'generate_local_status_reply'},
            self.tool_schemas,
        )
        policy = authorize(decision, self.state.policy_context(), self.tool_schemas)
        if policy.allowed:
            result = self.tools.execute(decision, policy)
        else:
            result = tool_result(decision['tool_call']['name'], False, 'rejected', policy.reason, error_code='policy_rejected')
            self.tools.publish_event(result)
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {
            'decision': decision,
            'result': result,
            'assistant_text': decision.get('final_answer') or decision.get('reason_cn') or '',
            'role': 'tool',
        }
