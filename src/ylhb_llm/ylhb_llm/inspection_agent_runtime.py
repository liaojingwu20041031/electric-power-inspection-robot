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
    return any(
        word in normalized
        for word in (
            '自我介绍',
            '介绍一下',
            '你是谁',
            '能做什么',
            '能够做什么',
            '有什么功能',
            '现在有什么问题',
            '哪里有问题',
            '当前状态',
        )
    )


@dataclass
class Message:
    role: str
    content: str


@dataclass
class ToolCall:
    name: str
    arguments: Dict[str, Any]


@dataclass
class ToolResult:
    name: str
    result: Dict[str, Any]


class InspectionAgentRuntime:
    def __init__(
        self,
        qwen,
        tools,
        state,
        spec,
        tool_schemas: Dict[str, Any],
        route_toolpack=None,
        model: str = '',
        timeout_sec: float = 12.0,
        enabled: bool = True,
        max_steps: int = 3,
    ) -> None:
        self.qwen = qwen
        self.tools = tools
        self.state = state
        self.spec = spec
        self.tool_schemas = dict(tool_schemas)
        self.route_toolpack = route_toolpack
        self.model = model
        self.timeout_sec = timeout_sec
        self.enabled = enabled
        self.max_steps = max_steps
        self.messages: List[Message] = []
        self.allowed_tool_names = set(self.tool_schemas) | self.tools.registry.names()
        self.allowed_tool_names.discard('send_motion_command')

    def run_turn(self, request: Dict[str, Any]) -> Dict[str, Any]:
        local = decide_local(request, self.state.policy_context())
        if local:
            return self._execute_decision(local)
        text = str(request.get('text') or request.get('command') or '').strip()
        if not self.enabled or not self.qwen.available():
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

        self.messages.append(Message('user', text))
        tool_results: List[ToolResult] = []
        assistant_text = ''
        decision: Dict[str, Any] = {}
        result: Dict[str, Any] = {}
        for _ in range(max(1, self.max_steps)):
            response = self.qwen.chat_tools(
                model=self.model,
                system_prompt=self.spec.system_prompt(),
                messages=[message.__dict__ for message in self.messages],
                tools=self.openai_tools(),
                timeout_sec=self.timeout_sec,
                temperature=0.0,
                extra_body={'enable_thinking': False},
            )
            assistant_text = str(response.get('content') or '')
            tool_calls = response.get('tool_calls') or []
            if not tool_calls:
                self.messages.append(Message('assistant', assistant_text))
                decision = make_decision('assistant_chat', 'generate_local_status_reply', assistant_text, speak=assistant_text, final_answer=assistant_text, response_type='final_answer')
                result = tool_result('assistant_chat', True, 'ok', assistant_text, {'answer': assistant_text})
                self.state.latest_decision = decision
                self.state.latest_result = result
                return {'decision': decision, 'result': result, 'assistant_text': assistant_text, 'role': 'assistant'}

            call = self._parse_tool_call(tool_calls[0])
            decision = self._decision_from_tool_call(call, assistant_text)
            result = self._execute_decision(decision)['result']
            tool_results.append(ToolResult(call.name, result))
            self.messages.append(Message('assistant', assistant_text or f'调用工具 {call.name}'))
            self.messages.append(Message('tool', json.dumps(result, ensure_ascii=False)))
            if not result.get('ok'):
                break

        final_text = assistant_text or str(result.get('message') or '工具已执行。')
        return {'decision': decision, 'result': result, 'assistant_text': final_text, 'role': 'tool', 'tool_results': tool_results}

    def openai_tools(self) -> List[Dict[str, Any]]:
        tools = []
        for name in sorted(self.allowed_tool_names):
            schema = self.tool_schemas.get(name) or self.tools.registry.schemas().get(name) or {}
            tools.append({
                'type': 'function',
                'function': {
                    'name': name,
                    'description': f'Inspection robot tool: {name}',
                    'parameters': {
                        'type': 'object',
                        'properties': schema.get('properties') or {},
                        'required': schema.get('required') or [],
                    },
                },
            })
        return tools

    def _parse_tool_call(self, raw: Dict[str, Any]) -> ToolCall:
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
        return ToolCall(name, arguments)

    def _decision_from_tool_call(self, call: ToolCall, assistant_text: str = '') -> Dict[str, Any]:
        decision = make_decision(
            call.name,
            call.name,
            assistant_text or call.name,
            speak='',
            final_answer=assistant_text,
            arguments=call.arguments,
        )
        return validate_decision(decision, self.allowed_tool_names, self.tool_schemas)

    def _execute_decision(self, decision: Dict[str, Any]) -> Dict[str, Any]:
        decision = validate_decision(decision, self.allowed_tool_names | {'emergency_stop', 'generate_local_status_reply'}, self.tool_schemas)
        policy = authorize(decision, self.state.policy_context(), self.tool_schemas)
        if policy.allowed:
            result = self.tools.execute(decision, policy)
        else:
            result = tool_result(decision['tool_call']['name'], False, 'rejected', policy.reason, error_code='policy_rejected')
            self.tools.publish_event(result)
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {'decision': decision, 'result': result, 'assistant_text': decision.get('final_answer') or decision.get('reason_cn') or '', 'role': 'tool'}
