from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List

from .agent_policy import authorize
from .agent_schema import SchemaError, tool_result, validate_decision
from .robot_reply_style import prepare_speech_text, speak as styled_speak


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
    if any(word in normalized for word in ('取消巡逻', '停止巡逻', '结束巡逻')):
        return make_decision('cancel_patrol', 'cancel_patrol', text, speak='正在取消巡逻。')
    if any(word in normalized for word in ('暂停巡逻', '暂停任务')):
        return make_decision('pause_patrol', 'pause_patrol', text, speak='正在暂停巡逻。')
    if any(word in normalized for word in ('恢复巡逻', '继续巡逻')):
        return make_decision('resume_patrol', 'resume_patrol', text, speak='正在恢复巡逻。')
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
        history_max_turns: int = 6,
        history_max_chars: int = 10_000,
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
        self.history_max_turns = history_max_turns
        self.history_max_chars = history_max_chars
        self.messages: List[Dict[str, Any]] = []
        self.pending_turn: Dict[str, Any] | None = None
        self.allowed_tool_names = set(self.tool_schemas) | self.tools.registry.names()
        self.allowed_tool_names.discard('send_motion_command')

    def run_turn(self, request: Dict[str, Any]) -> Dict[str, Any]:
        local = decide_local(request, self.state.policy_context())
        if local:
            if self.pending_turn is not None:
                local['target_operation_id'] = str(
                    self.pending_turn.get('pending_operation_id') or '')
            return self._execute_decision(local)
        text = str(request.get('text') or request.get('command') or '').strip()
        if self.pending_turn is not None:
            if self._looks_like_new_goal(text):
                turn = self._waiting_turn(self.pending_turn)
                turn['assistant_text'] = '当前目标仍在执行，请先取消当前任务再开始新目标。'
                turn['display_text'] = turn['assistant_text']
                turn['decision']['speak'] = {}
                return turn
            self.pending_turn.setdefault('supplements', []).append(text)
            self.state.add_step(f'收到补充：{text}')
            return self._waiting_turn(self.pending_turn)
        if not self.enabled or not self.planner.available():
            return self._planner_unavailable(text)

        run_id = str(request.get('run_id') or f'run_{int(time.time() * 1000)}')
        request_id = str(request.get('request_id') or request.get('client_msg_id') or '')
        self.state.start_goal(text)
        self.messages.append({'role': 'user', 'content': text})
        context = {
            'request': dict(request),
            'run_id': run_id,
            'request_id': request_id,
            'tool_results': [],
            'decision': {},
            'side_effect_tools': 0,
            'previous_call_key': '',
            'identical_call_count': 0,
            'steps_used': 0,
            'evidence_kind': self._evidence_kind(text),
            'required_retry_used': False,
            'force_tool_once': False,
        }
        return self._continue_turn(context)

    def resume_turn(self, operation: Dict[str, Any]) -> Dict[str, Any]:
        context = self.pending_turn
        if context is None:
            raise ValueError('no agent turn is waiting for feedback')
        if str(operation.get('operation_id') or '') != str(context.get('pending_operation_id') or ''):
            raise ValueError('operation feedback does not match pending agent turn')
        call = context.pop('pending_call')
        result = self._operation_result(operation)
        self._append_tool_result(call, result)
        context['tool_results'].append(ToolResult(call.id, call.name, result))
        for supplement in context.pop('supplements', []):
            self.messages.append({'role': 'user', 'content': f'用户补充：{supplement}'})
        context.pop('pending_operation_id', None)
        self.pending_turn = None
        self.state.state = 'planning'
        self.state.pending_operation_id = ''
        self.state.add_step(f'收到真实反馈：{result.get("status")}', {'tool': call.name, 'result': result})
        return self._continue_turn(context)

    def _continue_turn(self, context: Dict[str, Any]) -> Dict[str, Any]:
        tool_results: List[ToolResult] = context['tool_results']
        decision: Dict[str, Any] = context['decision']
        while context['steps_used'] < max(1, self.max_steps):
            context['steps_used'] += 1
            self.trim_history()
            tool_choice = 'required' if context['force_tool_once'] else 'auto'
            response = self.planner.chat_tools(
                model=self.model,
                system_prompt=self.spec.system_prompt(),
                messages=self.messages,
                tools=self.openai_tools(),
                timeout_sec=self.timeout_sec,
                temperature=0.0,
                tool_choice=tool_choice,
            )
            assistant_message = response.get('message')
            if not isinstance(assistant_message, dict):
                raise SchemaError('tool response is missing assistant message')
            self.messages.append(assistant_message)
            assistant_text = str(assistant_message.get('content') or response.get('content') or '')
            raw_calls = assistant_message.get('tool_calls') or []
            context['force_tool_once'] = False
            if not isinstance(raw_calls, list):
                raise SchemaError('assistant tool_calls must be an array')
            if not raw_calls:
                if context['evidence_kind'] and not self._has_valid_evidence(
                    context['evidence_kind'], tool_results,
                ):
                    if not context['required_retry_used']:
                        context['required_retry_used'] = True
                        context['force_tool_once'] = True
                        self.messages.pop()
                        continue
                    return self._missing_evidence_turn(tool_results)
                speech_text = prepare_speech_text(assistant_text)
                decision = make_decision(
                    'assistant_chat', 'generate_local_status_reply', assistant_text,
                    speak=speech_text, final_answer=assistant_text, response_type='final_answer',
                )
                result = tool_result('assistant_chat', True, 'ok', assistant_text, {'answer': assistant_text})
                self.state.latest_decision = decision
                self.state.latest_result = result
                self.state.finish(assistant_text)
                return {
                    'state': 'finished',
                    'decision': decision, 'result': result, 'assistant_text': assistant_text,
                    'display_text': assistant_text, 'speech_text': speech_text,
                    'role': 'assistant', 'tool_results': tool_results,
                }

            for raw_call in raw_calls:
                call = self._parse_tool_call(raw_call)
                decision = self._decision_from_tool_call(
                    call, assistant_text, context['run_id'], context['request_id'])
                context['decision'] = decision
                call_key = self._tool_call_key(call)
                if call_key == context['previous_call_key']:
                    context['identical_call_count'] += 1
                else:
                    context['previous_call_key'] = call_key
                    context['identical_call_count'] = 1
                if context['identical_call_count'] > self.max_identical_tool_calls:
                    result = tool_result(
                        call.name, False, 'failed', '相同工具调用重复，已停止任务',
                        error_code='repeated_tool_call',
                    )
                    return self._append_and_return(call, decision, result, tool_results)

                side_effect = self._is_side_effect_tool(call.name)
                if side_effect and context['side_effect_tools'] >= self.max_side_effect_tools_per_turn:
                    result = tool_result(
                        call.name, False, 'failed', '已达到本轮副作用工具上限，已停止任务',
                        error_code='max_side_effect_tools',
                    )
                    return self._append_and_return(call, decision, result, tool_results)

                result = self._execute_decision(decision)['result']
                self.state.add_step(
                    f'调用工具：{call.name}',
                    {'tool': call.name, 'arguments': call.arguments, 'result': result},
                )
                if side_effect and result.get('status') == 'sent' and (result.get('data') or {}).get('operation_id'):
                    context['side_effect_tools'] += 1
                    context['pending_call'] = call
                    context['pending_operation_id'] = str(result['data']['operation_id'])
                    context['pending_result'] = result
                    self.pending_turn = context
                    self.state.wait_feedback(
                        context['pending_operation_id'], f'等待 {call.name} 的真实反馈')
                    return self._waiting_turn(context)
                self._append_tool_result(call, result)
                tool_results.append(ToolResult(call.id, call.name, result))
                if side_effect:
                    context['side_effect_tools'] += 1
                if result.get('error_code') == 'policy_rejected':
                    self.state.finish(str(result.get('message') or '工具请求被拒绝。'))
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
        self.state.finish(str(result.get('message') or ''))
        return self._tool_turn(decision, result, tool_results)

    def _waiting_turn(self, context: Dict[str, Any]) -> Dict[str, Any]:
        decision = dict(context.get('decision') or {})
        decision['speak'] = {
            'reply_key': 'agent.waiting_feedback',
            'text': '已开始执行，等待结果。',
            'priority': 5,
            'interrupt': False,
        }
        result = context.get('pending_result') or self.state.latest_result or {}
        return {
            'state': 'waiting_feedback',
            'pending_operation_id': str(context.get('pending_operation_id') or ''),
            'decision': decision,
            'result': result,
            'assistant_text': '动作请求已发送，正在等待机器人真实反馈。',
            'display_text': '动作请求已发送，正在等待机器人真实反馈。',
            'speech_text': '已开始执行，等待结果。',
            'role': 'tool',
            'tool_results': context.get('tool_results') or [],
        }

    def _missing_evidence_turn(self, tool_results: List[ToolResult]) -> Dict[str, Any]:
        answer = '未取得机器人真实工具结果，无法确认当前状态或动作结果。'
        decision = make_decision(
            'missing_tool_evidence', 'generate_local_status_reply', answer,
            speak=answer, final_answer=answer, response_type='reject', safety_level='blocked')
        result = tool_result(
            'inspection_agent', False, 'failed', answer, error_code='missing_tool_evidence')
        self.state.latest_decision = decision
        self.state.latest_result = result
        self.state.finish(answer)
        return {
            'state': 'finished', 'decision': decision, 'result': result,
            'assistant_text': answer, 'display_text': answer,
            'speech_text': answer, 'role': 'system', 'tool_results': tool_results,
        }

    @staticmethod
    def _evidence_kind(text: str) -> str:
        normalized = text.replace(' ', '')
        if any(word in normalized for word in (
            '开始巡逻', '去巡检', '去目标', '导航到', '移动', '前进', '后退', '左转', '右转',
            '旋转', '转向', '暂停巡逻', '恢复巡逻', '取消巡逻', '停止巡逻',
        )):
            return 'action'
        if any(word in normalized for word in (
            '当前状态', '实时状态', '机器人状态', '巡逻状态', '导航状态', '定位状态',
            '现在有什么问题', '哪里有问题', '是否正常', '故障', '告警', '当前位置',
            '传感器状态', '底盘状态',
        )):
            return 'status'
        return ''

    def _has_valid_evidence(self, kind: str, tool_results: List[ToolResult]) -> bool:
        for item in tool_results:
            result = item.result or {}
            side_effect = self._is_side_effect_tool(item.name)
            if kind == 'status' and not side_effect and result.get('ok'):
                return True
            if kind == 'action' and side_effect and result.get('status') in {
                'succeeded', 'failed', 'canceled', 'timeout', 'rejected',
            }:
                return True
        return False

    @staticmethod
    def _looks_like_new_goal(text: str) -> bool:
        normalized = text.replace(' ', '')
        return any(word in normalized for word in (
            '开始新任务', '执行新任务', '换个任务', '开始巡逻', '导航到', '去巡检点',
        ))

    @staticmethod
    def _operation_result(operation: Dict[str, Any]) -> Dict[str, Any]:
        state = str(operation.get('state') or 'failed')
        payload = dict(operation.get('result') or {})
        payload.setdefault('ok', state == 'succeeded')
        payload['status'] = state
        payload.setdefault('message', f'操作终态：{state}')
        data = dict(payload.get('data') or {})
        data.update({
            'operation_id': str(operation.get('operation_id') or ''),
            'operation_state': state,
        })
        payload['data'] = data
        return payload

    def _planner_unavailable(self, text: str) -> Dict[str, Any]:
        answer = 'LLM Planner 不可用，未执行动作。'
        if _wants_local_status(text):
            context = self.state.policy_context()
            patrol_state = str(context.get('patrol_state') or 'unknown')
            voice_state = str((context.get('voice_status') or {}).get('state') or '')
            answer = f'我是巡检机器人语言 Agent。当前巡逻状态：{patrol_state}；语音状态：{voice_state or "unknown"}。LLM Planner 不可用，未执行动作。'
        speech_text = prepare_speech_text(answer)
        decision = make_decision(
            'planner_unavailable', 'generate_local_status_reply', answer,
            speak=speech_text, final_answer=answer, response_type='reject')
        result = tool_result('inspection_agent', False, 'failed', answer, error_code='planner_unavailable')
        self.state.latest_decision = decision
        self.state.latest_result = result
        self.state.last_error = 'planner_unavailable'
        self.state.start_goal(text)
        self.state.finish(answer)
        return {
            'state': 'finished', 'decision': decision, 'result': result, 'assistant_text': answer,
            'display_text': answer, 'speech_text': speech_text, 'role': 'system',
        }

    def trim_history(self) -> None:
        turns: List[List[Dict[str, Any]]] = []
        for message in self.messages:
            if message.get('role') == 'user' or not turns:
                turns.append([])
            turns[-1].append(message)
        turns = turns[-max(1, int(self.history_max_turns)):]
        while len(turns) > 1 and self._history_chars(turns) > self.history_max_chars:
            turns.pop(0)
        self.messages = [message for turn in turns for message in turn]

    @staticmethod
    def _history_chars(turns: List[List[Dict[str, Any]]]) -> int:
        return sum(
            len(json.dumps(message, ensure_ascii=False, separators=(',', ':')))
            for turn in turns for message in turn
        )

    @staticmethod
    def _tool_turn(decision: Dict[str, Any], result: Dict[str, Any], tool_results: List[ToolResult]) -> Dict[str, Any]:
        return {
            'state': 'finished',
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
        self.state.finish(str(result.get('message') or '工具执行未完成。'))
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
