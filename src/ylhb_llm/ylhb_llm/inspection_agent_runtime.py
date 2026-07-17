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
    if not text:
        return make_decision('empty', 'generate_local_status_reply', text, response_type='ignore')
    return None


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
        max_steps: int = 12,
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
        self.model_tool_names = {
            name for name in self.allowed_tool_names
            if (self.tool_schemas.get(name) or {}).get('model_visible', True)
        }

    def run_turn(self, request: Dict[str, Any]) -> Dict[str, Any]:
        local = decide_local(request, self.state.policy_context())
        if local:
            if self.pending_turn is not None:
                local['target_operation_id'] = str(
                    self.pending_turn.get('pending_operation_id') or '')
            return self._execute_decision(local)
        text = str(request.get('text') or request.get('command') or '').strip()
        if self.pending_turn is not None:
            interrupt = self._interrupt_pending(request)
            if interrupt is not None:
                return interrupt
            turn = self._waiting_turn(self.pending_turn)
            turn['assistant_text'] = '当前目标仍在执行，请先停止或等待完成。'
            turn['display_text'] = turn['assistant_text']
            turn['decision']['speak'] = {}
            return turn
        if not self.enabled or not self.planner.available():
            return self._planner_unavailable(text)

        run_id = str(request.get('run_id') or f'run_{int(time.time() * 1000)}')
        request_id = str(request.get('request_id') or request.get('client_msg_id') or '')
        self.state.start_goal(text)
        self.messages.append({'role': 'user', 'content': text})
        interaction = {
            key: request[key]
            for key in ('interaction_phase', 'contains_wake_phrase')
            if key in request
        }
        if interaction:
            self.messages.append({
                'role': 'system',
                'content': 'INTERNAL_INTERACTION_CONTEXT ' + json.dumps(
                    interaction, ensure_ascii=False, sort_keys=True, separators=(',', ':')),
            })
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
            'required_retry_used': False,
            'force_tool_once': True,
            'started_components': [],
            'progress_announced': False,
            'preparation_seen': False,
            'target_tool': '',
            'terminal_side_effect_tools': [],
        }
        self._inject_robot_summary(context)
        return self._continue_safely(context)

    def resume_turn(self, operation: Dict[str, Any]) -> Dict[str, Any]:
        context = self.pending_turn
        if context is None:
            raise ValueError('no agent turn is waiting for feedback')
        if str(operation.get('operation_id') or '') != str(context.get('pending_operation_id') or ''):
            raise ValueError('operation feedback does not match pending agent turn')
        call = context.pop('pending_call')
        result = self._operation_result(operation)
        self._update_component_ledger(context, call, result)
        if call.name in {'start_component', 'stop_component'}:
            self._inject_robot_summary(context)
        component_status = str((result.get('data') or {}).get('component_status') or '')
        if call.name == 'start_component' and component_status == 'started':
            context['preparation_seen'] = True
        if (
            call.name not in {'start_component', 'stop_component'}
            and self._is_side_effect_tool(call.name)
            and result.get('status') in {'succeeded', 'failed', 'canceled', 'timeout', 'rejected'}
        ):
            completed = context.setdefault('terminal_side_effect_tools', [])
            if call.name not in completed:
                completed.append(call.name)
        if context.pop('cleanup_active', False):
            if str((result.get('data') or {}).get('component_status') or '') not in {
                'stopped', 'already_stopped',
            }:
                context.setdefault('cleanup_failures', []).append(
                    str(call.arguments.get('component') or 'unknown'))
            context.pop('pending_operation_id', None)
            self.pending_turn = None
            self.state.pending_operation_id = ''
            return self._finish_or_cleanup(context, context['terminal_turn'])
        self._append_tool_result(call, result)
        context['tool_results'].append(ToolResult(call.id, call.name, result))
        context.pop('pending_operation_id', None)
        self.pending_turn = None
        self.state.state = 'planning'
        self.state.pending_operation_id = ''
        self.state.add_step(f'收到真实反馈：{result.get("status")}', {'tool': call.name, 'result': result})
        if (self.tool_schemas.get(call.name) or {}).get('side_effect') == 'voice_session_control':
            self.state.finish(str(result.get('message') or '语音会话操作已结束。'))
            return self._finish_or_cleanup(
                context, self._tool_turn(context['decision'], result, context['tool_results']))
        return self._continue_safely(context)

    def _continue_safely(self, context: Dict[str, Any]) -> Dict[str, Any]:
        try:
            return self._continue_turn(context)
        except (SchemaError, ValueError, RuntimeError) as exc:
            message = f'{exc.__class__.__name__}: {str(exc).strip() or repr(exc)}'
            decision = make_decision(
                'agent_error', 'generate_local_status_reply', message,
                speak='语言模型暂不可用，未执行后续动作。', response_type='reject',
                safety_level='blocked',
            )
            result = tool_result(
                'inspection_agent', False, 'failed', message, error_code='agent_error')
            self.state.last_error = message
            self.state.latest_decision = decision
            self.state.latest_result = result
            self.tools.publish_event(result)
            return self._finish_or_cleanup(
                context, self._tool_turn(decision, result, context['tool_results']))

    def _continue_turn(self, context: Dict[str, Any]) -> Dict[str, Any]:
        tool_results: List[ToolResult] = context['tool_results']
        decision: Dict[str, Any] = context['decision']
        while context['steps_used'] < max(1, self.max_steps):
            context['steps_used'] += 1
            self.trim_history()
            tool_choice = 'required' if context['force_tool_once'] else 'auto'
            available_tool_names = set(self.model_tool_names)
            if not context.get('recovery_components'):
                available_tool_names.discard('start_component')
            response = self.planner.chat_tools(
                model=self.model,
                system_prompt=self.spec.system_prompt(),
                messages=self.messages,
                tools=self.openai_tools(available_tool_names),
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
                target_tool = str(context.get('target_tool') or '')
                if (
                    (target_tool or context.get('preparation_seen'))
                    and not self._has_valid_evidence(target_tool, tool_results)
                ):
                    if not context['required_retry_used']:
                        context['required_retry_used'] = True
                        context['force_tool_once'] = True
                        self.messages.pop()
                        continue
                    return self._finish_or_cleanup(
                        context, self._missing_evidence_turn(tool_results))
                speech_text = '' if context.get('suppress_speech') else prepare_speech_text(assistant_text)
                decision = make_decision(
                    'assistant_chat', 'generate_local_status_reply', assistant_text,
                    speak=speech_text, final_answer=assistant_text, response_type='final_answer',
                )
                result = tool_result('assistant_chat', True, 'ok', assistant_text, {'answer': assistant_text})
                self.state.latest_decision = decision
                self.state.latest_result = result
                self.state.finish(assistant_text)
                return self._finish_or_cleanup(context, {
                    'state': 'finished',
                    'decision': decision, 'result': result, 'assistant_text': assistant_text,
                    'display_text': assistant_text, 'speech_text': speech_text,
                    'role': 'assistant', 'tool_results': tool_results,
                })

            for raw_call in raw_calls:
                function = raw_call.get('function') if isinstance(raw_call, dict) else {}
                call = ToolCall(
                    str(raw_call.get('id') or 'invalid') if isinstance(raw_call, dict) else 'invalid',
                    str((function or raw_call or {}).get('name') or '') if isinstance(raw_call, dict) else '',
                    {},
                )
                if (self.tool_schemas.get(call.name) or {}).get('suppress_speech_after_call'):
                    context['suppress_speech'] = True
                try:
                    call = self._parse_tool_call(raw_call)
                    decision = self._decision_from_tool_call(
                        call,
                        assistant_text,
                        context['run_id'],
                        context['request_id'],
                        available_tool_names,
                    )
                except (SchemaError, ValueError) as exc:
                    decision = make_decision(
                        call.name, call.name, str(exc), response_type='reject',
                        safety_level='blocked', arguments=call.arguments,
                    )
                    result = tool_result(
                        call.name, False, 'rejected', f'工具参数不符合契约：{exc}',
                        error_code='invalid_tool_arguments',
                    )
                    return self._append_and_return(context, call, decision, result, tool_results)
                context['decision'] = decision
                call_key = self._tool_call_key(call)
                side_effect = self._is_side_effect_tool(call.name)
                side_effect_type = str(
                    (self.tool_schemas.get(call.name) or {}).get('side_effect') or '')
                if side_effect_type != 'component_lifecycle' and side_effect and not context.get('target_tool'):
                    context['target_tool'] = call.name
                elif (
                    side_effect_type != 'component_lifecycle'
                    and side_effect
                    and call.name != context.get('target_tool')
                ):
                    result = tool_result(
                        call.name, False, 'rejected',
                        f'本轮目标已锁定为 {context["target_tool"]}，拒绝切换为 {call.name}',
                        error_code='target_tool_changed',
                    )
                    return self._append_and_return(context, call, decision, result, tool_results)
                if call.name in context.get('terminal_side_effect_tools', []):
                    result = tool_result(
                        call.name, False, 'rejected',
                        '同一动作已在本轮取得终态，拒绝重复执行',
                        error_code='duplicate_terminal_action',
                    )
                    self._append_tool_result(call, result)
                    tool_results.append(ToolResult(call.id, call.name, result))
                    self.state.latest_decision = decision
                    self.state.latest_result = result
                    break
                if not side_effect:
                    context['previous_call_key'] = ''
                    context['identical_call_count'] = 0
                elif call_key == context['previous_call_key']:
                    context['identical_call_count'] += 1
                else:
                    context['previous_call_key'] = call_key
                    context['identical_call_count'] = 1
                if side_effect and context['identical_call_count'] > self.max_identical_tool_calls:
                    result = tool_result(
                        call.name, False, 'failed', '相同工具调用重复，已停止任务',
                        error_code='repeated_tool_call',
                    )
                    return self._append_and_return(context, call, decision, result, tool_results)

                if side_effect and context['side_effect_tools'] >= self.max_side_effect_tools_per_turn:
                    result = tool_result(
                        call.name, False, 'failed', '已达到本轮副作用工具上限，已停止任务',
                        error_code='max_side_effect_tools',
                    )
                    return self._append_and_return(context, call, decision, result, tool_results)

                if call.name == 'start_component':
                    component = str(call.arguments.get('component') or '')
                    allowed_recovery = list(context.get('recovery_components') or [])
                    if not context.get('target_tool') or component not in allowed_recovery:
                        result = tool_result(
                            call.name, False, 'rejected',
                            f'{component} 不在本次允许的恢复组件中',
                            {'recovery_components': allowed_recovery},
                            error_code='unexpected_recovery_component',
                        )
                        self._append_tool_result(call, result)
                        tool_results.append(ToolResult(call.id, call.name, result))
                        self.state.latest_decision = decision
                        self.state.latest_result = result
                        break
                    context['recovery_components'] = []

                result = self._execute_decision(decision, context)['result']
                if result.get('error_code') == 'precondition_failed':
                    result_data = result.get('data') or {}
                    missing = set(result_data.get('missing_preconditions') or [])
                    if self._owned_readiness_missing(context, missing):
                        result_data.update({
                            'recoverable': True,
                            'recovery_components': [],
                        })
                side_effect_applied = side_effect and result.get('status') != 'rejected'
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
                if side_effect_applied:
                    context['side_effect_tools'] += 1
                if result.get('error_code') == 'precondition_failed':
                    result_data = result.get('data') or {}
                    recovery_components = list(result_data.get('recovery_components') or [])
                    context['recovery_components'] = recovery_components
                    if (
                        not recovery_components
                        and not result_data.get('recoverable')
                        and result_data.get('missing_preconditions') != ['started_this_turn']
                    ):
                        self.state.finish(str(result.get('message') or '工具前置条件不满足。'))
                        return self._finish_or_cleanup(
                            context, self._tool_turn(decision, result, tool_results))
                if result.get('error_code') == 'policy_rejected':
                    self.state.finish(str(result.get('message') or '工具请求被拒绝。'))
                    return self._finish_or_cleanup(
                        context, self._tool_turn(decision, result, tool_results))
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
        return self._finish_or_cleanup(
            context, self._tool_turn(decision, result, tool_results))

    @staticmethod
    def _owned_readiness_missing(context: Dict[str, Any], missing: set[str]) -> bool:
        owned = set(context.get('started_components') or [])
        readiness = {f'{component}_running' for component in owned}
        if 'bringup' in owned:
            readiness.update({'robot_ready', 'chassis_online', 'sensor_fresh'})
        return bool(missing) and missing.issubset(readiness)

    def _waiting_turn(self, context: Dict[str, Any]) -> Dict[str, Any]:
        decision = dict(context.get('decision') or {})
        announce = (
            not context.get('progress_announced', False)
            and not context.get('suppress_speech', False)
            and not context.get('cleanup_active', False)
        )
        context['progress_announced'] = True
        decision['speak'] = ({
            'reply_key': 'agent.waiting_feedback',
            'text': '已开始执行，等待结果。',
            'priority': 5,
            'interrupt': False,
        } if announce else {})
        result = context.get('pending_result') or self.state.latest_result or {}
        return {
            'state': 'waiting_feedback',
            'pending_operation_id': str(context.get('pending_operation_id') or ''),
            'decision': decision,
            'result': result,
            'assistant_text': '动作请求已发送，正在等待机器人真实反馈。',
            'display_text': '动作请求已发送，正在等待机器人真实反馈。',
            'speech_text': '已开始执行，等待结果。' if announce else '',
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

    def _has_valid_evidence(self, target_tool: str, tool_results: List[ToolResult]) -> bool:
        for item in tool_results:
            result = item.result or {}
            if (self.tool_schemas.get(item.name) or {}).get('side_effect') == 'component_lifecycle':
                continue
            if (
                result.get('error_code') == 'precondition_failed'
                and (result.get('data') or {}).get('recoverable')
            ):
                continue
            if target_tool and item.name == target_tool and result.get('status') in {
                'succeeded', 'failed', 'canceled', 'timeout', 'rejected',
            }:
                return True
        return False

    def _finish_or_cleanup(
        self, context: Dict[str, Any], terminal_turn: Dict[str, Any],
    ) -> Dict[str, Any]:
        context['terminal_turn'] = terminal_turn
        attempted = context.setdefault('cleanup_attempted', [])
        component = next((
            item for item in reversed(context.get('started_components') or [])
            if item not in attempted
        ), '')
        if not component:
            failures = list(dict.fromkeys(context.get('cleanup_failures') or []))
            turn = self._report_cleanup_failures(terminal_turn, failures)
            self.state.latest_decision = dict(turn.get('decision') or {})
            self.state.latest_result = dict(turn.get('result') or {})
            self.state.finish(str(turn.get('assistant_text') or ''))
            return turn

        attempted.append(component)
        call = ToolCall(f'cleanup_{component}_{len(attempted)}', 'stop_component', {
            'component': component,
        })
        decision = make_decision(
            'stop_component', 'stop_component', '自动回收本轮启动组件',
            arguments=call.arguments,
        )
        decision.update({
            'run_id': context.get('run_id', ''),
            'request_id': context.get('request_id', ''),
            'tool_call_id': call.id,
        })
        result = self._execute_decision(decision, context)['result']
        operation_id = str((result.get('data') or {}).get('operation_id') or '')
        if result.get('status') == 'sent' and operation_id:
            context['decision'] = decision
            context['pending_call'] = call
            context['pending_operation_id'] = operation_id
            context['pending_result'] = result
            context['cleanup_active'] = True
            self.pending_turn = context
            self.state.state = 'waiting_feedback'
            self.state.pending_operation_id = operation_id
            return self._waiting_turn(context)

        context.setdefault('cleanup_failures', []).append(component)
        return self._finish_or_cleanup(context, terminal_turn)

    @staticmethod
    def _report_cleanup_failures(
        terminal_turn: Dict[str, Any], failures: List[str],
    ) -> Dict[str, Any]:
        if not failures:
            return terminal_turn
        turn = dict(terminal_turn)
        warning = '；未能关闭本轮启动的组件：' + '、'.join(failures)
        original_text = str(turn.get('assistant_text') or '')
        original_display = str(turn.get('display_text') or original_text)
        turn['assistant_text'] = original_text + warning
        turn['display_text'] = original_display + warning
        turn['speech_text'] = str(turn.get('speech_text') or '') + warning
        decision = dict(turn.get('decision') or {})
        speak = dict(decision.get('speak') or {})
        speak['text'] = str(speak.get('text') or '') + warning
        decision['speak'] = speak
        turn['decision'] = decision
        result = dict(turn.get('result') or {})
        result['data'] = {
            **(result.get('data') or {}),
            'residual_components': failures,
        }
        turn['result'] = result
        return turn

    @staticmethod
    def _operation_result(operation: Dict[str, Any]) -> Dict[str, Any]:
        state = str(operation.get('state') or 'failed')
        payload = dict(operation.get('result') or {})
        payload.setdefault('ok', state == 'succeeded')
        component_status = str(payload.pop('result_status', '') or '')
        payload['status'] = state
        payload.setdefault('message', f'操作终态：{state}')
        data = dict(payload.get('data') or {})
        data.update({
            'operation_id': str(operation.get('operation_id') or ''),
            'operation_state': state,
            'arguments': dict(operation.get('arguments') or {}),
        })
        if str(operation.get('tool_name') or '') in {'start_component', 'stop_component'}:
            data['component_status'] = component_status or state
        payload['data'] = data
        return payload

    @staticmethod
    def _update_component_ledger(
        context: Dict[str, Any], call: ToolCall, result: Dict[str, Any],
    ) -> None:
        if call.name not in {'start_component', 'stop_component'}:
            return
        component = str(call.arguments.get('component') or '')
        started = context.setdefault('started_components', [])
        component_status = str((result.get('data') or {}).get('component_status') or '')
        if call.name == 'start_component' and component_status == 'started':
            if component and component not in started:
                started.append(component)
        elif call.name == 'stop_component' and component_status in {'stopped', 'already_stopped'}:
            if component in started:
                started.remove(component)

    def _planner_unavailable(self, text: str) -> Dict[str, Any]:
        answer = 'LLM Planner 不可用，未执行动作。'
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
        context: Dict[str, Any],
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
        return self._finish_or_cleanup(
            context, self._tool_turn(decision, result, tool_results))

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

    def openai_tools(self, names: set[str] | None = None) -> List[Dict[str, Any]]:
        tools = []
        for name in sorted(self.model_tool_names if names is None else self.model_tool_names & names):
            schema = self.tool_schemas.get(name) or self.tools.registry.schemas().get(name) or {}
            tools.append({
                'type': 'function',
                'function': {
                    'name': name,
                    'description': self._tool_description(name, schema),
                    'parameters': {
                        'type': 'object',
                        'properties': schema.get('properties') or {},
                        'required': schema.get('required') or [],
                        'additionalProperties': False,
                    },
                },
            })
        return tools

    def _interrupt_pending(self, request: Dict[str, Any]) -> Dict[str, Any] | None:
        context = self.pending_turn or {}
        pending_call = context.get('pending_call')
        if pending_call is None or not self.enabled or not self.planner.available():
            return None
        side_effect = str((self.tool_schemas.get(pending_call.name) or {}).get('side_effect') or '')
        stop_tool = {
            'robot_motion': 'stop_motion',
            'robot_navigation': 'cancel_patrol',
            'patrol_control': 'cancel_patrol',
            'patrol_cancel': 'cancel_patrol',
        }.get(side_effect)
        allowed = {'emergency_stop'} | ({stop_tool} if stop_tool else set())
        response = self.planner.chat_tools(
            model=self.model,
            system_prompt=self.spec.system_prompt(),
            messages=[
                {'role': 'user', 'content': str(request.get('text') or request.get('command') or '')},
                {'role': 'system', 'content': '当前有操作执行中，只能选择提供的停止工具；否则直接澄清。'},
            ],
            tools=self.openai_tools(allowed),
            timeout_sec=self.timeout_sec,
            temperature=0.0,
            tool_choice='auto',
        )
        message = response.get('message') or {}
        raw_calls = message.get('tool_calls') or []
        if len(raw_calls) != 1:
            return None
        try:
            call = self._parse_tool_call(raw_calls[0])
            if call.name not in allowed:
                return None
            decision = self._decision_from_tool_call(
                call,
                str(message.get('content') or ''),
                str(request.get('run_id') or context.get('run_id') or ''),
                str(request.get('request_id') or request.get('client_msg_id') or ''),
                allowed,
            )
        except (SchemaError, ValueError):
            return None
        decision['target_operation_id'] = str(context.get('pending_operation_id') or '')
        executed = self._execute_decision(decision)
        return self._tool_turn(decision, executed['result'], [])

    @staticmethod
    def _tool_description(name: str, schema: Dict[str, Any]) -> str:
        description = str(schema.get('description') or f'Inspection robot tool: {name}')
        preconditions = list(schema.get('preconditions') or [])
        argument_preconditions = schema.get('preconditions_by_argument') or {}
        results = list((schema.get('result_schema') or {}).get('status') or [])
        side_effect = str(schema.get('side_effect') or 'none')
        risk = str(schema.get('risk_level') or 'normal')
        timeout = schema.get('timeout_sec')
        use = '需要执行该能力并取得真实结果时。' if side_effect != 'none' else '需要读取这类实时信息时。'
        avoid = (
            '前置条件失败后，仅当 recovery_components 非空时启动其中一个组件。'
            if preconditions or argument_preconditions
            else '不要用于其他职责或猜测状态。'
        )
        argument_rules = '; '.join(
            f'{field}={value}: {", ".join(required)}'
            for field, choices in argument_preconditions.items()
            for value, required in (choices or {}).items()
        )
        after = (
            '调用后等待真实终态，再调用 get_robot_summary 重新查询确认。'
            if side_effect != 'none'
            else '读取后仅依据返回数据回答。'
        )
        return ' '.join(filter(None, (
            description,
            f'什么时候使用：{use}',
            f'不要使用：{avoid}',
            '前置条件：' + (', '.join(preconditions) if preconditions else '无'),
            f'按参数前置条件：{argument_rules}。' if argument_rules else '',
            f'副作用/风险：{side_effect}/{risk}',
            '结果状态：' + (', '.join(results) if results else '以 ToolResult 为准'),
            f'超时：{timeout:g} 秒。' if isinstance(timeout, (int, float)) else '',
            after,
        )))

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
        allowed_tool_names: set[str] | None = None,
    ) -> Dict[str, Any]:
        decision = make_decision(
            call.name, call.name, assistant_text or call.name, speak='',
            final_answer=assistant_text, arguments=call.arguments,
        )
        decision = validate_decision(
            decision,
            self.model_tool_names if allowed_tool_names is None else allowed_tool_names,
            self.tool_schemas,
        )
        decision['run_id'] = run_id
        decision['request_id'] = request_id
        decision['tool_call_id'] = call.id
        return decision

    def _execute_decision(
        self,
        decision: Dict[str, Any],
        run_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        decision = validate_decision(
            decision,
            self.allowed_tool_names | {'emergency_stop', 'generate_local_status_reply'},
            self.tool_schemas,
        )
        policy = authorize(decision, self._policy_context(run_context), self.tool_schemas)
        if policy.allowed:
            result = self.tools.execute(decision, policy)
        else:
            data = {}
            if policy.error_code == 'precondition_failed':
                data = {
                    'missing_preconditions': policy.missing_preconditions,
                    'recoverable': policy.recoverable,
                    'recovery_components': policy.recovery_components,
                    'current_state_summary': policy.state_summary,
                }
            result = tool_result(
                decision['tool_call']['name'], False, 'rejected', policy.reason,
                data=data,
                error_code=policy.error_code or 'policy_rejected',
            )
            self.tools.publish_event(result)
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {
            'decision': decision,
            'result': result,
            'assistant_text': decision.get('final_answer') or decision.get('reason_cn') or '',
            'role': 'tool',
        }

    def _policy_context(
        self, run_context: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        context = self.state.policy_context()
        if self.tools.status_aggregator is not None:
            context['robot_summary'] = self.tools.status_aggregator.summary()
        if run_context is not None:
            context['started_components'] = list(run_context.get('started_components') or [])
        return context

    def _inject_robot_summary(self, context: Dict[str, Any]) -> None:
        aggregator = self.tools.status_aggregator
        if aggregator is None:
            return
        summary = aggregator.summary()
        self.messages.append({
            'role': 'system',
            'content': 'INTERNAL_FRESH_ROBOT_SUMMARY 仅供推理，用户可见状态必须用自然简体中文概括：' + json.dumps(
                summary, ensure_ascii=False, sort_keys=True, separators=(',', ':')),
        })
        context['latest_robot_summary'] = summary
