from __future__ import annotations

import json
import re
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List

from .agent_policy import authorize
from .agent_schema import SchemaError, tool_result, validate_decision
from .robot_reply_style import prepare_speech_text, speak as styled_speak


READ_ONLY_STATES = {'ok', 'warning', 'failed'}
SIDE_EFFECT_STATES = {
    'accepted', 'running', 'submitted', 'succeeded', 'failed', 'rejected',
    'canceled', 'timeout',
}
FINAL_TASK_STATES = {
    'informational', 'needs_input', 'ok', 'warning', 'running', 'submitted',
    'succeeded', 'failed', 'canceled', 'timeout',
}
DEFAULT_TERMINAL_STATES = {'succeeded', 'failed', 'rejected', 'canceled', 'timeout'}
FINAL_RESPONSE_TOOL = 'submit_final_response'


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
    return (
        make_decision('empty', 'generate_local_status_reply', text, response_type='ignore')
        if not text else None
    )


@dataclass
class RuntimeLease:
    lease_id: str
    session_id: str
    acquired_at: float
    expires_at: float
    state: str = 'active'


@dataclass
class Observation:
    observation_id: str
    tool_name: str
    ok: bool
    state: str
    message: str
    data: Dict[str, Any]
    error_code: str
    terminal: bool
    observed_at: float
    freshness: str
    operation_id: str = ''
    idempotency_key: str = ''

    def as_dict(self) -> Dict[str, Any]:
        return {
            'observation_id': self.observation_id,
            'tool_name': self.tool_name,
            'ok': self.ok,
            'state': self.state,
            'message': self.message,
            'data': self.data,
            'error_code': self.error_code,
            'terminal': self.terminal,
            'observed_at': self.observed_at,
            'freshness': self.freshness,
            'operation_id': self.operation_id,
            'idempotency_key': self.idempotency_key,
        }


@dataclass
class FinalResponse:
    answer: str
    evidence_refs: List[str]
    task_state: str
    response_type: str = 'final'

    @classmethod
    def parse(cls, content: str) -> 'FinalResponse':
        raw = str(content or '').strip()
        if raw.startswith('```'):
            raw = re.sub(r'^```(?:json)?\s*|\s*```$', '', raw, flags=re.I)
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise SchemaError(f'final response must be JSON: {exc}') from exc
        if not isinstance(value, dict) or value.get('response_type') != 'final':
            raise SchemaError('final response_type must be final')
        answer = value.get('answer')
        evidence_refs = value.get('evidence_refs')
        task_state = str(value.get('task_state') or '')
        if not isinstance(answer, str) or not answer.strip():
            raise SchemaError('final answer is required')
        if not isinstance(evidence_refs, list) or any(not isinstance(item, str) for item in evidence_refs):
            raise SchemaError('evidence_refs must be an array of strings')
        if task_state not in FINAL_TASK_STATES:
            raise SchemaError('invalid final task_state')
        return cls(answer.strip(), evidence_refs, task_state)


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


@dataclass
class AgentTurnContext:
    run_id: str
    request: Dict[str, Any]
    history: List[Dict[str, Any]]
    observations: List[Observation]
    active_operations: List[Dict[str, Any]]
    available_tools: List[Dict[str, Any]]
    runtime_lease: RuntimeLease | None
    planner_steps: int = 0
    protocol_retries: int = 0
    request_id: str = ''
    decision: Dict[str, Any] = field(default_factory=dict)
    pending_call: ToolCall | None = None
    pending_operation_id: str = ''
    pending_result: Dict[str, Any] = field(default_factory=dict)
    progress_announced: bool = False
    suppress_speech: bool = False
    call_idempotency: Dict[str, str] = field(default_factory=dict)
    previous_no_progress_key: str = ''
    identical_no_progress_calls: int = 0


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
        max_steps: int = 24,
        max_side_effect_tools_per_turn: int = 4,
        max_identical_tool_calls: int = 2,
        history_max_turns: int = 6,
        history_max_chars: int = 10_000,
        runtime_lease_idle_sec: float = 60.0,
        clock=time.time,
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
        self.max_steps = max(1, int(max_steps))
        self.max_side_effect_tools_per_turn = max_side_effect_tools_per_turn
        self.max_identical_tool_calls = max(1, int(max_identical_tool_calls))
        self.history_max_turns = history_max_turns
        self.history_max_chars = history_max_chars
        self.runtime_lease_idle_sec = max(1.0, float(runtime_lease_idle_sec))
        self.clock = clock
        self.messages: List[Dict[str, Any]] = []
        self.pending_turn: AgentTurnContext | None = None
        self.runtime_lease: RuntimeLease | None = None
        self.allowed_tool_names = set(self.tool_schemas) | self.tools.registry.names()
        self.allowed_tool_names.discard('send_motion_command')
        self.model_tool_names = {
            name for name in self.allowed_tool_names
            if (self.tool_schemas.get(name) or {}).get('model_visible', True)
        }

    def run_turn(self, request: Dict[str, Any]) -> Dict[str, Any]:
        local = decide_local(request, self.state.policy_context())
        if local:
            return self._execute_local_decision(local)
        text = str(request.get('text') or request.get('command') or '').strip()
        if self.pending_turn is not None:
            interrupted = self._interrupt_pending(request)
            return interrupted or self._waiting_turn(self.pending_turn)
        if not self.enabled or not self.planner.available():
            return self._planner_unavailable(text)

        self._refresh_runtime_lease(str(request.get('session_id') or request.get('source') or 'default'))
        run_id = str(request.get('run_id') or f'run_{int(self.clock() * 1000)}')
        self.state.start_goal(text)
        self.messages.append({'role': 'user', 'content': text})
        interaction = {
            key: request[key] for key in ('interaction_phase', 'contains_wake_phrase') if key in request
        }
        if interaction:
            self.messages.append({
                'role': 'system',
                'content': 'INTERNAL_INTERACTION_CONTEXT ' + json.dumps(
                    interaction, ensure_ascii=False, sort_keys=True, separators=(',', ':')),
            })
        context = AgentTurnContext(
            run_id=run_id,
            request=dict(request),
            history=self.messages,
            observations=[],
            active_operations=self._active_operations(),
            available_tools=self.openai_tools(),
            runtime_lease=self.runtime_lease,
            request_id=str(request.get('request_id') or request.get('client_msg_id') or ''),
        )
        return self._continue_safely(context)

    def resume_turn(self, operation: Dict[str, Any]) -> Dict[str, Any]:
        context = self.pending_turn
        if context is None or context.pending_call is None:
            raise ValueError('no agent turn is waiting for feedback')
        if str(operation.get('operation_id') or '') != context.pending_operation_id:
            raise ValueError('operation feedback does not match pending agent turn')
        call = context.pending_call
        result = self._operation_result(operation)
        observation = self._observe(call, result, operation_feedback=True)
        self._append_observation(call, observation)
        context.observations.append(observation)
        self._log_observation(context, observation)
        context.pending_call = None
        context.pending_operation_id = ''
        context.pending_result = {}
        self.pending_turn = None
        self.state.state = 'planning'
        self.state.pending_operation_id = ''
        if call.name == 'prepare_robot_runtime' and observation.state == 'succeeded':
            self._acquire_runtime_lease(context)
        if (self.tool_schemas.get(call.name) or {}).get('side_effect') == 'voice_session_control':
            self.release_runtime_lease('voice_session_ended')
        return self._continue_safely(context)

    def _continue_safely(self, context: AgentTurnContext) -> Dict[str, Any]:
        try:
            return self._continue_turn(context)
        except (SchemaError, ValueError, RuntimeError) as exc:
            detail = f'{exc.__class__.__name__}: {str(exc).strip() or repr(exc)}'
            message = '智能助手回复格式异常，请重试。'
            result = tool_result(
                'inspection_agent', False, 'failed', message, error_code='agent_error')
            self.state.last_error = detail
            self._log(f'agent protocol error: run_id={context.run_id} detail={detail}')
            self.state.latest_result = result
            self.tools.publish_event(result)
            turn = self._tool_turn({}, result, context)
            turn['speech_text'] = prepare_speech_text(message)
            return turn

    def _continue_turn(self, context: AgentTurnContext) -> Dict[str, Any]:
        while context.planner_steps < self.max_steps:
            context.planner_steps += 1
            self.trim_history()
            available_tool_names = set(self.model_tool_names)
            if self.runtime_lease is not None and self.runtime_lease.state == 'active':
                available_tool_names.discard('prepare_robot_runtime')
            response = self.planner.chat_tools(
                model=self.model,
                system_prompt=self._system_prompt(),
                messages=self.messages,
                tools=self._planning_tools(context, available_tool_names),
                timeout_sec=self.timeout_sec,
                temperature=0.0,
                tool_choice='required',
            )
            assistant_message = response.get('message')
            if not isinstance(assistant_message, dict):
                raise SchemaError('tool response is missing assistant message')
            self.messages.append(assistant_message)
            raw_calls = assistant_message.get('tool_calls') or []
            if not isinstance(raw_calls, list):
                raise SchemaError('assistant tool_calls must be an array')
            self._log(
                'agent step: run_id=%s step=%s response_type=%s tool=%s' % (
                    context.run_id, context.planner_steps,
                    'tool_call' if raw_calls else 'final',
                    str(((raw_calls[0].get('function') or {}).get('name') or '') if raw_calls else ''),
                ))
            if not raw_calls:
                final_content = str(
                    assistant_message.get('content') or response.get('content') or '')
                try:
                    final = FinalResponse.parse(final_content)
                    self._validate_final(final, context)
                except SchemaError as exc:
                    if context.protocol_retries >= 1:
                        raise
                    context.protocol_retries += 1
                    self.messages.append({
                        'role': 'system',
                        'content': 'FINAL_RESPONSE_PROTOCOL_ERROR: ' + str(exc)
                        + '。请仅返回符合 FinalResponse schema 的 JSON，并引用本轮 Observation。',
                    })
                    continue
                return self._final_turn(final, context)

            for index, raw_call in enumerate(raw_calls):
                call = self._parse_tool_call(raw_call)
                if call.name == FINAL_RESPONSE_TOOL:
                    try:
                        answer = call.arguments.get('answer')
                        refs = call.arguments.get('evidence_refs')
                        task_state = str(call.arguments.get('task_state') or '')
                        if not isinstance(answer, str) or not answer.strip():
                            raise SchemaError('final answer is required')
                        if not isinstance(refs, list) or any(
                            not isinstance(item, str) for item in refs
                        ):
                            raise SchemaError('evidence_refs must be an array of strings')
                        if task_state not in FINAL_TASK_STATES:
                            raise SchemaError('invalid final task_state')
                        final = FinalResponse(
                            answer=answer.strip(),
                            evidence_refs=refs,
                            task_state=task_state,
                        )
                        self._validate_final(final, context)
                    except (SchemaError, TypeError, ValueError) as exc:
                        self.messages.append({
                            'role': 'tool',
                            'tool_call_id': call.id,
                            'name': call.name,
                            'content': json.dumps({
                                'ok': False,
                                'error_code': 'invalid_final_response',
                                'message': str(exc),
                            }, ensure_ascii=False),
                        })
                        if context.protocol_retries >= 1:
                            raise SchemaError(str(exc)) from exc
                        context.protocol_retries += 1
                        self.messages.append({
                            'role': 'system',
                            'content': 'FINAL_RESPONSE_PROTOCOL_ERROR: ' + str(exc)
                            + '。请按 submit_final_response schema 重新提交。',
                        })
                        break
                    return self._final_turn(final, context)
                if (self.tool_schemas.get(call.name) or {}).get('suppress_speech_after_call'):
                    context.suppress_speech = True
                idempotency_key = context.call_idempotency.setdefault(
                    call.id, f'{context.run_id}:{call.id}')
                call_key = self._tool_call_key(call)
                if (
                    context.previous_no_progress_key.startswith(call_key + ':')
                    and context.identical_no_progress_calls >= self.max_identical_tool_calls
                ):
                    decision = make_decision(
                        call.name, call.name, '重复调用被阻止', response_type='reject',
                        safety_level='blocked', arguments=call.arguments)
                    decision.update({
                        'run_id': context.run_id, 'request_id': context.request_id,
                        'tool_call_id': call.id, 'idempotency_key': idempotency_key,
                    })
                    result = tool_result(
                        call.name, False, 'rejected', '相同工具、参数和错误已连续出现，拒绝第三次执行',
                        error_code='repeated_tool_call')
                    self._log(f'agent loop guard: run_id={context.run_id} reason=repeated_tool_call')
                else:
                    try:
                        decision = self._decision_from_tool_call(
                            call, context.run_id, context.request_id, idempotency_key,
                            available_tool_names)
                        result = self._execute_decision(decision, context)['result']
                    except (SchemaError, ValueError) as exc:
                        decision = make_decision(
                            call.name, call.name, str(exc), response_type='reject',
                            safety_level='blocked', arguments=call.arguments)
                        decision.update({
                            'run_id': context.run_id,
                            'request_id': context.request_id,
                            'tool_call_id': call.id,
                            'idempotency_key': idempotency_key,
                        })
                        result = tool_result(
                            call.name, False, 'rejected', f'工具参数不符合契约：{exc}',
                            error_code='invalid_tool_arguments')
                context.decision = decision
                no_progress_key = self._no_progress_key(call, result)
                if no_progress_key and no_progress_key == context.previous_no_progress_key:
                    context.identical_no_progress_calls += 1
                else:
                    context.previous_no_progress_key = no_progress_key
                    context.identical_no_progress_calls = 1 if no_progress_key else 0
                operation_id = str((result.get('data') or {}).get('operation_id') or '')
                if self._is_side_effect_tool(call.name) and result.get('status') == 'sent' and operation_id:
                    context.pending_call = call
                    context.pending_operation_id = operation_id
                    context.pending_result = result
                    self.pending_turn = context
                    self.state.wait_feedback(operation_id, f'等待 {call.name} 的真实反馈')
                    for deferred in raw_calls[index + 1:]:
                        deferred_call = self._parse_tool_call(deferred)
                        deferred_result = self._observe(
                            deferred_call,
                            tool_result(
                                deferred_call.name, False, 'rejected',
                                '副作用工具必须串行调用，请等待当前 Operation 终态',
                                error_code='serial_side_effect_required'),
                        )
                        self._append_observation(deferred_call, deferred_result)
                        context.observations.append(deferred_result)
                    return self._waiting_turn(context)

                observation = self._observe(call, result)
                self._append_observation(call, observation)
                context.observations.append(observation)
                self._log_observation(context, observation)
                self.state.latest_decision = decision
                self.state.latest_result = result
                if self._is_side_effect_tool(call.name):
                    break

        result = tool_result(
            'inspection_agent', False, 'failed',
            f'已达到最大规划步骤数 {self.max_steps}，停止继续调用工具',
            {'completed_observations': len(context.observations)},
            error_code='max_agent_steps')
        self._log(f'agent loop guard: run_id={context.run_id} reason=max_agent_steps')
        return self._tool_turn(context.decision, result, context)

    def _validate_final(self, final: FinalResponse, context: AgentTurnContext) -> None:
        observations = {item.observation_id: item for item in context.observations}
        refs = set(final.evidence_refs)
        if not refs.issubset(observations):
            raise SchemaError('evidence_refs contains unknown observation')
        if observations and not refs:
            raise SchemaError('at least one observation must be referenced')
        side_effect_refs = {
            item.observation_id for item in context.observations
            if self._is_side_effect_tool(item.tool_name)
        }
        if not side_effect_refs.issubset(refs):
            raise SchemaError('all side-effect observations must be referenced')
        if not observations:
            if final.task_state not in {'informational', 'needs_input'}:
                raise SchemaError('final without observations must be informational or needs_input')
            return
        referenced = [observations[item] for item in final.evidence_refs]
        compatible = {
            'ok': {'ok'},
            'warning': {'warning'},
            'running': {'running', 'accepted'},
            'submitted': {'submitted'},
            'succeeded': {'succeeded'},
            'failed': {'failed', 'rejected'},
            'canceled': {'canceled'},
            'timeout': {'timeout'},
            'needs_input': {'failed', 'rejected', 'warning'},
            'informational': {'ok', 'warning'},
        }[final.task_state]
        if not any(item.state in compatible for item in referenced):
            raise SchemaError('task_state is incompatible with referenced observations')
        if final.task_state in {'ok', 'succeeded'} and any(
            not self._is_side_effect_tool(item.tool_name)
            and item.freshness in {'stale', 'unknown'} for item in referenced
        ):
            raise SchemaError('stale or unknown observations cannot support a current success state')

    def _final_turn(self, final: FinalResponse, context: AgentTurnContext) -> Dict[str, Any]:
        speech_text = '' if context.suppress_speech else prepare_speech_text(final.answer)
        tool_lines = [
            f'- `{item.tool_name}`：{item.state}' for item in context.observations
        ]
        display_text = (
            final.answer + '\n\n调用工具：\n' + '\n'.join(tool_lines)
            if tool_lines else final.answer
        )
        decision = make_decision(
            'assistant_chat', 'generate_local_status_reply', final.answer,
            speak=speech_text, final_answer=final.answer, response_type='final_answer')
        result_status = final.task_state if final.task_state in {
            'ok', 'warning', 'running', 'submitted', 'succeeded', 'failed',
            'canceled', 'timeout',
        } else 'ok'
        result = tool_result(
            'assistant_chat', result_status not in {'failed', 'timeout'}, result_status,
            final.answer,
            {'answer': final.answer, 'evidence_refs': final.evidence_refs, 'task_state': final.task_state})
        self.state.latest_decision = decision
        self.state.latest_result = result
        self.state.finish(display_text)
        self._log(
            f'agent final: run_id={context.run_id} evidence_refs={final.evidence_refs} '
            f'task_state={final.task_state}')
        return {
            'state': 'finished',
            'decision': decision,
            'result': result,
            'assistant_text': final.answer,
            'display_text': display_text,
            'speech_text': speech_text,
            'role': 'assistant',
            'tool_results': [ToolResult('', item.tool_name, item.as_dict()) for item in context.observations],
            'observations': [item.as_dict() for item in context.observations],
        }

    def _observe(
        self,
        call: ToolCall,
        result: Dict[str, Any],
        operation_feedback: bool = False,
    ) -> Observation:
        schema = self.tool_schemas.get(call.name) or {}
        side_effect = self._is_side_effect_tool(call.name)
        state = str(result.get('status') or ('succeeded' if result.get('ok') else 'failed'))
        if state == 'sent':
            state = 'accepted'
        allowed = SIDE_EFFECT_STATES if side_effect else READ_ONLY_STATES
        if state not in allowed:
            state = 'succeeded' if side_effect and result.get('ok') else 'failed'
        received_at = self.clock()
        source_time = self._source_timestamp(result)
        observed_at = source_time if source_time is not None else received_at
        if operation_feedback or side_effect:
            freshness = 'fresh'
        elif source_time is None:
            freshness = 'unknown'
        else:
            ttl = schema.get('freshness_ttl_sec')
            freshness = (
                'unknown' if not isinstance(ttl, (int, float))
                else 'fresh' if received_at - source_time <= float(ttl) else 'stale'
            )
        terminal_states = set(schema.get('terminal_states') or DEFAULT_TERMINAL_STATES)
        terminal = not side_effect or state in terminal_states
        data = dict(result.get('data') or {})
        idempotency_key = str(data.get('idempotency_key') or '')
        operation_id = str(data.get('operation_id') or '')
        return Observation(
            observation_id=f'obs_{uuid.uuid4().hex}',
            tool_name=call.name,
            ok=bool(result.get('ok')),
            state=state,
            message=str(result.get('message') or ''),
            data=data,
            error_code=str(result.get('error_code') or ''),
            terminal=terminal,
            observed_at=float(observed_at),
            freshness=freshness,
            operation_id=operation_id,
            idempotency_key=idempotency_key,
        )

    @staticmethod
    def _source_timestamp(result: Dict[str, Any]) -> float | None:
        data = result.get('data') or {}
        for key in ('observed_at', 'source_timestamp', 'generated_at', 'updated_at'):
            value = data.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                return float(value)
        return None

    def _append_observation(self, call: ToolCall, observation: Observation) -> None:
        self.messages.append({
            'role': 'tool',
            'tool_call_id': call.id,
            'name': call.name,
            'content': json.dumps(observation.as_dict(), ensure_ascii=False),
        })
        self.state.add_step(
            f'调用 {call.name}：{observation.state}', observation.as_dict())

    def _decision_from_tool_call(
        self,
        call: ToolCall,
        run_id: str,
        request_id: str,
        idempotency_key: str,
        allowed_tool_names: set[str] | None = None,
    ) -> Dict[str, Any]:
        decision = make_decision(call.name, call.name, call.name, arguments=call.arguments)
        decision = validate_decision(
            decision,
            self.model_tool_names if allowed_tool_names is None else allowed_tool_names,
            self.tool_schemas,
        )
        decision.update({
            'run_id': run_id,
            'request_id': request_id,
            'tool_call_id': call.id,
            'idempotency_key': idempotency_key,
        })
        return decision

    def _execute_decision(
        self,
        decision: Dict[str, Any],
        context: AgentTurnContext | None = None,
    ) -> Dict[str, Any]:
        decision = validate_decision(
            decision,
            self.allowed_tool_names | {'emergency_stop', 'generate_local_status_reply'},
            self.tool_schemas,
        )
        policy = authorize(decision, self._policy_context(), self.tool_schemas)
        if policy.allowed:
            result = self.tools.execute(decision, policy)
        else:
            data: Dict[str, Any] = {}
            if policy.error_code == 'precondition_failed':
                data = {
                    'missing_preconditions': policy.missing_preconditions,
                    'recovery_tools': (
                        ['prepare_robot_runtime'] if 'bringup' in policy.recovery_components else []),
                    'current_state_summary': policy.state_summary,
                }
            result = tool_result(
                str((decision.get('tool_call') or {}).get('name') or ''),
                False, 'rejected', policy.reason, data=data,
                error_code=policy.error_code or 'policy_rejected')
            self.tools.publish_event(result)
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {'decision': decision, 'result': result, 'role': 'tool'}

    def _policy_context(self) -> Dict[str, Any]:
        context = self.state.policy_context()
        if self.tools.status_aggregator is not None:
            context['robot_summary'] = self.tools.status_aggregator.mode_aware_summary()
        if self.tools.diagnostic_engine is not None:
            context['latest_diagnostic'] = dict(
                getattr(self.tools.diagnostic_engine, 'last_report', {}) or {})
            context['now'] = self.clock()
            context['diagnostic_freshness_sec'] = float(
                getattr(self.tools.diagnostic_engine, 'diagnostic_freshness_sec', 5.0))
        context['active_operations'] = self._active_operations()
        return context

    def _active_operations(self) -> List[Dict[str, Any]]:
        manager = self.tools.operation_manager
        return manager.list_active() if manager is not None else []

    def _acquire_runtime_lease(self, context: AgentTurnContext) -> None:
        now = self.clock()
        session_id = str(context.request.get('session_id') or context.request.get('source') or 'default')
        if self.runtime_lease is None or self.runtime_lease.state != 'active':
            self.runtime_lease = RuntimeLease(
                f'lease_{uuid.uuid4().hex}', session_id, now,
                now + self.runtime_lease_idle_sec)
        else:
            self.runtime_lease.session_id = session_id
            self.runtime_lease.expires_at = now + self.runtime_lease_idle_sec
        context.runtime_lease = self.runtime_lease
        self._log(
            f'runtime lease: lease_id={self.runtime_lease.lease_id} state=active '
            f'expires_at={self.runtime_lease.expires_at}')

    def _refresh_runtime_lease(self, session_id: str) -> None:
        if self.runtime_lease is None or self.runtime_lease.state != 'active':
            return
        self.runtime_lease.session_id = session_id
        self.runtime_lease.expires_at = self.clock() + self.runtime_lease_idle_sec

    def release_runtime_lease(self, reason: str = 'idle') -> Dict[str, Any] | None:
        lease = self.runtime_lease
        if lease is None or lease.state != 'active':
            return None
        if reason == 'idle' and self._runtime_busy():
            lease.expires_at = self.clock() + self.runtime_lease_idle_sec
            return None
        call_id = f'release_{lease.lease_id}'
        decision = make_decision(
            'release_robot_runtime', 'release_robot_runtime', reason, arguments={})
        decision.update({
            'run_id': lease.lease_id,
            'tool_call_id': call_id,
            'idempotency_key': f'{lease.lease_id}:{call_id}',
        })
        previous_decision = self.state.latest_decision
        previous_result = self.state.latest_result
        result = self._execute_decision(decision)['result']
        self.state.latest_decision = previous_decision
        self.state.latest_result = previous_result
        lease.state = 'release_requested'
        self._log(
            f'runtime lease: lease_id={lease.lease_id} state={lease.state} '
            f'expires_at={lease.expires_at}')
        return result

    def reap_runtime_lease(self) -> Dict[str, Any] | None:
        lease = self.runtime_lease
        if lease is None or lease.state != 'active' or self.clock() < lease.expires_at:
            return None
        return self.release_runtime_lease('idle')

    def _runtime_busy(self) -> bool:
        if self._active_operations():
            return True
        patrol_state = str((self.state.patrol_status or {}).get('state') or '')
        return patrol_state in {
            'starting', 'command_sent', 'running', 'paused', 'returning_home',
            'waiting_loop', 'manual_takeover', 'canceling',
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
            for turn in turns for message in turn)

    def openai_tools(self, names: set[str] | None = None) -> List[Dict[str, Any]]:
        tools = []
        selected = self.model_tool_names if names is None else self.model_tool_names & names
        for name in sorted(selected):
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

    def _planning_tools(
        self, context: AgentTurnContext, names: set[str],
    ) -> List[Dict[str, Any]]:
        refs = [item.observation_id for item in context.observations]
        return [
            *self.openai_tools(names),
            {
                'type': 'function',
                'function': {
                    'name': FINAL_RESPONSE_TOOL,
                    'description': (
                        '结束本轮并向用户返回最终回答。必须引用给定的全部 Observation，'
                        'task_state 必须使用 schema 允许值。'
                    ),
                    'parameters': {
                        'type': 'object',
                        'properties': {
                            'answer': {'type': 'string', 'minLength': 1},
                            'evidence_refs': {
                                'type': 'array',
                                'items': {'type': 'string'},
                                'enum': [refs],
                            },
                            'task_state': {
                                'type': 'string',
                                'enum': self._final_task_states(context),
                            },
                        },
                        'required': ['answer', 'evidence_refs', 'task_state'],
                        'additionalProperties': False,
                    },
                },
            },
        ]

    def _final_task_states(self, context: AgentTurnContext) -> List[str]:
        if not context.observations:
            return ['informational', 'needs_input']
        evidence = next((
            item for item in reversed(context.observations)
            if self._is_side_effect_tool(item.tool_name)
        ), context.observations[-1])
        return {
            'accepted': ['running'],
            'running': ['running'],
            'submitted': ['submitted'],
            'succeeded': ['succeeded'],
            'failed': ['failed', 'needs_input'],
            'rejected': ['failed', 'needs_input'],
            'canceled': ['canceled'],
            'timeout': ['timeout', 'needs_input'],
            'warning': ['warning', 'needs_input'],
            'ok': ['informational'] if evidence.freshness != 'fresh' else ['ok', 'informational'],
        }[evidence.state]

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
        if self.route_toolpack and name in {'go_to_checkpoint', 'inspect_checkpoint'} and 'target_id' in arguments:
            arguments = dict(arguments)
            arguments['target_id'] = self.route_toolpack.catalog.resolve_target_id(
                str(arguments['target_id']))
        if self.route_toolpack and name in {'start_route', 'describe_route', 'list_checkpoints'} and 'route_id' in arguments:
            arguments = dict(arguments)
            arguments['route_id'] = self.route_toolpack.catalog.route(
                str(arguments['route_id']))['id']
        return ToolCall(call_id, name, arguments)

    def _interrupt_pending(self, request: Dict[str, Any]) -> Dict[str, Any] | None:
        context = self.pending_turn
        if context is None or context.pending_call is None or not self.planner.available():
            return None
        side_effect = str(
            (self.tool_schemas.get(context.pending_call.name) or {}).get('side_effect') or '')
        stop_tool = {
            'robot_motion': 'stop_motion',
            'robot_navigation': 'cancel_patrol',
            'patrol_control': 'cancel_patrol',
            'patrol_cancel': 'cancel_patrol',
        }.get(side_effect)
        allowed = {'emergency_stop'} | ({stop_tool} if stop_tool else set())
        response = self.planner.chat_tools(
            model=self.model,
            system_prompt=self._system_prompt(),
            messages=[{'role': 'user', 'content': str(request.get('text') or '')}],
            tools=self.openai_tools(allowed), timeout_sec=self.timeout_sec,
            temperature=0.0, tool_choice='auto')
        calls = (response.get('message') or {}).get('tool_calls') or []
        if len(calls) != 1:
            return None
        call = self._parse_tool_call(calls[0])
        if call.name not in allowed:
            return None
        key = f'{context.run_id}:{call.id}'
        decision = self._decision_from_tool_call(
            call, context.run_id, context.request_id, key, allowed)
        decision['target_operation_id'] = context.pending_operation_id
        result = self._execute_decision(decision)['result']
        return self._tool_turn(decision, result, context)

    def _waiting_turn(self, context: AgentTurnContext) -> Dict[str, Any]:
        decision = dict(context.decision)
        announce = not context.progress_announced and not context.suppress_speech
        context.progress_announced = True
        decision['speak'] = ({
            'reply_key': 'agent.waiting_feedback', 'text': '已开始执行，等待结果。',
            'priority': 5, 'interrupt': False,
        } if announce else {})
        return {
            'state': 'waiting_feedback',
            'pending_operation_id': context.pending_operation_id,
            'decision': decision,
            'result': context.pending_result,
            'assistant_text': '动作请求已发送，正在等待机器人真实反馈。',
            'display_text': '动作请求已发送，正在等待机器人真实反馈。',
            'speech_text': '已开始执行，等待结果。' if announce else '',
            'role': 'tool',
            'tool_results': [ToolResult('', item.tool_name, item.as_dict()) for item in context.observations],
            'observations': [item.as_dict() for item in context.observations],
        }

    def _tool_turn(
        self, decision: Dict[str, Any], result: Dict[str, Any], context: AgentTurnContext,
    ) -> Dict[str, Any]:
        message = str(result.get('message') or '工具执行未完成。')
        self.state.latest_decision = decision
        self.state.latest_result = result
        self.state.finish(message)
        return {
            'state': 'finished', 'decision': decision, 'result': result,
            'assistant_text': message, 'display_text': message, 'speech_text': '',
            'role': 'tool',
            'tool_results': [ToolResult('', item.tool_name, item.as_dict()) for item in context.observations],
            'observations': [item.as_dict() for item in context.observations],
        }

    def _execute_local_decision(self, decision: Dict[str, Any]) -> Dict[str, Any]:
        result = tool_result(
            'assistant_chat', True, 'ok', str(decision.get('final_answer') or ''),
            {'answer': str(decision.get('final_answer') or '')})
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {
            'state': 'finished', 'decision': decision, 'result': result,
            'assistant_text': str(decision.get('final_answer') or ''),
            'display_text': str(decision.get('final_answer') or ''),
            'speech_text': str((decision.get('speak') or {}).get('text') or ''),
            'role': 'tool', 'tool_results': [], 'observations': [],
        }

    def _planner_unavailable(self, text: str) -> Dict[str, Any]:
        answer = 'LLM Planner 不可用，未执行动作。'
        decision = make_decision(
            'planner_unavailable', 'generate_local_status_reply', answer,
            speak=prepare_speech_text(answer), final_answer=answer, response_type='reject')
        result = tool_result(
            'inspection_agent', False, 'failed', answer,
            error_code='planner_unavailable')
        self.state.start_goal(text)
        self.state.finish(answer)
        self.state.latest_decision = decision
        self.state.latest_result = result
        return {
            'state': 'finished', 'decision': decision, 'result': result,
            'assistant_text': answer, 'display_text': answer,
            'speech_text': prepare_speech_text(answer), 'role': 'system',
        }

    @staticmethod
    def _operation_result(operation: Dict[str, Any]) -> Dict[str, Any]:
        operation_state = str(operation.get('state') or 'failed')
        payload = dict(operation.get('result') or {})
        business_state = str(payload.pop('result_status', '') or '')
        status = business_state if business_state in SIDE_EFFECT_STATES else operation_state
        payload['status'] = status
        payload['ok'] = status in {'succeeded', 'submitted', 'running', 'accepted'}
        payload.setdefault('message', f'操作终态：{operation_state}')
        data = dict(payload.get('data') or {})
        data.update({
            'operation_id': str(operation.get('operation_id') or ''),
            'operation_state': operation_state,
            'arguments': dict(operation.get('arguments') or {}),
            'idempotency_key': str(operation.get('idempotency_key') or ''),
        })
        payload['data'] = data
        return payload

    @staticmethod
    def _update_component_ledger(
        context: Dict[str, Any], call: ToolCall, result: Dict[str, Any],
    ) -> None:
        # Retained for compatibility; RuntimeLease replaced per-turn component ownership.
        return None

    @staticmethod
    def _no_progress_key(call: ToolCall, result: Dict[str, Any]) -> str:
        error = str(result.get('error_code') or '')
        if not error:
            return ''
        arguments = json.dumps(
            call.arguments, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        return f'{call.name}:{arguments}:{error}'

    @staticmethod
    def _tool_call_key(call: ToolCall) -> str:
        arguments = json.dumps(
            call.arguments, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        return f'{call.name}:{arguments}'

    def _is_side_effect_tool(self, name: str) -> bool:
        schema = self.tool_schemas.get(name) or {}
        return schema.get('risk_level') != 'read_only' and schema.get('side_effect', 'none') != 'none'

    def _system_prompt(self) -> str:
        return self.spec.system_prompt() + (
            '\n\n最终响应协议：完成规划或需要用户输入时，必须调用 submit_final_response；'
            '不得用普通 assistant content 代替。answer 使用自然简体中文；'
            '先给结论，使用简短完整的句子，避免标题、长复句和重复说明；'
            '单一操作完成时只需一句结果，不复述运行环境准备等中间过程；'
            'submitted/running 不得表述为 succeeded。'
        )

    @staticmethod
    def _tool_description(name: str, schema: Dict[str, Any]) -> str:
        description = str(schema.get('description') or f'Inspection robot tool: {name}')
        preconditions = ', '.join(schema.get('preconditions') or []) or '无'
        states = ', '.join((schema.get('result_schema') or {}).get('status') or []) or '以 Observation 为准'
        return (
            f'{description} 前置条件：{preconditions}。结果状态：{states}。'
            '工具结果仅作为 Observation，必须依据真实状态继续规划。')

    def _runtime_busy_message(self) -> str:
        return '当前目标仍在执行，请先停止或等待完成。'

    def _log_observation(self, context: AgentTurnContext, observation: Observation) -> None:
        self._log(
            'agent observation: run_id=%s observation_id=%s tool=%s state=%s '
            'terminal=%s freshness=%s observed_at=%s' % (
                context.run_id, observation.observation_id, observation.tool_name,
                observation.state, observation.terminal, observation.freshness,
                observation.observed_at,
            ))

    def _log(self, message: str) -> None:
        logger = getattr(self.tools.node, 'get_logger', lambda: None)()
        if logger is not None and hasattr(logger, 'info'):
            logger.info(message)
