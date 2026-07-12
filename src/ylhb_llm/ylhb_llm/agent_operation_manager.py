from __future__ import annotations

import time
import threading
import uuid
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Dict

from .agent_protocol import (
    LEGACY_OPERATION_STATES,
    OPERATION_STATES,
    OPERATION_TRANSITIONS,
    TERMINAL_OPERATION_STATES,
)

TERMINAL_STATES = TERMINAL_OPERATION_STATES


@dataclass
class AgentOperation:
    operation_id: str
    run_id: str
    tool_call_id: str
    tool_name: str
    arguments: dict
    state: str
    created_at: float
    accepted_at: float | None
    finished_at: float | None
    timeout_sec: float
    result: dict = field(default_factory=dict)


class AgentOperationManager:
    """In-memory operation ledger; ROS callbacks provide all state changes."""

    def __init__(self, max_operations: int = 128, clock: Callable[[], float] = time.time) -> None:
        self.clock = clock
        self.max_operations = max(1, int(max_operations))
        self._operations: Dict[str, AgentOperation] = {}
        self._order: deque[str] = deque()
        self._by_run_tool: Dict[tuple[str, str], str] = {}
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)

    def create(
        self,
        run_id: str,
        tool_call_id: str,
        tool_name: str,
        arguments: dict,
        timeout_sec: float,
        now: float | None = None,
    ) -> AgentOperation:
        created_at = self.clock() if now is None else now
        operation = AgentOperation(
            operation_id=f'op_{uuid.uuid4().hex}',
            run_id=run_id,
            tool_call_id=tool_call_id,
            tool_name=tool_name,
            arguments=dict(arguments),
            state='created',
            created_at=created_at,
            accepted_at=None,
            finished_at=None,
            timeout_sec=float(timeout_sec),
            result={'ok': True, 'status': 'created', 'message': '操作已创建'},
        )
        with self._changed:
            while len(self._order) >= self.max_operations:
                old_id = self._order.popleft()
                old = self._operations.pop(old_id, None)
                if old is not None:
                    self._by_run_tool.pop((old.run_id, old.tool_call_id), None)
            self._operations[operation.operation_id] = operation
            self._order.append(operation.operation_id)
            self._by_run_tool[(run_id, tool_call_id)] = operation.operation_id
            self._changed.notify_all()
        return operation

    def mark_sent(self, operation_id: str, now: float | None = None) -> AgentOperation:
        return self.update(operation_id, 'sent', {'ok': True, 'status': 'sent', 'message': '工具请求已发送'}, now)

    def update(
        self,
        operation_id: str,
        state: str,
        result: dict | None = None,
        now: float | None = None,
    ) -> AgentOperation:
        state = LEGACY_OPERATION_STATES.get(state, state)
        if state not in OPERATION_STATES:
            raise ValueError(f'unknown operation state: {state}')
        with self._changed:
            operation = self._operations[operation_id]
            if operation.state in TERMINAL_STATES:
                return operation
            if state != operation.state and state not in OPERATION_TRANSITIONS.get(operation.state, set()):
                raise ValueError(f'invalid operation transition: {operation.state} -> {state}')
            timestamp = self.clock() if now is None else now
            operation.state = state
            if state in {'accepted', 'running'} and operation.accepted_at is None:
                operation.accepted_at = timestamp
            if state in TERMINAL_STATES:
                operation.finished_at = timestamp
            if result is not None:
                operation.result = dict(result)
            self._changed.notify_all()
            return operation

    def get(self, operation_id: str, now: float | None = None) -> dict:
        with self._changed:
            operation = self._operations[operation_id]
            self._expire(operation, self.clock() if now is None else now)
            return asdict(operation)

    def list_active(self, now: float | None = None) -> list[dict]:
        timestamp = self.clock() if now is None else now
        with self._changed:
            return [
                asdict(operation)
                for operation in self._operations.values()
                if not self._expire(operation, timestamp) and operation.state not in TERMINAL_STATES
            ]

    def find(self, run_id: str, tool_call_id: str) -> dict | None:
        with self._lock:
            operation_id = self._by_run_tool.get((run_id, tool_call_id))
            return self.get(operation_id) if operation_id else None

    def wait(self, operation_id: str, timeout_sec: float, poll_sec: float = 0.1) -> dict:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._changed:
            while True:
                operation = self.get(operation_id)
                if operation['state'] in TERMINAL_STATES or time.monotonic() >= deadline:
                    return operation
                self._changed.wait(min(poll_sec, max(0.0, deadline - time.monotonic())))

    @staticmethod
    def _expire(operation: AgentOperation, now: float) -> bool:
        if operation.state in TERMINAL_STATES or now < operation.created_at + operation.timeout_sec:
            return operation.state == 'timeout'
        operation.state = 'timeout'
        operation.finished_at = now
        operation.result = {
            'ok': False,
            'status': 'timeout',
            'message': '操作等待真实反馈超时',
            'error_code': 'operation_timeout',
        }
        return True
