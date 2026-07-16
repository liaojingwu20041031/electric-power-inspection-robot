import time
from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class AgentState:
    state: str = 'idle'
    current_goal: str = ''
    current_step: str = ''
    steps: List[Dict[str, Any]] = field(default_factory=list)
    final_result: str = ''
    pending_operation_id: str = ''
    latest_request: Dict[str, Any] = field(default_factory=dict)
    latest_decision: Dict[str, Any] = field(default_factory=dict)
    latest_result: Dict[str, Any] = field(default_factory=dict)
    system_status: Dict[str, Any] = field(default_factory=dict)
    patrol_status: Dict[str, Any] = field(default_factory=dict)
    voice_status: Dict[str, Any] = field(default_factory=dict)
    last_error: str = ''
    waiting_confirm: bool = False

    def start_goal(self, goal: str) -> None:
        self.state = 'planning'
        self.current_goal = goal
        self.current_step = '正在规划'
        self.steps = []
        self.final_result = ''
        self.pending_operation_id = ''

    def add_step(self, summary: str, detail: Dict[str, Any] | None = None) -> None:
        self.current_step = summary
        self.steps.append({
            'summary': summary,
            'detail': detail or {},
            'timestamp': time.time(),
        })
        del self.steps[:-30]

    def wait_feedback(self, operation_id: str, summary: str) -> None:
        self.state = 'waiting_feedback'
        self.pending_operation_id = operation_id
        self.add_step(summary)

    def finish(self, text: str) -> None:
        self.state = 'finished'
        self.current_step = '已结束'
        self.final_result = text
        self.pending_operation_id = ''

    def snapshot(self) -> Dict[str, Any]:
        return {
            'schema_version': '1.0',
            'state': self.state,
            'current_goal': self.current_goal,
            'current_step': self.current_step,
            'steps': list(self.steps[-30:]),
            'final_result': self.final_result,
            'pending_operation_id': self.pending_operation_id,
            'last_text': str(self.latest_request.get('text') or ''),
            'last_intent': str(self.latest_decision.get('intent') or ''),
            'last_tool': str((self.latest_decision.get('tool_call') or {}).get('name') or ''),
            'last_result_status': str(self.latest_result.get('status') or ''),
            'last_error': self.last_error,
            'waiting_confirm': self.waiting_confirm,
            'patrol_state': self.patrol_state(),
            'timestamp': time.time(),
        }

    def policy_context(self) -> Dict[str, Any]:
        return {
            'patrol_state': self.patrol_state(),
            'system_status': self.system_status,
            'patrol_status': self.patrol_status,
            'voice_status': self.voice_status,
        }

    def patrol_state(self) -> str:
        return str(
            self.patrol_status.get('state')
            or self.system_status.get('patrol_mode_state')
            or 'unknown'
        )
