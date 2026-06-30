import time
from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class AgentState:
    latest_request: Dict[str, Any] = field(default_factory=dict)
    latest_decision: Dict[str, Any] = field(default_factory=dict)
    latest_result: Dict[str, Any] = field(default_factory=dict)
    system_status: Dict[str, Any] = field(default_factory=dict)
    patrol_status: Dict[str, Any] = field(default_factory=dict)
    voice_status: Dict[str, Any] = field(default_factory=dict)
    last_error: str = ''
    waiting_confirm: bool = False

    def snapshot(self) -> Dict[str, Any]:
        return {
            'schema_version': '1.0',
            'state': 'ready',
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
