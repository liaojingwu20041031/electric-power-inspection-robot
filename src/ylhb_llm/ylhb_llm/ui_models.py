import time
from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class UiState:
    system_status: Dict[str, Any] = field(default_factory=dict)
    task_context: Dict[str, Any] = field(default_factory=dict)
    localized_objects: str = ''
    voice_status: str = '-'
    agent_status: Dict[str, Any] = field(default_factory=dict)
    agent_events: List[Dict[str, Any]] = field(default_factory=list)
    robot_mode: str = 'ready'
    patrol_status: Dict[str, Any] = field(default_factory=dict)
    patrol_events: List[Dict[str, Any]] = field(default_factory=list)
    route_preview: Dict[str, Any] = field(default_factory=dict)
    patrol_tasks: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    events: List[Dict[str, str]] = field(default_factory=list)
    max_events: int = 200

    def add_event(self, message: str, timestamp: str = '') -> None:
        self.events.append({
            'timestamp': timestamp or time.strftime('%H:%M:%S'),
            'message': str(message),
        })
        if len(self.events) > self.max_events:
            del self.events[:-self.max_events]
