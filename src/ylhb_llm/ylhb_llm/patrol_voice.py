from collections import deque
from dataclasses import dataclass
import json
import os
from string import Formatter
from typing import Any, Dict, Optional

import yaml


ALLOWED_FIELDS = {'route_id', 'target_id', 'target_name', 'execution_id'}
DEFAULT_FIELDS = {
    'route_id': '当前路线',
    'target_id': '当前检查点',
    'target_name': '检查点',
    'execution_id': '当前任务',
}


@dataclass(frozen=True)
class VoiceRequest:
    task_id: str
    text: str
    priority: int
    interrupt: bool
    audio_path: str = ''
    inventory_path: str = ''
    max_delay_sec: float = 0.0


class PatrolVoice:
    def __init__(self, config: Dict[str, Any], audio_dir: str, inventory_dir: str = '') -> None:
        self.enabled = bool(config.get('enabled', True))
        announcements = config.get('announcements', {})
        if not isinstance(announcements, dict):
            raise ValueError('announcements must be a mapping')
        self.announcements = announcements
        self.audio_dir = audio_dir
        self.inventory_dir = inventory_dir
        self._seen_order = deque(maxlen=128)
        self._seen = set()
        self._validate_rules()

    @classmethod
    def from_file(
        cls, config_file: str, audio_dir: str, inventory_dir: str = ''
    ) -> 'PatrolVoice':
        with open(config_file, encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        if not isinstance(config, dict):
            raise ValueError('patrol voice config must be a mapping')
        if int(config.get('version', 0)) != 1:
            raise ValueError('unsupported patrol voice config version')
        return cls(config, audio_dir, inventory_dir)

    def request_for_json(self, raw: str) -> Optional[VoiceRequest]:
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, TypeError, ValueError):
            return None
        if not isinstance(payload, dict):
            return None
        return self.request_for_event(payload)

    def request_for_event(self, payload: Dict[str, Any]) -> Optional[VoiceRequest]:
        if not self.enabled:
            return None
        event = str(payload.get('event') or '')
        if event == 'target_task_finished' and str(
            payload.get('result_status') or ''
        ) != 'succeeded':
            return None
        rule_name = (
            'return_home_after_failure'
            if event == 'return_home_started' and payload.get('after_failure')
            else event
        )
        rule = self.announcements.get(rule_name)
        if not isinstance(rule, dict) or not rule.get('enabled', False):
            return None

        dedupe_key = tuple(str(payload.get(name) or '') for name in (
            'boot_id', 'execution_id', 'event', 'target_id', 'timestamp'))
        if dedupe_key in self._seen:
            return None

        fields = {
            name: str(payload.get(name) or default)
            for name, default in DEFAULT_FIELDS.items()
        }
        text = str(rule.get('text') or '').format_map(fields).strip()
        audio_file = str(rule.get('audio_file') or '').strip()
        audio_path = os.path.join(self.audio_dir, audio_file) if audio_file else ''
        inventory_path = (
            os.path.join(self.inventory_dir, audio_file)
            if audio_file and self.inventory_dir else ''
        )
        task_id = 'patrol:' + ':'.join((
            fields['execution_id'], event, fields['target_id'], str(payload.get('timestamp') or '')))
        self._remember(dedupe_key)
        return VoiceRequest(
            task_id=task_id,
            text=text,
            priority=int(rule.get('priority', 0)),
            interrupt=bool(rule.get('interrupt', False)),
            audio_path=audio_path,
            inventory_path=inventory_path,
            max_delay_sec=float(rule.get('max_delay_sec', 0.0)),
        )

    def _validate_rules(self) -> None:
        for name, rule in self.announcements.items():
            if not isinstance(rule, dict):
                raise ValueError(f'announcement {name} must be a mapping')
            for _literal, field, format_spec, conversion in Formatter().parse(
                str(rule.get('text') or '')
            ):
                if field and (field not in ALLOWED_FIELDS or format_spec or conversion):
                    raise ValueError(f'announcement {name} has unsupported template field: {field}')
            audio_file = str(rule.get('audio_file') or '')
            if audio_file and os.path.basename(audio_file) != audio_file:
                raise ValueError(f'announcement {name} audio_file must be a file name')

    def _remember(self, key: tuple[str, ...]) -> None:
        if len(self._seen_order) == self._seen_order.maxlen:
            self._seen.discard(self._seen_order[0])
        self._seen_order.append(key)
        self._seen.add(key)
