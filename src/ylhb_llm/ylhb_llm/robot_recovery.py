from __future__ import annotations

from pathlib import Path
import re
from typing import Any, Iterable

import yaml


class RecoveryCatalog:
    ALLOWED_ACTIONS = {'restart_managed_process'}

    def __init__(self, items: dict[str, dict[str, Any]], invalid_items=None) -> None:
        self._items = items
        self.invalid_items = dict(invalid_items or {})

    @classmethod
    def from_file(
        cls,
        path: Path,
        managed_processes: Iterable[str] | None = None,
    ) -> 'RecoveryCatalog':
        try:
            data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            return cls({})
        recoveries = data.get('recoveries') or {}
        if not isinstance(recoveries, dict):
            return cls({})
        managed = set(managed_processes) if managed_processes is not None else None
        valid: dict[str, dict[str, Any]] = {}
        invalid: dict[str, str] = {}
        for component, raw in recoveries.items():
            reason = cls._invalid_reason(raw, managed)
            if reason:
                invalid[str(component)] = reason
            else:
                valid[str(component)] = dict(raw)
        return cls(valid, invalid)

    @classmethod
    def _invalid_reason(cls, raw: Any, managed: set[str] | None) -> str:
        if not isinstance(raw, dict):
            return 'recovery item must be an object'
        if raw.get('action_type') not in cls.ALLOWED_ACTIONS:
            return 'unsupported action_type'
        process = str(raw.get('process') or '')
        if not re.fullmatch(r'[A-Za-z0-9_-]+', process) or (managed is not None and process not in managed):
            return 'unmanaged process'
        if raw.get('verification') not in {None, 'process_running', 'bridge_tcp_ok'}:
            return 'unsupported verification'
        for field, minimum, maximum, default in (
            ('timeout_sec', 1.0, 120.0, 30.0),
            ('cooldown_sec', 0.0, 86400.0, 60.0),
            ('max_attempts_per_incident', 1, 3, 1),
        ):
            value = raw.get(field, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f'invalid {field}'
            if not minimum <= float(value) <= maximum:
                return f'invalid {field}'
        return ''

    def names(self) -> list[str]:
        return sorted(self._items)

    def get(self, component: str) -> dict:
        if component not in self._items:
            raise KeyError(component)
        return dict(self._items[component])
