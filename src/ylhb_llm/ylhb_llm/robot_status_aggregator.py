from __future__ import annotations

import time
from typing import Any, Callable, Dict


class RobotStatusAggregator:
    """Stores timestamped read-only ROS observations without side effects."""

    def __init__(self, default_max_age_sec: float = 2.5, clock: Callable[[], float] = time.time) -> None:
        self.default_max_age_sec = float(default_max_age_sec)
        self.clock = clock
        self._observations: Dict[str, Dict[str, Any]] = {}
        self.expected_components: Dict[str, tuple[str, ...]] = {}

    def configure_expected_components(self, expected: Dict[str, Any]) -> None:
        self.expected_components = {
            str(mode).lower(): tuple(str(name) for name in names)
            for mode, names in (expected or {}).items()
            if isinstance(names, (list, tuple))
        }

    def update(
        self,
        source: str,
        payload: Dict[str, Any],
        now: float | None = None,
        max_age_sec: float | None = None,
    ) -> None:
        timestamp = self.clock() if now is None else now
        self._observations[source] = {
            'payload': dict(payload),
            'updated_at': timestamp,
            'max_age_sec': self.default_max_age_sec if max_age_sec is None else float(max_age_sec),
        }

    def get(self, source: str, now: float | None = None) -> Dict[str, Any]:
        timestamp = self.clock() if now is None else now
        observation = self._observations.get(source)
        if observation is None:
            return {'state': 'unknown', 'updated_at': 0.0, 'age_sec': None, 'fresh': False}
        age_sec = max(0.0, timestamp - float(observation['updated_at']))
        fresh = age_sec <= float(observation['max_age_sec'])
        payload = dict(observation['payload'])
        return {
            **payload,
            'state': str(payload.get('state') or ('ok' if fresh else 'stale')) if fresh else 'stale',
            'updated_at': observation['updated_at'],
            'age_sec': age_sec,
            'fresh': fresh,
        }

    def raw(self, source: str) -> Dict[str, Any]:
        observation = self._observations.get(source)
        return dict(observation['payload']) if observation else {}

    def snapshot(self, now: float | None = None) -> Dict[str, Dict[str, Any]]:
        return {source: self.get(source, now) for source in self._observations}

    def mode_aware_summary(self, now: float | None = None) -> Dict[str, Any]:
        summary = self.summary(now)
        system = self.get('system_status', now)
        mode = str(
            system.get('system_mode') or system.get('mode')
            or system.get('patrol_mode_state') or 'unknown'
        ).lower()
        expected = self.expected_components.get(mode, ())
        unhealthy = not system.get('fresh') or any(
            system.get(name) not in {'running', 'embedded'} for name in expected
        )
        summary['health'] = 'warning' if unhealthy else 'ok'
        return summary

    def summary(self, now: float | None = None) -> Dict[str, Any]:
        system = self.get('system_status', now)
        patrol = self.get('patrol_status', now)
        pose = self.get('amcl_pose', now)
        base = self.get('base_skill_status', now)
        chassis = self.get('chassis_status', now)
        chassis['state'] = str(chassis.get('state') or '').split(maxsplit=1)[0] or 'unknown'
        return {
            'robot_mode': system.get('mode') or system.get('system_mode') or 'unknown',
            'health': 'warning' if not system.get('fresh') else 'ok',
            'pose': {key: pose.get(key) for key in ('x', 'y', 'yaw', 'fresh')},
            'navigation': {'state': system.get('navigation_state', 'unknown'), 'profile': system.get('navigation_profile', 'unknown')},
            'patrol': {'state': patrol.get('state', 'unknown'), 'target': patrol.get('target_id', ''), 'progress': patrol.get('progress', '')},
            'sensors': {'lidar': self.get('scan', now).get('state'), 'odom': self.get('odom', now).get('state')},
            'base': base,
            'chassis': chassis,
            'components': {
                name: system.get(name, 'unknown') if system.get('fresh') else 'unknown'
                for name in ('bringup', 'navigation', 'perception', 'patrol_executor')
            },
        }
