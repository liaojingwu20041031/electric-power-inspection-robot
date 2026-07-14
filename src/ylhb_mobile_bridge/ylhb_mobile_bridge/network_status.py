"""Cached Linux network diagnostics for local APP and cloud route display."""

from __future__ import annotations

import ipaddress
import json
import re
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable


_IGNORED_PREFIXES = (
    'docker',
    'veth',
    'virbr',
    'br-',
    'zt',
    'tailscale',
)
_ETHERNET_NAME = re.compile(r'^(?:eth|en|end|enx)', re.IGNORECASE)
_TYPE_LABELS = {
    'wifi': 'Wi-Fi 网络',
    'ethernet': '5G 有线网络',
    'other': '其他网络',
}


class NetworkStatusProvider:
    """Read interface and route state without making network changes."""

    def __init__(
        self,
        cache_seconds: float = 3.0,
        runner: Callable[..., Any] = subprocess.run,
        sys_class_net: Path | str = Path('/sys/class/net'),
        clock: Callable[[], float] = time.monotonic,
        resolver: Callable[[str], str] = socket.gethostbyname,
    ) -> None:
        self.cache_seconds = max(0.0, float(cache_seconds))
        self._runner = runner
        self._sys_class_net = Path(sys_class_net)
        self._clock = clock
        self._resolver = resolver
        self._lock = threading.Lock()
        self._snapshot_at = 0.0
        self._snapshot: dict[str, list[dict[str, Any]]] | None = None
        self._route_cache: dict[str, tuple[float, dict[str, Any]]] = {}

    def _run_json(self, args: list[str]) -> list[dict[str, Any]]:
        try:
            completed = self._runner(
                args,
                capture_output=True,
                text=True,
                check=True,
                shell=False,
                timeout=2,
            )
            payload = json.loads(completed.stdout or '[]')
            return payload if isinstance(payload, list) else []
        except (
            FileNotFoundError,
            subprocess.CalledProcessError,
            subprocess.TimeoutExpired,
            json.JSONDecodeError,
            OSError,
            TypeError,
            ValueError,
        ):
            return []

    def _interface_type(self, name: str) -> str:
        if (self._sys_class_net / name / 'wireless').exists():
            return 'wifi'
        if _ETHERNET_NAME.match(name):
            return 'ethernet'
        return 'other'

    @staticmethod
    def _ignored_interface(name: str) -> bool:
        return name == 'lo' or name.startswith(_IGNORED_PREFIXES)

    @staticmethod
    def _metric(value: Any) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0

    def _build_snapshot(self) -> dict[str, list[dict[str, Any]]]:
        address_rows = self._run_json(
            ['ip', '-j', '-4', 'address', 'show', 'up']
        )
        route_rows = self._run_json(
            ['ip', '-j', '-4', 'route', 'show']
        )
        default_routes = sorted(
            (
                {
                    'interface': str(row.get('dev') or ''),
                    'gateway': str(row.get('gateway') or ''),
                    'metric': self._metric(row.get('metric')),
                }
                for row in route_rows
                if row.get('dst') == 'default' and row.get('dev')
            ),
            key=lambda item: (item['metric'], item['interface']),
        )
        route_by_interface = {
            route['interface']: route for route in default_routes
        }
        interfaces: list[dict[str, Any]] = []
        for row in address_rows:
            name = str(row.get('ifname') or '')
            if not name or self._ignored_interface(name):
                continue
            address_info = next(
                (
                    item
                    for item in row.get('addr_info', [])
                    if item.get('family') == 'inet' and item.get('local')
                ),
                None,
            )
            if not address_info:
                continue
            address = str(address_info['local'])
            try:
                parsed_address = ipaddress.ip_address(address)
            except ValueError:
                continue
            if parsed_address.is_link_local:
                continue
            interface_type = self._interface_type(name)
            route = route_by_interface.get(name, {})
            interfaces.append(
                {
                    'name': name,
                    'type': interface_type,
                    'label': _TYPE_LABELS[interface_type],
                    'address': address,
                    'prefixLength': int(address_info.get('prefixlen') or 0),
                    'gateway': str(route.get('gateway') or ''),
                    'defaultRoute': bool(route),
                    'metric': self._metric(route.get('metric')) if route else None,
                    'up': str(row.get('operstate') or '').upper() in {'UP', 'UNKNOWN'},
                }
            )
        type_rank = {'wifi': 0, 'ethernet': 1, 'other': 2}
        interfaces.sort(
            key=lambda item: (
                not item['up'],
                type_rank[item['type']],
                item['name'],
            )
        )
        warnings: list[dict[str, str]] = []
        physical = [
            item for item in interfaces if item['type'] in {'wifi', 'ethernet'}
        ]
        networks = []
        for item in physical:
            try:
                network = ipaddress.ip_network(
                    f"{item['address']}/{item['prefixLength']}",
                    strict=False,
                )
            except ValueError:
                continue
            networks.append((item, network))
        if any(
            first['type'] != second['type'] and first_network.overlaps(second_network)
            for index, (first, first_network) in enumerate(networks)
            for second, second_network in networks[index + 1:]
        ):
            warnings.append(
                {
                    'code': 'OVERLAPPING_SUBNET',
                    'message': 'Wi-Fi 和有线网络处于相同子网，可能产生路由歧义',
                }
            )
        return {
            'interfaces': interfaces,
            'defaultRoutes': default_routes,
            'warnings': warnings,
        }

    def snapshot(self) -> dict[str, list[dict[str, Any]]]:
        now = self._clock()
        with self._lock:
            if (
                self._snapshot is not None
                and now - self._snapshot_at < self.cache_seconds
            ):
                return self._snapshot
            snapshot = self._build_snapshot()
            self._snapshot = snapshot
            self._snapshot_at = now
            return snapshot

    def app_endpoints(self, host: str, port: int) -> list[dict[str, Any]]:
        snapshot = self.snapshot()
        interfaces = snapshot['interfaces']
        configured_host = str(host or '').strip()
        selected = (
            [
                item
                for item in interfaces
                if item['type'] in {'wifi', 'ethernet'}
            ]
            if configured_host in {'', '0.0.0.0'}
            else [item for item in interfaces if item['address'] == configured_host]
        )
        if not selected and configured_host not in {'', '0.0.0.0'}:
            selected = [
                {
                    'name': '',
                    'type': 'other',
                    'label': '配置地址',
                    'address': configured_host,
                    'up': False,
                }
            ]
        return [
            {
                'interface': item['name'],
                'type': item['type'],
                'label': item['label'],
                'address': item['address'],
                'port': int(port),
                'url': f"http://{item['address']}:{int(port)}",
                'available': bool(item['up']),
            }
            for item in selected
        ]

    def route_to_host(self, hostname: str) -> dict[str, Any]:
        hostname = str(hostname or '').strip()
        if not hostname:
            return {}
        now = self._clock()
        with self._lock:
            cached = self._route_cache.get(hostname)
            if cached and now - cached[0] < self.cache_seconds:
                return cached[1]
        try:
            target = self._resolver(hostname)
        except (OSError, socket.gaierror, TypeError, ValueError):
            return {}
        rows = self._run_json(
            ['ip', '-j', '-4', 'route', 'get', target]
        )
        if not rows:
            return {}
        row = rows[0]
        interface_name = str(row.get('dev') or '')
        snapshot = self.snapshot()
        interface = next(
            (
                item
                for item in snapshot['interfaces']
                if item['name'] == interface_name
            ),
            {},
        )
        default_route = next(
            (
                route
                for route in snapshot['defaultRoutes']
                if route['interface'] == interface_name
            ),
            {},
        )
        alternates = []
        for route in snapshot['defaultRoutes']:
            if route['interface'] == interface_name:
                continue
            alternate_interface = next(
                (
                    item
                    for item in snapshot['interfaces']
                    if item['name'] == route['interface']
                ),
                {},
            )
            if not alternate_interface:
                continue
            alternates.append(
                {
                    'interface': route['interface'],
                    'type': alternate_interface.get('type', 'other'),
                    'label': alternate_interface.get('label', '其他网络'),
                    'sourceAddress': alternate_interface.get('address', ''),
                    'gateway': route.get('gateway', ''),
                    'metric': route.get('metric', 0),
                }
            )
        result = {
            'interface': interface_name,
            'type': interface.get('type', 'other'),
            'label': interface.get('label', '其他网络'),
            'sourceAddress': str(
                row.get('prefsrc') or interface.get('address') or ''
            ),
            'gateway': str(
                row.get('gateway') or default_route.get('gateway') or ''
            ),
            'metric': self._metric(
                row.get('metric', default_route.get('metric', 0))
            ),
            'alternateCloudRoutes': alternates,
            'failoverAvailable': bool(alternates),
        }
        with self._lock:
            self._route_cache[hostname] = (now, result)
        return result
