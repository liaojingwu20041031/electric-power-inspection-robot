#!/usr/bin/env python3
"""Read-only preflight for the inspection Agent."""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
for package_root in (ROOT / 'src' / 'ylhb_llm', ROOT / 'src' / 'ylhb_mobile_bridge'):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from ylhb_mobile_bridge.patrol_route_store import load_route_file, resolve_route_file_path


REQUIRED_TOPICS = (
    '/inspection_ai/system_status',
    '/patrol/status',
    '/inspection_ai/base_skill_status',
)


def agent_parameters(config_path: Path) -> dict[str, Any]:
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    params = config.get('inspection_agent_node', {}).get('ros__parameters', {})
    if not isinstance(params, dict):
        raise ValueError('inspection_agent_node.ros__parameters must be an object')
    return params


def endpoint_available(endpoint: str, api_key: str, timeout_sec: float) -> bool:
    request = urllib.request.Request(endpoint.rstrip('/') + '/models')
    if api_key:
        request.add_header('Authorization', f'Bearer {api_key}')
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec):
            return True
    except urllib.error.HTTPError:
        return True  # A HTTP response proves the configured endpoint is reachable.
    except (OSError, urllib.error.URLError):
        return False


def ros_graph_status() -> tuple[bool, list[str]]:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        return False, ['rclpy is unavailable']

    warnings: list[str] = []
    rclpy.init(args=None)
    node = Node('check_agent_setup')
    try:
        names = {name for name, _types in node.get_topic_names_and_types()}
        missing = [topic for topic in REQUIRED_TOPICS if topic not in names]
        if missing:
            warnings.append('missing ROS topics: ' + ', '.join(missing))
        if node.count_publishers('/inspection_ai/system_status') < 1:
            warnings.append('system supervisor has no /inspection_ai/system_status publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return not warnings, warnings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'src' / 'ylhb_llm' / 'config' / 'llm.yaml')
    parser.add_argument('--route-file', default='')
    parser.add_argument('--route-directory', type=Path, default=ROOT / 'maps')
    parser.add_argument('--capabilities-file', type=Path, default=ROOT / 'src' / 'ylhb_llm' / 'config' / 'robot_capabilities.yaml')
    parser.add_argument('--endpoint', default='')
    parser.add_argument('--model', default='')
    parser.add_argument('--endpoint-timeout-sec', type=float, default=3.0)
    parser.add_argument('--skip-endpoint', action='store_true')
    parser.add_argument('--skip-ros', action='store_true')
    args = parser.parse_args()

    warnings: list[str] = []
    try:
        params = agent_parameters(args.config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        params = {}
        warnings.append(f'cannot load agent configuration: {exc}')

    route_requested = args.route_file or str(params.get('route_file_path') or 'auto')
    try:
        route_path = resolve_route_file_path(route_requested, args.route_directory)
        load_route_file(str(route_path))
        route_catalog_available = True
    except (OSError, ValueError) as exc:
        route_path = None
        route_catalog_available = False
        warnings.append(f'route catalog unavailable: {exc}')

    try:
        with args.capabilities_file.open(encoding='utf-8') as stream:
            capabilities = (yaml.safe_load(stream) or {}).get('capabilities') or {}
        capability_catalog_available = isinstance(capabilities, dict) and bool(capabilities)
        if not capability_catalog_available:
            warnings.append('capability catalog is empty')
    except (OSError, yaml.YAMLError) as exc:
        capability_catalog_available = False
        warnings.append(f'capability catalog unavailable: {exc}')

    model = args.model or str(params.get('chat_model') or '')
    if not model.strip():
        warnings.append('chat model is empty')
    endpoint = args.endpoint or str(params.get('dashscope_base_url') or '')
    api_key = os.getenv('DASHSCOPE_API_KEY', '')
    planner_available = bool(api_key and model.strip())
    if not api_key:
        warnings.append('DASHSCOPE_API_KEY is missing')

    if args.skip_endpoint:
        endpoint_ok = True
    elif not endpoint:
        endpoint_ok = False
        warnings.append('DashScope endpoint is empty')
    else:
        endpoint_ok = endpoint_available(endpoint, api_key, args.endpoint_timeout_sec)
        if not endpoint_ok:
            warnings.append('DashScope endpoint is unreachable')

    if args.skip_ros:
        ros_ok = True
    else:
        ros_ok, ros_warnings = ros_graph_status()
        warnings.extend(ros_warnings)

    ready = all((planner_available, route_catalog_available, capability_catalog_available, endpoint_ok, ros_ok))
    payload = {
        'status': 'ok' if ready else 'warning',
        'planner_available': planner_available,
        'route_catalog_available': route_catalog_available,
        'capability_catalog_available': capability_catalog_available,
        'route_file_path': str(route_path) if route_path else '',
        'endpoint_available': endpoint_ok,
        'ros_graph_available': ros_ok,
        'warnings': warnings,
    }
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
