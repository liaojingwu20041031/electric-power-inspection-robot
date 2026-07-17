"""Read-only setup check for the inspection Agent planner."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_path
except ImportError:
    get_package_share_path = None

from ylhb_mobile_bridge.patrol_route_store import (
    default_workspace_dir,
    load_route_file,
    resolve_route_file_path,
)


def agent_parameters(config_path: Path) -> dict[str, Any]:
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    params = config.get('inspection_agent_node', {}).get('ros__parameters', {})
    if not isinstance(params, dict):
        raise ValueError('inspection_agent_node.ros__parameters must be an object')
    return params


def package_share_path() -> Path:
    if get_package_share_path is not None:
        return get_package_share_path('ylhb_llm')
    return Path(__file__).resolve().parents[1]


def capability_path(configured: str) -> Path:
    directory = package_share_path() / 'config'
    path = Path(configured).expanduser() if configured else directory / 'robot_capabilities.yaml'
    return path if path.is_absolute() else directory / path


def endpoint_status(
    base_url: str, models_path: str, api_key: str, timeout_sec: float,
) -> tuple[bool, bool | None, str, bool]:
    if not base_url:
        return False, None, 'planner endpoint is empty', True
    headers = {'Authorization': f'Bearer {api_key}'} if api_key else {}
    request = urllib.request.Request(base_url.rstrip('/') + '/' + models_path.lstrip('/'), headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec):
            return True, True, '', False
    except urllib.error.HTTPError as exc:
        if exc.code in (401, 403):
            return True, False, f'planner authentication failed: HTTP {exc.code}', True
        if exc.code == 404:
            return False, None, 'planner models path is invalid: HTTP 404', True
        return False, None, f'planner endpoint failed: HTTP {exc.code}', False
    except (OSError, urllib.error.URLError) as exc:
        return False, None, f'planner endpoint is unreachable: {exc}', False


def ros_graph_status(required_topics: list[str], system_status_topic: str) -> tuple[list[str], list[str]]:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        return required_topics, ['rclpy is unavailable']
    node = None
    try:
        rclpy.init(args=None)
        node = Node('check_agent_setup')
        names = {name for name, _types in node.get_topic_names_and_types()}
        missing = [topic for topic in required_topics if topic not in names]
        warnings = []
        if node.count_publishers(system_status_topic) < 1:
            warnings.append(f'system supervisor has no {system_status_topic} publisher')
        return missing, warnings
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    share = package_share_path()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=share / 'config' / 'llm.yaml')
    parser.add_argument('--route-file', default='')
    parser.add_argument('--route-directory', default='')
    parser.add_argument('--capabilities-file', default='')
    parser.add_argument('--endpoint', default='')
    parser.add_argument('--model', default='')
    parser.add_argument('--endpoint-timeout-sec', type=float, default=3.0)
    parser.add_argument('--skip-endpoint', action='store_true')
    parser.add_argument('--skip-ros', action='store_true')
    parser.add_argument(
        '--network-optional', action='store_true',
        help='endpoint 暂时不可达时仅警告；认证失败仍返回错误',
    )
    args = parser.parse_args(argv)

    warnings: list[str] = []
    try:
        params = agent_parameters(args.config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        params = {}
        warnings.append(f'cannot load agent configuration: {exc}')

    workspace = str(params.get('workspace_dir') or '').strip()
    workspace_path = Path(workspace).expanduser() if workspace else default_workspace_dir()
    route_directory = args.route_directory or str(params.get('route_directory') or '') or str(workspace_path / 'maps')
    route_requested = args.route_file or str(params.get('patrol_route_path') or params.get('route_file_path') or 'auto')
    try:
        route_path = resolve_route_file_path(route_requested, route_directory)
        load_route_file(str(route_path))
        route_catalog_available = True
    except (OSError, ValueError) as exc:
        route_path = None
        route_catalog_available = False
        warnings.append(f'route catalog unavailable: {exc}')

    try:
        caps_path = Path(args.capabilities_file).expanduser() if args.capabilities_file else capability_path(str(params.get('robot_capabilities_file') or ''))
        with caps_path.open(encoding='utf-8') as stream:
            capabilities = (yaml.safe_load(stream) or {}).get('capabilities') or {}
        capability_catalog_available = isinstance(capabilities, dict) and bool(capabilities)
        if not capability_catalog_available:
            warnings.append('capability catalog is empty')
    except (OSError, ValueError, yaml.YAMLError) as exc:
        caps_path = None
        capability_catalog_available = False
        warnings.append(f'capability catalog unavailable: {exc}')

    provider = str(params.get('planner_provider_name') or 'planner')
    model = args.model or str(params.get('planner_model') or '')
    api_key_env = str(params.get('planner_api_key_env') or '')
    api_key_required = bool(params.get('planner_api_key_required', True))
    api_key = os.environ.get(api_key_env, '') if api_key_env else ''
    planner_available = bool(model.strip() and (not api_key_required or api_key))
    if not model.strip():
        warnings.append('planner model is empty')
    if api_key_required and not api_key:
        warnings.append(f'{api_key_env or "planner API key"} is missing')

    endpoint = args.endpoint or str(params.get('planner_base_url') or '')
    if args.skip_endpoint:
        endpoint_reachable, authentication_ok, endpoint_permanent_error = True, None, False
    else:
        endpoint_reachable, authentication_ok, endpoint_warning, endpoint_permanent_error = endpoint_status(
            endpoint, str(params.get('planner_models_path') or '/models'), api_key, args.endpoint_timeout_sec,
        )
        if endpoint_warning:
            warnings.append(endpoint_warning)
    if authentication_ok is False:
        planner_available = False

    required_topics = [
        str(params.get(name) or '')
        for name in ('system_status_topic', 'patrol_status_topic', 'base_skill_status_topic', 'agent_request_topic')
    ]
    required_topics = [topic for topic in required_topics if topic]
    if args.skip_ros:
        missing_topics: list[str] = []
    else:
        missing_topics, ros_warnings = ros_graph_status(required_topics, str(params.get('system_status_topic') or ''))
        warnings.extend(ros_warnings)
    if missing_topics:
        warnings.append('missing ROS topics: ' + ', '.join(missing_topics))

    ready = all((
        planner_available,
        route_catalog_available,
        capability_catalog_available,
        endpoint_reachable or (args.network_optional and not endpoint_permanent_error),
        authentication_ok is not False,
        not missing_topics,
    ))
    print(json.dumps({
        'status': 'ok' if ready and not warnings else 'warning',
        'provider': provider,
        'model': model,
        'route_file': str(route_path) if route_path else '',
        'capabilities_file': str(caps_path) if caps_path else '',
        'planner_available': planner_available,
        'endpoint_reachable': endpoint_reachable,
        'authentication_ok': authentication_ok,
        'endpoint_permanent_error': endpoint_permanent_error,
        'route_catalog_available': route_catalog_available,
        'capability_catalog_available': capability_catalog_available,
        'missing_topics': missing_topics,
        'warnings': warnings,
    }, ensure_ascii=False))
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
