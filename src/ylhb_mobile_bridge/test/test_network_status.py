import json
import subprocess
from pathlib import Path

from ylhb_mobile_bridge.network_status import NetworkStatusProvider


class FakeCompleted:
    def __init__(self, payload):
        self.stdout = json.dumps(payload)


def make_provider(tmp_path: Path, address_payload, route_payload, route_get=None):
    calls = []

    def runner(args, **kwargs):
        calls.append((tuple(args), kwargs))
        if args[:5] == ['ip', '-j', '-4', 'address', 'show']:
            return FakeCompleted(address_payload)
        if args[:5] == ['ip', '-j', '-4', 'route', 'show']:
            return FakeCompleted(route_payload)
        if args[:5] == ['ip', '-j', '-4', 'route', 'get']:
            return FakeCompleted(route_get or [])
        raise AssertionError(args)

    sys_class_net = tmp_path / 'sys-class-net'
    sys_class_net.mkdir()
    for name in ('wlan0', 'eth0', 'docker0', 'tun0'):
        (sys_class_net / name).mkdir()
    (sys_class_net / 'wlan0' / 'wireless').mkdir()
    provider = NetworkStatusProvider(
        runner=runner,
        sys_class_net=sys_class_net,
        resolver=lambda _hostname: '203.0.113.10',
    )
    return provider, calls


def physical_addresses():
    return [
        {
            'ifname': 'lo',
            'operstate': 'UNKNOWN',
            'addr_info': [{'family': 'inet', 'local': '127.0.0.1', 'prefixlen': 8}],
        },
        {
            'ifname': 'wlan0',
            'operstate': 'UP',
            'addr_info': [{'family': 'inet', 'local': '192.168.137.100', 'prefixlen': 24}],
        },
        {
            'ifname': 'eth0',
            'operstate': 'UP',
            'addr_info': [{'family': 'inet', 'local': '192.168.8.20', 'prefixlen': 24}],
        },
        {
            'ifname': 'docker0',
            'operstate': 'UP',
            'addr_info': [{'family': 'inet', 'local': '172.17.0.1', 'prefixlen': 16}],
        },
        {
            'ifname': 'veth123',
            'operstate': 'UP',
            'addr_info': [{'family': 'inet', 'local': '10.2.0.1', 'prefixlen': 24}],
        },
    ]


def default_routes():
    return [
        {'dst': 'default', 'gateway': '192.168.8.1', 'dev': 'eth0', 'metric': 100},
        {'dst': 'default', 'gateway': '192.168.137.1', 'dev': 'wlan0', 'metric': 600},
    ]


def test_snapshot_filters_virtual_interfaces_and_sorts_default_routes(tmp_path):
    provider, calls = make_provider(tmp_path, physical_addresses(), default_routes())

    snapshot = provider.snapshot()

    assert [item['name'] for item in snapshot['interfaces']] == ['wlan0', 'eth0']
    assert snapshot['interfaces'][0] == {
        'name': 'wlan0',
        'type': 'wifi',
        'label': 'Wi-Fi 网络',
        'address': '192.168.137.100',
        'prefixLength': 24,
        'gateway': '192.168.137.1',
        'defaultRoute': True,
        'metric': 600,
        'up': True,
    }
    assert snapshot['interfaces'][1]['type'] == 'ethernet'
    assert [route['metric'] for route in snapshot['defaultRoutes']] == [100, 600]
    assert snapshot['warnings'] == []
    assert all(call[1]['shell'] is False for call in calls)
    assert all(call[1]['timeout'] == 2 for call in calls)


def test_app_endpoints_reports_both_physical_addresses_and_preserves_explicit_host(tmp_path):
    provider, _calls = make_provider(tmp_path, physical_addresses(), default_routes())

    endpoints = provider.app_endpoints('0.0.0.0', 8000)

    assert [item['interface'] for item in endpoints] == ['wlan0', 'eth0']
    assert endpoints[0]['url'] == 'http://192.168.137.100:8000'
    assert endpoints[1]['url'] == 'http://192.168.8.20:8000'
    assert provider.app_endpoints('192.168.8.20', 9000) == [
        {
            'interface': 'eth0',
            'type': 'ethernet',
            'label': '5G 有线网络',
            'address': '192.168.8.20',
            'port': 9000,
            'url': 'http://192.168.8.20:9000',
            'available': True,
        }
    ]


def test_overlapping_physical_subnets_add_warning(tmp_path):
    addresses = physical_addresses()
    addresses[2]['addr_info'][0]['local'] = '192.168.137.200'
    routes = [
        {'dst': 'default', 'gateway': '192.168.137.1', 'dev': 'eth0', 'metric': 100},
        {'dst': 'default', 'gateway': '192.168.137.1', 'dev': 'wlan0', 'metric': 600},
    ]
    provider, _calls = make_provider(tmp_path, addresses, routes)

    warning = provider.snapshot()['warnings'][0]

    assert warning == {
        'code': 'OVERLAPPING_SUBNET',
        'message': 'Wi-Fi 和有线网络处于相同子网，可能产生路由歧义',
    }


def test_snapshot_is_cached_and_network_command_failures_are_nonfatal(tmp_path):
    calls = []

    def failing_runner(args, **kwargs):
        calls.append(tuple(args))
        raise subprocess.TimeoutExpired(args, timeout=kwargs['timeout'])

    provider = NetworkStatusProvider(
        runner=failing_runner,
        sys_class_net=tmp_path,
        resolver=lambda _hostname: '203.0.113.10',
    )

    first = provider.snapshot()
    second = provider.snapshot()

    assert first == second == {'interfaces': [], 'defaultRoutes': [], 'warnings': []}
    assert len(calls) == 2
    assert provider.route_to_host('cloud.example') == {}


def test_route_to_host_returns_interface_source_gateway_and_alternates(tmp_path):
    route_get = [{
        'dst': '203.0.113.10',
        'gateway': '192.168.8.1',
        'dev': 'eth0',
        'prefsrc': '192.168.8.20',
        'metric': 100,
    }]
    provider, _calls = make_provider(
        tmp_path,
        physical_addresses(),
        default_routes(),
        route_get=route_get,
    )

    route = provider.route_to_host('cloud.example')

    assert route['interface'] == 'eth0'
    assert route['type'] == 'ethernet'
    assert route['label'] == '5G 有线网络'
    assert route['sourceAddress'] == '192.168.8.20'
    assert route['gateway'] == '192.168.8.1'
    assert route['metric'] == 100
    assert route['alternateCloudRoutes'][0]['interface'] == 'wlan0'
    assert route['failoverAvailable'] is True
