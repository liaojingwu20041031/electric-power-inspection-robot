import json
from types import SimpleNamespace

from ylhb_mobile_bridge.platform_cloud_client import PlatformCloudClient


class FakeStore:
    def __init__(self):
        self.values = {}

    def cloud_state(self, key, default=''):
        return self.values.get(key, default)

    def set_cloud_state(self, key, value):
        self.values[key] = str(value)

    def pending_event_count(self):
        return 0

    def pending_command_count(self):
        return 0

    def latest_event_sequence(self):
        return 7


class FakeNetworkStatusProvider:
    def __init__(self):
        self.calls = []

    def route_to_host(self, hostname):
        self.calls.append(hostname)
        return {
            'interface': 'eth0',
            'type': 'ethernet',
            'label': '5G 有线网络',
            'sourceAddress': '192.168.8.20',
            'gateway': '192.168.8.1',
            'metric': 100,
            'alternateCloudRoutes': [
                {
                    'interface': 'wlan0',
                    'type': 'wifi',
                    'label': 'Wi-Fi 网络',
                    'sourceAddress': '192.168.137.100',
                    'gateway': '192.168.137.1',
                    'metric': 600,
                }
            ],
            'failoverAvailable': True,
        }


class FakeResponse:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return json.dumps({'ok': True}).encode('utf-8')


def make_client(monkeypatch):
    monkeypatch.setenv('YLHB_CLOUD_BASE_URL', 'https://cloud.example')
    monkeypatch.setenv('YLHB_CLOUD_ROBOT_TOKEN', 'secret')
    bridge = SimpleNamespace(
        network_status=FakeNetworkStatusProvider(),
        cloud_status_snapshot=lambda: {
            'state': 'idle',
            'platformContext': {},
            'mapPose': None,
            'odomPose': None,
            'health': {'ok': True},
        },
    )
    return PlatformCloudClient(FakeStore(), bridge, 'robot-1', 'boot-1')


def test_status_adds_system_routing_diagnostics(monkeypatch):
    client = make_client(monkeypatch)

    status = client.status()

    assert status['networkMode'] == 'system-routing'
    assert status['cloudEgress'] == {
        'interface': 'eth0',
        'type': 'ethernet',
        'label': '5G 有线网络',
        'sourceAddress': '192.168.8.20',
        'gateway': '192.168.8.1',
        'metric': 100,
    }
    assert status['alternateCloudRoutes'][0]['interface'] == 'wlan0'
    assert status['failoverAvailable'] is True
    assert status['lastSuccessfulEgress'] == {}


def test_heartbeat_payload_does_not_include_local_network_diagnostics(monkeypatch):
    client = make_client(monkeypatch)

    payload = client._heartbeat_payload()

    assert payload == {
        'protocolVersion': '1.0',
        'robotId': 'robot-1',
        'bootId': 'boot-1',
        'softwareVersion': 'unknown',
        'state': 'idle',
        'activeExecutionId': None,
        'activeDeploymentId': None,
        'lastReceivedCommandId': None,
        'latestLocalEventSequence': 7,
        'mapPose': None,
        'odomPose': None,
        'health': {'ok': True},
    }
    assert 'cloudEgress' not in payload
    assert 'networkMode' not in payload


def test_successful_cloud_request_records_egress_without_changing_request(monkeypatch):
    client = make_client(monkeypatch)
    monkeypatch.setattr(
        'ylhb_mobile_bridge.platform_cloud_client.urllib.request.urlopen',
        lambda *_args, **_kwargs: FakeResponse(),
    )

    assert client._request('GET', '/health') == {'ok': True}

    status = client.status()
    assert status['lastSuccessfulEgress']['interface'] == 'eth0'
    assert client.bridge.network_status.calls == [
        'cloud.example',
        'cloud.example',
    ]
