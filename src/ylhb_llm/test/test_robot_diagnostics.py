from types import SimpleNamespace

from ylhb_llm.robot_diagnostics import RobotDiagnosticEngine
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator


class FakeNetwork:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def snapshot(self):
        return self._snapshot

    def app_endpoints(self, host, port):
        return [
            {
                'interface': item['name'], 'label': item['label'],
                'address': item['address'], 'url': f"http://{item['address']}:{port}",
                'available': item['up'],
            }
            for item in self._snapshot['interfaces']
        ]


def make_engine(now=100.0):
    aggregator = RobotStatusAggregator(clock=lambda: now)
    network = FakeNetwork({'interfaces': [], 'defaultRoutes': [], 'warnings': []})
    config = {
        'freshness': {'system_status_sec': 3, 'odom_sec': 2.5, 'scan_sec': 2.5},
        'expected_components': {
            'ready': [], 'navigation': ['bringup', 'navigation'],
            'patrol': ['bringup', 'navigation', 'patrol_executor'],
        },
        'mobile_bridge_port': 8000,
    }
    return RobotDiagnosticEngine(aggregator, network, config, clock=lambda: now), aggregator


def test_ready_mode_does_not_require_amcl_or_patrol():
    engine, status = make_engine()
    status.update('system_status', {'system_mode': 'ready', 'bringup': 'stopped'}, now=100.0)

    report = engine.run_self_check('navigation')

    assert report['overall'] == 'ok'
    assert not any(issue['component'] == 'navigation' for issue in report['issues'])


def test_patrol_mode_requires_active_navigation():
    engine, status = make_engine()
    status.update('system_status', {
        'system_mode': 'patrol', 'patrol_mode_state': 'running',
        'bringup': 'running', 'navigation': 'running', 'patrol_executor': 'running',
        'patrol_readiness': {'nav2_active': False},
    }, now=100.0)

    report = engine.run_self_check('navigation')

    assert report['overall'] == 'failed'
    assert any(issue['code'] == 'NAV2_NOT_ACTIVE' for issue in report['issues'])


def test_missing_chassis_heartbeat_reports_possible_not_confirmed_cause():
    engine, status = make_engine()
    status.update('chassis_status', {
        'state': 'online', 'heartbeat_seen': False, 'fault_latched': False,
    }, now=100.0)
    status.update('odom', {'state': 'ok'}, now=90.0)

    report = engine.run_self_check('base')
    issue = next(item for item in report['issues'] if item['code'] == 'CHASSIS_HEARTBEAT_MISSING')

    assert issue['confirmed_cause'] is False
    assert issue['recoverable'] is False
    assert '驱动供电异常' in issue['likely_causes']
    assert all('已确认未上电' not in item for item in issue['evidence'])


def test_connection_info_distinguishes_disabled_local_app_from_network_failure():
    engine, status = make_engine()
    engine.network_status_provider = FakeNetwork({
        'interfaces': [{'name': 'wifi-test', 'label': 'Wi-Fi 网络', 'address': '10.0.0.8', 'up': True}],
        'defaultRoutes': [], 'warnings': [],
    })
    status.update('system_status', {
        'mobile_bridge_owner': 'supervisor', 'mobile_bridge_core_state': 'running',
        'mobile_bridge_tcp': 'tcp_ok',
    }, now=100.0)
    status.update('local_app_status', {'enabled': False}, now=100.0)

    info = engine.get_connection_info()

    assert info['bridge']['local_app_enabled'] is False
    assert info['recommended_endpoint']['address'] == '10.0.0.8'
    assert any(item['code'] == 'LOCAL_APP_DISABLED' for item in info['warnings'])


def test_sensor_diagnostic_distinguishes_no_publisher_from_stale_messages():
    engine, status = make_engine()
    status.update('system_status', {'topic_publishers': {'scan': 0, 'imu': 1, 'odom': 1}}, now=100.0)
    status.update('scan', {'state': 'ok'}, now=90.0)
    status.update('imu', {'state': 'ok'}, now=90.0)
    status.update('odom', {'state': 'ok'}, now=90.0)

    report = engine.run_self_check('sensors')

    assert any(issue['code'] == 'SCAN_NO_PUBLISHER' for issue in report['issues'])
    assert any(issue['code'] == 'IMU_STALE' for issue in report['issues'])
