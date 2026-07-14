import json
import queue
import time
from types import SimpleNamespace
from typing import Dict, List

import pytest

from ylhb_mobile_bridge.ros_bridge import MobileRosBridge


class FakePublisherInfo:
    def __init__(self, node_name: str = "fake_node") -> None:
        self.node_name = node_name


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class FailingPublisher(FakePublisher):
    def publish(self, msg) -> None:
        raise RuntimeError("publish failed")


class FakePlatformStore:
    def __init__(self) -> None:
        self.states = []
        self.events = []

    def set_command_state(self, command_id, state, result=None) -> None:
        self.states.append((command_id, state, result or {}))

    def append_event(self, event):
        saved = {**event, "sequence": len(self.events) + 1}
        self.events.append(saved)
        return saved

    def bridge_setting(self, key, default=''):
        return getattr(self, 'bridge_settings', {}).get(key, default)

    def set_bridge_setting(self, key, value):
        if not hasattr(self, 'bridge_settings'):
            self.bridge_settings = {}
        self.bridge_settings[key] = str(value)


class FakeNetworkStatusProvider:
    def snapshot(self):
        return {
            'interfaces': [
                {
                    'name': 'wlan0',
                    'type': 'wifi',
                    'label': 'Wi-Fi 网络',
                    'address': '192.168.137.100',
                    'prefixLength': 24,
                    'gateway': '192.168.137.1',
                    'defaultRoute': True,
                    'metric': 600,
                    'up': True,
                },
                {
                    'name': 'eth0',
                    'type': 'ethernet',
                    'label': '5G 有线网络',
                    'address': '192.168.8.20',
                    'prefixLength': 24,
                    'gateway': '192.168.8.1',
                    'defaultRoute': True,
                    'metric': 100,
                    'up': True,
                },
            ],
            'defaultRoutes': [],
            'warnings': [],
        }

    def app_endpoints(self, _host, port):
        return [
            {
                'interface': 'wlan0',
                'type': 'wifi',
                'label': 'Wi-Fi 网络',
                'address': '192.168.137.100',
                'port': port,
                'url': f'http://192.168.137.100:{port}',
                'available': True,
            },
            {
                'interface': 'eth0',
                'type': 'ethernet',
                'label': '5G 有线网络',
                'address': '192.168.8.20',
                'port': port,
                'url': f'http://192.168.8.20:{port}',
                'available': True,
            },
        ]


class FakeTimer:
    def __init__(self, interval, callback) -> None:
        self.interval = interval
        self.callback = callback
        self.daemon = False
        self.cancelled = False

    def start(self) -> None:
        return None

    def cancel(self) -> None:
        self.cancelled = True


def make_bridge(
    node_names: List[str] = None,
    topic_names_and_types: Dict[str, list] = None,
    topic_publishers: Dict[str, List[FakePublisherInfo]] = None,
) -> MobileRosBridge:
    node_names = node_names or []
    topic_names_and_types = topic_names_and_types or {}
    topic_publishers = topic_publishers or {}

    bridge = MobileRosBridge.__new__(MobileRosBridge)
    bridge.cmd_vel_topic = "/cmd_vel"
    bridge.text_command_topic = "/inspection_ai/text_command"
    bridge.odom_topic = "/odom"
    bridge.scan_topic = "/scan"
    bridge.map_topic = "/map"
    bridge.imu_topic = "/imu/data"
    bridge.zlac_status_topic = "/zlac8015d/status"
    bridge.zlac_fault_topic = "/zlac8015d/fault"
    bridge.system_mode_topic = "/inspection_ai/system_mode"
    bridge.system_command_topic = "/inspection_ai/system_command"
    bridge.system_status_topic = "/inspection_ai/system_status"
    bridge.patrol_command_topic = "/patrol/command"
    bridge.patrol_status_topic = "/patrol/status"
    bridge._last_odom_time = None
    bridge._last_scan_time = None
    bridge._last_map_time = None
    bridge._last_imu_time = None
    bridge._latest_map = None
    bridge._pose = None
    bridge._map_pose = None
    bridge._velocity = None
    bridge._scan_range_min = None
    bridge._scan_range_max = None
    bridge._zlac_status = "unknown"
    bridge._task_status = "idle"
    bridge._system_mode = "unknown"
    bridge._system_status = {}
    bridge._patrol_status = {}
    bridge._last_command_result_key = ""
    bridge._status_cache = {}
    bridge.host = '0.0.0.0'
    bridge.port = 8000
    bridge.network_status = FakeNetworkStatusProvider()
    bridge._last_stop_motion_time = 0.0
    bridge._last_stop_text_time = 0.0
    bridge._node_names = node_names
    bridge._topic_names_and_types = topic_names_and_types
    bridge._topic_publishers = topic_publishers

    bridge.get_node_names = lambda: bridge._node_names
    bridge.get_topic_names_and_types = lambda: list(
        bridge._topic_names_and_types.items()
    )
    bridge.get_publishers_info_by_topic = lambda topic: (
        bridge._topic_publishers.get(topic, [])
    )
    return bridge


def make_velocity_bridge(
    max_linear_speed: float = 0.30,
    max_angular_speed: float = 0.55,
) -> MobileRosBridge:
    bridge = make_bridge()
    bridge.max_linear_speed = max_linear_speed
    bridge.max_angular_speed = max_angular_speed
    bridge.default_cmd_duration_ms = 300
    bridge._cmd_pub = FakePublisher()
    bridge._text_pub = FakePublisher()
    bridge._system_command_pub = FakePublisher()
    bridge._patrol_command_pub = FakePublisher()
    bridge._stop_timer = None
    return bridge


def test_configured_speed_limits_are_capped_by_chassis_safety_limits():
    assert MobileRosBridge._safe_speed_limit(0.50, 0.35) == 0.35
    assert MobileRosBridge._safe_speed_limit(0.80, 0.55) == 0.55


def test_publish_velocity_allows_new_default_speed_limits(monkeypatch):
    monkeypatch.setattr(
        "ylhb_mobile_bridge.ros_bridge.threading.Timer",
        FakeTimer,
    )
    bridge = make_velocity_bridge()

    bridge.publish_velocity(0.30, 0.55, 300)

    msg = bridge._cmd_pub.messages[-1]
    assert msg.linear.x == pytest.approx(0.30)
    assert msg.angular.z == pytest.approx(0.55)


def test_publish_velocity_clamps_requests_to_configured_limits(monkeypatch):
    monkeypatch.setattr(
        "ylhb_mobile_bridge.ros_bridge.threading.Timer",
        FakeTimer,
    )
    bridge = make_velocity_bridge(
        max_linear_speed=0.20,
        max_angular_speed=0.40,
    )

    bridge.publish_velocity(0.30, -0.55, 300)

    msg = bridge._cmd_pub.messages[-1]
    assert msg.linear.x == pytest.approx(0.20)
    assert msg.angular.z == pytest.approx(-0.40)


def test_publish_zero_velocity_does_not_create_delayed_stop_timer(monkeypatch):
    monkeypatch.setattr(
        "ylhb_mobile_bridge.ros_bridge.threading.Timer",
        FakeTimer,
    )
    bridge = make_velocity_bridge()

    bridge.publish_velocity(0.0, 0.0, 300)

    assert bridge._stop_timer is None
    msg = bridge._cmd_pub.messages[-1]
    assert msg.linear.x == 0.0
    assert msg.angular.z == 0.0


def test_publish_motion_replaces_existing_delayed_stop_timer(monkeypatch):
    monkeypatch.setattr(
        "ylhb_mobile_bridge.ros_bridge.threading.Timer",
        FakeTimer,
    )
    bridge = make_velocity_bridge()

    bridge.publish_velocity(0.1, 0.0, 300)
    first_timer = bridge._stop_timer
    bridge.publish_velocity(0.1, 0.0, 300)

    assert first_timer.cancelled is True
    assert bridge._stop_timer is not first_timer


def test_stop_motion_debounces_repeated_zero_velocity(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("ylhb_mobile_bridge.ros_bridge.time.time", lambda: now[0])
    bridge = make_velocity_bridge()

    bridge.stop_motion()
    bridge.stop_motion()
    now[0] = 100.2
    bridge.stop_motion()

    assert len(bridge._cmd_pub.messages) == 2


def test_stop_motion_still_publishes_zero_velocity():
    bridge = make_velocity_bridge()

    bridge.stop_motion()

    msg = bridge._cmd_pub.messages[-1]
    assert msg.linear.x == 0.0
    assert msg.angular.z == 0.0


def test_stop_all_deduplicates_stop_text_but_keeps_zero_velocity(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("ylhb_mobile_bridge.ros_bridge.time.time", lambda: now[0])
    bridge = make_velocity_bridge()

    bridge.stop_all()
    now[0] = 100.2
    bridge.stop_all()
    now[0] = 102.2
    bridge.stop_all()

    assert len(bridge._cmd_pub.messages) == 3
    assert [msg.data for msg in bridge._text_pub.messages] == [
        "停止当前任务",
        "停止当前任务",
    ]


def test_scan_callback_captures_range_min_max():
    bridge = make_bridge()
    msg = SimpleNamespace(range_min=0.05, range_max=40.0)
    bridge._on_scan(msg)
    assert bridge._scan_range_min == 0.05
    assert bridge._scan_range_max == 40.0
    assert bridge._last_scan_time is not None


def test_map_callback_updates_last_map_time():
    bridge = make_bridge(node_names=["slam_toolbox"])
    assert bridge._last_map_time is None
    msg = SimpleNamespace()
    bridge._on_map(msg)
    assert bridge._last_map_time is not None
    assert bridge._latest_map is msg


def test_map_callback_ignores_map_when_slam_toolbox_is_not_running():
    bridge = make_bridge(node_names=["map_server"])
    msg = SimpleNamespace()

    bridge._on_map(msg)

    assert bridge._last_map_time is None
    assert bridge._latest_map is None


def test_map_callback_ignores_map_when_map_server_and_slam_run_together():
    bridge = make_bridge(node_names=["slam_toolbox", "map_server"])
    msg = SimpleNamespace()

    bridge._on_map(msg)

    assert bridge._last_map_time is None
    assert bridge._latest_map is None


def test_reset_mapping_map_clears_cached_map_and_timestamp():
    bridge = make_bridge(node_names=["slam_toolbox"])
    bridge._latest_map = object()
    bridge._last_map_time = time.time()

    bridge.reset_mapping_map()

    assert bridge._latest_map is None
    assert bridge._last_map_time is None


def test_odom_callback_caches_pose_and_velocity():
    bridge = make_bridge()
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom"),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.25, y=-0.5),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.5,
                    w=0.8660254,
                ),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.1),
                angular=SimpleNamespace(z=-0.2),
            )
        ),
    )

    bridge._on_odom(msg)

    assert bridge._pose["frame"] == "odom"
    assert bridge._pose["x"] == 1.25
    assert bridge._pose["y"] == -0.5
    assert round(bridge._pose["yaw"], 3) == 1.047
    assert bridge._velocity == {"linear_x": 0.1, "angular_z": -0.2}


def test_system_mode_callback_strips_whitespace():
    bridge = make_bridge()
    bridge._on_system_mode(SimpleNamespace(data="  mapping  "))
    assert bridge._system_mode == "mapping"


def test_system_mode_callback_empty_falls_back_to_unknown():
    bridge = make_bridge()
    bridge._on_system_mode(SimpleNamespace(data=""))
    assert bridge._system_mode == "unknown"


def test_robot_status_includes_system_mode():
    bridge = make_bridge()
    bridge._system_mode = "mapping"
    status = bridge.robot_status()
    assert status["system_mode"] == "mapping"
    assert "task_status" in status
    assert "pose" in status
    assert "velocity" in status
    assert "timestamp" in status


def test_robot_status_includes_optional_network_snapshot():
    bridge = make_bridge()

    network = bridge.robot_status()['network']

    assert [item['interface'] for item in network['appEndpoints']] == [
        'wlan0',
        'eth0',
    ]
    assert network['preferredAppEndpoint'] == {}
    assert network['candidateEndpoints'] == [
        {
            'url': 'http://192.168.137.100:8000',
            'interface': 'wlan0',
            'type': 'wifi',
            'linkUp': True,
        },
        {
            'url': 'http://192.168.8.20:8000',
            'interface': 'eth0',
            'type': 'ethernet',
            'linkUp': True,
        },
    ]
    assert len(network['interfaces']) == 2
    assert network['warnings'] == []


def test_debug_status_includes_expected_node_keys():
    bridge = make_bridge()
    status = bridge.debug_status()
    expected_nodes = {
        "zlac8015d_canopen_controller",
        "slam_toolbox",
        "bt_navigator",
        "controller_server",
        "planner_server",
        "amcl",
        "map_server",
        "bringup",
        "rplidar_node",
        "imu",
        "tf",
    }
    assert set(status["nodes"].keys()) == expected_nodes


def test_debug_status_bringup_detects_robot_state_publisher():
    bridge = make_bridge(node_names=["robot_state_publisher"])
    status = bridge.debug_status()
    assert status["nodes"]["bringup"] is True
    assert status["nodes"]["rplidar_node"] is False


def test_debug_status_rplidar_node_detected():
    bridge = make_bridge(node_names=["rplidar_node"])
    status = bridge.debug_status()
    assert status["nodes"]["rplidar_node"] is True
    assert status["nodes"]["bringup"] is False


def test_debug_status_tf_detected_via_topic_publishers():
    bridge = make_bridge(
        topic_publishers={"/tf": [FakePublisherInfo("robot_state_publisher")]}
    )
    status = bridge.debug_status()
    assert status["nodes"]["tf"] is True


def test_debug_status_tf_false_when_no_publishers():
    bridge = make_bridge(topic_publishers={"/tf": []})
    status = bridge.debug_status()
    assert status["nodes"]["tf"] is False


def test_debug_status_includes_scan_and_map_age_and_range():
    bridge = make_bridge()
    now = time.time()
    bridge._last_scan_time = now
    bridge._last_map_time = now
    bridge._scan_range_min = 0.05
    bridge._scan_range_max = 40.0
    status = bridge.debug_status()
    assert status["last_scan_age_sec"] is not None
    assert status["last_map_age_sec"] is not None
    assert status["scan_range_min"] == 0.05
    assert status["scan_range_max"] == 40.0


def test_debug_status_includes_task_status_and_system_mode():
    bridge = make_bridge()
    bridge._task_status = "emergency_stop"
    bridge._system_mode = "fault"
    status = bridge.debug_status()
    assert status["task_status"] == "emergency_stop"
    assert status["system_mode"] == "fault"


def test_status_graph_queries_are_cached_for_short_window(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("ylhb_mobile_bridge.ros_bridge.time.time", lambda: now[0])
    bridge = make_bridge(node_names=["robot_state_publisher"])
    calls = {"topics": 0, "nodes": 0}

    def get_topics():
        calls["topics"] += 1
        return []

    def get_nodes():
        calls["nodes"] += 1
        return bridge._node_names

    bridge.get_topic_names_and_types = get_topics
    bridge.get_node_names = get_nodes

    bridge.debug_status()
    bridge.debug_status()
    now[0] = 101.1
    bridge.debug_status()

    assert calls["topics"] == 2
    assert calls["nodes"] == 2


def test_system_and_patrol_commands_publish_json_and_plain_command():
    bridge = make_velocity_bridge()

    bridge.publish_system_command("start_mapping", map_name="demo")
    bridge.publish_patrol_command("pause")

    payload = bridge._system_command_pub.messages[-1].data
    assert '"command": "start_mapping"' in payload
    assert '"map_name": "demo"' in payload
    assert bridge._patrol_command_pub.messages[-1].data == "pause"


def test_cloud_command_is_dispatched_only_after_ros_publish():
    bridge = make_velocity_bridge()
    bridge._cloud_command_queue = queue.Queue()
    bridge._platform_context = {}
    bridge.platform_store = FakePlatformStore()
    bridge.enqueue_cloud_command({
        "type": "PAUSE", "commandId": "command-1", "requestId": "request-1",
        "executionId": "execution-1", "deploymentId": "deployment-1",
    })

    bridge._drain_cloud_commands()

    assert bridge.platform_store.states[-1][1] == "DISPATCHED"
    assert len(bridge._system_command_pub.messages) == 1


def test_local_app_toggle_persists_without_changing_cloud_state():
    bridge = make_bridge()
    bridge.platform_store = FakePlatformStore()
    bridge.cloud_client = SimpleNamespace(desired_enabled=True)
    bridge._local_app_enabled = True
    bridge._local_app_http_available = True
    bridge._local_app_last_changed_at = ''
    bridge._local_app_last_error = ''
    bridge._local_app_clients = {'status': 0, 'map': 0}

    status = bridge.set_local_app_enabled(False)

    assert status['enabled'] is False
    assert bridge.platform_store.bridge_settings['local_app_enabled_override'] == 'false'
    assert bridge.cloud_client.desired_enabled is True


def test_local_app_override_wins_and_status_has_ui_contract():
    bridge = make_bridge()
    bridge.platform_store = FakePlatformStore()
    bridge.platform_store.set_bridge_setting('local_app_enabled_override', 'false')
    bridge._local_app_enabled = True
    bridge._local_app_http_available = True
    bridge._local_app_last_changed_at = ''
    bridge._local_app_last_error = ''
    bridge._local_app_clients = {'status': 2, 'map': 1}
    bridge.require_token = False
    bridge._system_status = {
        'mobile_bridge_url': 'http://192.168.1.50:8000',
        'mobile_bridge_managed_externally': True,
    }

    bridge.initialize_local_app_settings(bridge.platform_store)
    status = bridge.local_app_status_snapshot()

    assert status == {
        'enabled': False,
        'state': 'DISABLED',
        'httpAvailable': True,
        'appUrl': 'http://192.168.1.50:8000',
        'authRequired': False,
        'activeStatusClients': 2,
        'activeMapClients': 1,
        'managedExternally': True,
        'appEndpoints': bridge.network_status.app_endpoints('0.0.0.0', 8000),
        'candidateEndpoints': status['candidateEndpoints'],
        'preferredAppEndpoint': {},
        'networkInterfaces': bridge.network_status.snapshot()['interfaces'],
        'networkWarnings': [],
        'lastChangedAt': status['lastChangedAt'],
        'lastError': '',
    }


def test_cloud_toggle_does_not_change_local_app_state():
    bridge = make_bridge()
    bridge._local_app_enabled = True
    bridge.cloud_client = SimpleNamespace(
        set_enabled=lambda enabled: {
            'configured': True,
            'desiredEnabled': bool(enabled),
            'state': 'DISABLED' if not enabled else 'CONNECTING',
        }
    )
    request = SimpleNamespace(data=False)
    response = SimpleNamespace(success=False, message='')

    bridge._set_cloud_enabled(request, response)

    assert response.success is True
    assert bridge._local_app_enabled is True


def test_cloud_toggle_publishes_status_immediately():
    bridge = make_bridge()
    bridge.cloud_client = SimpleNamespace(
        set_enabled=lambda enabled: {
            'configured': True, 'desiredEnabled': bool(enabled), 'state': 'CONNECTING',
        },
        status=lambda: {'configured': True, 'desiredEnabled': True, 'state': 'CONNECTING'},
    )
    bridge._cloud_status_pub = FakePublisher()
    request = SimpleNamespace(data=True)
    response = SimpleNamespace(success=False, message='')

    bridge._set_cloud_enabled(request, response)

    assert response.success is True
    assert len(bridge._cloud_status_pub.messages) == 1


def test_invalid_cloud_queue_item_is_rejected_with_event():
    bridge = make_velocity_bridge()
    bridge._cloud_command_queue = queue.Queue()
    bridge._platform_context = {}
    bridge.platform_store = FakePlatformStore()
    bridge.enqueue_cloud_command({"type": "PAUSE", "commandId": "command-2", "requestId": "request-2"})

    bridge._drain_cloud_commands()

    assert bridge.platform_store.states[-1][1] == "REJECTED"
    assert bridge.platform_store.events[-1]["event"] == "command_rejected"


def test_cloud_ros_publish_exception_is_failed_with_event():
    bridge = make_velocity_bridge()
    bridge._system_command_pub = FailingPublisher()
    bridge._cloud_command_queue = queue.Queue()
    bridge._platform_context = {}
    bridge.platform_store = FakePlatformStore()
    bridge.enqueue_cloud_command({
        "type": "CANCEL", "commandId": "command-3", "requestId": "request-3",
        "executionId": "execution-3", "deploymentId": "deployment-3",
    })

    bridge._drain_cloud_commands()

    assert bridge.platform_store.states[-1][1] == "FAILED"
    assert bridge.platform_store.events[-1]["error_code"] == "ROS_PUBLISH_FAILED"


def test_patrol_event_keeps_its_own_command_context():
    bridge = make_bridge()
    bridge.platform_store = FakePlatformStore()
    bridge.platform_robot_id = "robot-1"
    bridge.platform_boot_id = "boot-1"
    bridge._platform_context = {
        "active_execution_id": "wrong-execution", "active_deployment_id": "wrong-deployment",
        "active_request_id": "wrong-request", "active_command_id": "wrong-command",
    }
    event = {
        "event": "route_paused", "execution_id": "execution-1", "deployment_id": "deployment-1",
        "request_id": "request-1", "command_id": "command-1",
    }

    bridge._on_patrol_event(SimpleNamespace(data=json.dumps(event)))

    saved = bridge.platform_store.events[-1]
    assert (saved["execution_id"], saved["request_id"], saved["command_id"]) == ("execution-1", "request-1", "command-1")
    assert bridge.platform_store.states[-1][1] == "APPLIED"


def test_debug_status_topics_include_all_four():
    bridge = make_bridge(
        topic_names_and_types={
            "/cmd_vel": [],
            "/odom": [],
            "/scan": [],
            "/imu/data": [],
        }
    )
    status = bridge.debug_status()
    assert status["topics"] == {
        "/cmd_vel": True,
        "/odom": True,
        "/scan": True,
        "/map": False,
        "/imu/data": True,
    }


def test_debug_status_includes_pose_velocity_and_map_meta():
    bridge = make_bridge()
    bridge._pose = {"frame": "odom", "x": 1.0, "y": 2.0, "yaw": 0.3}
    bridge._velocity = {"linear_x": 0.1, "angular_z": 0.2}
    bridge.map_metadata = lambda: {"width": 10, "height": 20}

    status = bridge.debug_status()

    assert status["pose"] == bridge._pose
    assert status["velocity"] == bridge._velocity
    assert status["map_meta"] == {"width": 10, "height": 20}


def test_mapping_status_recommends_starting_bringup_when_prereqs_missing():
    bridge = make_bridge()

    status = bridge.mapping_status({"mode": "mapping", "running": False})

    assert status["bringup_ready"] is False
    assert status["map_available"] is False
    assert status["recommended_next_action"] == "start_bringup"


def test_mapping_status_recommends_starting_mapping_after_bringup_ready():
    bridge = make_bridge(
        topic_names_and_types={
            "/odom": [],
            "/scan": [],
            "/imu/data": [],
        },
        topic_publishers={"/tf": [FakePublisherInfo("robot_state_publisher")]},
    )

    status = bridge.mapping_status({"mode": "mapping", "running": False})

    assert status["bringup_ready"] is True
    assert status["map_available"] is False
    assert status["recommended_next_action"] == "start_mapping"


def test_mapping_status_does_not_report_cached_map_without_slam_toolbox():
    bridge = make_bridge(
        node_names=["map_server"],
        topic_names_and_types={
            "/odom": [],
            "/scan": [],
            "/imu/data": [],
        },
        topic_publishers={"/tf": [FakePublisherInfo("robot_state_publisher")]},
    )
    bridge._latest_map = object()
    bridge._last_map_time = time.time()

    status = bridge.mapping_status({"mode": "mapping", "running": False})

    assert status["map_available"] is False
    assert status["last_map_age_sec"] is None
    assert status["map_meta"] is None


def test_mapping_status_recommends_waiting_for_map_while_mapping_runs():
    bridge = make_bridge(
        topic_names_and_types={
            "/odom": [],
            "/scan": [],
            "/imu/data": [],
        },
        topic_publishers={"/tf": [FakePublisherInfo("robot_state_publisher")]},
    )

    status = bridge.mapping_status({"mode": "mapping", "running": True})

    assert status["bringup_ready"] is True
    assert status["map_available"] is False
    assert status["recommended_next_action"] == "wait_for_map"


def test_mapping_status_recommends_save_when_map_is_available():
    bridge = make_bridge(
        node_names=["async_slam_toolbox_node"],
        topic_names_and_types={
            "/odom": [],
            "/scan": [],
            "/imu/data": [],
        },
        topic_publishers={"/tf": [FakePublisherInfo("robot_state_publisher")]},
    )
    bridge._latest_map = object()
    bridge.map_metadata = lambda: {"width": 10, "height": 10}

    status = bridge.mapping_status({"mode": "mapping", "running": True})

    assert status["bringup_ready"] is True
    assert status["map_available"] is True
    assert status["recommended_next_action"] == "continue_mapping_or_save"


def test_zlac_fault_updates_status():
    bridge = make_bridge()
    bridge._on_zlac_fault(SimpleNamespace(data="over_current"))
    assert bridge._zlac_status == "fault: over_current"


def test_zlac_status_online_when_data_empty():
    bridge = make_bridge()
    bridge._on_zlac_status(SimpleNamespace(data=""))
    assert bridge._zlac_status == "online"
