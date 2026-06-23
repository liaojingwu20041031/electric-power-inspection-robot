import time
from types import SimpleNamespace
from typing import Dict, List

from ylhb_mobile_bridge.ros_bridge import MobileRosBridge


class FakePublisherInfo:
    def __init__(self, node_name: str = "fake_node") -> None:
        self.node_name = node_name


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
    bridge.zlac_status_topic = "/zlac8015d/status"
    bridge.zlac_fault_topic = "/zlac8015d/fault"
    bridge.system_mode_topic = "/inspection_ai/system_mode"
    bridge._last_odom_time = None
    bridge._last_scan_time = None
    bridge._last_map_time = None
    bridge._scan_range_min = None
    bridge._scan_range_max = None
    bridge._zlac_status = "unknown"
    bridge._task_status = "idle"
    bridge._system_mode = "unknown"
    bridge._node_names = node_names
    bridge._topic_names_and_types = topic_names_and_types
    bridge._topic_publishers = topic_publishers

    bridge.get_node_names = lambda: bridge._node_names
    bridge.get_topic_names_and_types = lambda: list(bridge._topic_names_and_types.items())
    bridge.get_publishers_info_by_topic = lambda topic: bridge._topic_publishers.get(topic, [])
    return bridge


def test_scan_callback_captures_range_min_max():
    bridge = make_bridge()
    msg = SimpleNamespace(range_min=0.05, range_max=40.0)
    bridge._on_scan(msg)
    assert bridge._scan_range_min == 0.05
    assert bridge._scan_range_max == 40.0
    assert bridge._last_scan_time is not None


def test_map_callback_updates_last_map_time():
    bridge = make_bridge()
    assert bridge._last_map_time is None
    bridge._on_map(SimpleNamespace())
    assert bridge._last_map_time is not None


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
    assert "timestamp" in status


def test_debug_status_includes_all_ten_node_keys():
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


def test_debug_status_topics_include_all_four():
    bridge = make_bridge(
        topic_names_and_types={"/cmd_vel": [], "/odom": [], "/scan": []}
    )
    status = bridge.debug_status()
    assert status["topics"] == {
        "/cmd_vel": True,
        "/odom": True,
        "/scan": True,
        "/map": False,
    }


def test_zlac_fault_updates_status():
    bridge = make_bridge()
    bridge._on_zlac_fault(SimpleNamespace(data="over_current"))
    assert bridge._zlac_status == "fault: over_current"


def test_zlac_status_online_when_data_empty():
    bridge = make_bridge()
    bridge._on_zlac_status(SimpleNamespace(data=""))
    assert bridge._zlac_status == "online"
