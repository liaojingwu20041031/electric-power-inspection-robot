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
    bridge.imu_topic = "/imu/data"
    bridge.zlac_status_topic = "/zlac8015d/status"
    bridge.zlac_fault_topic = "/zlac8015d/fault"
    bridge.system_mode_topic = "/inspection_ai/system_mode"
    bridge._last_odom_time = None
    bridge._last_scan_time = None
    bridge._last_map_time = None
    bridge._last_imu_time = None
    bridge._latest_map = None
    bridge._pose = None
    bridge._velocity = None
    bridge._scan_range_min = None
    bridge._scan_range_max = None
    bridge._zlac_status = "unknown"
    bridge._task_status = "idle"
    bridge._system_mode = "unknown"
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
    msg = SimpleNamespace()
    bridge._on_map(msg)
    assert bridge._last_map_time is not None
    assert bridge._latest_map is msg


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
