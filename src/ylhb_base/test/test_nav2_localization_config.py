import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
NAV2_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params.yaml"
NAV2_KEEPOUT_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params_keepout.yaml"
NAV2_BT_PATH = PACKAGE_DIR / "config" / "nav2_no_recovery.xml"
CMAKE_PATH = PACKAGE_DIR / "CMakeLists.txt"
MAP_YAML_PATH = WORKSPACE_DIR / "maps" / "my_map.yaml"
MAP_PGM_PATH = WORKSPACE_DIR / "maps" / "my_map.pgm"
EXPECTED_FOOTPRINT_POINTS = 16


def load_nav2_params():
    return yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))


def load_keepout_nav2_params():
    return yaml.safe_load(NAV2_KEEPOUT_CONFIG_PATH.read_text(encoding="utf-8"))


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_mapping_without_duplicate_keys(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        assert key not in mapping, f"Duplicate YAML key: {key}"
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_mapping_without_duplicate_keys,
)


def test_nav2_params_yaml_has_no_duplicate_keys():
    yaml.load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)


def load_nav2_behavior_tree():
    return ET.parse(NAV2_BT_PATH).getroot()


def load_map_metadata():
    return yaml.safe_load(MAP_YAML_PATH.read_text(encoding="utf-8"))


def parse_pgm_pixels(path):
    data = path.read_bytes()
    offset = 0

    def token():
        nonlocal offset
        while offset < len(data) and chr(data[offset]).isspace():
            offset += 1
        if offset < len(data) and data[offset] == ord("#"):
            while offset < len(data) and data[offset] not in (ord("\n"), ord("\r")):
                offset += 1
            return token()

        start = offset
        while offset < len(data) and not chr(data[offset]).isspace():
            offset += 1
        return data[start:offset].decode("ascii")

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if offset < len(data) and chr(data[offset]).isspace():
        offset += 1

    assert magic == "P5"
    assert max_value == 255
    pixels = data[offset:]
    assert len(pixels) == width * height
    return pixels


def nav2_trinary_cell_value(gray, metadata):
    occupancy = (255 - gray) / 255.0
    if occupancy > metadata["occupied_thresh"]:
        return 100
    if occupancy < metadata["free_thresh"]:
        return 0
    return -1


def test_saved_map_contains_205_gray_unknown_pixels():
    pixels = parse_pgm_pixels(MAP_PGM_PATH)

    assert 205 in pixels


def test_amcl_waits_for_operator_initial_pose_instead_of_forcing_origin():
    params = load_nav2_params()["amcl"]["ros__parameters"]

    assert params["set_initial_pose"] is False
    assert params["always_reset_initial_pose"] is False
    assert params["update_min_d"] == 0.02
    assert params["update_min_a"] == 0.02
    assert params["min_particles"] == 1000
    assert params["max_particles"] == 4000


def test_costmaps_use_static_global_map_and_stable_inflation_baseline():
    config = load_nav2_params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_obstacle = local["obstacle_layer"]
    global_obstacle = global_map["obstacle_layer"]
    local_scan = local_obstacle["scan"]
    global_scan = global_obstacle["scan"]
    local_footprint = ast.literal_eval(local["footprint"])
    global_footprint = ast.literal_eval(global_map["footprint"])

    assert global_map["resolution"] == 0.025
    assert local["resolution"] == 0.05
    assert local["width"] == 4
    assert local["height"] == 4
    assert local["footprint_padding"] == 0.01
    assert "robot_radius" not in local
    assert 0.30 <= local["inflation_layer"]["inflation_radius"] <= 0.70
    assert local["inflation_layer"]["cost_scaling_factor"] > 0

    assert global_map["update_frequency"] == 2.0
    assert global_map["track_unknown_space"] is True
    assert global_map["footprint_padding"] == 0.01
    assert "robot_radius" not in global_map
    assert local_footprint == global_footprint
    assert len(local_footprint) == EXPECTED_FOOTPRINT_POINTS
    assert {"static_layer", "obstacle_layer", "inflation_layer"} <= set(global_map["plugins"])
    assert global_obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert global_obstacle["enabled"] is True
    assert global_obstacle["observation_sources"] == "scan"
    assert global_scan["topic"] == "/scan"
    assert global_scan["data_type"] == "LaserScan"
    assert global_scan == local_scan
    assert global_scan["clearing"] is local_scan["clearing"] is True
    assert global_scan["marking"] is local_scan["marking"] is True
    assert global_scan["max_obstacle_height"] == local_scan["max_obstacle_height"] == 2.0
    assert global_scan["raytrace_max_range"] == local_scan["raytrace_max_range"] == 3.0
    assert global_scan["raytrace_min_range"] == local_scan["raytrace_min_range"] == 0.10
    assert global_scan["obstacle_max_range"] == local_scan["obstacle_max_range"] == 2.5
    assert global_scan["obstacle_min_range"] == local_scan["obstacle_min_range"] == 0.10
    assert 0.30 <= global_map["inflation_layer"]["inflation_radius"] <= 0.70
    assert global_map["inflation_layer"]["cost_scaling_factor"] > 0


def test_dwb_low_speed_limits_match_velocity_smoother():
    config = load_nav2_params()
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    keepout_follow_path = load_keepout_nav2_params()["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert "current_goal_checker" not in controller
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["min_x_velocity_threshold"] == 0.001
    assert controller["min_y_velocity_threshold"] == 0.5
    assert controller["min_theta_velocity_threshold"] == 0.001

    assert follow_path["max_vel_x"] == 0.12
    assert follow_path["min_vel_x"] == 0.0
    assert follow_path["max_speed_xy"] == 0.12
    assert follow_path["max_vel_theta"] == 0.40
    assert follow_path["min_speed_theta"] == 0.18
    assert follow_path["vx_samples"] >= 3
    assert follow_path["vtheta_samples"] >= 3
    assert 1.0 <= follow_path["sim_time"] <= 3.0
    assert follow_path["linear_granularity"] > 0
    assert follow_path["angular_granularity"] > 0
    assert follow_path["short_circuit_trajectory_evaluation"] is True
    assert follow_path["stateful"] is True
    assert follow_path == keepout_follow_path
    assert {"RotateToGoal", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"} <= set(follow_path["critics"])
    assert "BaseObstacle" not in follow_path["critics"]
    assert "BaseObstacle.scale" not in follow_path
    assert follow_path["ObstacleFootprint.scale"] == 0.02
    assert follow_path["PathAlign.scale"] == 8.0
    assert follow_path["PathAlign.forward_point_distance"] > 0
    assert follow_path["PathDist.scale"] == 10.0
    assert follow_path["GoalAlign.scale"] == 0.0
    assert follow_path["GoalAlign.forward_point_distance"] > 0
    assert follow_path["GoalDist.scale"] > 0
    assert follow_path["RotateToGoal.lookahead_time"] >= 0

    assert smoother["max_velocity"] == [0.12, 0.0, 0.40]
    assert smoother["min_velocity"] == [-0.05, 0.0, -0.40]
    assert smoother["max_accel"] == [
        follow_path["acc_lim_x"],
        follow_path["acc_lim_y"],
        follow_path["acc_lim_theta"],
    ]
    assert smoother["max_decel"] == [
        follow_path["decel_lim_x"],
        follow_path["decel_lim_y"],
        follow_path["decel_lim_theta"],
    ]


def test_navigation_goal_tolerances_are_conservative_and_consistent():
    config = load_nav2_params()
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    goal_checker = controller["goal_checker"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]

    assert 0.08 <= follow_path["xy_goal_tolerance"] <= 0.15
    assert 0.08 <= goal_checker["xy_goal_tolerance"] <= 0.15
    assert abs(follow_path["xy_goal_tolerance"] - goal_checker["xy_goal_tolerance"]) <= 0.01
    assert 0.08 <= goal_checker["yaw_goal_tolerance"] <= 0.15
    assert 0.08 <= planner["tolerance"] <= 0.10


def test_navigation_recovers_from_stalls_with_checked_low_speed_backup():
    config = load_nav2_params()
    controller = config["controller_server"]["ros__parameters"]
    bt = config["bt_navigator"]["ros__parameters"]
    progress = controller["progress_checker"]
    behavior_tree = load_nav2_behavior_tree()
    navigate_recovery = behavior_tree.find(".//RecoveryNode[@name='NavigateWaitRetry']")
    follow_path_recovery = behavior_tree.find(".//RecoveryNode[@name='FollowPath']")
    outer_recovery = behavior_tree.find(".//ReactiveFallback[@name='SmallBackupRecovery']")

    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_radius"] == 0.05
    assert progress["required_movement_angle"] == 0.12
    assert progress["movement_time_allowance"] == 12.0
    assert bt["bt_loop_duration"] == 20
    assert bt["default_server_timeout"] >= 500
    assert bt["wait_for_service_timeout"] >= 1000
    assert behavior_tree.find(".//BackUp") is None
    assert navigate_recovery is not None
    assert navigate_recovery.attrib["number_of_retries"] == "2"

    assert follow_path_recovery is not None
    assert follow_path_recovery.attrib["number_of_retries"] == "1"
    follow_path_children = list(follow_path_recovery)
    assert [child.tag for child in follow_path_children[:2]] == ["FollowPath", "Sequence"]

    local_recovery = follow_path_children[1]
    assert local_recovery.attrib["name"] == "ClearLocalCostmapAndWait"
    local_recovery_children = list(local_recovery)
    assert [child.tag for child in local_recovery_children[:2]] == ["ClearEntireCostmap", "Wait"]
    assert local_recovery_children[1].attrib["wait_duration"] == "1"

    assert outer_recovery is not None
    outer_recovery_children = list(outer_recovery)
    assert [child.tag for child in outer_recovery_children[:2]] == ["GoalUpdated", "Sequence"]
    outer_sequence = outer_recovery_children[1]
    assert outer_sequence.attrib["name"] == "WaitOnly"
    outer_sequence_children = list(outer_sequence)
    assert [child.tag for child in outer_sequence_children] == ["Wait"]
    assert outer_sequence_children[0].attrib["wait_duration"] == "1"


def test_smac_planner_avoids_unknown_and_prefers_centered_costs():
    planner = load_nav2_params()["planner_server"]["ros__parameters"]["GridBased"]

    assert planner["allow_unknown"] is False
    assert planner["cost_travel_multiplier"] == 1.5


def test_relocalization_scripts_are_installed_without_py_extension():
    cmake = CMAKE_PATH.read_text(encoding="utf-8")

    assert "scripts/amcl_swing_relocalization_node.py" in cmake
    assert "RENAME amcl_swing_relocalization_node" in cmake
    assert "scripts/scan_map_relocalization_node.py" in cmake
    assert "RENAME scan_map_relocalization_node" in cmake
