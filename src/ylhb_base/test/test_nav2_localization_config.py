import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
NAV2_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params.yaml"
NAV2_KEEPOUT_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params_keepout.yaml"
NAV2_BT_PATH = PACKAGE_DIR / "config" / "nav2_no_recovery.xml"
BRINGUP_LAUNCH_PATH = PACKAGE_DIR / "launch" / "bringup.launch.py"
PACKAGE_XML_PATH = PACKAGE_DIR / "package.xml"
ZLAC_CONTROLLER_PATH = PACKAGE_DIR / "src" / "zlac8015d_canopen_controller.cpp"
STM32_CONTROLLER_PATH = PACKAGE_DIR / "src" / "base_controller.cpp"
ZLAC_CONFIG_PATH = PACKAGE_DIR / "config" / "zlac8015d.yaml"
BASE_KINEMATICS_PATH = PACKAGE_DIR / "config" / "base_kinematics.yaml"
LLM_CONFIG_PATH = WORKSPACE_DIR / "src" / "ylhb_llm" / "config" / "llm.yaml"
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
    keepout_config = load_keepout_nav2_params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]
    keepout_local = keepout_config["local_costmap"]["local_costmap"]["ros__parameters"]
    keepout_global = keepout_config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_footprint = ast.literal_eval(local["footprint"])
    global_footprint = ast.literal_eval(global_map["footprint"])

    assert global_map["resolution"] == 0.025
    assert local["resolution"] == 0.05
    assert local["width"] == 4
    assert local["height"] == 4
    assert local["footprint_padding"] == 0.01
    assert "robot_radius" not in local
    assert 0.20 <= local["inflation_layer"]["inflation_radius"] < global_map["inflation_layer"]["inflation_radius"]
    assert local["inflation_layer"] == keepout_local["inflation_layer"]
    assert local["inflation_layer"]["cost_scaling_factor"] > 0

    assert global_map["update_frequency"] == 2.0
    assert global_map["track_unknown_space"] is True
    assert global_map["footprint_padding"] == 0.01
    assert "robot_radius" not in global_map
    assert local_footprint == global_footprint
    assert len(local_footprint) == EXPECTED_FOOTPRINT_POINTS
    assert 0.30 <= global_map["inflation_layer"]["inflation_radius"] <= 0.70
    assert global_map["inflation_layer"] == keepout_global["inflation_layer"]
    assert global_map["inflation_layer"]["cost_scaling_factor"] > 0


def test_global_costmaps_use_nonpersistent_lidar_for_dynamic_replanning():
    normal_global = load_nav2_params()["global_costmap"]["global_costmap"]["ros__parameters"]
    keepout_global = load_keepout_nav2_params()["global_costmap"]["global_costmap"]["ros__parameters"]
    package = ET.parse(PACKAGE_XML_PATH).getroot()
    exec_dependencies = {dependency.text for dependency in package.findall("exec_depend")}

    assert normal_global["always_send_full_costmap"] is True
    assert keepout_global["always_send_full_costmap"] is True
    assert normal_global["plugins"] == ["static_layer", "nonpersistent_obstacle_layer", "inflation_layer"]
    assert keepout_global["plugins"] == [
        "static_layer",
        "nonpersistent_obstacle_layer",
        "keepout_filter",
        "inflation_layer",
    ]
    assert normal_global["plugins"][-1] == "inflation_layer"
    assert keepout_global["plugins"][-1] == "inflation_layer"
    assert normal_global["transform_tolerance"] == 0.3
    assert keepout_global["transform_tolerance"] == 0.3
    assert normal_global["nonpersistent_obstacle_layer"] == keepout_global["nonpersistent_obstacle_layer"]
    assert normal_global["inflation_layer"] == keepout_global["inflation_layer"]
    assert "nonpersistent_voxel_layer" in exec_dependencies

    obstacle = normal_global["nonpersistent_obstacle_layer"]
    scan = obstacle["scan"]
    assert obstacle["plugin"] == "nav2_costmap_2d/NonPersistentVoxelLayer"
    assert obstacle["enabled"] is True
    assert obstacle["combination_method"] == 1
    assert obstacle["origin_z"] == 0.0
    assert obstacle["z_resolution"] == 0.05
    assert obstacle["z_voxels"] == 16
    assert obstacle["mark_threshold"] == 0
    assert "tf_filter_tolerance" not in obstacle
    assert scan["topic"] == "/scan"
    assert scan["data_type"] == "LaserScan"
    assert scan["marking"] is True
    assert scan["clearing"] is False
    assert scan["observation_persistence"] == 0.0
    assert scan["obstacle_max_range"] == 2.5
    assert not any(key.startswith("raytrace_") for key in scan)


def test_local_costmap_uses_lidar_marking_and_clearing():
    for params in (load_nav2_params(), load_keepout_nav2_params()):
        local = params["local_costmap"]["local_costmap"]["ros__parameters"]
        obstacle = local["obstacle_layer"]
        scan = obstacle["scan"]

        assert "obstacle_layer" in local["plugins"]
        assert local["always_send_full_costmap"] is True
        assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
        assert obstacle["footprint_clearing_enabled"] is True
        assert scan["topic"] == "/scan"
        assert scan["clearing"] is True
        assert scan["marking"] is True
        assert scan["inf_is_valid"] is True
        assert scan["observation_persistence"] == 0.0
        assert scan["raytrace_max_range"] > scan["obstacle_max_range"]
        assert scan["raytrace_min_range"] == 0.15
        assert scan["obstacle_min_range"] == 0.15


def test_rotation_shim_handles_large_initial_heading_changes():
    config = load_nav2_params()
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    keepout_follow_path = load_keepout_nav2_params()["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]
    keepout_smoother = load_keepout_nav2_params()["velocity_smoother"]["ros__parameters"]

    assert follow_path == keepout_follow_path
    assert smoother == keepout_smoother
    assert follow_path["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow_path["primary_controller"] == "dwb_core::DWBLocalPlanner"
    assert 0 < follow_path["angular_disengage_threshold"] < follow_path["angular_dist_threshold"] <= 3.1416
    assert follow_path["forward_sampling_distance"] > 0
    assert 0.55 <= follow_path["rotate_to_heading_angular_vel"] <= smoother["max_velocity"][2]
    assert follow_path["max_angular_accel"] == smoother["max_accel"][2]
    assert follow_path["simulate_ahead_time"] > 0
    assert follow_path["rotate_to_goal_heading"] is False
    assert follow_path["closed_loop"] is True


def test_behavior_tree_keeps_two_hz_global_replanning():
    behavior_tree = load_nav2_behavior_tree()
    pipeline = behavior_tree.find(".//PipelineSequence")
    rate = behavior_tree.find(".//RateController")

    assert pipeline is not None
    assert rate is not None
    assert rate.attrib["hz"] == "2.0"
    assert rate.find(".//ComputePathToPose") is not None
    assert pipeline.find(".//FollowPath") is not None


def test_chassis_uses_direct_cmd_vel_with_driver_watchdogs():
    bringup = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")
    package_xml = PACKAGE_XML_PATH.read_text(encoding="utf-8")
    zlac = ZLAC_CONTROLLER_PATH.read_text(encoding="utf-8")
    stm32 = STM32_CONTROLLER_PATH.read_text(encoding="utf-8")
    zlac_config = yaml.safe_load(ZLAC_CONFIG_PATH.read_text(encoding="utf-8"))[
        "zlac8015d_canopen_controller"
    ]["ros__parameters"]

    assert "nav2_collision_monitor" not in bringup
    assert "collision_monitor" not in bringup
    assert "nav2_collision_monitor" not in package_xml
    assert "cmd_vel_topic" in zlac
    assert "cmd_vel_topic" in stm32
    assert zlac_config["cmd_vel_topic"] == "/cmd_vel"
    assert zlac_config["scan_topic"] == "/scan"
    assert zlac_config["require_fresh_scan"] is True
    assert zlac_config["scan_timeout_sec"] == 0.3
    assert zlac_config["cmd_timeout_sec"] == 0.5
    assert "'cmd_vel_topic': '/cmd_vel'" in bringup
    assert "'require_fresh_scan': True" in bringup
    assert "'scan_timeout_sec': 0.3" in bringup
    assert "'cmd_timeout_sec': 0.5" in bringup
    assert 'declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")' in zlac
    assert 'declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")' in stm32
    assert 'create_subscription<geometry_msgs::msg::Twist>(\n      cmd_vel_topic_' in zlac
    assert 'create_subscription<geometry_msgs::msg::Twist>(\n            cmd_vel_topic_' in stm32
    assert "/cmd_vel_safe" not in bringup
    assert "/cmd_vel_safe" not in ZLAC_CONFIG_PATH.read_text(encoding="utf-8")
    assert '"cmd_vel_safe"' not in zlac
    assert '"cmd_vel_safe"' not in stm32
    assert "last_scan_stamp_sec_" in zlac
    assert "last_scan_stamp_sec_" in stm32
    llm = yaml.safe_load(LLM_CONFIG_PATH.read_text(encoding="utf-8"))
    for node_name in (
        "basic_motion_command_node",
        "base_motion_skill_node",
        "inspection_display_ui_node",
        "system_supervisor_node",
    ):
        assert llm[node_name]["ros__parameters"]["cmd_vel_topic"] == "/cmd_vel"


def test_dwb_low_speed_limits_match_velocity_smoother():
    config = load_nav2_params()
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    keepout_follow_path = load_keepout_nav2_params()["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]
    base = yaml.safe_load(BASE_KINEMATICS_PATH.read_text(encoding="utf-8"))[
        "zlac8015d_canopen_controller"
    ]["ros__parameters"]
    zlac = yaml.safe_load(ZLAC_CONFIG_PATH.read_text(encoding="utf-8"))[
        "zlac8015d_canopen_controller"
    ]["ros__parameters"]

    assert "current_goal_checker" not in controller
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["min_x_velocity_threshold"] == 0.001
    assert controller["min_y_velocity_threshold"] == 0.5
    assert controller["min_theta_velocity_threshold"] == 0.001

    assert follow_path["max_vel_x"] == 0.12
    assert follow_path["min_vel_x"] == 0.0
    assert follow_path["max_speed_xy"] == 0.12
    assert follow_path["max_vel_theta"] == smoother["max_velocity"][2]
    assert follow_path["max_vel_theta"] >= 0.65
    assert follow_path["min_speed_xy"] > 0.0
    assert 0.50 <= follow_path["min_speed_theta"] <= follow_path["rotate_to_heading_angular_vel"]
    assert follow_path["acc_lim_theta"] >= 0.80
    assert follow_path["decel_lim_theta"] <= -0.80
    assert base["max_angular_speed"] >= follow_path["max_vel_theta"]
    assert zlac["profile_acceleration"] >= 200
    assert zlac["profile_deceleration"] >= 200
    assert follow_path["vx_samples"] >= 3
    assert follow_path["vtheta_samples"] >= 14
    theta_step = 2 * follow_path["max_vel_theta"] / (follow_path["vtheta_samples"] - 1)
    theta_samples = [
        -follow_path["max_vel_theta"] + index * theta_step
        for index in range(follow_path["vtheta_samples"])
    ]
    assert any(
        follow_path["min_speed_theta"] <= sample
        <= follow_path["rotate_to_heading_angular_vel"] + 1e-9
        for sample in theta_samples
    )
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

    assert smoother["max_velocity"][:2] == [0.12, 0.0]
    assert smoother["min_velocity"][:2] == [-0.05, 0.0]
    assert smoother["min_velocity"][2] == -follow_path["max_vel_theta"]
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
    keepout_goal_checker = load_keepout_nav2_params()["controller_server"]["ros__parameters"]["goal_checker"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]

    assert goal_checker == keepout_goal_checker
    assert 0.08 <= follow_path["xy_goal_tolerance"] <= 0.15
    assert 0.08 <= goal_checker["xy_goal_tolerance"] <= 0.15
    assert abs(follow_path["xy_goal_tolerance"] - goal_checker["xy_goal_tolerance"]) <= 0.01
    assert 0.04 <= goal_checker["yaw_goal_tolerance"] <= 0.05
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
    backups = behavior_tree.findall(".//BackUp")

    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_radius"] == 0.05
    assert progress["required_movement_angle"] == 0.12
    assert progress["movement_time_allowance"] == 12.0
    assert bt["bt_loop_duration"] == 20
    assert bt["default_server_timeout"] >= 500
    assert bt["wait_for_service_timeout"] >= 1000
    assert navigate_recovery is not None
    assert navigate_recovery.attrib["number_of_retries"] == "1"

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
    assert [child.tag for child in outer_recovery_children] == ["GoalUpdated", "BackUp"]
    assert len(backups) == 1
    assert backups[0].attrib == {
        "backup_dist": "0.08",
        "backup_speed": "0.03",
        "time_allowance": "4.0",
    }

    for params in (load_nav2_params(), load_keepout_nav2_params()):
        behavior = params["behavior_server"]["ros__parameters"]
        assert behavior["behavior_plugins"] == ["backup", "wait"]
        assert behavior["costmap_topic"] == "local_costmap/costmap_raw"
        assert behavior["footprint_topic"] == "local_costmap/published_footprint"


def test_smac_planner_avoids_unknown_and_prefers_centered_costs():
    planner = load_nav2_params()["planner_server"]["ros__parameters"]["GridBased"]
    keepout_planner = load_keepout_nav2_params()["planner_server"]["ros__parameters"]["GridBased"]

    assert planner["allow_unknown"] is False
    assert planner == keepout_planner
    assert planner["cost_travel_multiplier"] >= 2.0


def test_relocalization_scripts_are_installed_without_py_extension():
    cmake = CMAKE_PATH.read_text(encoding="utf-8")

    assert "scripts/amcl_swing_relocalization_node.py" in cmake
    assert "RENAME amcl_swing_relocalization_node" in cmake
    assert "scripts/scan_map_relocalization_node.py" in cmake
    assert "RENAME scan_map_relocalization_node" in cmake
