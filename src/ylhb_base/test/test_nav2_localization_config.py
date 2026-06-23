from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
NAV2_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params.yaml"
NAV2_BT_PATH = PACKAGE_DIR / "config" / "nav2_no_recovery.xml"
CMAKE_PATH = PACKAGE_DIR / "CMakeLists.txt"
MAP_YAML_PATH = WORKSPACE_DIR / "maps" / "my_map.yaml"
MAP_PGM_PATH = WORKSPACE_DIR / "maps" / "my_map.pgm"


def load_nav2_params():
    return yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))


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


def test_map_thresholds_keep_205_gray_pixels_unknown():
    metadata = load_map_metadata()

    assert metadata["mode"] == "trinary"
    assert metadata["negate"] == 0
    assert metadata["free_thresh"] == 0.196
    assert metadata["occupied_thresh"] == 0.65
    assert nav2_trinary_cell_value(205, metadata) == -1
    assert nav2_trinary_cell_value(254, metadata) == 0
    assert nav2_trinary_cell_value(0, metadata) == 100


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

    assert local["width"] == 4
    assert local["height"] == 4
    assert local["footprint_padding"] == 0.02
    assert local["inflation_layer"]["inflation_radius"] == 0.35
    assert local["inflation_layer"]["cost_scaling_factor"] == 3.0

    assert global_map["update_frequency"] == 2.0
    assert global_map["track_unknown_space"] is True
    assert global_map["footprint_padding"] == 0.02
    assert global_map["plugins"] == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" not in global_map
    assert global_map["inflation_layer"]["inflation_radius"] == 0.35
    assert global_map["inflation_layer"]["cost_scaling_factor"] == 3.0


def test_dwb_low_speed_limits_match_velocity_smoother():
    config = load_nav2_params()
    controller = config["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert "current_goal_checker" not in controller
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["min_x_velocity_threshold"] == 0.001
    assert controller["min_y_velocity_threshold"] == 0.5
    assert controller["min_theta_velocity_threshold"] == 0.001

    assert follow_path["max_vel_x"] == 0.12
    assert follow_path["max_speed_xy"] == 0.12
    assert follow_path["max_vel_theta"] == 0.30
    assert follow_path["vx_samples"] == 20
    assert follow_path["vtheta_samples"] == 20
    assert follow_path["sim_time"] == 1.7
    assert follow_path["linear_granularity"] == 0.05
    assert follow_path["angular_granularity"] == 0.025
    assert follow_path["short_circuit_trajectory_evaluation"] is True
    assert follow_path["stateful"] is True
    assert follow_path["critics"] == [
        "RotateToGoal",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist",
    ]
    assert "ObstacleFootprint.scale" not in follow_path
    assert follow_path["BaseObstacle.scale"] == 5.0
    assert follow_path["PathAlign.scale"] == 8.0
    assert follow_path["PathAlign.forward_point_distance"] == 0.1
    assert follow_path["PathDist.scale"] == 10.0
    assert follow_path["GoalAlign.scale"] == 8.0
    assert follow_path["GoalAlign.forward_point_distance"] == 0.1
    assert follow_path["GoalDist.scale"] == 8.0

    assert smoother["max_velocity"] == [0.12, 0.0, 0.30]
    assert smoother["min_velocity"] == [-0.05, 0.0, -0.30]
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


def test_navigation_recovers_from_stalls_with_checked_low_speed_backup():
    controller = load_nav2_params()["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    backup = load_nav2_behavior_tree().find(".//BackUp")

    assert progress["required_movement_radius"] == 0.05
    assert progress["movement_time_allowance"] == 30.0
    assert backup is not None
    assert backup.attrib["backup_dist"] == "0.08"
    assert backup.attrib["backup_speed"] == "0.03"
    assert "time_allowance" not in backup.attrib


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
