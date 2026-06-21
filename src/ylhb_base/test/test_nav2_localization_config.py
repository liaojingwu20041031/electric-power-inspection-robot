from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
NAV2_CONFIG_PATH = PACKAGE_DIR / "config" / "nav2_params.yaml"
CMAKE_PATH = PACKAGE_DIR / "CMakeLists.txt"


def load_nav2_params():
    return yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))


def test_amcl_waits_for_operator_initial_pose_instead_of_forcing_origin():
    params = load_nav2_params()["amcl"]["ros__parameters"]

    assert params["set_initial_pose"] is False
    assert params["always_reset_initial_pose"] is False
    assert params["update_min_d"] == 0.02
    assert params["update_min_a"] == 0.02
    assert params["max_beams"] == 100
    assert params["min_particles"] == 800
    assert params["max_particles"] == 3000


def test_nav2_transform_tolerances_limit_motion_time_skew():
    config = load_nav2_params()

    assert config["amcl"]["ros__parameters"]["transform_tolerance"] == 0.3
    assert config["bt_navigator"]["ros__parameters"]["transform_tolerance"] == 0.3
    assert (
        config["controller_server"]["ros__parameters"]["FollowPath"][
            "transform_tolerance"
        ]
        == 0.3
    )
    assert (
        config["local_costmap"]["local_costmap"]["ros__parameters"][
            "transform_tolerance"
        ]
        == 0.3
    )
    assert (
        config["global_costmap"]["global_costmap"]["ros__parameters"][
            "transform_tolerance"
        ]
        == 0.3
    )
    assert config["behavior_server"]["ros__parameters"]["transform_tolerance"] == 0.3


def test_costmaps_keep_local_scan_obstacles_out_of_global_map():
    config = load_nav2_params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = config["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local["width"] == 4
    assert local["height"] == 4
    assert local["footprint_padding"] == 0.02
    assert local["inflation_layer"]["inflation_radius"] == 0.35
    assert local["inflation_layer"]["cost_scaling_factor"] == 3.0
    assert local["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert local["obstacle_layer"]["scan"]["topic"] == "/scan"

    assert global_map["update_frequency"] == 2.0
    assert global_map["track_unknown_space"] is True
    assert global_map["plugins"] == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" not in global_map


def test_dwb_low_speed_limits_match_velocity_smoother():
    config = load_nav2_params()
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert follow_path["max_vel_x"] == 0.12
    assert follow_path["max_speed_xy"] == 0.12
    assert follow_path["max_vel_theta"] == 0.30
    assert follow_path["sim_time"] == 1.5
    assert follow_path["BaseObstacle.scale"] == 5.0
    assert follow_path["PathAlign.scale"] == 12.0
    assert follow_path["PathDist.scale"] == 12.0

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


def test_relocalization_scripts_are_installed_without_py_extension():
    cmake = CMAKE_PATH.read_text(encoding="utf-8")

    assert "scripts/amcl_swing_relocalization_node.py" in cmake
    assert "RENAME amcl_swing_relocalization_node" in cmake
    assert "scripts/scan_map_relocalization_node.py" in cmake
    assert "RENAME scan_map_relocalization_node" in cmake
