from pathlib import Path

import yaml


SLAM_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "slam_toolbox_params.yaml"
)


def test_slam_uses_declared_min_laser_range_parameter():
    config = SLAM_CONFIG_PATH.read_text(encoding="utf-8")

    assert "    min_laser_range:" in config
    assert "    minimum_laser_range:" not in config


def test_slam_adds_keyframes_for_small_robot_motion():
    config = yaml.safe_load(SLAM_CONFIG_PATH.read_text(encoding="utf-8"))
    params = config["slam_toolbox"]["ros__parameters"]

    assert params["resolution"] == 0.025
    assert params["minimum_travel_distance"] == 0.05
    assert params["minimum_travel_heading"] == 0.05
    assert params["scan_buffer_size"] == 20
    assert params["loop_search_space_resolution"] == 0.025
    assert params["coarse_angle_resolution"] == 0.01745

    assert params["throttle_scans"] == 1
    assert params["minimum_time_interval"] == 0.1
    assert params["correlation_search_space_resolution"] == 0.01
    assert params["use_scan_matching"] is True
    assert params["do_loop_closing"] is True
