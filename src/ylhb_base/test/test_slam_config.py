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

    assert params["minimum_travel_distance"] <= 0.10
    assert params["minimum_travel_heading"] <= 0.10
