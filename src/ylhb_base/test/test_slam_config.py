from pathlib import Path


SLAM_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "slam_toolbox_params.yaml"
)


def test_slam_uses_declared_min_laser_range_parameter():
    config = SLAM_CONFIG_PATH.read_text(encoding="utf-8")

    assert "    min_laser_range:" in config
    assert "    minimum_laser_range:" not in config
