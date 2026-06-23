import importlib.util
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def load_bringup_module():
    launch_path = PACKAGE_DIR / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_rtk_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_argument_defaults(actions):
    return {
        action.name: action.default_value[0].text
        for action in actions
        if action.__class__.__name__ == "DeclareLaunchArgument"
    }


def test_rtk_launch_defaults_are_optional_and_stable(monkeypatch, tmp_path):
    bringup = load_bringup_module()
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        bringup, "get_package_share_directory", lambda package_name: str(PACKAGE_DIR)
    )

    actions = bringup.generate_launch_description().entities
    defaults = launch_argument_defaults(actions)

    assert defaults["enable_rtk"] == "false"
    assert defaults["rtk_port"] == "/dev/rtk_4g"
    assert defaults["rtk_baud"] == "115200"
    assert defaults["rtk_frame_id"] == "gps_link"


def test_rtk_node_is_conditioned_by_enable_rtk(monkeypatch, tmp_path):
    bringup = load_bringup_module()
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        bringup, "get_package_share_directory", lambda package_name: str(PACKAGE_DIR)
    )

    actions = bringup.generate_launch_description().entities
    rtk_node = next(
        action
        for action in actions
        if action.__class__.__name__ == "Node"
        and getattr(action, "_Node__node_executable", None) == "wtrtk980_nmea_node"
    )

    assert rtk_node.condition is not None
