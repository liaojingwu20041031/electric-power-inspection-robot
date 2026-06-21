import importlib.util
from pathlib import Path


def load_bringup_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_n300_usb_id_is_ch9102():
    bringup = load_bringup_module()

    assert bringup.N300_VENDOR_ID == "1a86"
    assert bringup.N300_PRODUCT_ID == "55d4"


def test_default_n300_imu_can_resolve_tty_acm(monkeypatch):
    bringup = load_bringup_module()

    monkeypatch.setattr(bringup.os.path, "exists", lambda path: False)
    monkeypatch.setattr(bringup, "_find_n300_tty_ports", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(bringup, "_can_open_serial", lambda path: (True, ""))

    port, warning, error = bringup._resolve_required_imu_port(bringup.DEFAULT_IMU_PORT)

    assert port == "/dev/ttyACM0"
    assert "N300WP PRO" in warning
    assert error is None


def test_hardware_preflight_runs_before_robot_state_publisher(monkeypatch, tmp_path):
    bringup = load_bringup_module()
    package_dir = Path(__file__).resolve().parents[1]

    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        bringup, "get_package_share_directory", lambda package_name: str(package_dir)
    )

    actions = bringup.generate_launch_description().entities
    preflight_index = next(
        index
        for index, action in enumerate(actions)
        if action.__class__.__name__ == "OpaqueFunction"
        and getattr(action, "_OpaqueFunction__function", None) is bringup.serial_nodes
    )
    robot_state_index = next(
        index
        for index, action in enumerate(actions)
        if action.__class__.__name__ == "Node"
        and getattr(action, "_Node__node_executable", None) == "robot_state_publisher"
    )

    assert preflight_index < robot_state_index
