from pathlib import Path


def test_mobile_bridge_preserves_ros_velocity_coordinates():
    source = (
        Path(__file__).resolve().parents[1]
        / "ylhb_mobile_bridge"
        / "ros_bridge.py"
    ).read_text(encoding="utf-8")
    assert "msg.linear.x = linear_x" in source
    assert "msg.linear.x = -linear_x" not in source
    assert "msg.angular.z = angular_z" in source


def test_zlac_driver_owns_longitudinal_axis_calibration():
    source_root = Path(__file__).resolve().parents[2]
    config = (
        source_root / "ylhb_base" / "config" / "base_kinematics.yaml"
    ).read_text(encoding="utf-8")
    controller = (
        source_root / "ylhb_base" / "src" / "zlac8015d_canopen_controller.cpp"
    ).read_text(encoding="utf-8")

    assert "linear_axis_direction: 1.0" in config
    assert "command_to_internal(msg->linear.x, msg->angular.z)" in controller
    assert "feedback_to_external(internal_vx, internal_wz)" in controller
