from ylhb_llm.base_motion_skill_node import BaseMotionSkillLogic


def test_base_motion_rejects_out_of_range_angle_and_distance():
    logic = BaseMotionSkillLogic()

    assert logic.validate("rotate_relative", {"angle_deg": 999}, "ready", True) == "angle_deg out of range"
    assert logic.validate("move_relative", {"distance_m": 9}, "ready", True) == "distance_m out of range"


def test_base_motion_rejects_fault_mode_and_offline_chassis():
    logic = BaseMotionSkillLogic()

    assert logic.validate("rotate_relative", {"angle_deg": 90}, "fault", True) == "system_mode blocks base motion"
    assert logic.validate("rotate_relative", {"angle_deg": 90}, "ready", False) == "chassis offline"


def test_stop_motion_is_always_valid():
    logic = BaseMotionSkillLogic()

    assert logic.validate("stop_motion", {}, "fault", False) == ""
