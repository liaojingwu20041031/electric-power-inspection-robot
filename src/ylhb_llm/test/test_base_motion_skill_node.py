import json
import math

from ylhb_llm.base_motion_skill_node import BaseMotionSkillLogic, BaseMotionSkillNode


def test_base_motion_rejects_out_of_range_angle_and_distance():
    logic = BaseMotionSkillLogic()

    assert logic.validate("rotate_relative", {"angle_deg": 999}, "ready", True) == "angle_deg out of range"
    assert logic.validate("move_relative", {"distance_m": 9}, "ready", True) == "distance_m out of range"


def test_base_motion_rejects_fault_mode_and_offline_chassis():
    logic = BaseMotionSkillLogic()

    assert logic.validate("rotate_relative", {"angle_deg": 90}, "fault", True) == "system_mode blocks base motion"
    assert logic.validate("rotate_relative", {"angle_deg": 90}, "ready", False) == "chassis offline"
    assert logic.chassis_status_online("offline controller_ready") is False
    assert logic.chassis_status_online("stale/offline") is False
    assert logic.chassis_status_online("feedback_timeout") is False
    assert logic.chassis_status_online("online controller_ready") is True


def test_stop_motion_is_always_valid():
    logic = BaseMotionSkillLogic()

    assert logic.validate("stop_motion", {}, "fault", False) == ""


def test_active_motion_stops_when_chassis_goes_offline():
    stopped = []
    node = BaseMotionSkillNode.__new__(BaseMotionSkillNode)
    node.active = {
        'command': 'rotate_relative',
        'arguments': {'angle_deg': 90},
        'deadline': float('inf'),
    }
    node.logic = BaseMotionSkillLogic()
    node.system_mode = 'ready'
    node.chassis_online = lambda: False
    node.stop = lambda status, message='': stopped.append((status, message))

    node.tick()

    assert stopped == [('failed', 'chassis offline')]


def test_rotation_progress_accumulates_across_pi_wrap():
    node = BaseMotionSkillNode.__new__(BaseMotionSkillNode)
    node.active = {
        'command': 'rotate_relative',
        'arguments': {'angle_deg': 180},
        'start_pose': (0.0, 0.0, 0.0),
        'last_yaw': math.radians(179),
        'accumulated_yaw': math.radians(179),
    }
    node.current_pose = lambda: (0.0, 0.0, math.radians(-179))

    assert node.reached_target() is True

    node.active['last_yaw'] = math.radians(-179)
    node.active['accumulated_yaw'] = math.radians(-179)
    node.current_pose = lambda: (0.0, 0.0, math.radians(179))

    assert node.reached_target() is False


def test_base_motion_status_returns_agent_correlation_ids():
    published = []
    node = BaseMotionSkillNode.__new__(BaseMotionSkillNode)
    node.status_pub = type(
        'Publisher', (), {'publish': lambda _self, msg: published.append(json.loads(msg.data))}
    )()

    node.publish_status('done', 'done', {
        'run_id': 'run_1',
        'tool_call_id': 'call_1',
        'operation_id': 'op_1',
    })

    assert published[0]['status'] == 'done'
    assert published[0]['run_id'] == 'run_1'
    assert published[0]['tool_call_id'] == 'call_1'
    assert published[0]['operation_id'] == 'op_1'
