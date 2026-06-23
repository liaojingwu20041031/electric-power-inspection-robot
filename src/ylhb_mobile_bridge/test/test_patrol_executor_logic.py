import copy
from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from ylhb_mobile_bridge import patrol_executor_node
from ylhb_mobile_bridge.patrol_executor_node import (
    PatchedScheduleClock,
    PatrolExecutorNode,
    PatrolExecutorLogic,
    can_publish_initial_pose,
    due_interval_schedules,
)
from ylhb_mobile_bridge.patrol_route_store import (
    expand_route_targets,
    get_route,
    load_route_file,
)


TEST_ROUTE_PATH = (
    Path(__file__).resolve().parents[1]
    / "test"
    / "fixtures"
    / "patrol_routes.json"
)


def route_file_data():
    return load_route_file(str(TEST_ROUTE_PATH))


def scenario(**route_overrides):
    data = copy.deepcopy(route_file_data())
    route = data["routes"][0]
    route.update(route_overrides)
    targets = expand_route_targets(data, route["id"])
    start_pose = data["start_pose"]["pose"]
    return route, targets, start_pose, data


def first_target_scenario(**route_overrides):
    data = copy.deepcopy(route_file_data())
    route = data["routes"][0]
    route["target_ids"] = [data["targets"][0]["id"]]
    route.update(route_overrides)
    targets = expand_route_targets(data, route["id"])
    start_pose = data["start_pose"]["pose"]
    return route, targets, start_pose, data


class FakeAdapter:
    def __init__(self, navigation_results=None):
        self.navigation_results = list(navigation_results or [])
        self.navigation_requests = []
        self.cancel_count = 0
        self.stop_count = 0
        self.statuses = []
        self.events = []
        self.text_commands = []
        self.scheduled = []
        self.now = 100.0

    def request_navigation(self, pose, timeout_sec, on_result):
        self.navigation_requests.append((pose, timeout_sec))
        if self.navigation_results:
            on_result(self.navigation_results.pop(0))

    def cancel_navigation(self):
        self.cancel_count += 1

    def stop_motion(self):
        self.stop_count += 1

    def publish_status(self, status):
        self.statuses.append(status.copy())

    def publish_event(self, event):
        self.events.append(event.copy())

    def publish_text_command(self, text):
        self.text_commands.append(text)

    def schedule_once(self, delay_sec, callback):
        self.scheduled.append((delay_sec, callback))

    def time(self):
        return self.now

    def run_next_scheduled(self):
        _delay, callback = self.scheduled.pop(0)
        callback()


def make_logic(adapter):
    return PatrolExecutorLogic(
        request_navigation=adapter.request_navigation,
        cancel_navigation=adapter.cancel_navigation,
        stop_motion=adapter.stop_motion,
        publish_status=adapter.publish_status,
        publish_event=adapter.publish_event,
        publish_text_command=adapter.publish_text_command,
        schedule_once=adapter.schedule_once,
        time_source=adapter.time,
    )


def start(logic, route_data=None, targets=None, start_pose=None):
    default_route, default_targets, default_start_pose, _data = scenario()
    return logic.start_route(
        route_data or default_route,
        targets or default_targets,
        start_pose or default_start_pose,
    )


def finish_all_targets(adapter, targets):
    for _target in targets:
        adapter.run_next_scheduled()


def test_navigation_success_executes_targets_in_order():
    route, targets, start_pose, _data = scenario(return_to_start=False)
    adapter = FakeAdapter([True for _target in targets])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    finish_all_targets(adapter, targets)

    assert [request[0] for request in adapter.navigation_requests] == [
        target["pose"] for target in targets
    ]
    assert logic.state == "succeeded"


def test_target_task_duration_delays_navigation_to_next_target():
    route, targets, start_pose, _data = scenario(return_to_start=False)
    adapter = FakeAdapter([True for _target in targets])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)

    assert adapter.scheduled[0][0] == targets[0]["task_duration_sec"]
    assert len(adapter.navigation_requests) == 1

    adapter.run_next_scheduled()

    assert len(adapter.navigation_requests) == 2


def test_return_to_start_navigates_to_route_file_pose_last():
    route, targets, start_pose, _data = scenario(return_to_start=True)
    adapter = FakeAdapter([True for _target in targets] + [True])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    finish_all_targets(adapter, targets)

    assert adapter.navigation_requests[-1][0] == start_pose
    assert logic.state == "succeeded"


def test_target_events_text_and_status_use_target_semantics():
    route, targets, start_pose, _data = scenario(return_to_start=False)
    adapter = FakeAdapter([True for _target in targets])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    first_status = logic.status()
    finish_all_targets(adapter, targets)

    event_types = [event["event"] for event in adapter.events]
    assert "target_reached" in event_types
    assert "target_task_finished" in event_types
    assert adapter.text_commands[0] == (
        f"已到达{targets[0]['name']}，开始执行任务"
    )
    assert first_status["target_id"] == targets[0]["id"]
    assert first_status["target_index"] == 0
    assert first_status["cycle_index"] == 1
    assert first_status["loop_wait_sec"] == route["loop"]["wait_sec"]
    assert first_status["home_pose_source"] == "route_file"


def test_loop_waits_after_return_then_starts_next_cycle():
    loop = {"enabled": True, "wait_sec": 12.0, "max_cycles": 0}
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=True,
        loop=loop,
    )
    adapter = FakeAdapter([True, True, True, True])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    adapter.run_next_scheduled()

    assert logic.state == "waiting_loop"
    assert adapter.scheduled[0][0] == route["loop"]["wait_sec"]
    assert logic.status()["cycle_index"] == 1

    adapter.run_next_scheduled()

    assert logic.state == "running"
    assert logic.status()["cycle_index"] == 2
    assert [request[0] for request in adapter.navigation_requests] == [
        targets[0]["pose"],
        start_pose,
        targets[0]["pose"],
    ]


def test_max_cycles_one_finishes_after_first_cycle():
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=True,
        loop={"enabled": True, "wait_sec": 12.0, "max_cycles": 1},
    )
    adapter = FakeAdapter([True, True])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    adapter.run_next_scheduled()

    assert logic.state == "succeeded"
    assert adapter.scheduled == []


def test_pause_resume_and_cancel_during_loop_wait():
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=True,
        loop={"enabled": True, "wait_sec": 12.0, "max_cycles": 0},
    )
    adapter = FakeAdapter([True, True])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    adapter.run_next_scheduled()
    assert logic.state == "waiting_loop"

    assert logic.pause()
    assert logic.state == "paused"
    assert logic.resume()
    assert logic.state == "waiting_loop"
    assert logic.cancel()
    assert logic.state == "canceled"

    while adapter.scheduled:
        adapter.run_next_scheduled()

    assert len(adapter.navigation_requests) == 2


def test_cancel_cancels_current_goal_and_does_not_return_to_start():
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=True,
    )
    adapter = FakeAdapter([])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    logic.cancel()

    assert adapter.cancel_count == 1
    assert adapter.stop_count == 1
    assert logic.state == "canceled"
    assert len(adapter.navigation_requests) == 1


def test_pause_then_resume_restarts_current_target():
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=False,
    )
    adapter = FakeAdapter([])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    logic.pause()
    logic.resume()

    assert adapter.cancel_count == 1
    assert adapter.stop_count == 1
    assert logic.state == "running"
    assert len(adapter.navigation_requests) == 2
    assert adapter.navigation_requests[1][0] == targets[0]["pose"]


def test_target_failure_retries_extra_attempts():
    route, targets, start_pose, _data = scenario(
        return_to_start=False,
        max_retries_per_checkpoint=1,
    )
    adapter = FakeAdapter([False] + [True for _target in targets])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)
    finish_all_targets(adapter, targets)

    assert [request[0] for request in adapter.navigation_requests] == [
        targets[0]["pose"],
        targets[0]["pose"],
        targets[1]["pose"],
    ]
    assert logic.state == "succeeded"


def test_abort_and_return_home_attempts_start_pose_and_finally_fails():
    route, targets, start_pose, _data = first_target_scenario(
        return_to_start=False,
        failure_policy="abort_and_return_home",
        max_retries_per_checkpoint=0,
    )
    adapter = FakeAdapter([False, True])
    logic = make_logic(adapter)

    start(logic, route, targets, start_pose)

    assert adapter.navigation_requests[-1][0] == start_pose
    assert logic.state == "failed"


def test_interval_schedule_triggers_only_when_idle_or_waiting():
    data = route_file_data()
    schedules = copy.deepcopy(data["schedules"])
    schedules[0]["enabled"] = True
    schedules[0]["period_sec"] = 10.0
    clock = PatchedScheduleClock({schedules[0]["id"]: 100.0})

    assert due_interval_schedules(
        schedules,
        clock,
        111.0,
        "idle",
    ) == [schedules[0]]
    assert due_interval_schedules(schedules, clock, 111.0, "running") == []


def test_can_publish_initial_pose_only_when_navigation_not_active():
    assert can_publish_initial_pose("idle")
    assert can_publish_initial_pose("waiting_schedule")
    assert can_publish_initial_pose("failed")
    assert not can_publish_initial_pose("running")
    assert not can_publish_initial_pose("paused")
    assert not can_publish_initial_pose("returning_home")
    assert not can_publish_initial_pose("waiting_loop")


def test_initial_pose_publisher_uses_transient_local_reliable_qos():
    qos = patrol_executor_node.initial_pose_qos_profile()

    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result


class FakeGoalHandle:
    accepted = True

    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def test_late_goal_acceptance_is_canceled_after_pending_request_cancel():
    node = PatrolExecutorNode.__new__(PatrolExecutorNode)
    goal_handle = FakeGoalHandle()
    context = {"completed": True, "canceled": True}

    node._on_goal_response(context, FakeFuture(goal_handle))

    assert goal_handle.cancel_count == 1


def test_start_route_uses_start_pose_from_reloaded_route_file():
    data = route_file_data()
    route = get_route(data, data["active_route_id"])
    targets = expand_route_targets(data, route["id"])
    node = PatrolExecutorNode.__new__(PatrolExecutorNode)
    node._route_data = data
    node._reload_route_file = lambda: True
    started = []
    node.logic = type(
        "Logic",
        (),
        {
            "state": "idle",
            "start_route": lambda _self, selected, expanded, start_pose: (
                started.append((selected, expanded, start_pose)) or True
            ),
            "fail_to_start": lambda *_args: None,
        },
    )()

    assert node._start_route_from_file(None)
    assert started[0] == (route, targets, data["start_pose"]["pose"])


def test_restarting_initial_pose_sequence_destroys_previous_timer():
    data = route_file_data()
    node = PatrolExecutorNode.__new__(PatrolExecutorNode)
    node._route_data = data
    node._initial_pose_timer = "old_timer"
    destroyed = []
    node.destroy_timer = destroyed.append
    node.get_parameter = lambda name: type(
        "Parameter",
        (),
        {"value": 3 if name == "initial_pose_publish_count" else 0.5},
    )()
    node._publish_one_initial_pose = lambda: setattr(
        node,
        "_initial_pose_remaining",
        node._initial_pose_remaining - 1,
    )
    node.create_timer = lambda _period, _callback: "new_timer"

    assert node._publish_initial_pose_from_route()
    assert destroyed == ["old_timer"]
    assert node._initial_pose_timer == "new_timer"


def test_auto_start_waits_for_initial_pose_sequence_completion():
    node = PatrolExecutorNode.__new__(PatrolExecutorNode)
    node._startup_timer = "startup_timer"
    node._auto_start_after_initial_pose = False
    destroyed = []
    started = []
    node.destroy_timer = destroyed.append
    node.get_parameter = lambda name: type(
        "Parameter",
        (),
        {
            "value": (
                True
                if name
                in {"publish_initial_pose_on_startup", "auto_start"}
                else None
            )
        },
    )()
    node._publish_initial_pose_from_route = lambda: True
    node._start_route_from_file = lambda route_id: started.append(route_id)

    node._on_startup_timer()

    assert started == []
    assert node._auto_start_after_initial_pose

    node._finish_initial_pose_sequence()

    assert started == [None]
