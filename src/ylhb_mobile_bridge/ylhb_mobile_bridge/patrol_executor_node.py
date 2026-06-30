from __future__ import annotations

import json
import math
import time
from typing import Any, Callable, Dict, List, Optional

from .patrol_route_store import (
    expand_route_targets,
    get_route,
    load_route_file,
    resolve_route_file_path,
)
from .patrol_qos import patrol_status_qos_profile

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:
    # Allows pure logic tests outside a sourced ROS environment.
    rclpy = None
    GoalStatus = None
    Node = object


ACTIVE_STATES = {
    "running",
    "paused",
    "returning_home",
    "waiting_loop",
    "canceling",
}
INITIAL_POSE_ALLOWED_STATES = {
    "idle",
    "waiting_schedule",
    "failed",
    "succeeded",
    "canceled",
}
TERMINAL_STATES = {"succeeded", "failed", "canceled"}


def initial_pose_qos_profile() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class PatchedScheduleClock:
    def __init__(
        self,
        last_started: Optional[Dict[str, float]] = None,
    ) -> None:
        self._last_started = dict(last_started or {})

    def get(self, schedule_id: str) -> Optional[float]:
        return self._last_started.get(schedule_id)

    def mark_started(self, schedule_id: str, started_at: float) -> None:
        self._last_started[schedule_id] = started_at


def due_interval_schedules(
    schedules: List[Dict[str, Any]],
    clock: PatchedScheduleClock,
    now: float,
    state: str,
) -> List[Dict[str, Any]]:
    if state not in {"idle", "waiting_schedule"}:
        return []
    due = []
    for schedule in schedules:
        if not schedule.get("enabled") or schedule.get("mode") != "interval":
            continue
        last_started = clock.get(schedule["id"])
        if (
            last_started is None
            or now - last_started >= float(schedule["period_sec"])
        ):
            due.append(schedule)
    return due


def can_publish_initial_pose(state: str) -> bool:
    return state in INITIAL_POSE_ALLOWED_STATES


class PatrolExecutorLogic:
    def __init__(
        self,
        request_navigation: Callable[
            [Dict[str, float], float, Callable[[bool], None]],
            None,
        ],
        cancel_navigation: Callable[[], None],
        stop_motion: Callable[[], None],
        publish_status: Callable[[Dict[str, Any]], None],
        publish_event: Callable[[Dict[str, Any]], None],
        publish_text_command: Callable[[str], None],
        schedule_once: Callable[[float, Callable[[], None]], None],
        time_source: Callable[[], float] = time.time,
    ) -> None:
        self._request_navigation = request_navigation
        self._cancel_navigation = cancel_navigation
        self._stop_motion = stop_motion
        self._publish_status = publish_status
        self._publish_event = publish_event
        self._publish_text_command = publish_text_command
        self._schedule_once = schedule_once
        self._time_source = time_source

        self.state = "idle"
        self.route: Optional[Dict[str, Any]] = None
        self.targets: List[Dict[str, Any]] = []
        self.home_pose: Optional[Dict[str, float]] = None
        self.current_target_index = 0
        self.cycle_index = 1
        self._retry_count = 0
        self._navigation_token = 0
        self._navigation_purpose: Optional[str] = None
        self._return_home_then_fail = False
        self._loop_wait_token = 0
        self._loop_wait_started_at: Optional[float] = None
        self._loop_wait_until: Optional[float] = None
        self._paused_from_state: Optional[str] = None
        self.last_error: Optional[str] = None

    def status(self) -> Dict[str, Any]:
        target = self._current_target()
        loop = self.route.get("loop", {}) if self.route else {}
        loop_enabled = bool(loop.get("enabled"))
        loop_max_cycles = int(loop.get("max_cycles", 0)) if loop else 0
        loop_wait_sec = None
        if loop:
            loop_wait_sec = float(loop.get("wait_sec", 0.0))
        loop_wait_remaining_sec = None
        if self.state == "waiting_loop" and self._loop_wait_until is not None:
            loop_wait_remaining_sec = max(
                0,
                int(math.ceil(self._loop_wait_until - self._time_source())),
            )
        navigation_phase = self._navigation_phase()
        current_target_label = self._current_target_label(target, navigation_phase, len(self.targets))
        return {
            "state": self.state,
            "route_id": self.route["id"] if self.route else None,
            "target_id": target["id"] if target else None,
            "target_name": target["name"] if target else None,
            "target_index": (
                self.current_target_index if target is not None else None
            ),
            "target_count": len(self.targets),
            "navigation_phase": navigation_phase,
            "current_target_label": current_target_label,
            "cycle_index": self.cycle_index if self.route else None,
            "loop_wait_sec": loop_wait_sec,
            "loop_enabled": loop_enabled,
            "loop_max_cycles": loop_max_cycles,
            "loop_is_infinite": loop_enabled and loop_max_cycles == 0,
            "next_cycle_index": self.cycle_index + 1 if self.route and loop_enabled else None,
            "loop_wait_remaining_sec": loop_wait_remaining_sec,
            "loop_wait_until": self._loop_wait_until if self.state == "waiting_loop" else None,
            "home_pose_source": "route_file" if self.home_pose else None,
            "last_error": self.last_error,
            "timestamp": self._time_source(),
        }

    def _navigation_phase(self) -> str:
        if self.state == "running":
            return "target"
        if self.state == "returning_home":
            return "return_home"
        if self.state == "waiting_loop":
            return "waiting_next_cycle"
        return self.state

    def _current_target_label(
        self,
        target: Optional[Dict[str, Any]],
        navigation_phase: str,
        target_count: int,
    ) -> str:
        if navigation_phase == "target" and target:
            name = str(target.get("name") or target.get("id") or "")
            return f"第 {self.current_target_index + 1} / {target_count} 个检查点：{name}"
        if navigation_phase == "failed":
            return f"巡逻失败：{self.last_error}" if self.last_error else "巡逻失败"
        labels = {
            "return_home": "返回初始点",
            "waiting_next_cycle": "等待下一轮",
            "canceling": "正在取消",
            "canceled": "已取消",
            "succeeded": "巡逻完成",
            "idle": "待命",
        }
        if navigation_phase in labels:
            return labels[navigation_phase]
        if target:
            return str(target.get("name") or target.get("id") or "")
        return ""

    def _set_state(self, state: str, error: Optional[str] = None) -> None:
        self.state = state
        self.last_error = error
        self._publish_status(self.status())

    def _event(self, event: str, **details: Any) -> None:
        payload = {
            "event": event,
            "route_id": self.route["id"] if self.route else None,
            "timestamp": self._time_source(),
        }
        payload.update(details)
        self._publish_event(payload)

    def start_route(
        self,
        route: Dict[str, Any],
        targets: List[Dict[str, Any]],
        home_pose: Dict[str, float],
    ) -> bool:
        if self.state in ACTIVE_STATES:
            return False
        self.route = route
        self.targets = targets
        self.home_pose = dict(home_pose)
        self.current_target_index = 0
        self.cycle_index = 1
        self._retry_count = 0
        self._return_home_then_fail = False
        self._loop_wait_token += 1
        self._clear_loop_wait()
        self._paused_from_state = None
        self.last_error = None
        self._set_state("running")
        self._event("route_started")
        if not self.targets:
            self._complete_targets()
        else:
            self._navigate_current_target()
        return True

    def fail_to_start(self, route_id: Optional[str], message: str) -> None:
        self.route = {"id": route_id} if route_id else None
        self.targets = []
        self.home_pose = None
        self._event("route_failed", reason=message)
        self._set_state("failed", message)

    def reset_to_idle(self, waiting_schedule: bool = False) -> None:
        if self.state not in TERMINAL_STATES and self.state not in {
            "idle",
            "waiting_schedule",
        }:
            return
        self.route = None
        self.targets = []
        self.home_pose = None
        self.current_target_index = 0
        self.cycle_index = 1
        self._clear_loop_wait()
        self._set_state("waiting_schedule" if waiting_schedule else "idle")

    def pause(self) -> bool:
        if self.state not in {"running", "returning_home", "waiting_loop"}:
            return False
        self._paused_from_state = self.state
        self._navigation_token += 1
        self._loop_wait_token += 1
        self._clear_loop_wait()
        if self.state != "waiting_loop":
            self._cancel_navigation()
        self._stop_motion()
        self._set_state("paused")
        return True

    def resume(self) -> bool:
        if self.state != "paused":
            return False
        self._retry_count = 0
        paused_from_state = self._paused_from_state
        self._paused_from_state = None
        if paused_from_state == "waiting_loop":
            self._start_loop_wait()
        elif paused_from_state == "returning_home":
            self._start_return_home(self._return_home_then_fail)
        elif self.current_target_index < len(self.targets):
            self._set_state("running")
            self._navigate_current_target()
        elif self.route and self.route.get("return_to_start"):
            self._start_return_home(self._return_home_then_fail)
        else:
            self._complete_success()
        return True

    def cancel(self) -> bool:
        if self.state not in {
            "running",
            "paused",
            "returning_home",
            "waiting_loop",
        }:
            return False
        self._navigation_token += 1
        self._loop_wait_token += 1
        self._clear_loop_wait()
        self._set_state("canceling")
        self._cancel_navigation()
        self._stop_motion()
        self._set_state("canceled")
        return True

    def _current_target(self) -> Optional[Dict[str, Any]]:
        if 0 <= self.current_target_index < len(self.targets):
            return self.targets[self.current_target_index]
        return None

    def _navigate_current_target(self) -> None:
        target = self._current_target()
        if target is None or self.route is None:
            self._complete_targets()
            return
        self._set_state("running")
        self._start_navigation(
            target["pose"],
            float(self.route["goal_timeout_sec"]),
            "target",
        )

    def _start_navigation(
        self,
        pose: Dict[str, float],
        timeout_sec: float,
        purpose: str,
    ) -> None:
        self._navigation_token += 1
        token = self._navigation_token
        self._navigation_purpose = purpose
        self._request_navigation(
            dict(pose),
            timeout_sec,
            lambda success: self._navigation_finished(token, success),
        )

    def _navigation_finished(self, token: int, success: bool) -> None:
        if token != self._navigation_token:
            return
        if self.state in {"paused", "canceling", "canceled"}:
            return
        if self._navigation_purpose == "home":
            if self._return_home_then_fail:
                self._finish_failure("target navigation failed")
            elif success:
                self._complete_cycle_success()
            else:
                self._finish_failure("return home navigation failed")
            return
        if success:
            self._target_reached()
        else:
            self._target_failed()

    def _target_reached(self) -> None:
        target = self._current_target()
        if target is None:
            return
        self._event(
            "target_reached",
            target_id=target["id"],
            target_name=target["name"],
        )
        self._publish_text_command(
            f"已到达{target['name']}，开始执行任务"
        )
        self._schedule_once(
            float(target.get("task_duration_sec", 0.0)),
            self._target_task_finished,
        )

    def _target_task_finished(self) -> None:
        if self.state != "running":
            return
        target = self._current_target()
        if target is None:
            return
        self._event(
            "target_task_finished",
            target_id=target["id"],
            target_name=target["name"],
        )
        self.current_target_index += 1
        self._retry_count = 0
        if self.current_target_index >= len(self.targets):
            self._complete_targets()
        else:
            self._navigate_current_target()

    def _target_failed(self) -> None:
        if self.route is None:
            return
        max_retries = int(self.route["max_retries_per_checkpoint"])
        if self._retry_count < max_retries:
            self._retry_count += 1
            self._navigate_current_target()
            return
        if self.route["failure_policy"] == "abort_and_return_home":
            self._start_return_home(then_fail=True)
        else:
            self._finish_failure("target navigation failed")

    def _complete_targets(self) -> None:
        if self.route and self.route.get("return_to_start"):
            self._start_return_home(then_fail=False)
        else:
            self._complete_success()

    def _start_return_home(self, then_fail: bool) -> None:
        if self.route is None or self.home_pose is None:
            self._finish_failure("home pose is unavailable")
            return
        self._return_home_then_fail = then_fail
        self._set_state("returning_home")
        self._event("return_home_started", after_failure=then_fail)
        self._start_navigation(
            self.home_pose,
            float(self.route["goal_timeout_sec"]),
            "home",
        )

    def _complete_cycle_success(self) -> None:
        if self._should_continue_loop():
            self._start_loop_wait()
        else:
            self._complete_success()

    def _should_continue_loop(self) -> bool:
        if self.route is None:
            return False
        loop = self.route.get("loop", {})
        if not loop.get("enabled"):
            return False
        max_cycles = int(loop.get("max_cycles", 0))
        return max_cycles == 0 or self.cycle_index < max_cycles

    def _start_loop_wait(self) -> None:
        if self.route is None:
            self._complete_success()
            return
        self._loop_wait_token += 1
        token = self._loop_wait_token
        wait_sec = float(self.route["loop"]["wait_sec"])
        self._loop_wait_started_at = self._time_source()
        self._loop_wait_until = self._loop_wait_started_at + wait_sec
        self._set_state("waiting_loop")
        self._schedule_once(
            wait_sec,
            lambda: self._loop_wait_finished(token),
        )

    def _loop_wait_finished(self, token: int) -> None:
        if token != self._loop_wait_token or self.state != "waiting_loop":
            return
        self._clear_loop_wait()
        self.cycle_index += 1
        self.current_target_index = 0
        self._retry_count = 0
        if not self.targets:
            self._complete_targets()
        else:
            self._navigate_current_target()

    def _complete_success(self) -> None:
        self._event("route_finished", result="succeeded")
        self._clear_loop_wait()
        self._set_state("succeeded")

    def _finish_failure(self, message: str) -> None:
        self._event("route_failed", reason=message)
        self._clear_loop_wait()
        self._set_state("failed", message)

    def _clear_loop_wait(self) -> None:
        self._loop_wait_started_at = None
        self._loop_wait_until = None


class PatrolExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_executor")
        self._declare_parameters()
        self.route_file_path = str(self.get_parameter("route_file_path").value)
        self.resolved_route_file_path: Optional[str] = None
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.text_command_topic = str(
            self.get_parameter("text_command_topic").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._route_data: Optional[Dict[str, Any]] = None
        self._schedule_clock = PatchedScheduleClock()
        self._active_goal_handle = None
        self._active_navigation: Optional[Dict[str, Any]] = None
        self._navigation_request_id = 0
        self._terminal_reset_timer = None
        self._initial_pose_timer = None
        self._initial_pose_remaining = 0
        self._auto_start_after_initial_pose = False

        self._status_pub = self.create_publisher(
            String,
            self.status_topic,
            patrol_status_qos_profile(),
        )
        self._event_pub = self.create_publisher(
            String,
            self.event_topic,
            patrol_status_qos_profile(),
        )
        self._text_pub = self.create_publisher(
            String,
            self.text_command_topic,
            10,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            initial_pose_qos_profile(),
        )
        self.create_subscription(
            String,
            self.command_topic,
            self._on_command,
            10,
        )
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )
        self.logic = PatrolExecutorLogic(
            request_navigation=self._request_navigation,
            cancel_navigation=self._cancel_navigation,
            stop_motion=self._stop_motion,
            publish_status=self._publish_status,
            publish_event=self._publish_event,
            publish_text_command=self._publish_text_command,
            schedule_once=self._schedule_once,
        )

        self._reload_route_file(log_errors=False)
        schedule_period = float(
            self.get_parameter("schedule_check_period_sec").value
        )
        self.create_timer(schedule_period, self._check_schedules)
        self.create_timer(1.0, self._publish_current_status)
        self._startup_timer = self.create_timer(
            float(self.get_parameter("initial_pose_startup_delay_sec").value),
            self._on_startup_timer,
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "route_file_path": "auto",
            "command_topic": "/patrol/command",
            "status_topic": "/patrol/status",
            "event_topic": "/patrol/event",
            "text_command_topic": "/inspection_ai/text_command",
            "map_frame": "map",
            "cmd_vel_topic": "/cmd_vel",
            "auto_start": False,
            "schedule_check_period_sec": 1.0,
            "publish_initial_pose_on_startup": True,
            "initial_pose_publish_count": 3,
            "initial_pose_publish_period_sec": 0.5,
            "initial_pose_startup_delay_sec": 2.0,
            "initial_pose_stamp_zero": True,
            "nav2_action_wait_timeout_sec": 30.0,
            "nav2_action_wait_retry_period_sec": 1.0,
            "nav2_goal_reject_retry_delay_sec": 2.0,
            "nav2_goal_reject_max_retries": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _publish_json(self, publisher, payload: Dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)

    def _publish_status(self, status: Dict[str, Any]) -> None:
        status = {**status, **self._navigation_status_fields()}
        self._publish_json(self._status_pub, status)
        if (
            status["state"] in TERMINAL_STATES
            and self._terminal_reset_timer is None
        ):
            self._terminal_reset_timer = self.create_timer(
                1.0,
                self._reset_terminal_state,
            )

    def _publish_current_status(self) -> None:
        self._publish_status(self.logic.status())

    def _navigation_status_fields(self) -> Dict[str, Any]:
        context = self._active_navigation
        if not context or context.get("completed"):
            return {}
        phase = str(context.get("navigation_phase") or "")
        fields: Dict[str, Any] = {
            "navigation_phase": phase,
            "nav2_action_ready": bool(context.get("nav2_action_ready")),
            "nav2_wait_elapsed_sec": max(
                0.0,
                time.time()
                - float(context.get("nav2_wait_started_at") or time.time()),
            ),
            "goal_reject_count": int(context.get("goal_reject_count") or 0),
            "active_navigation_id": context.get("id"),
        }
        labels = {
            "waiting_nav2": "等待导航服务",
            "sending_goal": "发送导航目标",
            "retrying_goal": "导航目标重试",
        }
        if phase in labels:
            fields["current_target_label"] = labels[phase]
        if context.get("last_error"):
            fields["last_error"] = context["last_error"]
        return fields

    def _publish_event(self, event: Dict[str, Any]) -> None:
        self._publish_json(self._event_pub, event)

    def _publish_text_command(self, text: str) -> None:
        message = String()
        message.data = text
        self._text_pub.publish(message)

    def _stop_motion(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _schedule_once(
        self,
        delay_sec: float,
        callback: Callable[[], None],
    ) -> None:
        holder = {}

        def run_once() -> None:
            timer = holder.get("timer")
            if timer is not None:
                self.destroy_timer(timer)
            callback()

        holder["timer"] = self.create_timer(max(delay_sec, 0.001), run_once)

    def _reset_terminal_state(self) -> None:
        if self._terminal_reset_timer is not None:
            self.destroy_timer(self._terminal_reset_timer)
            self._terminal_reset_timer = None
        waiting = bool(
            self._route_data and self._route_data.get("schedules")
        )
        self.logic.reset_to_idle(waiting_schedule=waiting)

    def _reload_route_file(self, log_errors: bool = True) -> bool:
        try:
            resolved_path = resolve_route_file_path(self.route_file_path)
            self._route_data = load_route_file(str(resolved_path))
            self.resolved_route_file_path = str(resolved_path)
            self.get_logger().info(
                f"Loaded patrol routes from {resolved_path}"
            )
            return True
        except ValueError as exc:
            self._route_data = None
            self.resolved_route_file_path = None
            if log_errors:
                self.get_logger().error(str(exc))
            else:
                self.get_logger().warning(str(exc))
            return False

    def _on_startup_timer(self) -> None:
        self.destroy_timer(self._startup_timer)
        auto_start = bool(self.get_parameter("auto_start").value)
        self._auto_start_after_initial_pose = auto_start
        initial_pose_started = False
        if bool(
            self.get_parameter("publish_initial_pose_on_startup").value
        ):
            initial_pose_started = self._publish_initial_pose_from_route()
        if auto_start and not initial_pose_started:
            self._auto_start_after_initial_pose = False
            self._start_route_from_file(None)

    def _parse_command(self, text: str) -> Dict[str, Any]:
        stripped = text.strip()
        if not stripped:
            raise ValueError("empty patrol command")
        if stripped.startswith("{"):
            payload = json.loads(stripped)
            if not isinstance(payload, dict):
                raise ValueError("patrol command JSON must be an object")
            return payload
        return {"command": stripped}

    def _on_command(self, message: String) -> None:
        try:
            payload = self._parse_command(message.data)
            command = str(payload.get("command", "")).strip().lower()
            route_id = payload.get("route_id")
            if command == "start":
                self._start_route_from_file(route_id)
            elif command == "pause":
                if not self.logic.pause():
                    raise ValueError(f"cannot pause while {self.logic.state}")
            elif command == "resume":
                if not self.logic.resume():
                    raise ValueError(f"cannot resume while {self.logic.state}")
            elif command == "cancel":
                if not self.logic.cancel():
                    raise ValueError(f"cannot cancel while {self.logic.state}")
            elif command == "reload":
                if not self._reload_route_file():
                    raise ValueError("route file reload failed")
            elif command in {"initialize", "relocalize"}:
                if not can_publish_initial_pose(self.logic.state):
                    raise ValueError(
                        f"cannot publish initial pose while {self.logic.state}"
                    )
                if not self._reload_route_file():
                    raise ValueError("route file reload failed")
                if not self._publish_initial_pose_from_route():
                    raise ValueError("enabled start_pose is unavailable")
            else:
                raise ValueError(f"unknown patrol command: {command}")
        except (ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(str(exc))

    def _start_route_from_file(self, route_id: Optional[str]) -> bool:
        if self.logic.state in ACTIVE_STATES:
            self.get_logger().warning(
                f"Cannot start route while {self.logic.state}"
            )
            return False
        if not self._reload_route_file():
            self.logic.fail_to_start(route_id, "route file load failed")
            return False
        selected_route_id = route_id or self._route_data.get("active_route_id")
        if not selected_route_id:
            self.logic.fail_to_start(None, "route_id is required")
            return False
        try:
            route = get_route(self._route_data, selected_route_id)
            targets = expand_route_targets(
                self._route_data,
                selected_route_id,
            )
            home_pose = self._route_data["start_pose"]["pose"]
        except ValueError as exc:
            self.logic.fail_to_start(selected_route_id, str(exc))
            return False
        started = self.logic.start_route(route, targets, home_pose)
        return started

    def _request_navigation(
        self,
        pose: Dict[str, float],
        timeout_sec: float,
        on_result: Callable[[bool], None],
    ) -> None:
        self._navigation_request_id += 1
        request_id = self._navigation_request_id
        context = {
            "id": request_id,
            "callback": on_result,
            "completed": False,
            "canceled": False,
            "pose": dict(pose),
            "timeout_sec": timeout_sec,
            "nav2_wait_started_at": time.time(),
            "nav2_wait_deadline": time.time()
            + float(self.get_parameter("nav2_action_wait_timeout_sec").value),
            "goal_reject_count": 0,
            "wait_timer": None,
            "retry_timer": None,
            "timeout_timer": None,
            "navigation_phase": "waiting_nav2",
            "nav2_action_ready": False,
            "last_error": None,
        }
        self._active_navigation = context
        self._active_goal_handle = None
        self._publish_current_status()
        context["wait_timer"] = self.create_timer(
            float(
                self.get_parameter(
                    "nav2_action_wait_retry_period_sec"
                ).value
            ),
            lambda: self._check_nav2_ready(context),
        )
        context["timeout_timer"] = self.create_timer(
            float(self.get_parameter("nav2_action_wait_timeout_sec").value),
            lambda: self._on_nav2_wait_timeout(context),
        )
        self._check_nav2_ready(context)

    def _nav2_server_ready(self) -> bool:
        server_is_ready = getattr(self._nav_client, "server_is_ready", None)
        if callable(server_is_ready) and server_is_ready():
            return True
        return bool(self._nav_client.wait_for_server(timeout_sec=0.1))

    def _check_nav2_ready(self, context: Dict[str, Any]) -> None:
        if context["completed"] or context["canceled"]:
            return
        if not self._nav2_server_ready():
            self._publish_current_status()
            return
        context["nav2_action_ready"] = True
        self._send_navigation_goal(context)

    def _send_navigation_goal(self, context: Dict[str, Any]) -> None:
        if context["completed"] or context["canceled"]:
            return
        for name in ("wait_timer", "retry_timer", "timeout_timer"):
            timer = context.get(name)
            if timer is not None:
                self.destroy_timer(timer)
                context[name] = None
        context["timeout_timer"] = self.create_timer(
            float(context["timeout_sec"]),
            lambda: self._on_navigation_timeout(context),
        )
        context["navigation_phase"] = "sending_goal"
        self._publish_current_status()
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(context["pose"])
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(context, future)
        )

    def _on_nav2_wait_timeout(self, context: Dict[str, Any]) -> None:
        if context["completed"]:
            return
        wait_timeout = float(
            self.get_parameter("nav2_action_wait_timeout_sec").value
        )
        message = f"navigate_to_pose action server unavailable after {wait_timeout:g}s"
        context["last_error"] = message
        if hasattr(self.logic, "last_error"):
            self.logic.last_error = message
        self.get_logger().error(message)
        self._finish_navigation(context, False)

    def _pose_stamped(self, pose: Dict[str, float]) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(pose["x"])
        message.pose.position.y = float(pose["y"])
        message.pose.orientation.z = math.sin(float(pose["yaw"]) / 2.0)
        message.pose.orientation.w = math.cos(float(pose["yaw"]) / 2.0)
        return message

    def _on_goal_response(self, context: Dict[str, Any], future) -> None:
        if context["completed"]:
            if context.get("canceled"):
                try:
                    goal_handle = future.result()
                except Exception:
                    return
                if goal_handle is not None and goal_handle.accepted:
                    goal_handle.cancel_goal_async()
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"navigation goal request failed: {exc}")
            self._finish_navigation(context, False)
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(
                f"navigation goal rejected phase={context.get('navigation_phase')}"
            )
            self._retry_rejected_goal(context)
            return
        if context["canceled"]:
            goal_handle.cancel_goal_async()
            self._finish_navigation(context, False, notify=False)
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_navigation_result(context, result)
        )

    def _retry_rejected_goal(self, context: Dict[str, Any]) -> None:
        max_retries = int(
            self.get_parameter("nav2_goal_reject_max_retries").value
        )
        if int(context["goal_reject_count"]) >= max_retries:
            self._finish_navigation(context, False)
            return
        context["goal_reject_count"] = int(context["goal_reject_count"]) + 1
        context["navigation_phase"] = "retrying_goal"
        self._publish_current_status()
        context["retry_timer"] = self.create_timer(
            float(
                self.get_parameter("nav2_goal_reject_retry_delay_sec").value
            ),
            lambda: self._send_navigation_goal(context),
        )

    def _on_navigation_result(self, context: Dict[str, Any], future) -> None:
        if context["completed"]:
            return
        try:
            wrapped_result = future.result()
            succeeded = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:
            self.get_logger().error(f"navigation result failed: {exc}")
            succeeded = False
        self._finish_navigation(context, succeeded)

    def _on_navigation_timeout(self, context: Dict[str, Any]) -> None:
        if context["completed"]:
            return
        self.get_logger().warning("navigation goal timed out")
        context["canceled"] = True
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
        self._stop_motion()
        self._finish_navigation(context, False)

    def _finish_navigation(
        self,
        context: Dict[str, Any],
        success: bool,
        notify: bool = True,
    ) -> None:
        if context["completed"]:
            return
        context["completed"] = True
        for name in ("wait_timer", "retry_timer", "timeout_timer"):
            timer = context.get(name)
            if timer is not None:
                self.destroy_timer(timer)
                context[name] = None
        if self._active_navigation is context:
            self._active_navigation = None
            self._active_goal_handle = None
        if notify:
            context["callback"](success)

    def _cancel_navigation(self) -> None:
        context = self._active_navigation
        if context is None:
            return
        context["canceled"] = True
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
        self._finish_navigation(context, False, notify=False)

    def _publish_initial_pose_from_route(self) -> bool:
        if not self._route_data:
            return False
        start_pose = self._route_data.get("start_pose")
        if not start_pose or not start_pose.get("publish_initial_pose"):
            return False
        if self._initial_pose_timer is not None:
            self.destroy_timer(self._initial_pose_timer)
            self._initial_pose_timer = None
        self._initial_pose_remaining = max(
            1,
            int(self.get_parameter("initial_pose_publish_count").value),
        )
        self._publish_one_initial_pose()
        if self._initial_pose_remaining > 0:
            period = float(
                self.get_parameter(
                    "initial_pose_publish_period_sec"
                ).value
            )
            self._initial_pose_timer = self.create_timer(
                period,
                self._publish_one_initial_pose,
            )
        return True

    def _publish_one_initial_pose(self) -> None:
        if self._initial_pose_remaining <= 0 or not self._route_data:
            if self._initial_pose_timer is not None:
                self.destroy_timer(self._initial_pose_timer)
                self._initial_pose_timer = None
            return
        start_pose = self._route_data["start_pose"]
        pose = start_pose["pose"]
        covariance = start_pose["covariance"]
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        stamp_zero = bool(self.get_parameter("initial_pose_stamp_zero").value)
        message.header.stamp = (
            Time(sec=0, nanosec=0)
            if stamp_zero
            else self.get_clock().now().to_msg()
        )
        message.pose.pose.position.x = pose["x"]
        message.pose.pose.position.y = pose["y"]
        message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        message.pose.covariance[0] = covariance["x"]
        message.pose.covariance[7] = covariance["y"]
        message.pose.covariance[35] = covariance["yaw"]
        self._initial_pose_pub.publish(message)
        self._initial_pose_remaining -= 1
        self._publish_event(
            {
                "event": "initial_pose_published",
                "remaining": self._initial_pose_remaining,
                "stamp_zero": stamp_zero,
                "timestamp": time.time(),
            }
        )
        if (
            self._initial_pose_remaining <= 0
            and self._initial_pose_timer is not None
        ):
            self.destroy_timer(self._initial_pose_timer)
            self._initial_pose_timer = None
        if self._initial_pose_remaining <= 0:
            self._finish_initial_pose_sequence()

    def _finish_initial_pose_sequence(self) -> None:
        if self._auto_start_after_initial_pose:
            self._auto_start_after_initial_pose = False
            self._start_route_from_file(None)

    def _check_schedules(self) -> None:
        if not self._route_data:
            return
        now = time.time()
        due = due_interval_schedules(
            self._route_data.get("schedules", []),
            self._schedule_clock,
            now,
            self.logic.state,
        )
        if not due:
            return
        schedule = due[0]
        if self._start_route_from_file(schedule["route_id"]):
            self._schedule_clock.mark_started(schedule["id"], now)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 Python dependencies are unavailable")
    rclpy.init(args=args)
    node = PatrolExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
