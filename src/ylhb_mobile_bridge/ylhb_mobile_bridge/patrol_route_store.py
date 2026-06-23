import copy
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Union


FAILURE_POLICIES = {"abort", "abort_and_return_home"}
SCHEDULE_MODES = {"interval", "daily"}
DEFAULT_ROUTE_DIRECTORY = Path("/home/nvidia/ros2_DL/maps")
ROUTE_FILE_PATTERN = "route_patrol_*.json"
ROUTE_NUMBER_PATTERN = re.compile(r"^route_patrol_(\d+)\.json$")


def _require_dict(value: Any, field: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _require_list(value: Any, field: str) -> List[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{field} must be an array")
    return value


def _require_id(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _require_bool(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _require_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field} must be finite")
    return number


def _validate_pose(value: Any, field: str) -> Dict[str, float]:
    pose = _require_dict(value, field)
    return {
        axis: _require_number(pose.get(axis), f"{field}.{axis}")
        for axis in ("x", "y", "yaw")
    }


def _validate_nonnegative(value: Any, field: str) -> float:
    number = _require_number(value, field)
    if number < 0.0:
        raise ValueError(f"{field} must be >= 0")
    return number


def _validate_positive(value: Any, field: str) -> float:
    number = _require_number(value, field)
    if number <= 0.0:
        raise ValueError(f"{field} must be > 0")
    return number


def _validate_nonnegative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be an integer >= 0")
    return value


def resolve_route_file_path(
    route_file_path: str,
    route_directory: Union[str, Path] = DEFAULT_ROUTE_DIRECTORY,
) -> Path:
    requested_path = str(route_file_path).strip()
    if requested_path != "auto":
        explicit_path = Path(requested_path).expanduser()
        if not explicit_path.is_absolute():
            raise ValueError(
                "route_file_path must be 'auto' or an absolute path"
            )
        return explicit_path

    directory = Path(route_directory).expanduser()
    candidates = list(directory.glob(ROUTE_FILE_PATTERN))
    if not candidates:
        raise ValueError(
            f"no patrol route files found in {directory} "
            f"matching {ROUTE_FILE_PATTERN}"
        )

    numbered_candidates = []
    for candidate in candidates:
        match = ROUTE_NUMBER_PATTERN.match(candidate.name)
        if match:
            numbered_candidates.append((int(match.group(1)), candidate))

    if numbered_candidates:
        return max(
            numbered_candidates,
            key=lambda item: (
                item[0],
                item[1].stat().st_mtime,
                item[1].name,
            ),
        )[1]

    return max(
        candidates,
        key=lambda candidate: (
            candidate.stat().st_mtime,
            candidate.name,
        ),
    )


def load_route_file(path: str) -> Dict[str, Any]:
    route_path = Path(path).expanduser()
    try:
        with route_path.open("r", encoding="utf-8") as route_file:
            data = json.load(route_file)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(
            f"failed to load route file {route_path}: {exc}"
        ) from exc
    return validate_route_file(data)


def validate_route_file(data: Any) -> Dict[str, Any]:
    source = _require_dict(data, "route file")
    normalized = copy.deepcopy(source)

    version = normalized.get("version")
    if isinstance(version, bool) or version != 2:
        raise ValueError("version must be 2")
    if normalized.get("frame_id") != "map":
        raise ValueError('frame_id must be "map"')

    start_pose = _require_dict(normalized.get("start_pose"), "start_pose")
    start_name = start_pose.get("name", "start")
    if not isinstance(start_name, str) or not start_name.strip():
        raise ValueError("start_pose.name must be a non-empty string")
    publish_initial_pose = _require_bool(
        start_pose.get("publish_initial_pose", False),
        "start_pose.publish_initial_pose",
    )
    normalized_start_pose = {
        **start_pose,
        "name": start_name,
        "pose": _validate_pose(start_pose.get("pose"), "start_pose.pose"),
        "publish_initial_pose": publish_initial_pose,
    }
    if publish_initial_pose or "covariance" in start_pose:
        covariance = _require_dict(
            start_pose.get("covariance"),
            "start_pose.covariance",
        )
        normalized_start_pose["covariance"] = {
            axis: _validate_nonnegative(
                covariance.get(axis),
                f"start_pose.covariance.{axis}",
            )
            for axis in ("x", "y", "yaw")
        }
    normalized["start_pose"] = normalized_start_pose

    targets = _require_list(normalized.get("targets"), "targets")
    target_ids = set()
    normalized_targets = []
    for index, target_value in enumerate(targets):
        field = f"targets[{index}]"
        target = _require_dict(target_value, field)
        target_id = _require_id(target.get("id"), f"{field}.id")
        if target_id in target_ids:
            raise ValueError(f"duplicate target id: {target_id}")
        target_ids.add(target_id)
        name = target.get("name", target_id)
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{field}.name must be a non-empty string")
        normalized_targets.append(
            {
                **target,
                "id": target_id,
                "name": name,
                "pose": _validate_pose(
                    target.get("pose"),
                    f"{field}.pose",
                ),
                "task_duration_sec": _validate_nonnegative(
                    target.get("task_duration_sec", 0.0),
                    f"{field}.task_duration_sec",
                ),
            }
        )
    normalized["targets"] = normalized_targets

    routes = _require_list(normalized.get("routes"), "routes")
    route_ids = set()
    normalized_routes = []
    for index, route_value in enumerate(routes):
        field = f"routes[{index}]"
        route = _require_dict(route_value, field)
        route_id = _require_id(route.get("id"), f"{field}.id")
        if route_id in route_ids:
            raise ValueError(f"duplicate route id: {route_id}")
        route_ids.add(route_id)
        target_refs = _require_list(
            route.get("target_ids"),
            f"{field}.target_ids",
        )
        for target_id in target_refs:
            _require_id(target_id, f"{field}.target_ids item")
            if target_id not in target_ids:
                raise ValueError(
                    f"route {route_id} references unknown target "
                    f"{target_id}"
                )
        return_to_start = _require_bool(
            route.get("return_to_start", False),
            f"{field}.return_to_start",
        )
        retries = _validate_nonnegative_int(
            route.get("max_retries_per_checkpoint", 0),
            f"{field}.max_retries_per_checkpoint",
        )
        failure_policy = route.get("failure_policy", "abort")
        if failure_policy not in FAILURE_POLICIES:
            raise ValueError(
                f"{field}.failure_policy must be one of "
                f"{sorted(FAILURE_POLICIES)}"
            )
        name = route.get("name", route_id)
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{field}.name must be a non-empty string")
        loop = _require_dict(route.get("loop", {}), f"{field}.loop")
        loop_enabled = _require_bool(
            loop.get("enabled", False),
            f"{field}.loop.enabled",
        )
        loop_wait_sec = _validate_nonnegative(
            loop.get("wait_sec", 600.0),
            f"{field}.loop.wait_sec",
        )
        max_cycles = _validate_nonnegative_int(
            loop.get("max_cycles", 0),
            f"{field}.loop.max_cycles",
        )
        normalized_routes.append(
            {
                **route,
                "id": route_id,
                "name": name,
                "target_ids": list(target_refs),
                "return_to_start": return_to_start,
                "loop": {
                    **loop,
                    "enabled": loop_enabled,
                    "wait_sec": loop_wait_sec,
                    "max_cycles": max_cycles,
                },
                "goal_timeout_sec": _validate_positive(
                    route.get("goal_timeout_sec", 120.0),
                    f"{field}.goal_timeout_sec",
                ),
                "max_retries_per_checkpoint": retries,
                "failure_policy": failure_policy,
            }
        )
    normalized["routes"] = normalized_routes

    active_route_id = normalized.get("active_route_id")
    if active_route_id is not None:
        _require_id(active_route_id, "active_route_id")
        if active_route_id not in route_ids:
            raise ValueError(
                f"active_route_id references unknown route {active_route_id}"
            )

    schedules = _require_list(normalized.get("schedules", []), "schedules")
    schedule_ids = set()
    normalized_schedules = []
    for index, schedule_value in enumerate(schedules):
        field = f"schedules[{index}]"
        schedule = _require_dict(schedule_value, field)
        schedule_id = _require_id(schedule.get("id"), f"{field}.id")
        if schedule_id in schedule_ids:
            raise ValueError(f"duplicate schedule id: {schedule_id}")
        schedule_ids.add(schedule_id)
        route_id = _require_id(schedule.get("route_id"), f"{field}.route_id")
        if route_id not in route_ids:
            raise ValueError(
                f"schedule {schedule_id} references unknown route {route_id}"
            )
        enabled = _require_bool(
            schedule.get("enabled", False),
            f"{field}.enabled",
        )
        mode = schedule.get("mode")
        if mode not in SCHEDULE_MODES:
            raise ValueError(
                f"{field}.mode must be one of {sorted(SCHEDULE_MODES)}"
            )
        normalized_schedule = {
            **schedule,
            "id": schedule_id,
            "route_id": route_id,
            "enabled": enabled,
            "mode": mode,
        }
        if mode == "interval":
            normalized_schedule["period_sec"] = _validate_positive(
                schedule.get("period_sec"),
                f"{field}.period_sec",
            )
        normalized_schedules.append(normalized_schedule)
    normalized["schedules"] = normalized_schedules

    return normalized


def get_route(data: Dict[str, Any], route_id: str) -> Dict[str, Any]:
    for route in data["routes"]:
        if route["id"] == route_id:
            return copy.deepcopy(route)
    raise ValueError(f"unknown route: {route_id}")


def expand_route_targets(
    data: Dict[str, Any],
    route_id: str,
) -> List[Dict[str, Any]]:
    route = get_route(data, route_id)
    targets_by_id = {
        target["id"]: target for target in data["targets"]
    }
    return [
        copy.deepcopy(targets_by_id[target_id])
        for target_id in route["target_ids"]
    ]
