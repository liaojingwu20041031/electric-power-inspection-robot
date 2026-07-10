import copy
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

import yaml


FAILURE_POLICIES = {"abort", "abort_and_return_home"}
SCHEDULE_MODES = {"interval", "daily"}
DEFAULT_ROUTE_DIRECTORY = Path("/home/nvidia/ros2_DL/maps")
ROUTE_FILE_PATTERN = "route_patrol_*.json"
ROUTE_NUMBER_PATTERN = re.compile(r"^route_patrol_(\d+)\.json$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
MAP_BINDING_ERROR = (
    "当前巡逻标注不属于正在加载的地图，请使用路线标注工具在新地图上重新绘制"
    "起点、巡检点、路线和禁行区。"
)


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


def _validate_optional_string(value: Any, field: str) -> Union[str, None]:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    return value


def _validate_optional_string_list(value: Any, field: str) -> List[str]:
    items = _require_list(value, field)
    normalized = []
    for item in items:
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"{field} items must be non-empty strings")
        normalized.append(item)
    return normalized


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


def _validate_v3_location_pose(
    pose: Union[Dict[str, float], None],
    location: Any,
    field: str,
) -> Dict[str, float]:
    if location is None:
        raise ValueError(f"{field}.location must be a map_pose")
    location_dict = _require_dict(location, f"{field}.location")
    if location_dict.get("type") != "map_pose":
        raise ValueError(f"{field}.location.type must be map_pose")
    if location_dict.get("frame_id", "map") != "map":
        raise ValueError(f"{field}.location.frame_id must be map")
    location_pose = {
        axis: _require_number(location_dict.get(axis), f"{field}.location.{axis}")
        for axis in ("x", "y", "yaw")
    }
    if pose is None:
        return location_pose
    for axis in ("x", "y", "yaw"):
        if abs(pose[axis] - location_pose[axis]) > 1e-6:
            raise ValueError(f"{field}.pose and location disagree on {axis}")
    return pose


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


def _read_pgm_header(path: Path) -> Tuple[int, int]:
    data = path.read_bytes()
    index = 0

    def token() -> str:
        nonlocal index
        while index < len(data):
            if data[index] == ord("#"):
                while index < len(data) and data[index] not in b"\r\n":
                    index += 1
            elif chr(data[index]).isspace():
                index += 1
            else:
                break
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        return data[start:index].decode("ascii")

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if magic != "P5" or width <= 0 or height <= 0 or max_value != 255:
        raise ValueError(f"invalid 8-bit binary PGM: {path}")
    return width, height


def _validate_map_identity(value: Any) -> Dict[str, Any]:
    identity = _require_dict(value, "map")
    origin = _require_list(identity.get("origin"), "map.origin")
    if len(origin) != 3:
        raise ValueError("map.origin must contain 3 numbers")
    image_sha256 = _require_id(identity.get("image_sha256"), "map.image_sha256")
    if not SHA256_PATTERN.fullmatch(image_sha256):
        raise ValueError("map.image_sha256 must be a SHA256 hex string")
    width = identity.get("width")
    height = identity.get("height")
    if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
        raise ValueError("map.width must be a positive integer")
    if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
        raise ValueError("map.height must be a positive integer")
    return {
        **identity,
        "yaml": _require_id(identity.get("yaml"), "map.yaml"),
        "image": _require_id(identity.get("image"), "map.image"),
        "resolution": _validate_positive(identity.get("resolution"), "map.resolution"),
        "origin": [_require_number(item, f"map.origin[{index}]") for index, item in enumerate(origin)],
        "width": width,
        "height": height,
        "image_sha256": image_sha256,
    }


def _orientation(a: Dict[str, float], b: Dict[str, float], c: Dict[str, float]) -> float:
    return (b["x"] - a["x"]) * (c["y"] - a["y"]) - (b["y"] - a["y"]) * (c["x"] - a["x"])


def _on_segment(a: Dict[str, float], b: Dict[str, float], point: Dict[str, float], epsilon: float = 1e-9) -> bool:
    return (
        abs(_orientation(a, b, point)) <= epsilon
        and min(a["x"], b["x"]) - epsilon <= point["x"] <= max(a["x"], b["x"]) + epsilon
        and min(a["y"], b["y"]) - epsilon <= point["y"] <= max(a["y"], b["y"]) + epsilon
    )


def segments_intersect(a: Dict[str, float], b: Dict[str, float], c: Dict[str, float], d: Dict[str, float], epsilon: float = 1e-9) -> bool:
    first = _orientation(a, b, c)
    second = _orientation(a, b, d)
    third = _orientation(c, d, a)
    fourth = _orientation(c, d, b)
    if first * second < -epsilon and third * fourth < -epsilon:
        return True
    return any((
        _on_segment(a, b, c, epsilon),
        _on_segment(a, b, d, epsilon),
        _on_segment(c, d, a, epsilon),
        _on_segment(c, d, b, epsilon),
    ))


def _polygon_area(polygon: List[Dict[str, float]]) -> float:
    return 0.5 * sum(
        point["x"] * polygon[(index + 1) % len(polygon)]["y"]
        - point["y"] * polygon[(index + 1) % len(polygon)]["x"]
        for index, point in enumerate(polygon)
    )


def _polygon_self_intersects(polygon: List[Dict[str, float]]) -> bool:
    count = len(polygon)
    for first in range(count):
        for second in range(first + 1, count):
            if second in (first, (first + 1) % count) or (first == 0 and second == count - 1):
                continue
            if segments_intersect(
                polygon[first], polygon[(first + 1) % count],
                polygon[second], polygon[(second + 1) % count],
            ):
                return True
    return False


def _validate_keepout_zones(value: Any, field: str) -> List[Dict[str, Any]]:
    zones = _require_list(value, field)
    zone_ids = set()
    normalized = []
    for index, zone_value in enumerate(zones):
        zone_field = f"{field}[{index}]"
        zone = _require_dict(zone_value, zone_field)
        zone_id = _require_id(zone.get("id"), f"{zone_field}.id")
        if zone_id in zone_ids:
            raise ValueError(f"duplicate keepout zone id: {zone_id}")
        zone_ids.add(zone_id)
        if zone.get("type") != "hard_keepout":
            raise ValueError(f"{zone_field}.type must be hard_keepout")
        polygon_value = _require_list(zone.get("polygon"), f"{zone_field}.polygon")
        if len(polygon_value) < 3:
            raise ValueError(f"{zone_field}.polygon must contain at least 3 points")
        polygon = [
            {
                "x": _require_number(_require_dict(point, f"{zone_field}.polygon[{point_index}]").get("x"), f"{zone_field}.polygon[{point_index}].x"),
                "y": _require_number(_require_dict(point, f"{zone_field}.polygon[{point_index}]").get("y"), f"{zone_field}.polygon[{point_index}].y"),
            }
            for point_index, point in enumerate(polygon_value)
        ]
        if _polygon_self_intersects(polygon):
            raise ValueError(f"{zone_field}.polygon self-intersects")
        if abs(_polygon_area(polygon)) <= 1e-9:
            raise ValueError(f"{zone_field}.polygon area must not be zero")
        normalized.append({
            **zone,
            "id": zone_id,
            "type": "hard_keepout",
            "enabled": _require_bool(zone.get("enabled"), f"{zone_field}.enabled"),
            "polygon": polygon,
        })
    return normalized


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
    if isinstance(version, bool) or version not in (2, 3):
        raise ValueError("version must be 2 or 3")
    if normalized.get("frame_id") != "map":
        raise ValueError('frame_id must be "map"')
    if version == 3:
        normalized["map"] = _validate_map_identity(normalized.get("map"))

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
        "pose": _validate_v3_location_pose(
            _validate_pose(start_pose.get("pose"), "start_pose.pose"),
            start_pose.get("location"),
            "start_pose",
        ) if version == 3 else _validate_pose(start_pose.get("pose"), "start_pose.pose"),
        "publish_initial_pose": publish_initial_pose,
    }
    if version == 3:
        if start_pose.get("frame_id") != "map":
            raise ValueError("start_pose.frame_id must be map")
        normalized_start_pose["frame_id"] = "map"
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
        pose = _validate_v3_location_pose(
            _validate_pose(target.get("pose"), f"{field}.pose")
            if target.get("pose") is not None
            else None,
            target.get("location"),
            field,
        ) if version == 3 else _validate_pose(target.get("pose"), f"{field}.pose")
        normalized_targets.append(
            {
                **target,
                "id": target_id,
                "name": name,
                "aliases": _validate_optional_string_list(
                    target.get("aliases", []),
                    f"{field}.aliases",
                ),
                "area_id": _validate_optional_string(
                    target.get("area_id"),
                    f"{field}.area_id",
                ),
                "inspection_items": _validate_optional_string_list(
                    target.get("inspection_items", []),
                    f"{field}.inspection_items",
                ),
                "pose": pose,
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
                "aliases": _validate_optional_string_list(
                    route.get("aliases", []),
                    f"{field}.aliases",
                ),
                "description": _validate_optional_string(
                    route.get("description"),
                    f"{field}.description",
                ),
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
    if version == 3:
        normalized["keepout_zones"] = _validate_keepout_zones(
            normalized.get("keepout_zones"),
            "keepout_zones",
        )
    elif "keepout_zones" in normalized:
        normalized["keepout_zones"] = _validate_keepout_zones(
            normalized["keepout_zones"],
            "keepout_zones",
        )
    if "site" in normalized:
        normalized["site"] = _require_dict(normalized["site"], "site")
    if "areas" in normalized:
        areas = _require_list(normalized["areas"], "areas")
        area_ids = set()
        normalized_areas = []
        for index, area_value in enumerate(areas):
            field = f"areas[{index}]"
            area = _require_dict(area_value, field)
            area_id = _require_id(area.get("id"), f"{field}.id")
            if area_id in area_ids:
                raise ValueError(f"duplicate area id: {area_id}")
            area_ids.add(area_id)
            name = area.get("name", area_id)
            if not isinstance(name, str) or not name.strip():
                raise ValueError(f"{field}.name must be a non-empty string")
            normalized_areas.append({**area, "id": area_id, "name": name})
        normalized["areas"] = normalized_areas

    return normalized


def validate_route_map_binding(data: Any, map_yaml_path: Union[str, Path]) -> Dict[str, Any]:
    route = validate_route_file(data)
    if route["version"] == 2:
        return route
    map_yaml_path = Path(map_yaml_path).expanduser()
    try:
        metadata = _require_dict(
            yaml.safe_load(map_yaml_path.read_text(encoding="utf-8")),
            "map yaml",
        )
        image_path = Path(_require_id(metadata.get("image"), "map yaml.image"))
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path
        width, height = _read_pgm_header(image_path)
        route_map = route["map"]
        matches = (
            route_map["yaml"] == map_yaml_path.name
            and route_map["image"] == image_path.name
            and math.isclose(route_map["resolution"], _validate_positive(metadata.get("resolution"), "map yaml.resolution"), abs_tol=1e-12)
            and len(metadata.get("origin", [])) == 3
            and all(math.isclose(route_map["origin"][index], _require_number(metadata["origin"][index], f"map yaml.origin[{index}]"), abs_tol=1e-12) for index in range(3))
            and route_map["width"] == width
            and route_map["height"] == height
            and route_map["image_sha256"] == hashlib.sha256(image_path.read_bytes()).hexdigest()
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"invalid map binding: {exc}") from exc
    if not matches:
        raise ValueError(MAP_BINDING_ERROR)
    return route


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
