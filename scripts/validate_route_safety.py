#!/usr/bin/env python3
import argparse
import ast
import json
import math
import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ylhb_mobile_bridge"))
from ylhb_mobile_bridge.patrol_route_store import (  # noqa: E402
    segments_intersect,
    validate_route_map_binding,
)


def read_pgm(path):
    data = Path(path).read_bytes()
    index = 0

    def token():
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

    magic, width, height, maximum = token(), int(token()), int(token()), int(token())
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    pixels = data[index:]
    if magic != "P5" or maximum != 255 or len(pixels) != width * height:
        raise ValueError(f"invalid PGM: {path}")
    return width, height, pixels


def image_path(map_yaml, metadata):
    image = Path(metadata["image"])
    return image if image.is_absolute() else map_yaml.parent / image


def map_cell(gray, metadata):
    occupancy = ((255 - gray) if int(metadata.get("negate", 0)) == 0 else gray) / 255.0
    if occupancy > float(metadata["occupied_thresh"]):
        return "occupied"
    if occupancy < float(metadata["free_thresh"]):
        return "free"
    return "unknown"


def point_in_polygon(point, polygon, epsilon=1e-9):
    for index, current in enumerate(polygon):
        other = polygon[(index + 1) % len(polygon)]
        cross = (other["x"] - current["x"]) * (point["y"] - current["y"]) - (other["y"] - current["y"]) * (point["x"] - current["x"])
        if abs(cross) <= epsilon and min(current["x"], other["x"]) - epsilon <= point["x"] <= max(current["x"], other["x"]) + epsilon and min(current["y"], other["y"]) - epsilon <= point["y"] <= max(current["y"], other["y"]) + epsilon:
            return True
    inside = False
    for index, current in enumerate(polygon):
        other = polygon[index - 1]
        if (current["y"] > point["y"]) != (other["y"] > point["y"]):
            at_x = (other["x"] - current["x"]) * (point["y"] - current["y"]) / (other["y"] - current["y"]) + current["x"]
            if point["x"] < at_x:
                inside = not inside
    return inside


def polygons_intersect(first, second):
    if any(point_in_polygon(point, second) for point in first) or any(point_in_polygon(point, first) for point in second):
        return True
    return any(
        segments_intersect(point, first[(index + 1) % len(first)], other, second[(other_index + 1) % len(second)])
        for index, point in enumerate(first)
        for other_index, other in enumerate(second)
    )


def point_segment_distance(point, first, second):
    dx, dy = second["x"] - first["x"], second["y"] - first["y"]
    if dx == 0 and dy == 0:
        return math.hypot(point["x"] - first["x"], point["y"] - first["y"])
    ratio = max(0.0, min(1.0, ((point["x"] - first["x"]) * dx + (point["y"] - first["y"]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(point["x"] - (first["x"] + ratio * dx), point["y"] - (first["y"] + ratio * dy))


def polygon_distance(first, second):
    if polygons_intersect(first, second):
        return 0.0
    return min(
        min(
            point_segment_distance(point, other, second[(index + 1) % len(second)])
            for index, other in enumerate(second)
        )
        for point in first
    ) if first else math.inf


def polygon_clearance(first, second):
    return min(
        polygon_distance(first, second),
        polygon_distance(second, first),
    )


def padded_footprint(footprint, padding):
    return [
        [x + (padding if x > 0 else -padding if x < 0 else 0.0), y + (padding if y > 0 else -padding if y < 0 else 0.0)]
        for x, y in footprint
    ]


def transform_footprint(pose, footprint):
    cosine, sine = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    return [
        {"x": pose["x"] + point[0] * cosine - point[1] * sine, "y": pose["y"] + point[0] * sine + point[1] * cosine}
        for point in footprint
    ]


def footprint_map_failures(polygon, metadata, width, height, pixels):
    resolution, origin = float(metadata["resolution"]), metadata["origin"]
    min_x, max_x = min(point["x"] for point in polygon), max(point["x"] for point in polygon)
    min_y, max_y = min(point["y"] for point in polygon), max(point["y"] for point in polygon)
    if min_x < origin[0] or min_y < origin[1] or max_x >= origin[0] + width * resolution or max_y >= origin[1] + height * resolution:
        return ["footprint outside map"]
    failures = []
    start_x, end_x = max(0, int(math.floor((min_x - origin[0]) / resolution))), min(width - 1, int(math.floor((max_x - origin[0]) / resolution)))
    start_y, end_y = max(0, int(math.floor((min_y - origin[1]) / resolution))), min(height - 1, int(math.floor((max_y - origin[1]) / resolution)))
    for cell_y in range(start_y, end_y + 1):
        for cell_x in range(start_x, end_x + 1):
            center = {"x": origin[0] + (cell_x + 0.5) * resolution, "y": origin[1] + (cell_y + 0.5) * resolution}
            half = resolution * 0.5
            cell_polygon = [
                {"x": center["x"] - half, "y": center["y"] - half},
                {"x": center["x"] + half, "y": center["y"] - half},
                {"x": center["x"] + half, "y": center["y"] + half},
                {"x": center["x"] - half, "y": center["y"] + half},
            ]
            if polygons_intersect(polygon, cell_polygon):
                status = map_cell(pixels[(height - 1 - cell_y) * width + cell_x], metadata)
                if status != "free":
                    failures.append(f"footprint touches {status}")
                    return failures
    return failures


def required_clearance(zone, goal_xy_tolerance, map_resolution, safety_margin):
    return (
        float(zone.get("mask_padding_m", 0.05))
        + goal_xy_tolerance
        + map_resolution
        + safety_margin
    )


def navigation_geometry(map_yaml_path, nav2_path):
    metadata = yaml.safe_load(Path(map_yaml_path).read_text(encoding="utf-8"))
    width, height, pixels = read_pgm(image_path(Path(map_yaml_path), metadata))
    nav2 = yaml.safe_load(Path(nav2_path).read_text(encoding="utf-8"))
    global_params = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    controller_params = nav2["controller_server"]["ros__parameters"]
    footprint = padded_footprint(
        ast.literal_eval(global_params["footprint"]),
        float(global_params.get("footprint_padding", 0.0)),
    )
    goal_xy_tolerance = float(
        controller_params["goal_checker"]["xy_goal_tolerance"]
    )
    return metadata, width, height, pixels, footprint, goal_xy_tolerance


def validate_route(
    map_yaml_path,
    route_path,
    nav2_path,
    min_keepout_clearance,
    warn_distance,
    safety_margin,
):
    route_raw = json.loads(Path(route_path).read_text(encoding="utf-8"))
    route = validate_route_map_binding(route_raw, map_yaml_path)
    metadata, width, height, pixels, footprint, goal_xy_tolerance = navigation_geometry(
        map_yaml_path, nav2_path
    )
    map_resolution = float(metadata["resolution"])
    route_by_id = {item["id"]: item for item in route["routes"]}
    active_route = route_by_id[route.get("active_route_id") or next(iter(route_by_id), "")]
    targets = {item["id"]: item for item in route["targets"]}
    checks = [("start_pose", route["start_pose"]["pose"])] + [(target_id, targets[target_id]["pose"]) for target_id in active_route["target_ids"]]
    zones = [
        zone for zone in route["keepout_zones"]
        if zone["enabled"] and zone["type"] == "hard_keepout"
    ]
    failures, warnings, distances, target_safety = [], [], [], {}
    for name, pose in checks:
        polygon = transform_footprint(pose, footprint)
        item_failures = footprint_map_failures(polygon, metadata, width, height, pixels)
        item_distances = []
        item_required = []
        for zone in zones:
            effective_required = max(
                required_clearance(
                    zone,
                    goal_xy_tolerance,
                    map_resolution,
                    safety_margin,
                ),
                min_keepout_clearance,
            )
            warning_threshold = max(effective_required + 0.05, warn_distance)
            if polygons_intersect(polygon, zone["polygon"]):
                item_failures.append(f"footprint intersects {zone['id']}")
                distance = 0.0
            else:
                distance = polygon_clearance(polygon, zone["polygon"])
                epsilon = 1e-6
                if distance + epsilon < effective_required:
                    item_failures.append(
                        f"footprint clearance to {zone['id']} is {distance:.3f}m, "
                        f"required {effective_required:.3f}m"
                    )
                elif distance < warning_threshold:
                    warnings.append(
                        f"{name} footprint clearance to {zone['id']} is {distance:.3f}m "
                        f"(warning below {warning_threshold:.3f}m)"
                    )
            distances.append(distance)
            item_distances.append(distance)
            item_required.append(effective_required)
        failures.extend(f"{name} {failure}" for failure in item_failures)
        if name in targets:
            target_safety[name] = {
                "validation_status": "unsafe" if item_failures else "warning" if any(warning.startswith(f"{name} ") for warning in warnings) else "ok",
                "min_keepout_distance_m": min(item_distances) if item_distances else None,
                "required_clearance_m": max(item_required) if item_required else None,
                "warnings": [warning for warning in warnings if warning.startswith(f"{name} ")],
            }
    status = "unsafe" if failures else "warning" if warnings else "ok"
    route_raw["safety"] = {"validation_status": status, "min_keepout_distance_m": min(distances) if distances else None, "warnings": failures + warnings}
    for target in route_raw.get("targets", []):
        if target.get("id") in target_safety:
            target["safety"] = target_safety[target["id"]]
    return route_raw, status, failures, warnings


def suggest_safe_target(
    map_yaml_path,
    route_path,
    nav2_path,
    target_id,
    safety_margin,
    max_distance,
):
    route = validate_route_map_binding(
        json.loads(Path(route_path).read_text(encoding="utf-8")),
        map_yaml_path,
    )
    targets = {target["id"]: target for target in route["targets"]}
    if target_id not in targets:
        raise ValueError(f"unknown target: {target_id}")
    zones = [
        zone for zone in route["keepout_zones"]
        if zone["enabled"] and zone["type"] == "hard_keepout"
    ]
    metadata, width, height, pixels, footprint, goal_xy_tolerance = navigation_geometry(
        map_yaml_path, nav2_path
    )
    resolution = float(metadata["resolution"])
    original = dict(targets[target_id]["pose"])
    original_polygon = transform_footprint(original, footprint)
    original_clearance = min(
        polygon_clearance(original_polygon, zone["polygon"])
        for zone in zones
    )
    steps = int(math.ceil(max_distance / resolution))
    offsets = sorted(
        (
            (math.hypot(dx, dy), dx, dy)
            for dx in range(-steps, steps + 1)
            for dy in range(-steps, steps + 1)
            if (dx or dy) and math.hypot(dx, dy) * resolution <= max_distance
        ),
        key=lambda item: item[0],
    )
    for _distance_cells, dx, dy in offsets:
        candidate = {
            **original,
            "x": original["x"] + dx * resolution,
            "y": original["y"] + dy * resolution,
        }
        polygon = transform_footprint(candidate, footprint)
        map_failures = footprint_map_failures(
            polygon, metadata, width, height, pixels
        )
        clearances = []
        thresholds = []
        safe = not map_failures
        for zone in zones:
            clearance = (
                0.0
                if polygons_intersect(polygon, zone["polygon"])
                else polygon_clearance(polygon, zone["polygon"])
            )
            threshold = required_clearance(
                zone,
                goal_xy_tolerance,
                resolution,
                safety_margin,
            )
            clearances.append(clearance)
            thresholds.append(threshold)
            safe = safe and clearance + 1e-6 >= threshold
        if safe:
            new_clearance = min(clearances)
            warning_threshold = max(thresholds) + 0.05
            return {
                "target_id": target_id,
                "original_pose": original,
                "candidate_pose": candidate,
                "movement_distance_m": math.hypot(dx, dy) * resolution,
                "original_clearance_m": original_clearance,
                "candidate_clearance_m": new_clearance,
                "required_clearance_m": max(thresholds),
                "map_collision_check": "free",
                "final_safety_status": (
                    "warning" if new_clearance < warning_threshold else "ok"
                ),
            }
    return None


def atomic_write_text(path, content):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, dest="map_yaml")
    parser.add_argument("--route", required=True)
    parser.add_argument("--nav2-params", required=True)
    parser.add_argument("--min-keepout-clearance", type=float, default=0.0)
    parser.add_argument("--warn-distance", type=float, default=0.0)
    parser.add_argument("--safety-margin-m", type=float, default=0.025)
    parser.add_argument("--suggest-target", default="")
    parser.add_argument("--suggest-max-distance-m", type=float, default=0.50)
    parser.add_argument("--write-back", action="store_true")
    parser.add_argument("--report", action="store_true")
    args = parser.parse_args()
    try:
        route, status, failures, warnings = validate_route(
            args.map_yaml,
            args.route,
            args.nav2_params,
            args.min_keepout_clearance,
            args.warn_distance,
            args.safety_margin_m,
        )
        suggestion = None
        if args.suggest_target:
            suggestion = suggest_safe_target(
                args.map_yaml,
                args.route,
                args.nav2_params,
                args.suggest_target,
                args.safety_margin_m,
                args.suggest_max_distance_m,
            )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if args.write_back:
        atomic_write_text(args.route, json.dumps(route, ensure_ascii=False, indent=2) + "\n")
    if args.report:
        targets = {
            target["id"]: target.get("safety", {})
            for target in route.get("targets", [])
        }
        print(json.dumps({
            "status": status,
            "ok": status != "unsafe",
            "failures": failures,
            "warnings": warnings,
            "targets": targets,
            "suggestion": suggestion,
        }, ensure_ascii=False))
        return 1 if status == "unsafe" else 0
    for warning in warnings:
        print(f"WARN: {warning}")
    for failure in failures:
        print(f"ERROR: {failure}", file=sys.stderr)
    print(f"route safety {status}")
    return 1 if status == "unsafe" else 0


if __name__ == "__main__":
    raise SystemExit(main())
