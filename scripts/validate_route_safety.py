#!/usr/bin/env python3
import argparse
import ast
import json
import math
from pathlib import Path

import yaml


def point_in_polygon(point, polygon):
    inside = False
    j = len(polygon) - 1
    for i, current in enumerate(polygon):
        previous = polygon[j]
        crosses = (current["y"] > point["y"]) != (previous["y"] > point["y"])
        if crosses:
            at_x = ((previous["x"] - current["x"]) * (point["y"] - current["y"])) / (
                previous["y"] - current["y"]
            ) + current["x"]
            if point["x"] < at_x:
                inside = not inside
        j = i
    return inside


def segments_intersect(a, b, c, d):
    def orient(p, q, r):
        return (q["x"] - p["x"]) * (r["y"] - p["y"]) - (q["y"] - p["y"]) * (r["x"] - p["x"])

    return orient(a, b, c) * orient(a, b, d) <= 0 and orient(c, d, a) * orient(c, d, b) <= 0


def polygon_intersects(a, b):
    if any(point_in_polygon(point, b) for point in a):
        return True
    if any(point_in_polygon(point, a) for point in b):
        return True
    for i, point in enumerate(a):
        next_point = a[(i + 1) % len(a)]
        for j, other in enumerate(b):
            next_other = b[(j + 1) % len(b)]
            if segments_intersect(point, next_point, other, next_other):
                return True
    return False


def point_segment_distance(point, a, b):
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    if dx == 0 and dy == 0:
        return math.hypot(point["x"] - a["x"], point["y"] - a["y"])
    t = max(0.0, min(1.0, ((point["x"] - a["x"]) * dx + (point["y"] - a["y"]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(point["x"] - (a["x"] + t * dx), point["y"] - (a["y"] + t * dy))


def polygon_distance(a, b):
    distances = []
    for point in a:
        distances.extend(point_segment_distance(point, b[i], b[(i + 1) % len(b)]) for i in range(len(b)))
    for point in b:
        distances.extend(point_segment_distance(point, a[i], a[(i + 1) % len(a)]) for i in range(len(a)))
    return min(distances)


def transform_footprint(pose, footprint):
    cos_yaw = math.cos(pose["yaw"])
    sin_yaw = math.sin(pose["yaw"])
    return [
        {
            "x": pose["x"] + point[0] * cos_yaw - point[1] * sin_yaw,
            "y": pose["y"] + point[0] * sin_yaw + point[1] * cos_yaw,
        }
        for point in footprint
    ]


def hard_zones(route):
    return [
        zone
        for zone in route.get("keepout_zones", [])
        if zone.get("enabled") is True and zone.get("type") == "hard_keepout"
    ]


def route_targets(route):
    if route.get("version") not in (2, 3):
        raise SystemExit("ERROR: route version must be 2 or 3")
    for target in route.get("targets", []):
        pose = target.get("pose")
        location = target.get("location")
        if pose is None and isinstance(location, dict) and location.get("type") == "map_pose":
            pose = {axis: location[axis] for axis in ("x", "y", "yaw")}
        if not pose:
            raise SystemExit(f"ERROR: target {target.get('id')} missing pose")
        if route.get("version") == 3 and isinstance(location, dict) and location.get("type") == "map_pose":
            for axis in ("x", "y", "yaw"):
                if abs(float(pose[axis]) - float(location[axis])) > 1e-6:
                    raise SystemExit(f"ERROR: target {target.get('id')} pose/location {axis} mismatch")
        yield target.get("id", "<unknown>"), {axis: float(pose[axis]) for axis in ("x", "y", "yaw")}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True)
    parser.add_argument("--nav2-params", required=True)
    parser.add_argument("--warn-distance", type=float, default=0.20)
    args = parser.parse_args()

    nav2 = yaml.safe_load(Path(args.nav2_params).read_text(encoding="utf-8"))
    footprint_text = nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    footprint = ast.literal_eval(footprint_text)
    failures = []
    warnings = []
    route = json.loads(Path(args.route).read_text(encoding="utf-8"))
    zones = hard_zones(route)

    for target_id, pose in route_targets(route):
        footprint_polygon = transform_footprint(pose, footprint)
        for zone in zones:
            polygon = zone["polygon"]
            if point_in_polygon(pose, polygon):
                failures.append(f"{target_id} center inside {zone['id']}")
            if polygon_intersects(footprint_polygon, polygon):
                failures.append(f"{target_id} footprint intersects {zone['id']}")
            distance = polygon_distance(footprint_polygon, polygon)
            if distance < args.warn_distance:
                warnings.append(f"{target_id} is {distance:.2f}m from {zone['id']}")

    if failures:
        raise SystemExit("ERROR:\n" + "\n".join(failures))
    for warning in warnings:
        print(f"WARN: {warning}")
    print("route safety OK")


if __name__ == "__main__":
    main()
