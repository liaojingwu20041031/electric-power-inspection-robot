#!/usr/bin/env python3
import argparse
import ast
import hashlib
import json
import math
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ylhb_mobile_bridge"))
from ylhb_mobile_bridge.patrol_route_store import validate_route_map_binding  # noqa: E402


def require(condition, message):
    if not condition:
        raise SystemExit(f"ERROR: {message}")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def image_path(yaml_path, metadata):
    image = Path(metadata["image"])
    return image if image.is_absolute() else Path(yaml_path).parent / image


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
    require(magic == "P5" and maximum == 255, f"invalid PGM: {path}")
    require(index < len(data) and chr(data[index]).isspace(), f"invalid PGM header: {path}")
    if data[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = data[index:]
    require(len(pixels) == width * height, f"invalid PGM pixel count: {path}")
    return width, height, pixels


def mask_stats(pixels):
    return {
        "hard_cells": pixels.count(100),
        "weighted_cells": sum(1 for value in pixels if 1 <= value <= 99),
        "free_cells": pixels.count(0),
    }


def filter_info_parameters(launch_path):
    tree = ast.parse(Path(launch_path).read_text(encoding="utf-8"))
    result = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or getattr(node.func, "id", None) != "LifecycleNode":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        name = keywords.get("name")
        values = keywords.get("parameters")
        if not isinstance(name, ast.Constant) or not isinstance(values, ast.List) or not values.elts:
            continue
        item = values.elts[0]
        if not isinstance(item, ast.Dict):
            continue
        result[name.value] = {
            key.value: value.value
            for key, value in zip(item.keys, item.values)
            if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)
        }
    return result


def weighted_zone_surrounds_hard_core(global_pixels, local_pixels, width, height):
    for index, value in enumerate(local_pixels):
        if value != 100:
            continue
        row, column = divmod(index, width)
        neighbors = [
            (row + dy, column + dx)
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
            if dy or dx
            if 0 <= row + dy < height and 0 <= column + dx < width
        ]
        for neighbor_row, neighbor_column in neighbors:
            neighbor = neighbor_row * width + neighbor_column
            if local_pixels[neighbor] != 100 and not 1 <= global_pixels[neighbor] <= 99:
                return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, dest="map_yaml")
    parser.add_argument("--route", required=True)
    parser.add_argument("--nav2-params", required=True)
    parser.add_argument("--output-dir", default="maps/keepout")
    args = parser.parse_args()

    map_path = Path(args.map_yaml).expanduser()
    route_path = Path(args.route).expanduser()
    nav2_path = Path(args.nav2_params).expanduser()
    normal_nav2_path = nav2_path.with_name("nav2_params.yaml")
    launch_path = ROOT / "src" / "ylhb_base" / "launch" / "navigation_keepout.launch.py"
    output_dir = Path(args.output_dir).expanduser()
    global_yaml_path = output_dir / "keepout_global_mask.yaml"
    local_yaml_path = output_dir / "keepout_local_mask.yaml"
    metadata_path = output_dir / "keepout_masks.metadata.json"
    for path in (map_path, route_path, nav2_path, normal_nav2_path, global_yaml_path, local_yaml_path, metadata_path):
        require(path.exists(), f"missing {path}")

    map_meta = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    global_meta = yaml.safe_load(global_yaml_path.read_text(encoding="utf-8"))
    local_meta = yaml.safe_load(local_yaml_path.read_text(encoding="utf-8"))
    generated = json.loads(metadata_path.read_text(encoding="utf-8"))
    route = validate_route_map_binding(
        json.loads(route_path.read_text(encoding="utf-8")), map_path
    )
    zones = [
        zone for zone in route.get("keepout_zones", [])
        if zone.get("enabled") and zone.get("type") == "hard_keepout"
    ]
    map_width, map_height, _ = read_pgm(image_path(map_path, map_meta))
    mask_pixels: dict = {}
    for name, yaml_path, meta in (
        ("global", global_yaml_path, global_meta),
        ("local", local_yaml_path, local_meta),
    ):
        width, height, pixels = read_pgm(image_path(yaml_path, meta))
        require((width, height) == (map_width, map_height), f"{name} mask size differs")
        require(float(meta["resolution"]) == float(map_meta["resolution"]), f"{name} mask resolution differs")
        require(meta["origin"][:2] == map_meta["origin"][:2], f"{name} mask origin differs")
        require(meta.get("mode") == "raw", f"{name} mask mode must be raw")
        mask_pixels[name] = pixels

    require(generated["map_yaml_sha256"] == sha256(map_path), "map metadata is stale")
    require(generated["map_pgm_sha256"] == sha256(image_path(map_path, map_meta)), "map image metadata is stale")
    require(generated["route_sha256"] == sha256(route_path), "route metadata is stale")
    require(generated["nav2_params_sha256"] == sha256(nav2_path), "Nav2 metadata is stale")

    expected_zone_ids = {str(zone["id"]) for zone in zones}
    actual_zone_ids = set((generated.get("zones") or {}).keys())
    require(actual_zone_ids == expected_zone_ids, "metadata zones differ from route")
    require(
        generated.get("enabled_hard_keepout_count") == len(zones),
        "metadata keepout zone count differs",
    )
    expected_mode = "active" if zones else "all_free"
    require(
        generated.get("keepout_mode") == expected_mode,
        "metadata keepout mode differs",
    )

    params = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    normal_params = yaml.safe_load(normal_nav2_path.read_text(encoding="utf-8"))
    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    normal_follow = normal_params["controller_server"]["ros__parameters"]["FollowPath"]
    expected_follow = dict(normal_follow)
    expected_follow.update({
        "critics": [
            "RotateToGoal", "Oscillation", "BaseObstacle", "ObstacleFootprint",
            "GoalAlign", "PathAlign", "PathDist", "GoalDist",
        ],
        "BaseObstacle.scale": 5.0,
        "ObstacleFootprint.scale": 0.02,
    })
    require(follow == expected_follow, "Keepout FollowPath differs from normal critics")
    normal_bt = normal_params["bt_navigator"]["ros__parameters"]
    keepout_bt = params["bt_navigator"]["ros__parameters"]
    expected_bt = dict(normal_bt)
    expected_bt.update({
        "default_bt_xml_filename": keepout_bt["default_bt_xml_filename"],
        "default_nav_to_pose_bt_xml": keepout_bt["default_nav_to_pose_bt_xml"],
    })
    require(keepout_bt == expected_bt, "Keepout BT settings differ from normal")
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    normal_global_params = normal_params["global_costmap"]["global_costmap"]["ros__parameters"]
    normal_local_params = normal_params["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = ast.literal_eval(global_params["footprint"])
    require(footprint == ast.literal_eval(local_params["footprint"]), "global/local footprint differs")
    footprint_padding = float(global_params.get("footprint_padding", 0.0))
    require(footprint_padding == float(local_params.get("footprint_padding", 0.0)), "global/local footprint padding differs")
    radius = max(math.hypot(float(x), float(y)) for x, y in footprint)
    require(math.isclose(generated["circumscribed_radius_m"], radius), "footprint radius metadata is stale")
    resolution = float(map_meta["resolution"])
    for zone in zones:
        requested_clearance = float(zone.get("mask_padding_m", 0.05))
        soft_radius = radius + footprint_padding + requested_clearance + resolution
        zone_meta = generated["zones"][zone["id"]]
        require(math.isclose(zone_meta["hard_padding_m"], requested_clearance), f"{zone['id']} hard padding differs")
        require(math.isclose(zone_meta["soft_radius_m"], soft_radius), f"{zone['id']} soft radius differs")
        require(zone_meta["soft_radius_m"] > zone_meta["hard_padding_m"], f"{zone['id']} soft radius must exceed hard padding")

    global_pixels = mask_pixels["global"]
    local_pixels = mask_pixels["local"]
    require(set(global_pixels) <= set(range(101)), "global mask contains invalid raw cost")
    require(set(local_pixels) <= {0, 100}, "local mask must contain only free and hard costs")
    global_stats = mask_stats(global_pixels)
    local_stats = mask_stats(local_pixels)
    require(generated.get("global_mask") == {
        **global_stats, "max_weight": 100, "min_nonzero_weight": 1,
    }, "global mask metadata differs")
    require(generated.get("local_mask") == local_stats, "local mask metadata differs")
    if zones:
        require(global_stats["hard_cells"] > 0, "global mask contains no hard core")
        require(global_stats["weighted_cells"] > 0, "global mask contains no weighted costs")
        require(global_stats["free_cells"] > 0, "global mask contains no free cells")
        require(local_stats["hard_cells"] > 0, "local mask contains no hard core")
        require(local_stats["weighted_cells"] == 0, "local mask must not contain weighted costs")
        require(local_stats["free_cells"] > 0, "local mask contains no free cells")
        require(global_stats["hard_cells"] + global_stats["weighted_cells"] < len(global_pixels), "global mask is fully blocked")
        require(
            all(
                (global_value == 100) == (local_value == 100)
                for global_value, local_value in zip(global_pixels, local_pixels)
            ),
            "global/local hard cores differ",
        )
        require(
            weighted_zone_surrounds_hard_core(
                global_pixels, local_pixels, map_width, map_height
            ),
            "global weighted zone does not surround hard core",
        )
    else:
        require(set(global_pixels) == {0}, "global mask is not all-free for zoneless route")
        require(set(local_pixels) == {0}, "local mask is not all-free for zoneless route")

    expected_topics = {
        "global": "/keepout_global_filter_info",
        "local": "/keepout_local_filter_info",
    }
    filter_info = filter_info_parameters(launch_path)
    for name, costmap, normal_costmap in (
        ("global", global_params, normal_global_params),
        ("local", local_params, normal_local_params),
    ):
        require("keepout_filter" in (costmap.get("filters") or []), f"{name} keepout filter missing")
        require(costmap["keepout_filter"]["filter_info_topic"] == expected_topics[name], f"{name} keepout topic differs")
        info = filter_info.get(f"keepout_{name}_filter_info_server", {})
        require(info.get("type") == 0, f"{name} filter info type differs")
        require(info.get("base") == 0.0, f"{name} filter info base differs")
        require(info.get("multiplier") == 1.0, f"{name} filter info multiplier differs")
        comparable = dict(costmap)
        comparable.pop("filters", None)
        comparable.pop("keepout_filter", None)
        require(comparable == normal_costmap, f"{name} non-filter settings differ from normal")
    require(
        params["velocity_smoother"]["ros__parameters"]
        == normal_params["velocity_smoother"]["ros__parameters"],
        "Keepout velocity smoother differs from normal",
    )
    print("keepout setup OK")


if __name__ == "__main__":
    main()
