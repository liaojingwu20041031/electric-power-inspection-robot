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
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    pixels = data[index:]
    require(len(pixels) == width * height, f"invalid PGM pixel count: {path}")
    return width, height, pixels


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
    output_dir = Path(args.output_dir).expanduser()
    global_yaml_path = output_dir / "keepout_global_mask.yaml"
    local_yaml_path = output_dir / "keepout_local_mask.yaml"
    metadata_path = output_dir / "keepout_masks.metadata.json"
    for path in (map_path, route_path, nav2_path, global_yaml_path, local_yaml_path, metadata_path):
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
    obstacle_scale = float(
        params[
            "controller_server"
        ][
            "ros__parameters"
        ][
            "FollowPath"
        ][
            "ObstacleFootprint.scale"
        ]
    )
    require(
        0.0 < obstacle_scale <= 0.10,
        "ObstacleFootprint.scale is too large; "
        "inflation costs will behave like hard obstacles",
    )
    critics = params[
        "controller_server"
    ][
        "ros__parameters"
    ][
        "FollowPath"
    ][
        "critics"
    ]
    require(
        "ObstacleFootprint" in critics,
        "ObstacleFootprint critic missing",
    )
    require(
        "BaseObstacle" not in critics,
        "BaseObstacle must not replace full footprint collision checking",
    )
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = ast.literal_eval(global_params["footprint"])
    require(footprint == ast.literal_eval(local_params["footprint"]), "global/local footprint differs")
    footprint_padding = float(global_params.get("footprint_padding", 0.0))
    require(footprint_padding == float(local_params.get("footprint_padding", 0.0)), "global/local footprint padding differs")
    radius = max(math.hypot(float(x), float(y)) for x, y in footprint)
    require(math.isclose(generated["circumscribed_radius_m"], radius), "footprint radius metadata is stale")
    resolution = float(map_meta["resolution"])
    for zone in zones:
        padding = float(zone.get("mask_padding_m", 0.05))
        zone_meta = generated["zones"][zone["id"]]
        require(math.isclose(zone_meta["local_padding_m"], padding), f"{zone['id']} local padding differs")
        require(math.isclose(zone_meta["global_padding_m"], radius + footprint_padding + padding + resolution), f"{zone['id']} global padding differs")

    global_pixels = mask_pixels["global"]
    local_pixels = mask_pixels["local"]
    if zones:
        require(0 in global_pixels, "global mask contains no keepout pixels")
        require(0 in local_pixels, "local mask contains no keepout pixels")
    else:
        require(set(global_pixels) == {254}, "global mask is not all-free for zoneless route")
        require(set(local_pixels) == {254}, "local mask is not all-free for zoneless route")

    expected_topics = {
        "global": "/keepout_global_filter_info",
        "local": "/keepout_local_filter_info",
    }
    for name, costmap in (("global", global_params), ("local", local_params)):
        require("keepout_filter" in (costmap.get("filters") or []), f"{name} keepout filter missing")
        require(costmap["keepout_filter"]["filter_info_topic"] == expected_topics[name], f"{name} keepout topic differs")
        require(float(costmap["inflation_layer"]["cost_scaling_factor"]) == 6.0, f"{name} inflation scaling changed")
        require(float(costmap["inflation_layer"]["inflation_radius"]) == 0.20, f"{name} inflation radius changed")
    print("keepout setup OK")


if __name__ == "__main__":
    main()
