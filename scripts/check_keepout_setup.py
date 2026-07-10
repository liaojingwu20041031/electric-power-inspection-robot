#!/usr/bin/env python3
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ylhb_mobile_bridge"))
from ylhb_mobile_bridge.patrol_route_store import validate_route_map_binding  # noqa: E402


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

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    pixels = data[index:]
    if magic != "P5" or max_value != 255 or len(pixels) != width * height:
        raise ValueError(f"invalid PGM: {path}")
    return width, height, pixels


def image_path(yaml_path, metadata):
    image = Path(metadata["image"])
    return image if image.is_absolute() else Path(yaml_path).parent / image


def require(condition, message):
    if not condition:
        raise SystemExit(f"ERROR: {message}")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_ros():
    commands = [
        ["ros2", "topic", "info", "/keepout_costmap_filter_info", "--no-daemon"],
        ["ros2", "topic", "info", "/keepout_filter_mask", "--no-daemon"],
        ["ros2", "lifecycle", "get", "/keepout_filter_mask_server"],
        ["ros2", "lifecycle", "get", "/costmap_filter_info_server"],
    ]
    for command in commands:
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        require(result.returncode == 0, f"{' '.join(command)} failed:\n{result.stdout}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, dest="map_yaml")
    parser.add_argument("--route", required=True)
    parser.add_argument("--mask", required=True, dest="mask_yaml")
    parser.add_argument("--nav2-params", required=True)
    parser.add_argument("--metadata", default="")
    parser.add_argument("--ros", action="store_true")
    args = parser.parse_args()

    for path in (args.map_yaml, args.route, args.mask_yaml, args.nav2_params):
        require(Path(path).expanduser().exists(), f"missing {path}")

    map_meta = yaml.safe_load(Path(args.map_yaml).read_text(encoding="utf-8"))
    mask_meta = yaml.safe_load(Path(args.mask_yaml).read_text(encoding="utf-8"))
    metadata_path = Path(args.metadata).expanduser() if args.metadata else Path(args.mask_yaml).with_suffix('.metadata.json')
    require(metadata_path.exists(), f"missing {metadata_path}")
    generated = json.loads(metadata_path.read_text(encoding="utf-8"))
    route = validate_route_map_binding(
        json.loads(Path(args.route).read_text(encoding="utf-8")),
        args.map_yaml,
    )
    require(route["version"] == 3, "keepout setup requires a v3 route with map binding")
    require(any(z.get("enabled") and z.get("type") == "hard_keepout" for z in route.get("keepout_zones", [])),
            "route has no enabled hard_keepout keepout_zones")

    map_width, map_height, _ = read_pgm(image_path(args.map_yaml, map_meta))
    mask_width, mask_height, mask_pixels = read_pgm(image_path(args.mask_yaml, mask_meta))
    require((mask_width, mask_height) == (map_width, map_height), "mask size differs from map")
    require(mask_meta["resolution"] == map_meta["resolution"], "mask resolution differs from map")
    require(mask_meta["origin"][:2] == map_meta["origin"][:2] and mask_meta["origin"][2] == 0,
            "mask origin x/y must match map and yaw must be 0")
    require(0 in mask_pixels, "mask contains no black keepout pixels")
    require(generated["map_yaml_sha256"] == sha256(args.map_yaml), "metadata map yaml hash differs")
    require(generated["map_pgm_sha256"] == sha256(image_path(args.map_yaml, map_meta)), "metadata map pgm hash differs")
    require(generated["route_sha256"] == sha256(args.route), "metadata route hash differs")
    require((generated["width"], generated["height"]) == (mask_width, mask_height), "metadata size differs")
    require(generated["resolution"] == mask_meta["resolution"], "metadata resolution differs")
    require(generated["origin"][:2] == mask_meta["origin"][:2], "metadata origin differs")

    params = yaml.safe_load(Path(args.nav2_params).read_text(encoding="utf-8"))
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]
    keepout = global_params["keepout_filter"]
    require("keepout_filter" in global_params["filters"], "global keepout_filter missing from filters")
    require(keepout["plugin"] == "nav2_costmap_2d::KeepoutFilter", "wrong keepout plugin")
    require(keepout["filter_info_topic"] == "keepout_costmap_filter_info", "wrong filter info topic")
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    require("keepout_filter" in local_params["filters"], "local keepout_filter missing from filters")
    require(local_params["keepout_filter"]["enabled"] is False, "local keepout should default false")

    if args.ros:
        check_ros()
    print("keepout setup OK")


if __name__ == "__main__":
    main()
