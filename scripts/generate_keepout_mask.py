#!/usr/bin/env python3
import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ylhb_mobile_bridge"))
from ylhb_mobile_bridge.patrol_route_store import validate_route_map_binding  # noqa: E402


def read_pgm_header(path):
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
    if magic != "P5" or maximum != 255:
        raise ValueError(f"{path} must be 8-bit binary PGM")
    return width, height


def point_in_polygon(x, y, polygon):
    inside = False
    for index, point in enumerate(polygon):
        other = polygon[index - 1]
        if (point["y"] > y) != (other["y"] > y):
            at_x = (other["x"] - point["x"]) * (y - point["y"]) / (
                other["y"] - point["y"]
            ) + point["x"]
            if x < at_x:
                inside = not inside
    return inside


def point_segment_distance(x, y, first, second):
    dx = float(second["x"]) - float(first["x"])
    dy = float(second["y"]) - float(first["y"])
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(x - float(first["x"]), y - float(first["y"]))
    ratio = max(0.0, min(1.0, (
        (x - float(first["x"])) * dx + (y - float(first["y"])) * dy
    ) / length_sq))
    return math.hypot(
        x - (float(first["x"]) + ratio * dx),
        y - (float(first["y"]) + ratio * dy),
    )


def distance_to_polygon(x, y, polygon):
    return min(
        point_segment_distance(x, y, point, polygon[(index + 1) % len(polygon)])
        for index, point in enumerate(polygon)
    )


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stage_write(path, data):
    temporary = Path(str(path) + ".tmp")
    with temporary.open("wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    return temporary


def make_pixels(width, height, resolution, origin, zones, hard_paddings):
    pixels = bytearray(width * height)
    for py in range(height):
        y = origin[1] + (height - py - 0.5) * resolution
        for px in range(width):
            x = origin[0] + (px + 0.5) * resolution
            pixel_value = 0
            for zone in zones:
                polygon = zone["polygon"]
                distance = 0.0 if point_in_polygon(x, y, polygon) else distance_to_polygon(
                    x, y, polygon
                )
                hard_padding = hard_paddings[zone["id"]]
                if distance <= hard_padding:
                    zone_value = 100
                else:
                    zone_value = 0
                pixel_value = max(pixel_value, zone_value)
            pixels[py * width + px] = pixel_value
    return pixels


def mask_yaml(image, resolution, origin):
    return {
        "image": image,
        "mode": "raw",
        "resolution": resolution,
        "origin": [float(origin[0]), float(origin[1]), 0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }


def mask_stats(pixels):
    return {
        "hard_cells": pixels.count(100),
        "weighted_cells": sum(1 for value in pixels if 1 <= value <= 99),
        "free_cells": pixels.count(0),
    }


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
    map_data = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    map_image = Path(map_data["image"])
    if not map_image.is_absolute():
        map_image = map_path.parent / map_image
    width, height = read_pgm_header(map_image)
    route = validate_route_map_binding(
        json.loads(route_path.read_text(encoding="utf-8")), map_path
    )
    zones = [
        zone for zone in route.get("keepout_zones", [])
        if zone.get("enabled") is True and zone.get("type") == "hard_keepout"
    ]

    resolution = float(map_data["resolution"])
    origin = map_data["origin"]
    requested_padding = {
        zone["id"]: float(zone.get("mask_padding_m", resolution)) for zone in zones
    }

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "global_pgm": output_dir / "keepout_global_mask.pgm",
        "global_yaml": output_dir / "keepout_global_mask.yaml",
        "local_pgm": output_dir / "keepout_local_mask.pgm",
        "local_yaml": output_dir / "keepout_local_mask.yaml",
        "metadata": output_dir / "keepout_masks.metadata.json",
    }
    metadata = {
        "map_yaml_sha256": sha256(map_path),
        "map_pgm_sha256": sha256(map_image),
        "route_sha256": sha256(route_path),
        "nav2_params_sha256": sha256(nav2_path),
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "zones": {
            zone["id"]: {
                "hard_padding_m": requested_padding[zone["id"]],
            }
            for zone in zones
        },
    }
    metadata["keepout_mode"] = "active" if zones else "all_free"
    metadata["enabled_hard_keepout_count"] = len(zones)
    global_pixels = make_pixels(
        width, height, resolution, origin, zones, requested_padding
    )
    local_pixels = make_pixels(
        width, height, resolution, origin, zones, requested_padding
    )
    metadata["global_mask"] = mask_stats(global_pixels)
    metadata["local_mask"] = mask_stats(local_pixels)
    staged = [
        (stage_write(outputs["global_pgm"], f"P5\n{width} {height}\n255\n".encode() + global_pixels), outputs["global_pgm"]),
        (stage_write(outputs["global_yaml"], yaml.safe_dump(mask_yaml(outputs["global_pgm"].name, resolution, origin), sort_keys=False).encode()), outputs["global_yaml"]),
        (stage_write(outputs["local_pgm"], f"P5\n{width} {height}\n255\n".encode() + local_pixels), outputs["local_pgm"]),
        (stage_write(outputs["local_yaml"], yaml.safe_dump(mask_yaml(outputs["local_pgm"].name, resolution, origin), sort_keys=False).encode()), outputs["local_yaml"]),
        (stage_write(outputs["metadata"], (json.dumps(metadata, indent=2) + "\n").encode()), outputs["metadata"]),
    ]
    try:
        for temporary, final in staged:
            os.replace(temporary, final)
    finally:
        for temporary, _final in staged:
            temporary.unlink(missing_ok=True)

    print(f"keepout mode: {metadata['keepout_mode']}")
    print(f"enabled hard keepout zones: {metadata['enabled_hard_keepout_count']}")
    if zones:
        for zone in zones:
            zone_id = zone["id"]
            print(f"{zone_id}:")
            print(f"hard padding = {requested_padding[zone_id]:.3f} m")
    else:
        print("no enabled hard_keepout zones; generated all-free global/local masks")
    print(outputs["global_yaml"])
    print(outputs["local_yaml"])


if __name__ == "__main__":
    main()
