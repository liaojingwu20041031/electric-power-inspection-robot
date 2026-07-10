#!/usr/bin/env python3
import argparse
import json
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

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if magic != "P5" or max_value != 255:
        raise ValueError(f"{path} must be 8-bit binary PGM (P5 max 255)")
    return width, height


def point_in_polygon(x, y, polygon):
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        other = polygon[j]
        crosses = (point["y"] > y) != (other["y"] > y)
        if crosses:
            at_x = ((other["x"] - point["x"]) * (y - point["y"])) / (
                other["y"] - point["y"]
            ) + point["x"]
            if x < at_x:
                inside = not inside
        j = i
    return inside


def active_hard_zones(route):
    for zone in route.get("keepout_zones", []):
        if zone.get("enabled") is True and zone.get("type") == "hard_keepout":
            polygon = zone.get("polygon")
            if not isinstance(polygon, list) or len(polygon) < 3:
                raise ValueError(f"zone {zone.get('id', '<unknown>')} needs >=3 polygon points")
            yield polygon


def write_pgm(path, width, height, pixels):
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, dest="map_yaml")
    parser.add_argument("--route", required=True)
    parser.add_argument("--output-dir", default="maps/keepout")
    parser.add_argument("--name", default="keepout_mask_power_room_a")
    args = parser.parse_args()

    map_yaml_path = Path(args.map_yaml).expanduser()
    map_data = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    map_image = Path(map_data["image"])
    if not map_image.is_absolute():
        map_image = map_yaml_path.parent / map_image
    width, height = read_pgm_header(map_image)

    route = validate_route_map_binding(
        json.loads(Path(args.route).expanduser().read_text(encoding="utf-8")),
        map_yaml_path,
    )
    if route["version"] != 3:
        raise ValueError("keepout mask generation requires a v3 route with map binding")
    hard_zones = list(active_hard_zones(route))
    if not hard_zones:
        raise ValueError("route has no enabled hard_keepout keepout_zones")
    origin = map_data["origin"]
    resolution = float(map_data["resolution"])
    pixels = bytearray([254] * (width * height))

    for py in range(height):
        y = origin[1] + (height - (py + 0.5)) * resolution
        for px in range(width):
            x = origin[0] + (px + 0.5) * resolution
            if any(point_in_polygon(x, y, polygon) for polygon in hard_zones):
                pixels[py * width + px] = 0

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / f"{args.name}.pgm"
    yaml_path = output_dir / f"{args.name}.yaml"
    write_pgm(pgm_path, width, height, pixels)
    mask_yaml = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": resolution,
        "origin": [float(origin[0]), float(origin[1]), 0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": float(map_data["free_thresh"]),
    }
    yaml_path.write_text(yaml.safe_dump(mask_yaml, sort_keys=False), encoding="utf-8")
    print(f"wrote {yaml_path} and {pgm_path}")


if __name__ == "__main__":
    main()
