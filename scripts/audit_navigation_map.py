#!/usr/bin/env python3
"""Compare Nav2 trinary map thresholds without modifying map assets."""

import argparse
import ast
import json
import math
import re
from collections import deque
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


FREE = "free"
UNKNOWN = "unknown"
OCCUPIED = "occupied"


def parse_scalar(value: str):
    value = value.strip()
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        return ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return value.strip("\"'")


def load_map_metadata(path: Path) -> Dict[str, object]:
    metadata: Dict[str, object] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        metadata[key.strip()] = parse_scalar(value)
    required = ("image", "mode", "resolution", "origin", "negate", "free_thresh", "occupied_thresh")
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError(f"map metadata missing: {', '.join(missing)}")
    if metadata["mode"] != "trinary":
        raise ValueError("only mode=trinary is supported")
    return metadata


def read_pgm(path: Path) -> Tuple[int, int, bytes]:
    data = path.read_bytes()
    offset = 0

    def token() -> str:
        nonlocal offset
        while offset < len(data):
            if data[offset] in b" \t\r\n\v\f":
                offset += 1
            elif data[offset] == ord("#"):
                while offset < len(data) and data[offset] not in b"\r\n":
                    offset += 1
            else:
                break
        start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n\v\f#":
            offset += 1
        if start == offset:
            raise ValueError(f"invalid PGM header: {path}")
        return data[start:offset].decode("ascii")

    magic = token()
    width = int(token())
    height = int(token())
    maximum = int(token())
    if offset >= len(data) or data[offset] not in b" \t\r\n\v\f":
        raise ValueError(f"PGM header missing raster separator: {path}")
    if data[offset:offset + 2] == b"\r\n":
        offset += 2
    else:
        offset += 1
    pixels = data[offset:]
    if magic != "P5" or maximum != 255 or len(pixels) != width * height:
        raise ValueError(f"invalid P5/255 PGM: {path}")
    return width, height, pixels


def classify(gray: int, negate: int, free_thresh: float, occupied_thresh: float) -> str:
    occupancy = (gray if negate else 255 - gray) / 255.0
    if occupancy > occupied_thresh:
        return OCCUPIED
    if occupancy < free_thresh:
        return FREE
    return UNKNOWN


def classify_pixels(pixels: bytes, metadata: Dict[str, object], free_thresh: float) -> List[str]:
    return [
        classify(
            gray,
            int(metadata["negate"]),
            free_thresh,
            float(metadata["occupied_thresh"]),
        )
        for gray in pixels
    ]


def statistics(classes: Sequence[str], free_thresh: float) -> Dict[str, object]:
    total = len(classes)
    counts = {kind: classes.count(kind) for kind in (FREE, UNKNOWN, OCCUPIED)}
    return {
        "free_thresh": free_thresh,
        "free_cells": counts[FREE],
        "unknown_cells": counts[UNKNOWN],
        "occupied_cells": counts[OCCUPIED],
        "free_ratio": counts[FREE] / total,
        "unknown_ratio": counts[UNKNOWN] / total,
        "occupied_ratio": counts[OCCUPIED] / total,
    }


def read_footprint(nav2_params: Path) -> Tuple[List[Tuple[float, float]], float]:
    source = nav2_params.read_text(encoding="utf-8")
    footprint_match = re.search(r"^\s*footprint:\s*(.+)$", source, flags=re.MULTILINE)
    padding_match = re.search(r"^\s*footprint_padding:\s*([^#\n]+)", source, flags=re.MULTILINE)
    if footprint_match is None or padding_match is None:
        raise ValueError("normal local_costmap footprint or footprint_padding is missing")
    value = ast.literal_eval(footprint_match.group(1).strip())
    if isinstance(value, str):
        value = ast.literal_eval(value)
    footprint = [(float(x), float(y)) for x, y in value]
    return footprint, float(padding_match.group(1).strip())


def padded_footprint(footprint: Iterable[Tuple[float, float]], padding: float) -> List[Tuple[float, float]]:
    return [
        (
            x + (padding if x > 0 else -padding if x < 0 else 0.0),
            y + (padding if y > 0 else -padding if y < 0 else 0.0),
        )
        for x, y in footprint
    ]


def world_to_cell(x: float, y: float, metadata: Dict[str, object], width: int, height: int):
    resolution = float(metadata["resolution"])
    origin_x, origin_y = metadata["origin"][:2]
    cell_x = math.floor((x - float(origin_x)) / resolution)
    cell_y = math.floor((y - float(origin_y)) / resolution)
    if not (0 <= cell_x < width and 0 <= cell_y < height):
        return None
    return int(cell_x), int(cell_y)


def class_at_cell(classes: Sequence[str], cell: Tuple[int, int], width: int, height: int) -> str:
    cell_x, cell_y = cell
    return classes[(height - 1 - cell_y) * width + cell_x]


def rotate_footprint(pose: Dict[str, object], footprint: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    yaw = float(pose.get("yaw", 0.0))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x, y = float(pose["x"]), float(pose["y"])
    return [
        (x + px * cosine - py * sine, y + px * sine + py * cosine)
        for px, py in footprint
    ]


def point_in_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[index - 1]
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def orientation(first, second, third) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (second[1] - first[1]) * (third[0] - first[0])


def on_segment(first, second, point) -> bool:
    return (
        min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
        and min(first[1], second[1]) <= point[1] <= max(first[1], second[1])
    )


def segments_intersect(first, second, third, fourth) -> bool:
    one = orientation(first, second, third)
    two = orientation(first, second, fourth)
    three = orientation(third, fourth, first)
    four = orientation(third, fourth, second)
    if one == 0 and on_segment(first, second, third):
        return True
    if two == 0 and on_segment(first, second, fourth):
        return True
    if three == 0 and on_segment(third, fourth, first):
        return True
    if four == 0 and on_segment(third, fourth, second):
        return True
    return (one > 0) != (two > 0) and (three > 0) != (four > 0)


def polygons_intersect(first: Sequence[Tuple[float, float]], second: Sequence[Tuple[float, float]]) -> bool:
    if any(point_in_polygon(point, second) for point in first):
        return True
    if any(point_in_polygon(point, first) for point in second):
        return True
    return any(
        segments_intersect(point, first[(index + 1) % len(first)], other, second[(other_index + 1) % len(second)])
        for index, point in enumerate(first)
        for other_index, other in enumerate(second)
    )


def footprint_cells(
    polygon: Sequence[Tuple[float, float]], metadata: Dict[str, object], width: int, height: int
) -> Tuple[List[Tuple[int, int]], bool]:
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    min_x, max_x = min(x for x, _ in polygon), max(x for x, _ in polygon)
    min_y, max_y = min(y for _, y in polygon), max(y for _, y in polygon)
    outside = min_x < origin_x or min_y < origin_y or max_x >= origin_x + width * resolution or max_y >= origin_y + height * resolution
    start_x = max(0, math.floor((min_x - origin_x) / resolution))
    end_x = min(width - 1, math.floor((max_x - origin_x) / resolution))
    start_y = max(0, math.floor((min_y - origin_y) / resolution))
    end_y = min(height - 1, math.floor((max_y - origin_y) / resolution))
    cells = []
    for cell_y in range(int(start_y), int(end_y) + 1):
        for cell_x in range(int(start_x), int(end_x) + 1):
            left = origin_x + cell_x * resolution
            bottom = origin_y + cell_y * resolution
            cell_polygon = [
                (left, bottom),
                (left + resolution, bottom),
                (left + resolution, bottom + resolution),
                (left, bottom + resolution),
            ]
            if polygons_intersect(polygon, cell_polygon):
                cells.append((cell_x, cell_y))
    return cells, outside


def footprint_report(
    pose: Dict[str, object], footprint: Sequence[Tuple[float, float]], classes: Sequence[str], metadata: Dict[str, object], width: int, height: int
) -> Dict[str, object]:
    cells, outside = footprint_cells(rotate_footprint(pose, footprint), metadata, width, height)
    counts = {kind: 0 for kind in (FREE, UNKNOWN, OCCUPIED)}
    for cell in cells:
        counts[class_at_cell(classes, cell, width, height)] += 1
    return {
        "outside_map": outside,
        "covered_cells": len(cells),
        "free_cells": counts[FREE],
        "unknown_cells": counts[UNKNOWN],
        "occupied_cells": counts[OCCUPIED],
        "touches_unknown": counts[UNKNOWN] > 0,
        "touches_occupied": outside or counts[OCCUPIED] > 0,
        "safe": not outside and counts[UNKNOWN] == 0 and counts[OCCUPIED] == 0,
    }


def route_poses(route: Dict[str, object]):
    start = (route.get("start_pose") or {}).get("pose") or {}
    yield "start_pose", start
    for target in route.get("targets") or []:
        yield str(target.get("id") or "target"), target.get("pose") or {}


def pose_reports(
    route: Dict[str, object], footprints: Sequence[Tuple[float, float]], maps: Dict[float, Sequence[str]], metadata: Dict[str, object], width: int, height: int
) -> Dict[str, object]:
    results = {}
    for name, pose in route_poses(route):
        x, y = float(pose["x"]), float(pose["y"])
        cell = world_to_cell(x, y, metadata, width, height)
        item = {
            "world_pose": {"x": x, "y": y, "yaw": float(pose.get("yaw", 0.0))},
            "map_cell": None if cell is None else {"x": cell[0], "y": cell[1]},
            "classifications": {},
            "footprint": {},
        }
        for threshold, classes in maps.items():
            cell_class = "outside_map" if cell is None else class_at_cell(classes, cell, width, height)
            item["classifications"][str(threshold)] = cell_class
            item[f"class_at_{threshold:g}"] = cell_class
            item["footprint"][str(threshold)] = footprint_report(pose, footprints, classes, metadata, width, height)
        results[name] = item
    return results


def connectivity(
    route: Dict[str, object], reports: Dict[str, object], footprint: Sequence[Tuple[float, float]], classes: Sequence[str], threshold: float, metadata: Dict[str, object], width: int, height: int
) -> Dict[str, object]:
    start_cell = world_to_cell(float(route["start_pose"]["pose"]["x"]), float(route["start_pose"]["pose"]["y"]), metadata, width, height)
    visited = set()
    if start_cell is not None and class_at_cell(classes, start_cell, width, height) == FREE:
        visited.add(start_cell)
        queue = deque([start_cell])
        while queue:
            cell_x, cell_y = queue.popleft()
            for next_x in range(cell_x - 1, cell_x + 2):
                for next_y in range(cell_y - 1, cell_y + 2):
                    candidate = (next_x, next_y)
                    if candidate == (cell_x, cell_y) or candidate in visited:
                        continue
                    if 0 <= next_x < width and 0 <= next_y < height and class_at_cell(classes, candidate, width, height) == FREE:
                        visited.add(candidate)
                        queue.append(candidate)
    targets = {}
    for target in route.get("targets") or []:
        name = str(target.get("id") or "target")
        pose = target.get("pose") or {}
        cell = world_to_cell(float(pose["x"]), float(pose["y"]), metadata, width, height)
        targets[name] = {
            "center_reachable": cell in visited if cell is not None else False,
            "footprint_safe": footprint_report(pose, footprint, classes, metadata, width, height)["safe"],
        }
    return {
        "free_thresh": threshold,
        "start_is_free": start_cell is not None and class_at_cell(classes, start_cell, width, height) == FREE,
        "reachable_free_cells": len(visited),
        "targets": targets,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_yaml")
    parser.add_argument("--route", required=True)
    parser.add_argument("--nav2-params", required=True)
    parser.add_argument("--compare-free-thresh", nargs=2, type=float, required=True, metavar=("FIRST", "SECOND"))
    parser.add_argument("--report", required=True)
    args = parser.parse_args()

    map_yaml = Path(args.map_yaml)
    metadata = load_map_metadata(map_yaml)
    image = Path(str(metadata["image"]))
    image = image if image.is_absolute() else map_yaml.parent / image
    width, height, pixels = read_pgm(image)
    route = json.loads(Path(args.route).read_text(encoding="utf-8"))
    footprint, padding = read_footprint(Path(args.nav2_params))
    footprint = padded_footprint(footprint, padding)
    thresholds = tuple(args.compare_free_thresh)
    maps = {threshold: classify_pixels(pixels, metadata, threshold) for threshold in thresholds}
    first, second = thresholds
    changes = {
        "cells_unknown_to_free": sum(a == UNKNOWN and b == FREE for a, b in zip(maps[first], maps[second])),
        "cells_free_to_unknown": sum(a == FREE and b == UNKNOWN for a, b in zip(maps[first], maps[second])),
        "cells_unchanged": sum(a == b for a, b in zip(maps[first], maps[second])),
    }
    reports = pose_reports(route, footprint, maps, metadata, width, height)
    connectivity_report = {
        str(threshold): connectivity(route, reports, footprint, maps[threshold], threshold, metadata, width, height)
        for threshold in thresholds
    }
    summaries = []
    if first == 0.19 and second == 0.25:
        before = connectivity_report[str(first)]["targets"]
        after = connectivity_report[str(second)]["targets"]
        if any(not before[name]["center_reachable"] and after[name]["center_reachable"] for name in before):
            summaries.append("free_thresh=0.19 causes route connectivity loss")
            summaries.append("free_thresh=0.25 restores connectivity")
    report = {
        "map": str(map_yaml),
        "image": str(image),
        "resolution": float(metadata["resolution"]),
        "origin": metadata["origin"],
        "footprint_padding": padding,
        "thresholds": [statistics(maps[threshold], threshold) for threshold in thresholds],
        "threshold_change": changes,
        "poses": reports,
        "connectivity": connectivity_report,
        "summary": summaries,
    }
    report_path = Path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
