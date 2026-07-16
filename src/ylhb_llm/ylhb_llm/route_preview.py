import hashlib
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    from ylhb_mobile_bridge.patrol_route_store import (
        expand_route_targets,
        get_route,
        load_route_file,
        validate_route_map_binding,
    )
except ModuleNotFoundError:
    bridge_package = Path(__file__).resolve().parents[2] / "ylhb_mobile_bridge"
    if bridge_package.exists():
        sys.path.insert(0, str(bridge_package))
    from ylhb_mobile_bridge.patrol_route_store import (
        expand_route_targets,
        get_route,
        load_route_file,
        validate_route_map_binding,
    )


WORKSPACE_DIR = Path(os.environ.get("WS_DIR", "/home/nvidia/ros2_DL")).expanduser()
DEFAULT_MAP_YAML = WORKSPACE_DIR / "maps" / "my_map.yaml"
ROUTE_PATTERN = re.compile(r"^route_patrol_(\d+)\.json$")
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True)
class RouteSelection:
    path: Optional[Path]
    error: str = ""


def map_to_pixel(
    x: float,
    y: float,
    *,
    resolution: float,
    origin: Tuple[float, float],
    image_height: int,
) -> Tuple[int, int]:
    px = int(round((float(x) - origin[0]) / resolution))
    py = int(round(image_height - 1 - ((float(y) - origin[1]) / resolution)))
    return px, py


def select_latest_route_file(directory: Path) -> RouteSelection:
    candidates: List[Tuple[int, float, str, Path]] = []
    for path in Path(directory).glob("route_patrol_*.json"):
        match = ROUTE_PATTERN.match(path.name)
        if not match:
            continue
        candidates.append((int(match.group(1)), path.stat().st_mtime, path.name, path))
    if not candidates:
        return RouteSelection(None, "未找到正式巡逻路线文件")
    return RouteSelection(max(candidates)[3])


def build_patrol_tasks(targets: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    tasks: Dict[str, Dict[str, Any]] = {}
    for target in targets:
        target_id = str(target.get("id") or "")
        if not target_id:
            continue
        tasks[target_id] = {
            "target_id": target_id,
            "name": str(target.get("name") or target_id),
            "task_duration_sec": target.get("task_duration_sec", 0),
            "task_type": "未配置",
            "task_params": {},
            "task_status": "预留接口",
        }
    return tasks


def _parse_map_yaml(path: Path) -> Dict[str, Any]:
    image = ""
    resolution = 0.05
    origin: List[float] = [0.0, 0.0, 0.0]
    lines = path.read_text(encoding="utf-8").splitlines()
    index = 0
    while index < len(lines):
        raw = lines[index].strip()
        if raw.startswith("image:"):
            image = raw.split(":", 1)[1].strip().strip("\"'")
        elif raw.startswith("resolution:"):
            resolution = float(raw.split(":", 1)[1].strip())
        elif raw.startswith("origin:"):
            tail = raw.split(":", 1)[1].strip()
            if tail.startswith("["):
                origin = [float(item.strip()) for item in tail.strip("[]").split(",")[:3]]
            else:
                values = []
                for offset in range(1, 4):
                    if index + offset >= len(lines):
                        break
                    item = lines[index + offset].strip()
                    if item.startswith("-"):
                        values.append(float(item[1:].strip()))
                if values:
                    origin = values[:3]
        index += 1
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    return {"image": image_path, "resolution": resolution, "origin": origin}


def _read_pgm_size(path: Path) -> Tuple[int, int]:
    with path.open("rb") as file_obj:
        magic = file_obj.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"unsupported PGM format: {magic!r}")
        tokens: List[bytes] = []
        while len(tokens) < 3:
            line = file_obj.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        if len(tokens) < 3:
            raise ValueError("invalid PGM header")
        return int(tokens[0]), int(tokens[1])


def _route_signature(
    paths: Iterable[Path], active_route_id: str, preview_mode: str
) -> str:
    digest = hashlib.sha256(
        f"route-preview-v5:{preview_mode}:{active_route_id}:footprint-v1:warn=0.20".encode("utf-8")
    )
    for path in paths:
        digest.update(str(path).encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def _active_route(route_data: Dict[str, Any]) -> Dict[str, Any]:
    routes = route_data.get("routes") or []
    active_route_id = route_data.get("active_route_id")
    if active_route_id:
        for route in routes:
            if route.get("id") == active_route_id:
                return route
    return routes[0] if routes else {}


def validate_png(path: Path) -> Tuple[bool, str]:
    path = Path(path)
    if not path.exists():
        return False, "missing"
    try:
        if path.stat().st_size <= 0:
            return False, "empty"
        with path.open("rb") as file_obj:
            if file_obj.read(len(PNG_MAGIC)) != PNG_MAGIC:
                return False, "invalid png header"
        from PIL import Image

        with Image.open(path) as image:
            image.verify()
        return True, ""
    except Exception as exc:
        return False, str(exc)


def needs_regenerate_preview(output_path: Path, force: bool) -> bool:
    if force:
        return True
    valid, _error = validate_png(output_path)
    if valid:
        return False
    try:
        output_path.unlink(missing_ok=True)
    except OSError:
        pass
    return True


def _draw_preview(
    map_image: Path,
    output_path: Path,
    width: int,
    height: int,
    resolution: float,
    origin: Tuple[float, float],
    route_data: Dict[str, Any],
    route: Dict[str, Any],
    preview_mode: str = "route_focus",
) -> None:
    from PIL import Image, ImageDraw, ImageFont

    try:
        image = Image.open(map_image).convert("RGB")
    except Exception:
        image = Image.new("RGB", (width, height), "#F3F4F6")
    width, height = image.size
    source_height = height
    crop_left = crop_top = 0
    targets = {str(item.get("id")): item for item in route_data.get("targets", [])}
    ordered = [targets[target_id] for target_id in route.get("target_ids", []) if target_id in targets]
    start_pose = (route_data.get("start_pose") or {}).get("pose") or {}
    if preview_mode == "route_focus":
        focus = [(float(start_pose.get("x", 0.0)), float(start_pose.get("y", 0.0)))]
        focus += [(float((item.get("pose") or {}).get("x", 0.0)), float((item.get("pose") or {}).get("y", 0.0))) for item in ordered]
        for zone in route_data.get("keepout_zones", []):
            if zone.get("enabled") is True and zone.get("type") == "hard_keepout":
                focus += [(float(point["x"]), float(point["y"])) for point in zone.get("polygon", [])]
        min_x, max_x = min(point[0] for point in focus), max(point[0] for point in focus)
        min_y, max_y = min(point[1] for point in focus), max(point[1] for point in focus)
        margin = max(0.5, max(max_x - min_x, max_y - min_y) * 0.1) + 0.38
        min_x, max_x = max(origin[0], min_x - margin), min(origin[0] + width * resolution, max_x + margin)
        min_y, max_y = max(origin[1], min_y - margin), min(origin[1] + height * resolution, max_y + margin)
        crop_left = max(0, int(math.floor((min_x - origin[0]) / resolution)))
        crop_right = min(width, int(math.ceil((max_x - origin[0]) / resolution)))
        crop_top = max(0, int(math.floor(height - (max_y - origin[1]) / resolution)))
        crop_bottom = min(height, int(math.ceil(height - (min_y - origin[1]) / resolution)))
        image = image.crop((crop_left, crop_top, max(crop_left + 1, crop_right), max(crop_top + 1, crop_bottom)))
        width, height = image.size
    scale = 1.0
    if width < 1000:
        scale = 1000.0 / max(1, width)
        resample = getattr(getattr(Image, "Resampling", Image), "NEAREST")
        image = image.resize((1000, max(1, int(round(height * scale)))), resample)
        width, height = image.size
    shortest_side = min(width, height)
    route_line_width = max(3, min(7, int(round(shortest_side / 180))))
    marker_radius = max(7, min(12, int(round(shortest_side / 90))))
    font_size = max(13, min(18, int(round(shortest_side / 65))))
    font = ImageFont.load_default()
    for font_path in (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(font_path).exists():
            try:
                font = ImageFont.truetype(font_path, font_size)
                break
            except OSError:
                pass

    def scaled_pixel(x_value: float, y_value: float) -> Tuple[int, int]:
        px, py = map_to_pixel(
            x_value,
            y_value,
            resolution=resolution,
            origin=origin,
            image_height=source_height,
        )
        return int(round((px - crop_left) * scale)), int(round((py - crop_top) * scale))

    route_points = [
        scaled_pixel(
            start_pose.get("x", 0.0),
            start_pose.get("y", 0.0),
        )
    ]
    for target in ordered:
        pose = target.get("pose") or {}
        route_points.append(scaled_pixel(pose.get("x", 0.0), pose.get("y", 0.0)))
    keepout_overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
    keepout_draw = ImageDraw.Draw(keepout_overlay)
    for zone in route_data.get("keepout_zones", []):
        if zone.get("enabled") is True and zone.get("type") == "hard_keepout":
            zone_points = [scaled_pixel(point["x"], point["y"]) for point in zone.get("polygon", [])]
            if len(zone_points) >= 3:
                keepout_draw.polygon(zone_points, fill=(239, 68, 68, 72), outline=(153, 27, 27, 255))
                keepout_draw.line(zone_points + [zone_points[0]], fill=(153, 27, 27, 255), width=2)
    image = Image.alpha_composite(image.convert("RGBA"), keepout_overlay).convert("RGB")
    draw = ImageDraw.Draw(image)

    footprint = [
        (0.18650, 0.00000), (0.16543, 0.10591), (0.10544, 0.19569), (0.01566, 0.25568),
        (-0.09025, 0.27675), (-0.19616, 0.25568), (-0.28594, 0.19569), (-0.34593, 0.10591),
        (-0.36700, 0.00000), (-0.34593, -0.10591), (-0.28594, -0.19569), (-0.19616, -0.25568),
        (-0.09025, -0.27675), (0.01566, -0.25568), (0.10544, -0.19569), (0.16543, -0.10591),
    ]
    for pose, status in [(start_pose, (route_data.get("safety") or {}).get("validation_status", "ok"))] + [
        (target.get("pose") or {}, (target.get("safety") or {}).get("validation_status", "ok")) for target in ordered
    ]:
        yaw = float(pose.get("yaw", 0.0))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        footprint_points = [
            scaled_pixel(
                float(pose.get("x", 0.0)) + (x + (0.01 if x > 0 else -0.01 if x < 0 else 0.0)) * cosine - (y + (0.01 if y > 0 else -0.01 if y < 0 else 0.0)) * sine,
                float(pose.get("y", 0.0)) + (x + (0.01 if x > 0 else -0.01 if x < 0 else 0.0)) * sine + (y + (0.01 if y > 0 else -0.01 if y < 0 else 0.0)) * cosine,
            )
            for x, y in footprint
        ]
        draw.line(
            footprint_points + [footprint_points[0]],
            fill="#DC2626" if status == "unsafe" else "#D97706" if status == "warning" else "#15803D",
            width=1,
        )

    line_points = route_points
    if route.get("return_to_start") and len(line_points) > 1:
        line_points = line_points + [line_points[0]]
    for first, second in zip(line_points, line_points[1:]):
        draw.line([first, second], fill="#FFFFFF", width=route_line_width + 4)
        draw.line([first, second], fill="#2563EB", width=route_line_width)

    label_overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
    label_draw = ImageDraw.Draw(label_overlay)
    labels: List[Tuple[Tuple[int, int], str]] = []

    start_x, start_y = route_points[0]
    draw.ellipse(
        [start_x - marker_radius - 3, start_y - marker_radius - 3, start_x + marker_radius + 3, start_y + marker_radius + 3],
        fill="#FFFFFF",
    )
    draw.ellipse(
        [start_x - marker_radius, start_y - marker_radius, start_x + marker_radius, start_y + marker_radius],
        fill="#F97316",
        outline="#C2410C",
        width=2,
    )
    labels.append(((start_x + marker_radius + 7, start_y - marker_radius - 5), "S 起点"))

    for index, target in enumerate(ordered, start=1):
        pose = target.get("pose") or {}
        px, py = scaled_pixel(pose.get("x", 0.0), pose.get("y", 0.0))
        target_status = (target.get("safety") or {}).get("validation_status", "ok")
        marker_color = "#DC2626" if target_status == "unsafe" else "#EAB308" if target_status == "warning" else "#22C55E"
        draw.ellipse(
            [px - marker_radius - 3, py - marker_radius - 3, px + marker_radius + 3, py + marker_radius + 3],
            fill="#FFFFFF",
        )
        draw.ellipse(
            [px - marker_radius, py - marker_radius, px + marker_radius, py + marker_radius],
            fill=marker_color,
            outline="#111827",
            width=1,
        )
        number = str(index)
        number_box = draw.textbbox((0, 0), number, font=font)
        draw.text(
            (px - (number_box[2] - number_box[0]) / 2, py - (number_box[3] - number_box[1]) / 2 - 1),
            number,
            fill="#FFFFFF",
            font=font,
        )
        yaw = float(pose.get("yaw", 0.0))
        end = (px + math.cos(yaw) * (marker_radius + 18), py - math.sin(yaw) * (marker_radius + 18))
        draw.line([(px, py), end], fill="#111827", width=2)
        draw.ellipse([end[0] - 3, end[1] - 3, end[0] + 3, end[1] + 3], fill="#111827")
        labels.append(((px + marker_radius + 7, py + marker_radius + 3), str(target.get("name") or target.get("id") or index)))

    for position, label in labels:
        bounds = draw.textbbox(position, label, font=font)
        label_draw.rounded_rectangle(
            [bounds[0] - 5, bounds[1] - 3, bounds[2] + 5, bounds[3] + 3],
            radius=5,
            fill=(255, 255, 255, 220),
            outline=(203, 213, 225, 230),
        )
    image = Image.alpha_composite(image.convert("RGBA"), label_overlay).convert("RGB")
    draw = ImageDraw.Draw(image)
    for position, label in labels:
        draw.text(position, label, fill="#111827", font=font)

    overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    legend_width = min(width - 16, 390)
    overlay_draw.rounded_rectangle([8, 8, legend_width, 66], radius=8, fill=(255, 255, 255, 220), outline="#CBD5E1")
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(image)
    draw.line([(18, 24), (48, 24)], fill="#FFFFFF", width=route_line_width + 3)
    draw.line([(18, 24), (48, 24)], fill="#2563EB", width=route_line_width)
    draw.text((56, 14), "蓝色路线", fill="#111827", font=font)
    legend_items = [(145, "#22C55E", "绿色安全"), (235, "#EAB308", "黄色警告")]
    for x_value, color, label in legend_items:
        draw.ellipse([x_value, 18, x_value + 10, 28], fill=color, outline="#FFFFFF")
        draw.text((x_value + 14, 14), label, fill="#111827", font=font)
    draw.ellipse([18, 43, 28, 53], fill="#DC2626", outline="#FFFFFF")
    draw.text((32, 37), "红色危险", fill="#111827", font=font)
    draw.rectangle([145, 43, 157, 53], fill="#EF4444", outline="#991B1B")
    draw.text((163, 37), "红色区域禁行", fill="#111827", font=font)
    tmp_path = output_path.with_suffix(".tmp.png")
    try:
        image.save(tmp_path, "PNG")
        valid, error = validate_png(tmp_path)
        if not valid:
            raise RuntimeError(f"临时 PNG 无效: {error}")
        os.replace(tmp_path, output_path)
    finally:
        try:
            tmp_path.unlink(missing_ok=True)
        except OSError:
            pass


def generate_route_preview(
    map_yaml_path: Path = DEFAULT_MAP_YAML,
    force: bool = False,
    route_file_path: Optional[Path] = None,
    preview_mode: str = "route_focus",
) -> Dict[str, Any]:
    map_yaml = Path(map_yaml_path).expanduser().resolve()
    base_failure = {
        "ok": False,
        "preview_type": "route_overlay",
        "overlay_ok": False,
        "image_url": "",
        "image_path": "",
        "image_exists": False,
        "image_valid": False,
        "image_error": "",
        "image_format": "png",
        "image_bytes": 0,
        "image_mtime_ns": 0,
        "source": str(map_yaml),
        "route_file": "",
        "active_route_id": "",
        "route_name": "",
        "target_count": 0,
        "targets": [],
        "map_identity": {},
        "keepout_count": 0,
        "safety_warnings": [],
        "preview_mode": preview_mode,
    }
    if route_file_path:
        route_path = Path(route_file_path).expanduser().resolve()
    else:
        selection = select_latest_route_file(map_yaml.parent)
        if selection.path is None:
            return {**base_failure, "message": f"路线预览失败: {selection.error}"}
        route_path = selection.path.resolve()
    if preview_mode not in {"route_focus", "full_map"}:
        return {**base_failure, "message": f"路线预览失败: unsupported preview mode: {preview_mode}"}
    try:
        map_info = _parse_map_yaml(map_yaml)
        width, height = _read_pgm_size(map_info["image"])
        route_data = load_route_file(str(route_path))
        route_data = validate_route_map_binding(route_data, map_yaml)
        active_route_id = str(route_data.get("active_route_id") or "")
        if not active_route_id and route_data.get("routes"):
            active_route_id = str(route_data["routes"][0]["id"])
        route = get_route(route_data, active_route_id)
        active_route_id = str(route.get("id") or route_data.get("active_route_id") or "")
        signature = _route_signature([map_yaml, map_info["image"], route_path], active_route_id, preview_mode)
        output_path = Path("/tmp") / f"ylhb_route_preview_{signature}.png"
        if needs_regenerate_preview(output_path, force):
            try:
                _draw_preview(
                    map_info["image"],
                    output_path,
                    width,
                    height,
                    float(map_info["resolution"]),
                    (float(map_info["origin"][0]), float(map_info["origin"][1])),
                    route_data,
                    route,
                    preview_mode,
                )
            except ModuleNotFoundError as exc:
                if exc.name == "PIL" or "PIL" in str(exc):
                    raise RuntimeError("缺少 python3-pil/Pillow 依赖") from exc
                raise
        image_valid, image_error = validate_png(output_path)
        if not image_valid:
            try:
                output_path.unlink(missing_ok=True)
            except OSError:
                pass
            return {
                **base_failure,
                "message": f"路线预览图生成失败: {image_error}",
                "image_error": image_error,
                "image_path": str(output_path),
                "image_url": output_path.as_uri(),
                "image_exists": output_path.exists(),
                "route_file": str(route_path),
            }
        targets = expand_route_targets(route_data, active_route_id)
        image_stat = output_path.stat() if output_path.exists() else None
        return {
            "ok": True,
            "preview_type": "route_overlay",
            "overlay_ok": True,
            "message": "路线预览已生成",
            "image_url": output_path.as_uri(),
            "image_path": str(output_path),
            "image_exists": output_path.exists(),
            "image_valid": image_valid,
            "image_error": image_error,
            "image_format": "png",
            "image_bytes": image_stat.st_size if image_stat else 0,
            "image_mtime_ns": image_stat.st_mtime_ns if image_stat else 0,
            "source": output_path.as_uri(),
            "route_file": str(route_path),
            "active_route_id": active_route_id,
            "route_name": str(route.get("name") or active_route_id),
            "target_count": len(targets),
            "targets": targets,
            "map_identity": route_data.get("map", {}),
            "map_resolution": float(map_info["resolution"]),
            "keepout_count": sum(1 for zone in route_data.get("keepout_zones", []) if zone.get("enabled") is True and zone.get("type") == "hard_keepout"),
            "safety_warnings": list((route_data.get("safety") or {}).get("warnings", [])),
            "preview_mode": preview_mode,
        }
    except Exception as exc:
        return {
            **base_failure,
            "message": f"路线预览失败: {exc}",
            "route_file": str(route_path),
        }
