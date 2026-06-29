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
        resolve_route_file_path,
    )
except ModuleNotFoundError:
    bridge_package = Path(__file__).resolve().parents[2] / "ylhb_mobile_bridge"
    if bridge_package.exists():
        sys.path.insert(0, str(bridge_package))
    from ylhb_mobile_bridge.patrol_route_store import (
        expand_route_targets,
        get_route,
        load_route_file,
        resolve_route_file_path,
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


def _route_signature(paths: Iterable[Path], active_route_id: str) -> str:
    digest = hashlib.sha1(active_route_id.encode("utf-8"))
    for path in paths:
        stat = path.stat()
        digest.update(str(path).encode("utf-8"))
        digest.update(str(stat.st_mtime_ns).encode("ascii"))
        digest.update(str(stat.st_size).encode("ascii"))
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
) -> None:
    from PIL import Image, ImageDraw, ImageFont

    try:
        image = Image.open(map_image).convert("RGB")
    except Exception:
        image = Image.new("RGB", (width, height), "#F3F4F6")
    width, height = image.size
    scale = 1.0
    if width < 1000:
        scale = 1000.0 / max(1, width)
        resample = getattr(getattr(Image, "Resampling", Image), "NEAREST")
        image = image.resize((1000, max(1, int(round(height * scale)))), resample)
        width, height = image.size
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    for font_path in (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(font_path).exists():
            try:
                font = ImageFont.truetype(font_path, 14)
                break
            except OSError:
                pass

    targets = {str(item.get("id")): item for item in route_data.get("targets", [])}
    ordered = [targets[target_id] for target_id in route.get("target_ids", []) if target_id in targets]
    start_pose = (route_data.get("start_pose") or {}).get("pose") or {}

    def scaled_pixel(x_value: float, y_value: float) -> Tuple[int, int]:
        px, py = map_to_pixel(
            x_value,
            y_value,
            resolution=resolution,
            origin=origin,
            image_height=int(round(height / scale)),
        )
        return int(round(px * scale)), int(round(py * scale))

    points = [
        scaled_pixel(
            start_pose.get("x", 0.0),
            start_pose.get("y", 0.0),
        )
    ]
    for target in ordered:
        pose = target.get("pose") or {}
        points.append(scaled_pixel(pose.get("x", 0.0), pose.get("y", 0.0)))
    line_points = points + ([points[0]] if route.get("return_to_start") and len(points) > 1 else [])
    for first, second in zip(line_points, line_points[1:]):
        draw.line([first, second], fill="#2563EB", width=3)

    start_x, start_y = points[0]
    draw.ellipse(
        [start_x - 7, start_y - 7, start_x + 7, start_y + 7],
        fill="#F97316",
        outline="#DC2626",
        width=2,
    )
    draw.text((start_x + 9, start_y - 9), "S", fill="#DC2626", font=font)

    for index, target in enumerate(ordered, start=1):
        pose = target.get("pose") or {}
        px, py = scaled_pixel(pose.get("x", 0.0), pose.get("y", 0.0))
        draw.ellipse(
            [px - 8, py - 8, px + 8, py + 8],
            fill="#22C55E",
            outline="#111827",
            width=2,
        )
        draw.text((px + 10, py + 5), str(index), fill="#111827", font=font)
        yaw = float(pose.get("yaw", 0.0))
        draw.line(
            [(px, py), (px + math.cos(yaw) * 22, py - math.sin(yaw) * 22)],
            fill="#111827",
            width=2,
        )

    overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    overlay_draw.rectangle([8, 8, 308, 52], fill=(255, 255, 255, 210), outline="#111827")
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(image)
    draw.text(
        (18, 25),
        f"{route.get('name', route.get('id', 'route'))} targets: {len(ordered)}",
        fill="#111827",
        font=font,
    )
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


def generate_route_preview(map_yaml_path: Path = DEFAULT_MAP_YAML, force: bool = False) -> Dict[str, Any]:
    map_yaml = Path(map_yaml_path).expanduser()
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
    }
    try:
        route_path = resolve_route_file_path("auto", map_yaml.parent)
    except ValueError:
        return {
            **base_failure,
            "message": "未找到正式巡逻路线文件",
        }
    try:
        map_info = _parse_map_yaml(map_yaml)
        width, height = _read_pgm_size(map_info["image"])
        route_data = load_route_file(str(route_path))
        active_route_id = str(route_data.get("active_route_id") or "")
        if not active_route_id and route_data.get("routes"):
            active_route_id = str(route_data["routes"][0]["id"])
        route = get_route(route_data, active_route_id)
        active_route_id = str(route.get("id") or route_data.get("active_route_id") or "")
        signature = _route_signature([map_yaml, map_info["image"], route_path], active_route_id)
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
        }
    except Exception as exc:
        return {
            **base_failure,
            "message": f"路线预览失败: {exc}",
            "route_file": str(route_path),
        }
