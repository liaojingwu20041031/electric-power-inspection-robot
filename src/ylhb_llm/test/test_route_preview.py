import json
import hashlib
from pathlib import Path

from ylhb_llm.route_preview import (
    _route_signature,
    build_patrol_tasks,
    generate_route_preview,
    map_to_pixel,
    select_latest_route_file,
)


def write_route(path: Path, number: int = 1) -> None:
    route_id = f"route_patrol_{number:03d}"
    path.write_text(
        json.dumps(
            {
                "version": 2,
                "frame_id": "map",
                "active_route_id": route_id,
                "start_pose": {
                    "name": "起点",
                    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    "publish_initial_pose": False,
                },
                "targets": [
                    {
                        "id": "target_001",
                        "name": "巡检点1",
                        "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
                        "task_duration_sec": 7,
                    }
                ],
                "routes": [
                    {
                        "id": route_id,
                        "name": "路线",
                        "target_ids": ["target_001"],
                        "return_to_start": True,
                        "loop": {"enabled": False, "wait_sec": 0, "max_cycles": 0},
                        "goal_timeout_sec": 120,
                        "max_retries_per_checkpoint": 1,
                        "failure_policy": "abort",
                    }
                ],
                "schedules": [],
            }
        ),
        encoding="utf-8",
    )


def test_map_to_pixel_uses_ros_map_origin_and_flipped_y_axis():
    assert map_to_pixel(0.0, 0.0, resolution=0.05, origin=(-2.0, -3.0), image_height=100) == (40, 39)


def test_select_latest_route_file_prefers_highest_number(tmp_path):
    write_route(tmp_path / "route_patrol_001.json", 1)
    write_route(tmp_path / "route_patrol_010.json", 10)

    result = select_latest_route_file(tmp_path)

    assert result.path == tmp_path / "route_patrol_010.json"
    assert result.error == ""


def test_select_latest_route_file_rejects_deprecated_only(tmp_path):
    write_route(tmp_path / "deprecated_route_patrol_001.json", 1)

    result = select_latest_route_file(tmp_path)

    assert result.path is None
    assert result.error == "未找到正式巡逻路线文件"


def test_build_patrol_tasks_preserves_target_level_placeholders():
    tasks = build_patrol_tasks(
        [
            {
                "id": "target_001",
                "name": "巡检点1",
                "task_duration_sec": 5,
            }
        ]
    )

    assert tasks["target_001"] == {
        "target_id": "target_001",
        "name": "巡检点1",
        "task_duration_sec": 5,
        "task_type": "未配置",
        "task_params": {},
        "task_status": "预留接口",
    }


def test_generate_route_preview_creates_png_without_gui_application(tmp_path):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n80 80\n255\n" + bytes([245]) * 6400)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "mode: trinary",
                "resolution: 0.05",
                "origin: [-0.5, -0.5, 0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )
    write_route(tmp_path / "route_patrol_001.json", 1)
    write_route(tmp_path / "route_patrol_003.json", 3)

    preview = generate_route_preview(map_yaml, force=True)

    assert preview["ok"] is True
    assert preview["preview_type"] == "route_overlay"
    assert preview["overlay_ok"] is True
    assert preview["image_url"].startswith("file:///tmp/ylhb_route_preview_")
    assert Path(preview["image_path"]).exists()
    assert preview["image_exists"] is True
    assert preview["image_bytes"] > 0
    assert preview["image_mtime_ns"] > 0
    assert preview["target_count"] == 1
    assert preview["route_file"] == str(tmp_path / "route_patrol_003.json")
    assert preview["image_valid"] is True
    assert preview["image_error"] == ""
    assert preview["image_format"] == "png"
    assert Path(preview["image_path"]).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")

    from PIL import Image

    with Image.open(preview["image_path"]) as image:
        image.verify()


def test_generate_route_preview_supports_route_focus_and_full_map(tmp_path):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n80 80\n255\n" + bytes([245]) * 6400)
    map_yaml.write_text("image: my_map.pgm\nresolution: 0.05\norigin: [-0.5, -0.5, 0]\n", encoding="utf-8")
    write_route(tmp_path / "route_patrol_001.json", 1)

    focus = generate_route_preview(map_yaml, force=True, preview_mode="route_focus")
    full = generate_route_preview(map_yaml, force=True, preview_mode="full_map")

    assert focus["preview_mode"] == "route_focus"
    assert full["preview_mode"] == "full_map"
    assert focus["image_path"] != full["image_path"]


def test_generate_route_preview_deletes_bad_cache_and_regenerates(tmp_path):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n80 80\n255\n" + bytes([245]) * 6400)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "resolution: 0.05",
                "origin: [-0.5, -0.5, 0]",
            ]
        ),
        encoding="utf-8",
    )
    write_route(tmp_path / "route_patrol_001.json", 1)
    first = generate_route_preview(map_yaml, force=True)
    image_path = Path(first["image_path"])
    image_path.write_bytes(b"not a png")

    preview = generate_route_preview(map_yaml, force=False)

    assert preview["ok"] is True
    assert preview["image_path"] == str(image_path)
    assert preview["image_valid"] is True
    assert preview["image_error"] == ""
    assert image_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_generate_route_preview_reports_png_write_failure(tmp_path, monkeypatch):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n10 10\n255\n" + bytes([245]) * 100)
    map_yaml.write_text("image: my_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    write_route(tmp_path / "route_patrol_001.json", 1)

    def write_bad_preview(_map_image, output_path, *_args, **_kwargs):
        output_path.write_bytes(b"bad")

    monkeypatch.setattr("ylhb_llm.route_preview._draw_preview", write_bad_preview)

    preview = generate_route_preview(map_yaml, force=True)

    assert preview["ok"] is False
    assert preview["overlay_ok"] is False
    assert preview["image_valid"] is False
    assert preview["image_format"] == "png"
    assert "路线预览图生成失败" in preview["message"]


def test_generate_route_preview_outputs_minimum_1000px_wide_png(tmp_path):
    from PIL import Image

    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n80 80\n255\n" + bytes([245]) * 6400)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "resolution: 0.05",
                "origin: [-0.5, -0.5, 0]",
            ]
        ),
        encoding="utf-8",
    )
    write_route(tmp_path / "route_patrol_001.json", 1)

    preview = generate_route_preview(map_yaml, force=True)
    image_path = Path(preview["image_url"].removeprefix("file://"))

    assert Image.open(image_path).size[0] >= 1000


def test_generate_route_preview_contains_route_overlay_pixels(tmp_path):
    from PIL import Image

    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n80 80\n255\n" + bytes([80]) * 6400)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "resolution: 0.05",
                "origin: [-0.5, -0.5, 0]",
            ]
        ),
        encoding="utf-8",
    )
    write_route(tmp_path / "route_patrol_001.json", 1)
    route_path = tmp_path / "route_patrol_001.json"
    route_data = json.loads(route_path.read_text(encoding="utf-8"))
    route_data["keepout_zones"] = [{
        "id": "keepout_001",
        "name": "禁行区",
        "type": "hard_keepout",
        "enabled": True,
        "polygon": [
            {"x": 2.2, "y": 0.4},
            {"x": 2.8, "y": 0.4},
            {"x": 2.8, "y": 1.0},
            {"x": 2.2, "y": 1.0},
        ],
    }]
    route_path.write_text(json.dumps(route_data), encoding="utf-8")

    preview = generate_route_preview(map_yaml, force=True)
    image = Image.open(preview["image_path"]).convert("RGB")
    colors = set(image.getdata())

    assert (37, 99, 235) in colors  # route line
    assert (34, 197, 94) in colors  # checkpoint marker
    assert (249, 115, 22) in colors  # start marker
    assert (125, 77, 77) in colors  # translucent keepout overlay over gray map

    blue_pixels = [
        (x, y)
        for y in range(image.height)
        for x in range(image.width)
        if image.getpixel((x, y)) == (37, 99, 235)
    ]
    assert any(
        image.getpixel((nx, ny))[0] >= 240
        and image.getpixel((nx, ny))[1] >= 240
        and image.getpixel((nx, ny))[2] >= 240
        for x, y in blue_pixels
        for nx in range(max(0, x - 6), min(image.width, x + 7))
        for ny in range(max(0, y - 6), min(image.height, y + 7))
    )


def test_route_preview_cache_signature_uses_v5(tmp_path):
    route = tmp_path / "route.json"
    route.write_text("{}", encoding="utf-8")

    expected = hashlib.sha256(b"route-preview-v5:route_focus:route:footprint-v1:warn=0.20")
    expected.update(str(route).encode("utf-8"))
    expected.update(route.read_bytes())

    assert _route_signature([route], "route", "route_focus") == expected.hexdigest()[:16]


def test_generate_route_preview_reports_missing_map(tmp_path):
    write_route(tmp_path / "route_patrol_001.json", 1)

    preview = generate_route_preview(tmp_path / "missing.yaml", force=True)

    assert preview["ok"] is False
    assert preview["preview_type"] == "route_overlay"
    assert preview["overlay_ok"] is False
    assert "路线预览失败" in preview["message"]
    assert "missing.yaml" in preview["message"]


def test_generate_route_preview_reuses_patrol_route_validation(tmp_path):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n10 10\n255\n" + bytes([245]) * 100)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "resolution: 0.05",
                "origin: [0, 0, 0]",
            ]
        ),
        encoding="utf-8",
    )
    (tmp_path / "route_patrol_001.json").write_text('{"routes": []}', encoding="utf-8")

    preview = generate_route_preview(map_yaml, force=True)

    assert preview["ok"] is False
    assert "version must be 2" in preview["message"]


def test_generate_route_preview_reports_missing_pil(tmp_path, monkeypatch):
    map_yaml = tmp_path / "my_map.yaml"
    map_pgm = tmp_path / "my_map.pgm"
    map_pgm.write_bytes(b"P5\n10 10\n255\n" + bytes([245]) * 100)
    map_yaml.write_text(
        "\n".join(
            [
                "image: my_map.pgm",
                "resolution: 0.05",
                "origin: [0, 0, 0]",
            ]
        ),
        encoding="utf-8",
    )
    write_route(tmp_path / "route_patrol_001.json", 1)

    def raise_missing_pil(*_args, **_kwargs):
        raise ModuleNotFoundError("No module named 'PIL'")

    monkeypatch.setattr("ylhb_llm.route_preview._draw_preview", raise_missing_pil)

    preview = generate_route_preview(map_yaml, force=True)

    assert preview["ok"] is False
    assert preview["message"] == "路线预览失败: 缺少 python3-pil/Pillow 依赖"


def test_workspace_patrol_route_preview_uses_latest_route_and_png_exists():
    preview = generate_route_preview(Path("maps/my_map.yaml"), force=True)
    latest = select_latest_route_file(Path("maps")).path

    assert preview["ok"] is True
    assert preview["preview_type"] == "route_overlay"
    assert preview["overlay_ok"] is True
    assert Path(preview["route_file"]) == latest.resolve()
    assert preview["target_count"] == len(preview["targets"])
    assert preview["map_identity"]["image"] == "my_map.pgm"
    assert isinstance(preview["safety_warnings"], list)
    assert Path(preview["image_url"].removeprefix("file://")).exists()
