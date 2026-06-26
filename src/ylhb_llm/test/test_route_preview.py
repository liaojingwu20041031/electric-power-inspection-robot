import json
from pathlib import Path

from ylhb_llm.route_preview import (
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
    assert preview["route_file"] == str(tmp_path / "route_patrol_001.json")


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
    image = Image.open(preview["image_path"]).convert("RGB")
    colors = set(image.getdata())

    assert (37, 99, 235) in colors  # route line
    assert (34, 197, 94) in colors  # checkpoint marker
    assert (249, 115, 22) in colors  # start marker


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


def test_workspace_patrol_route_preview_has_four_targets_and_png_exists():
    preview = generate_route_preview(Path("maps/my_map.yaml"), force=True)

    assert preview["ok"] is True
    assert preview["preview_type"] == "route_overlay"
    assert preview["overlay_ok"] is True
    assert preview["active_route_id"] == "route_patrol_001"
    assert preview["target_count"] == 4
    assert Path(preview["image_url"].removeprefix("file://")).exists()
