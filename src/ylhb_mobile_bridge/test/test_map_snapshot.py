import base64
import io
from types import SimpleNamespace

from PIL import Image

from ylhb_mobile_bridge.map_snapshot import occupancy_grid_to_png_snapshot


def make_grid(width=4, height=2, data=None):
    data = data or [-1, 0, 25, 26, 64, 65, 100, 10]
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=10, nanosec=500000000),
        ),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data,
    )


def decode_png(png_base64):
    return Image.open(io.BytesIO(base64.b64decode(png_base64))).convert("RGB")


def test_occupancy_grid_snapshot_includes_metadata_and_decodable_png():
    snapshot = occupancy_grid_to_png_snapshot(make_grid())

    assert snapshot["map_meta"]["width"] == 4
    assert snapshot["map_meta"]["height"] == 2
    assert snapshot["map_meta"]["resolution"] == 0.05
    assert snapshot["map_meta"]["origin"]["position"]["x"] == 1.0
    assert snapshot["map_meta"]["frame_id"] == "map"
    assert snapshot["map_meta"]["timestamp"] == 10.5

    image = decode_png(snapshot["png_base64"])
    assert image.size == (4, 2)


def test_occupancy_grid_snapshot_colors_thresholds_after_vertical_flip():
    snapshot = occupancy_grid_to_png_snapshot(make_grid())
    image = decode_png(snapshot["png_base64"])

    assert image.getpixel((0, 1)) == (205, 205, 205)
    assert image.getpixel((1, 1)) == (255, 255, 255)
    assert image.getpixel((2, 1)) == (255, 255, 255)
    assert image.getpixel((3, 1)) == (205, 205, 205)
    assert image.getpixel((0, 0)) == (205, 205, 205)
    assert image.getpixel((1, 0)) == (0, 0, 0)
    assert image.getpixel((2, 0)) == (0, 0, 0)
    assert image.getpixel((3, 0)) == (255, 255, 255)


def test_occupancy_grid_snapshot_downsamples_dimensions():
    grid = make_grid(width=4, height=4, data=[0] * 16)
    image = decode_png(
        occupancy_grid_to_png_snapshot(grid, downsample=2, max_size_px=100)[
            "png_base64"
        ]
    )

    assert image.size == (2, 2)


def test_occupancy_grid_snapshot_applies_max_size_px():
    grid = make_grid(width=8, height=4, data=[0] * 32)
    image = decode_png(
        occupancy_grid_to_png_snapshot(grid, downsample=1, max_size_px=4)[
            "png_base64"
        ]
    )

    assert image.size == (4, 2)
