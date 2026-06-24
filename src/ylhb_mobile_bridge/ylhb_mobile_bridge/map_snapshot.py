import base64
import io
import math
from typing import Any, Dict

import numpy as np
from PIL import Image


def occupancy_grid_metadata(grid: Any) -> Dict[str, Any]:
    stamp = grid.header.stamp
    origin = grid.info.origin
    return {
        'frame_id': grid.header.frame_id,
        'timestamp': float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0,
        'resolution': float(grid.info.resolution),
        'width': int(grid.info.width),
        'height': int(grid.info.height),
        'origin': {
            'position': {
                'x': float(origin.position.x),
                'y': float(origin.position.y),
                'z': float(origin.position.z),
            },
            'orientation': {
                'x': float(origin.orientation.x),
                'y': float(origin.orientation.y),
                'z': float(origin.orientation.z),
                'w': float(origin.orientation.w),
            },
        },
    }


def occupancy_grid_to_png_snapshot(
    grid: Any,
    downsample: int = 1,
    max_size_px: int = 1024,
) -> Dict[str, Any]:
    downsample = max(1, min(16, int(downsample)))
    max_size_px = max(1, int(max_size_px))
    width = int(grid.info.width)
    height = int(grid.info.height)
    values = np.asarray(grid.data, dtype=np.int16).reshape((height, width))

    pixels = np.full((height, width), 205, dtype=np.uint8)
    pixels[(values >= 0) & (values <= 25)] = 255
    pixels[values >= 65] = 0
    pixels = np.flipud(pixels)
    pixels = pixels[::downsample, ::downsample]

    image = Image.fromarray(pixels, mode='L').convert('RGB')
    longest_edge = max(image.size)
    if longest_edge > max_size_px:
        scale = max_size_px / longest_edge
        size = (
            max(1, math.floor(image.width * scale)),
            max(1, math.floor(image.height * scale)),
        )
        nearest = getattr(getattr(Image, 'Resampling', Image), 'NEAREST')
        image = image.resize(size, resample=nearest)

    buffer = io.BytesIO()
    image.save(buffer, format='PNG', optimize=True)
    return {
        'map_meta': occupancy_grid_metadata(grid),
        'png_base64': base64.b64encode(buffer.getvalue()).decode('ascii'),
    }
