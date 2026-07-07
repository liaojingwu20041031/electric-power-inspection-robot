import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from .zed_spatial_mapping_node import (
    RANGE_METERS,
    RESOLUTION_METERS,
    count_spatial_map_points,
)


CAPTURE_DEFAULTS = {
    'output_root': '/home/nvidia/ros2_DL/runs/3d_capture',
    'duration_sec': 30.0,
    'camera_resolution': 'HD720',
    'camera_fps': 30,
    'depth_mode': 'NONE',
    'compression_mode': 'H264',
}

PROFILES = {
    'fast_check': {
        'depth_mode': 'NEURAL',
        'resolution_preset': 'medium',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 768,
    },
    'quality_safe': {
        'depth_mode': 'NEURAL',
        'resolution_preset': 'high',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 1024,
    },
    'quality_plus': {
        'depth_mode': 'NEURAL_PLUS',
        'resolution_preset': 'high',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 1024,
    },
}


def parse_args(args: List[str]) -> Dict[str, str]:
    parsed = {}
    for arg in args:
        if ':=' in arg:
            key, value = arg.split(':=', 1)
            parsed[key] = value
    return parsed


def load_zed():
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(f'pyzed.sl import failed; install ZED SDK Python API: {exc}') from exc
    return sl


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def set_enum(enum_cls, obj: Any, attr: str, value: str) -> None:
    enum_value = getattr(enum_cls, str(value).upper(), None)
    if enum_value is None:
        return
    try:
        setattr(obj, attr, enum_value)
    except AttributeError:
        pass


def check_success(sl, result, action: str) -> None:
    if result is True:
        return
    success = getattr(getattr(sl, 'ERROR_CODE', object), 'SUCCESS', None)
    if success is not None and result != success:
        raise RuntimeError(f'{action} failed: {result}')


def output_dir(root: str, prefix: str, timestamp: float) -> Path:
    stamp = datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S')
    return Path(os.path.expanduser(root)) / f'{prefix}_{stamp}'


def capture_svo(
    args: List[str],
    *,
    sl=None,
    now: Callable[[], float] = time.time,
    monotonic: Callable[[], float] = time.monotonic,
) -> Dict[str, Any]:
    sl = sl or load_zed()
    opts = {**CAPTURE_DEFAULTS, **parse_args(args)}
    started_at = now()
    out_dir = output_dir(str(opts['output_root']), 'capture', started_at)
    svo_file = out_dir / 'capture.svo2'
    out_dir.mkdir(parents=True, exist_ok=True)
    write_json(out_dir / 'status.json', {'state': 'recording', 'svo_file': str(svo_file), 'parameters': opts})

    camera = sl.Camera()
    frames = 0
    try:
        init_params = sl.InitParameters()
        set_enum(sl.RESOLUTION, init_params, 'camera_resolution', str(opts['camera_resolution']))
        set_enum(sl.DEPTH_MODE, init_params, 'depth_mode', str(opts['depth_mode']))
        try:
            init_params.camera_fps = int(opts['camera_fps'])
        except AttributeError:
            pass
        check_success(sl, camera.open(init_params), 'Camera.open')

        rec_params = sl.RecordingParameters()
        rec_params.video_filename = str(svo_file)
        set_enum(sl.SVO_COMPRESSION_MODE, rec_params, 'compression_mode', str(opts['compression_mode']))
        check_success(sl, camera.enable_recording(rec_params), 'Camera.enable_recording')

        start = monotonic()
        duration = float(opts['duration_sec'])
        while duration <= 0 or monotonic() - start < duration:
            if camera.grab() == sl.ERROR_CODE.SUCCESS:
                frames += 1
            if duration <= 0:
                break
    finally:
        if hasattr(camera, 'disable_recording'):
            camera.disable_recording()
        camera.close()

    finished_at = now()
    metadata = {
        'schema_version': '1.0',
        'state': 'succeeded',
        'svo_file': str(svo_file),
        'output_dir': str(out_dir),
        'started_at': started_at,
        'finished_at': finished_at,
        'capture_duration_sec': max(0.0, finished_at - started_at),
        'svo_frame_count': frames,
        'parameters': opts,
    }
    write_json(out_dir / 'metadata.json', metadata)
    write_json(out_dir / 'status.json', metadata)
    return metadata


def reconstruct_svo(
    args: List[str],
    *,
    sl=None,
    now: Callable[[], float] = time.time,
) -> Dict[str, Any]:
    sl = sl or load_zed()
    opts = parse_args(args)
    if 'input' not in opts:
        raise ValueError('input:=/path/to/capture.svo2 is required')
    profile_name = opts.get('profile', 'quality_safe')
    params = {**PROFILES.get(profile_name, PROFILES['quality_safe']), **opts}
    svo_file = Path(str(params['input'])).expanduser()
    started_at = now()
    out_dir = output_dir(str(params.get('output_root', '/home/nvidia/ros2_DL/runs/3d_reconstruct')), 'reconstruct', started_at)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_file = out_dir / 'pointcloud.ply'
    write_json(out_dir / 'status.json', {'state': 'reconstructing', 'svo_file': str(svo_file), 'parameters': params})

    camera = sl.Camera()
    try:
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(str(svo_file))
        init_params.svo_real_time_mode = False
        set_enum(sl.DEPTH_MODE, init_params, 'depth_mode', str(params['depth_mode']))
        set_enum(sl.UNIT, init_params, 'coordinate_units', 'METER')
        set_enum(sl.COORDINATE_SYSTEM, init_params, 'coordinate_system', 'RIGHT_HANDED_Z_UP')
        check_success(sl, camera.open(init_params), 'Camera.open')
        check_success(sl, camera.enable_positional_tracking(sl.PositionalTrackingParameters()), 'enable_positional_tracking')

        spatial_params = sl.SpatialMappingParameters()
        spatial_params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        if hasattr(spatial_params, 'resolution_meter'):
            spatial_params.resolution_meter = RESOLUTION_METERS[str(params['resolution_preset'])]
        if hasattr(spatial_params, 'range_meter'):
            spatial_params.range_meter = RANGE_METERS[str(params['range_preset'])]
        if hasattr(spatial_params, 'max_memory_usage'):
            spatial_params.max_memory_usage = int(params['spatial_mapping_max_memory_mb'])
        check_success(sl, camera.enable_spatial_mapping(spatial_params), 'enable_spatial_mapping')

        runtime_params = sl.RuntimeParameters()
        end_code = getattr(sl.ERROR_CODE, 'END_OF_SVOFILE_REACHED', object())
        while True:
            result = camera.grab(runtime_params)
            if result == end_code:
                break
            check_success(sl, result, 'Camera.grab')

        spatial_map = sl.FusedPointCloud()
        camera.extract_whole_spatial_map(spatial_map)
        point_count = count_spatial_map_points(spatial_map)
        check_success(sl, spatial_map.save(str(output_file), sl.MESH_FILE_FORMAT.PLY), 'spatial_map.save')
        frame_count = camera.get_svo_number_of_frames() if hasattr(camera, 'get_svo_number_of_frames') else 0
    finally:
        for method_name in ('disable_spatial_mapping', 'disable_positional_tracking', 'close'):
            method = getattr(camera, method_name, None)
            if method is not None:
                method()

    finished_at = now()
    metadata = {
        'schema_version': '1.0',
        'state': 'succeeded',
        'svo_file': str(svo_file),
        'svo_frame_count': frame_count,
        'reconstruct_profile': profile_name,
        'output_dir': str(out_dir),
        'output_file': str(output_file),
        'started_at': started_at,
        'finished_at': finished_at,
        'duration_sec': max(0.0, finished_at - started_at),
        'export_point_count': point_count,
        'parameters': params,
    }
    write_json(out_dir / 'metadata.json', metadata)
    write_json(out_dir / 'status.json', metadata)
    return metadata


def main_capture(args: Optional[List[str]] = None) -> None:
    print(json.dumps(capture_svo(sys.argv[1:] if args is None else args), ensure_ascii=False))


def main_reconstruct(args: Optional[List[str]] = None) -> None:
    print(json.dumps(reconstruct_svo(sys.argv[1:] if args is None else args), ensure_ascii=False))
