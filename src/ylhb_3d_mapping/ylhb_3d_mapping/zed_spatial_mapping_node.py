import json
import os
import struct
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


VALID_COMMANDS = {'start', 'stop', 'stop_and_export', 'export', 'reset'}
VALID_MAP_TYPES = {'fused_point_cloud', 'mesh'}
FINAL_STATES = {'idle', 'succeeded', 'failed'}
RESOLUTION_METERS = {'low': 0.08, 'medium': 0.05, 'high': 0.02}
RANGE_METERS = {'near': 3.5, 'medium': 5.0, 'far': 10.0}


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def parse_command_json(data: str) -> Dict[str, Any]:
    try:
        payload = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ValueError(f'Invalid 3D mapping command JSON: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError('3D mapping command must be a JSON object.')
    command = str(payload.get('command') or '').strip()
    if command not in VALID_COMMANDS:
        raise ValueError(f'Unknown 3D mapping command: {command or "<empty>"}')
    payload['command'] = command
    return payload


def build_metadata(
    *,
    map_type: str,
    output_file: str,
    output_dir: str,
    started_at: float,
    finished_at: float,
    params: Dict[str, Any],
    preview_point_count: int = 0,
    export_point_count: int = 0,
) -> Dict[str, Any]:
    return {
        'schema_version': '1.0',
        'map_type': map_type,
        'output_file': output_file,
        'output_dir': output_dir,
        'started_at': started_at,
        'finished_at': finished_at,
        'duration_sec': max(0.0, finished_at - started_at),
        'preview_point_count': preview_point_count,
        'export_point_count': export_point_count,
        'parameters': params,
        'note': 'ZED SDK 3D spatial mapping export for inspection only; not a Nav2 occupancy map.',
    }


def _vertex_xyz(vertex: Any) -> Optional[Tuple[float, float, float]]:
    if all(hasattr(vertex, name) for name in ('x', 'y', 'z')):
        return float(vertex.x), float(vertex.y), float(vertex.z)
    try:
        return float(vertex[0]), float(vertex[1]), float(vertex[2])
    except (TypeError, ValueError, IndexError):
        return None


def _color_rgb(color: Any) -> Optional[int]:
    if color is None:
        return None
    if all(hasattr(color, name) for name in ('r', 'g', 'b')):
        return (int(color.r) & 0xFF) << 16 | (int(color.g) & 0xFF) << 8 | (int(color.b) & 0xFF)
    try:
        return (int(color[0]) & 0xFF) << 16 | (int(color[1]) & 0xFF) << 8 | (int(color[2]) & 0xFF)
    except (TypeError, ValueError, IndexError):
        return None


def iter_spatial_map_points(spatial_map: Any, max_points: int) -> Iterable[Tuple[float, ...]]:
    if max_points <= 0:
        return
    vertices = getattr(spatial_map, 'vertices', None)
    if vertices is None and hasattr(spatial_map, 'get_vertices'):
        vertices = spatial_map.get_vertices()
    if vertices is None:
        return
    colors = getattr(spatial_map, 'colors', None)
    if colors is None and hasattr(spatial_map, 'get_colors'):
        colors = spatial_map.get_colors()
    color_iter = iter(colors) if colors is not None else None
    count = 0
    for vertex in vertices:
        xyz = _vertex_xyz(vertex)
        if xyz is None:
            continue
        rgb = _color_rgb(next(color_iter, None)) if color_iter is not None else None
        yield (*xyz, rgb) if rgb is not None else xyz
        count += 1
        if count >= max_points:
            return


def count_spatial_map_points(spatial_map: Any) -> int:
    return sum(1 for _ in iter_spatial_map_points(spatial_map, 2**63 - 1))


def build_pointcloud2(
    points: Iterable[Tuple[float, float, float]],
    *,
    frame_id: str,
    stamp: Any,
) -> PointCloud2:
    packed = bytearray()
    width = 0
    has_rgb = False
    rows = []
    for point in points:
        rows.append(point)
        has_rgb = has_rgb or len(point) >= 4

    for point in rows:
        x, y, z = point[:3]
        if has_rgb:
            rgb = int(point[3]) if len(point) >= 4 else 0
            packed.extend(struct.pack('<fffI', x, y, z, rgb))
        else:
            packed.extend(struct.pack('<fff', x, y, z))
        width += 1

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if has_rgb:
        msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    msg.is_bigendian = False
    msg.point_step = 16 if has_rgb else 12
    msg.row_step = msg.point_step * width
    msg.data = bytes(packed)
    msg.is_dense = False
    return msg


class ZedSpatialMappingNode(Node):
    def __init__(self) -> None:
        super().__init__('zed_spatial_mapping_node')
        self.declare_parameter('command_topic', '/inspection_ai/mapping3d_command')
        self.declare_parameter('status_topic', '/inspection_ai/mapping3d_status')
        self.declare_parameter('result_topic', '/inspection_ai/mapping3d_result')
        self.declare_parameter('preview_topic', '/inspection_ai/mapping3d_pointcloud')
        self.declare_parameter('output_root', '/home/nvidia/ros2_DL/runs/3d_mapping')
        self.declare_parameter('map_type', 'fused_point_cloud')
        self.declare_parameter('resolution_preset', 'high')
        self.declare_parameter('range_preset', 'near')
        self.declare_parameter('save_texture', False)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('preview_rate_hz', 1.0)
        self.declare_parameter('preview_frame_id', 'zed_3d_map')
        self.declare_parameter('preview_max_points', 1000000)
        self.declare_parameter('camera_resolution', 'HD720')
        self.declare_parameter('camera_fps', 15)
        self.declare_parameter('coordinate_units', 'METER')
        self.declare_parameter('coordinate_system', 'RIGHT_HANDED_Z_UP')
        self.declare_parameter('depth_mode', 'NEURAL')
        self.declare_parameter('depth_minimum_distance', 0.25)
        self.declare_parameter('depth_maximum_distance', 4.0)
        self.declare_parameter('confidence_threshold', 60)
        self.declare_parameter('texture_confidence_threshold', 60)
        self.declare_parameter('spatial_mapping_max_memory_mb', 1024)
        self.declare_parameter('mesh_filter_preset', 'high')
        self.declare_parameter('max_duration_sec', 0.0)
        self.declare_parameter('max_consecutive_grab_failures', 30)
        self.declare_parameter('min_success_frames_before_export', 30)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('auto_export_on_shutdown', True)

        self.output_root = os.path.expanduser(str(self.get_parameter('output_root').value))
        self.map_type = str(self.get_parameter('map_type').value)
        self.resolution_preset = str(self.get_parameter('resolution_preset').value)
        self.range_preset = str(self.get_parameter('range_preset').value)
        self.save_texture = parse_bool(self.get_parameter('save_texture').value)
        self.publish_period = 1.0 / max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.camera_resolution = str(self.get_parameter('camera_resolution').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.coordinate_units = str(self.get_parameter('coordinate_units').value)
        self.coordinate_system = str(self.get_parameter('coordinate_system').value)
        self.depth_mode = str(self.get_parameter('depth_mode').value)
        self.depth_minimum_distance = float(self.get_parameter('depth_minimum_distance').value)
        self.depth_maximum_distance = float(self.get_parameter('depth_maximum_distance').value)
        self.confidence_threshold = int(self.get_parameter('confidence_threshold').value)
        self.texture_confidence_threshold = int(self.get_parameter('texture_confidence_threshold').value)
        self.spatial_mapping_max_memory_mb = int(self.get_parameter('spatial_mapping_max_memory_mb').value)
        self.mesh_filter_preset = str(self.get_parameter('mesh_filter_preset').value)
        self.max_duration_sec = float(self.get_parameter('max_duration_sec').value)
        self.max_consecutive_grab_failures = int(self.get_parameter('max_consecutive_grab_failures').value)
        self.min_success_frames_before_export = int(self.get_parameter('min_success_frames_before_export').value)
        self.auto_export_on_shutdown = parse_bool(self.get_parameter('auto_export_on_shutdown').value)
        self.preview_topic = str(self.get_parameter('preview_topic').value)
        self.preview_period = 1.0 / max(0.1, float(self.get_parameter('preview_rate_hz').value))
        self.preview_frame_id = str(self.get_parameter('preview_frame_id').value)
        self.preview_max_points = int(self.get_parameter('preview_max_points').value)

        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.result_pub = self.create_publisher(String, self.get_parameter('result_topic').value, 10)
        self.preview_pub = self.create_publisher(PointCloud2, self.preview_topic, 10)
        self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)

        self.lock = threading.Lock()
        self.worker: Optional[threading.Thread] = None
        self.stop_requested = False
        self.export_requested = False
        self.state = 'idle'
        self.message = 'ready'
        self.camera = None
        self.sl = None
        self.success_frames = 0
        self.failed_frames = 0
        self.last_grab_error = ''
        self.spatial_mapping_state = ''
        self.preview_point_count = 0
        self.export_point_count = 0
        self.mapping_started = False
        self.output_file = ''
        self.started_at = 0.0
        self.current_output_dir = ''
        self.preview_request_pending = False
        self.last_preview_publish = 0.0
        self.publish_status('idle', 'ready')
        if parse_bool(self.get_parameter('auto_start').value):
            self.auto_start_timer = self.create_timer(0.5, self._auto_start_once)

    def _auto_start_once(self) -> None:
        timer = getattr(self, 'auto_start_timer', None)
        if timer is not None:
            timer.cancel()
        self.start_mapping(export_on_stop=False)

    def command_callback(self, msg: String) -> None:
        try:
            payload = parse_command_json(msg.data)
        except ValueError as exc:
            self.publish_status('failed', str(exc))
            return
        self.handle_command(payload)

    def handle_command(self, payload: Dict[str, Any]) -> None:
        command = payload['command']
        if command == 'start':
            self.start_mapping(export_on_stop=False)
        elif command == 'stop':
            self.request_stop(export_on_stop=False)
        elif command == 'stop_and_export':
            self.request_stop(export_on_stop=True)
        elif command == 'export':
            self.export_current_map()
        elif command == 'reset':
            self.reset()

    def start_mapping(self, export_on_stop: bool = False) -> None:
        with self.lock:
            if self.worker and self.worker.is_alive():
                self.publish_status('running', '3D mapping already running')
                return
            self.stop_requested = False
            self.export_requested = export_on_stop
            self.started_at = time.time()
            self.current_output_dir = self.make_output_dir(self.started_at)
            self.worker = threading.Thread(target=self.mapping_loop, daemon=True)
            self.worker.start()

    def request_stop(self, export_on_stop: bool) -> None:
        with self.lock:
            self.stop_requested = True
            self.export_requested = self.export_requested or export_on_stop
            running = bool(self.worker and self.worker.is_alive())
        if not running:
            if export_on_stop:
                self.export_current_map()
            else:
                self.publish_status('idle', '3D mapping is not running')

    def reset(self) -> None:
        self.request_stop(export_on_stop=False)
        if self.worker and self.worker.is_alive():
            self.publish_status('running', 'reset requested; stopping 3D mapping')
            return
        self.camera = None
        self.sl = None
        self.current_output_dir = ''
        self.output_file = ''
        self.success_frames = 0
        self.failed_frames = 0
        self.last_grab_error = ''
        self.spatial_mapping_state = ''
        self.mapping_started = False
        self.preview_point_count = 0
        self.export_point_count = 0
        self.publish_status('idle', 'reset')

    def mapping_loop(self) -> None:
        try:
            sl = self.load_zed()
            self.sl = sl
            self.publish_status('opening_camera', 'opening ZED camera')
            camera = sl.Camera()
            self.camera = camera
            init_params = sl.InitParameters()
            self.apply_init_params(sl, init_params)
            self.check_success(camera.open(init_params), 'Camera.open')

            self.publish_status('tracking_enabled', 'enabling positional tracking')
            self.check_success(
                camera.enable_positional_tracking(sl.PositionalTrackingParameters()),
                'enable_positional_tracking',
            )

            self.publish_status('mapping_enabled', 'enabling spatial mapping')
            spatial_params = sl.SpatialMappingParameters()
            self.apply_spatial_mapping_params(sl, spatial_params)
            self.check_success(camera.enable_spatial_mapping(spatial_params), 'enable_spatial_mapping')
            self.mapping_started = True

            self.publish_status('running', '3D mapping running')
            runtime_params = sl.RuntimeParameters()
            self.apply_runtime_params(runtime_params)
            last_publish = 0.0
            consecutive_failures = 0
            while not self.stop_requested:
                result = camera.grab(runtime_params)
                if self.is_success(result):
                    self.success_frames += 1
                    consecutive_failures = 0
                    self.last_grab_error = ''
                else:
                    self.failed_frames += 1
                    consecutive_failures += 1
                    self.last_grab_error = str(result)
                    if consecutive_failures > self.max_consecutive_grab_failures:
                        raise RuntimeError(f'ZED grab failed too many times: {result}')
                now = time.time()
                if self.max_duration_sec > 0 and now - self.started_at >= self.max_duration_sec:
                    self.stop_requested = True
                    self.export_requested = True
                if now - last_publish >= self.publish_period:
                    message = self.mapping_state_message() or '3D mapping running'
                    self.publish_status('running', message)
                    last_publish = now
                self.maybe_publish_preview(now)
            if self.export_requested:
                self.export_current_map_in_worker()
            else:
                self.publish_status('idle', '3D mapping stopped')
        except Exception as exc:
            self.publish_status('failed', str(exc))
        finally:
            self.close_camera()

    def load_zed(self):
        try:
            import pyzed.sl as sl
        except Exception as exc:
            raise RuntimeError(f'pyzed.sl import failed; install ZED SDK Python API: {exc}') from exc
        return sl

    def apply_init_params(self, sl, params) -> None:
        self.set_enum_value(sl.RESOLUTION, params, 'camera_resolution', self.camera_resolution)
        self.set_enum_value(sl.UNIT, params, 'coordinate_units', self.coordinate_units)
        self.set_enum_value(sl.COORDINATE_SYSTEM, params, 'coordinate_system', self.coordinate_system)
        self.set_enum_value(sl.DEPTH_MODE, params, 'depth_mode', self.depth_mode)
        if hasattr(params, 'camera_fps'):
            params.camera_fps = self.camera_fps
        if hasattr(params, 'depth_minimum_distance'):
            params.depth_minimum_distance = self.depth_minimum_distance
        if hasattr(params, 'depth_maximum_distance'):
            params.depth_maximum_distance = self.depth_maximum_distance

    def apply_runtime_params(self, params) -> None:
        if hasattr(params, 'confidence_threshold'):
            params.confidence_threshold = self.confidence_threshold
        if hasattr(params, 'texture_confidence_threshold'):
            params.texture_confidence_threshold = self.texture_confidence_threshold

    def apply_spatial_mapping_params(self, sl, params) -> None:
        map_type = self.map_type if self.map_type in VALID_MAP_TYPES else 'fused_point_cloud'
        if map_type == 'mesh':
            params.map_type = sl.SPATIAL_MAP_TYPE.MESH
        else:
            params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        if hasattr(params, 'resolution_meter'):
            params.resolution_meter = RESOLUTION_METERS.get(self.resolution_preset, RESOLUTION_METERS['medium'])
        if hasattr(params, 'range_meter'):
            params.range_meter = RANGE_METERS.get(self.range_preset, RANGE_METERS['medium'])
        if hasattr(params, 'max_memory_usage'):
            params.max_memory_usage = self.spatial_mapping_max_memory_mb
        if hasattr(params, 'save_texture'):
            params.save_texture = self.save_texture if map_type == 'mesh' else False

    def set_enum_value(self, enum_cls, params, attr: str, preset: str) -> None:
        value = getattr(enum_cls, str(preset).upper(), None)
        if value is not None and hasattr(params, attr):
            setattr(params, attr, value)

    def export_current_map(self) -> None:
        with self.lock:
            running = bool(self.worker and self.worker.is_alive())
            self.stop_requested = True
            self.export_requested = True
        if not running:
            self.publish_status('failed', '3D mapping is not running; start mapping before export.')

    def export_current_map_in_worker(self) -> None:
        try:
            if self.camera is None or self.sl is None:
                raise RuntimeError('ZED camera is not open; start mapping before export.')
            if not self.mapping_started:
                raise RuntimeError('Spatial mapping has not started; cannot export.')
            if self.success_frames < self.min_success_frames_before_export:
                raise RuntimeError(
                    'Not enough successful ZED frames before export: '
                    f'{self.success_frames}/{self.min_success_frames_before_export}'
                )
            output_dir = self.current_output_dir or self.make_output_dir(time.time())
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            output_file = os.path.join(output_dir, 'mesh.obj' if self.map_type == 'mesh' else 'pointcloud.ply')
            self.output_file = output_file
            self.write_json(
                os.path.join(output_dir, 'status.json'),
                {
                    'state': 'exporting',
                    'output_dir': output_dir,
                    'output_file': output_file,
                    'started_at': self.started_at,
                    'parameters': self.mapping_params(),
                },
            )
            self.publish_status('exporting', 'exporting 3D map; do not press Ctrl+C again or metadata may be lost')
            self.publish_status('extracting', 'extracting whole spatial map')
            spatial_map = self.make_spatial_map()
            self.camera.extract_whole_spatial_map(spatial_map)
            self.export_point_count = count_spatial_map_points(spatial_map)
            self.publish_status('saving', f'saving {self.map_type}')
            self.prepare_spatial_map_for_save(spatial_map)
            file_format = self.sl.MESH_FILE_FORMAT.OBJ if self.map_type == 'mesh' else self.sl.MESH_FILE_FORMAT.PLY
            self.check_success(spatial_map.save(output_file, file_format), 'spatial_map.save')
            finished_at = time.time()
            metadata = build_metadata(
                map_type=self.map_type,
                output_file=output_file,
                output_dir=output_dir,
                started_at=self.started_at or finished_at,
                finished_at=finished_at,
                params=self.mapping_params(),
                preview_point_count=self.preview_point_count,
                export_point_count=self.export_point_count,
            )
            self.write_json(os.path.join(output_dir, 'metadata.json'), metadata)
            self.write_json(os.path.join(output_dir, 'status.json'), {'state': 'succeeded', **metadata})
            self.publish_result(metadata)
            self.publish_status('succeeded', f'3D map exported: {output_file}')
        except Exception as exc:
            self.publish_status('failed', str(exc))

    def make_spatial_map(self):
        return self.sl.Mesh() if self.map_type == 'mesh' else self.sl.FusedPointCloud()

    def prepare_spatial_map_for_save(self, spatial_map: Any) -> None:
        if self.map_type != 'mesh':
            return
        filter_method = getattr(spatial_map, 'filter', None)
        if filter_method is not None:
            try:
                result = filter_method()
            except TypeError:
                result = filter_method(self.make_mesh_filter_params())
            if result is not None:
                self.check_success(result, 'spatial_map.filter')
        if self.save_texture:
            texture_method = getattr(spatial_map, 'apply_texture', None)
            if texture_method is not None:
                result = texture_method()
                if result is not None:
                    self.check_success(result, 'spatial_map.apply_texture')

    def make_mesh_filter_params(self):
        params_cls = getattr(self.sl, 'MeshFilterParameters', None)
        if params_cls is None:
            return None
        params = params_cls()
        filter_enum = getattr(self.sl, 'MESH_FILTER', None)
        if filter_enum is not None and hasattr(params, 'set'):
            value = getattr(filter_enum, self.mesh_filter_preset.upper(), None)
            if value is not None:
                params.set(value)
        return params

    def maybe_publish_preview(self, now: float) -> None:
        if now - self.last_preview_publish < self.preview_period:
            return
        camera = getattr(self, 'camera', None)
        if camera is None or not hasattr(camera, 'request_spatial_map_async'):
            return
        if not self.preview_request_pending:
            camera.request_spatial_map_async()
            self.preview_request_pending = True
            return
        status_method = getattr(camera, 'get_spatial_map_request_status_async', None)
        if status_method is not None and not self.is_success(status_method()):
            return
        retrieve = getattr(camera, 'retrieve_spatial_map_async', None)
        if retrieve is None:
            return
        spatial_map = self.make_spatial_map()
        result = retrieve(spatial_map)
        self.preview_request_pending = False
        self.last_preview_publish = now
        if self.is_success(result):
            self.publish_preview_map(spatial_map)

    def publish_preview_map(self, spatial_map: Any) -> None:
        msg = build_pointcloud2(
            iter_spatial_map_points(spatial_map, self.preview_max_points),
            frame_id=self.preview_frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self.preview_point_count = msg.width
        self.preview_pub.publish(msg)

    def close_camera(self) -> None:
        camera = getattr(self, 'camera', None)
        self.camera = None
        if camera is None:
            return
        for method_name in ('disable_spatial_mapping', 'disable_positional_tracking', 'close'):
            method = getattr(camera, method_name, None)
            if method is not None:
                try:
                    method()
                except Exception:
                    pass

    def mapping_params(self) -> Dict[str, Any]:
        return {
            'map_type': self.map_type,
            'resolution_preset': self.resolution_preset,
            'range_preset': self.range_preset,
            'save_texture': self.save_texture,
            'camera_resolution': self.camera_resolution,
            'camera_fps': self.camera_fps,
            'depth_mode': self.depth_mode,
            'depth_minimum_distance': self.depth_minimum_distance,
            'depth_maximum_distance': self.depth_maximum_distance,
            'confidence_threshold': self.confidence_threshold,
            'texture_confidence_threshold': self.texture_confidence_threshold,
            'spatial_mapping_max_memory_mb': self.spatial_mapping_max_memory_mb,
            'mesh_filter_preset': self.mesh_filter_preset,
            'preview_topic': self.preview_topic,
            'preview_frame_id': self.preview_frame_id,
            'preview_max_points': self.preview_max_points,
        }

    def make_output_dir(self, timestamp: float) -> str:
        stamp = datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S')
        return os.path.join(self.output_root, f'map_{stamp}')

    def write_json(self, path: str, payload: Dict[str, Any]) -> None:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
            f.write('\n')

    def publish_status(self, state: str, message: str) -> None:
        self.state = state
        self.message = message
        payload = {
            'schema_version': '1.0',
            'timestamp': time.time(),
            'state': state,
            'message': message,
            'output_dir': getattr(self, 'current_output_dir', ''),
            'output_file': getattr(self, 'output_file', ''),
            'map_type': getattr(self, 'map_type', 'fused_point_cloud'),
            'preview_topic': getattr(self, 'preview_topic', ''),
            'preview_frame_id': getattr(self, 'preview_frame_id', ''),
            'success_frames': getattr(self, 'success_frames', 0),
            'failed_frames': getattr(self, 'failed_frames', 0),
            'last_grab_error': getattr(self, 'last_grab_error', ''),
            'spatial_mapping_state': getattr(self, 'spatial_mapping_state', ''),
            'preview_point_count': getattr(self, 'preview_point_count', 0),
            'export_point_count': getattr(self, 'export_point_count', 0),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def publish_result(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)

    def check_success(self, result, action: str) -> None:
        success = getattr(getattr(self.sl, 'ERROR_CODE', object), 'SUCCESS', None)
        if success is not None and result != success:
            raise RuntimeError(f'{action} failed: {result}')

    def is_success(self, result) -> bool:
        success = getattr(getattr(self.sl, 'ERROR_CODE', object), 'SUCCESS', None)
        return success is None or result == success

    def mapping_state_message(self) -> str:
        camera = getattr(self, 'camera', None)
        if camera is None or not hasattr(camera, 'get_spatial_mapping_state'):
            return ''
        try:
            state = camera.get_spatial_mapping_state()
        except Exception:
            return ''
        self.spatial_mapping_state = str(state)
        if 'FPS_TOO_LOW' in self.spatial_mapping_state or 'NOT_ENOUGH_MEMORY' in self.spatial_mapping_state:
            return (
                f'{self.spatial_mapping_state}; try lower resolution/range or disable texture'
            )
        return f'3D mapping running: {self.spatial_mapping_state}'

    def destroy_node(self) -> bool:
        worker = getattr(self, 'worker', None)
        if worker and worker.is_alive():
            with self.lock:
                self.stop_requested = True
                self.export_requested = (
                    self.export_requested
                    or self.auto_export_on_shutdown
                    and self.success_frames >= self.min_success_frames_before_export
                )
            worker.join(timeout=10.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedSpatialMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        worker = getattr(node, 'worker', None)
        if worker and worker.is_alive():
            node.get_logger().warn('Ctrl+C received; exporting if eligible. Do not press Ctrl+C again or metadata may be lost.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
