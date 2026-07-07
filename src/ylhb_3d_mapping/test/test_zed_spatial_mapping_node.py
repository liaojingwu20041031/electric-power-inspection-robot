import json
import struct
from pathlib import Path

import pytest
import yaml
from builtin_interfaces.msg import Time
from rclpy.validate_topic_name import validate_topic_name

from ylhb_3d_mapping.zed_spatial_mapping_node import (
    RANGE_METERS,
    RESOLUTION_METERS,
    ZedSpatialMappingNode,
    build_metadata,
    build_pointcloud2,
    iter_spatial_map_points,
    parse_command_json,
    parse_bool,
)


REPO_ROOT = Path(__file__).resolve().parents[3]


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


class RawPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_yaml_defaults_to_fused_point_cloud():
    config = yaml.safe_load(
        (REPO_ROOT / 'src/ylhb_3d_mapping/config/zed_spatial_mapping.yaml').read_text(encoding='utf-8')
    )

    params = config['zed_spatial_mapping_node']['ros__parameters']
    assert params['map_type'] == 'fused_point_cloud'
    assert params['output_root'].endswith('/runs/3d_mapping')
    assert params['resolution_preset'] == 'high'
    assert params['range_preset'] == 'near'
    assert params['publish_rate_hz'] == 2.0
    assert params['preview_topic'] == '/inspection_ai/mapping3d_pointcloud'
    assert params['preview_rate_hz'] == 1.0
    assert params['preview_frame_id'] == 'zed_3d_map'
    assert params['preview_max_points'] == 1000000
    assert params['save_texture'] is False
    assert params['depth_mode'] == 'NEURAL'
    assert params['depth_minimum_distance'] == 0.25
    assert params['depth_maximum_distance'] == 4.0
    assert params['confidence_threshold'] == 60
    assert params['texture_confidence_threshold'] == 60
    assert params['spatial_mapping_max_memory_mb'] == 1024
    assert params['mesh_filter_preset'] == 'high'


def test_yaml_topics_are_valid_ros_names():
    config_text = (REPO_ROOT / 'src/ylhb_3d_mapping/config/zed_spatial_mapping.yaml').read_text(
        encoding='utf-8'
    )
    config = yaml.safe_load(config_text)
    params = config['zed_spatial_mapping_node']['ros__parameters']

    for key in ('command_topic', 'status_topic', 'result_topic', 'preview_topic'):
        validate_topic_name(params[key])

    old_prefix = '/inspection_ai/' + '3d' + '_mapping_'
    assert old_prefix not in config_text


def test_command_json_accepts_known_commands_and_rejects_bad_input():
    for command in ('start', 'stop', 'stop_and_export', 'export', 'reset'):
        assert parse_command_json(f'{{"command":"{command}"}}')['command'] == command

    with pytest.raises(ValueError, match='Invalid 3D mapping command JSON'):
        parse_command_json('{bad')
    with pytest.raises(ValueError, match='Unknown 3D mapping command'):
        parse_command_json('{"command":"fly"}')


def test_metadata_declares_non_nav2_use():
    metadata = build_metadata(
        map_type='fused_point_cloud',
        output_file='/tmp/map/pointcloud.ply',
        output_dir='/tmp/map',
        started_at=10.0,
        finished_at=13.5,
        params={'range_preset': 'medium'},
    )

    assert metadata['schema_version'] == '1.0'
    assert metadata['map_type'] == 'fused_point_cloud'
    assert metadata['duration_sec'] == 3.5
    assert 'not a Nav2 occupancy map' in metadata['note']
    assert metadata['parameters']['range_preset'] == 'medium'


def test_pyzed_missing_publishes_readable_failed_status(monkeypatch):
    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.status_pub = FakePublisher()
    node.result_pub = FakePublisher()
    node.current_output_dir = ''
    node.map_type = 'fused_point_cloud'
    node.load_zed = lambda: (_ for _ in ()).throw(RuntimeError('pyzed.sl import failed; install ZED SDK Python API'))

    node.mapping_loop()

    assert node.status_pub.messages[-1]['state'] == 'failed'
    assert 'pyzed.sl import failed' in node.status_pub.messages[-1]['message']


def test_parse_bool_keeps_string_false_false():
    assert parse_bool('false') is False
    assert parse_bool('true') is True
    assert parse_bool(False) is False


def test_spatial_mapping_presets_use_meter_values_and_texture_only_for_mesh():
    class Enum:
        MESH = 'mesh'
        FUSED_POINT_CLOUD = 'fused'

    class FakeSl:
        SPATIAL_MAP_TYPE = Enum

    class Params:
        resolution_meter = 0.0
        range_meter = 0.0
        max_memory_usage = 0
        save_texture = False
        map_type = ''

    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.map_type = 'fused_point_cloud'
    node.resolution_preset = 'high'
    node.range_preset = 'far'
    node.spatial_mapping_max_memory_mb = 1024
    node.save_texture = True
    params = Params()
    node.apply_spatial_mapping_params(FakeSl, params)
    assert params.resolution_meter == RESOLUTION_METERS['high']
    assert params.range_meter == RANGE_METERS['far']
    assert params.max_memory_usage == 1024
    assert params.save_texture is False

    node.map_type = 'mesh'
    params = Params()
    node.apply_spatial_mapping_params(FakeSl, params)
    assert params.map_type == 'mesh'
    assert params.save_texture is True


def test_zed_quality_parameters_are_applied_to_init_and_runtime_params():
    class Enum:
        NEURAL_PLUS = 'neural_plus'

    class Sl:
        RESOLUTION = Enum
        UNIT = Enum
        COORDINATE_SYSTEM = Enum
        DEPTH_MODE = Enum

    class InitParams:
        depth_mode = ''
        depth_minimum_distance = 0.0
        depth_maximum_distance = 0.0

    class RuntimeParams:
        confidence_threshold = 0
        texture_confidence_threshold = 0

    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.camera_resolution = ''
    node.coordinate_units = ''
    node.coordinate_system = ''
    node.depth_mode = 'NEURAL_PLUS'
    node.camera_fps = 15
    node.depth_minimum_distance = 0.25
    node.depth_maximum_distance = 4.0
    node.confidence_threshold = 60
    node.texture_confidence_threshold = 60

    init_params = InitParams()
    runtime_params = RuntimeParams()
    node.apply_init_params(Sl, init_params)
    node.apply_runtime_params(runtime_params)

    assert init_params.depth_mode == 'neural_plus'
    assert init_params.depth_minimum_distance == 0.25
    assert init_params.depth_maximum_distance == 4.0
    assert runtime_params.confidence_threshold == 60
    assert runtime_params.texture_confidence_threshold == 60


def test_status_payload_includes_runtime_mapping_fields():
    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.status_pub = FakePublisher()
    node.map_type = 'mesh'
    node.current_output_dir = '/tmp/map'
    node.output_file = '/tmp/map/mesh.obj'
    node.success_frames = 42
    node.failed_frames = 1
    node.last_grab_error = 'TIMEOUT'
    node.spatial_mapping_state = 'OK'
    node.preview_topic = '/inspection_ai/mapping3d_pointcloud'
    node.preview_frame_id = 'zed_3d_map'
    node.preview_point_count = 123
    node.export_point_count = 456

    node.publish_status('running', 'ok')

    payload = node.status_pub.messages[-1]
    assert payload['preview_topic'] == '/inspection_ai/mapping3d_pointcloud'
    assert payload['preview_frame_id'] == 'zed_3d_map'
    assert payload['success_frames'] == 42
    assert payload['failed_frames'] == 1
    assert payload['last_grab_error'] == 'TIMEOUT'
    assert payload['spatial_mapping_state'] == 'OK'
    assert payload['output_dir'] == '/tmp/map'
    assert payload['output_file'] == '/tmp/map/mesh.obj'
    assert payload['preview_point_count'] == 123
    assert payload['export_point_count'] == 456


def test_fake_zed_vertices_convert_to_pointcloud2_xyz_fields():
    class Map:
        vertices = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]

    msg = build_pointcloud2(
        iter_spatial_map_points(Map(), 10),
        frame_id='zed_3d_map',
        stamp=Time(),
    )

    assert msg.header.frame_id == 'zed_3d_map'
    assert [field.name for field in msg.fields] == ['x', 'y', 'z']
    assert msg.point_step == 12
    assert msg.width == 2
    assert struct.unpack('<ffffff', bytes(msg.data)) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_fake_zed_vertices_with_colors_convert_to_pointcloud2_rgb_field():
    class Map:
        vertices = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        colors = [(255, 0, 0), (0, 128, 255)]

    msg = build_pointcloud2(
        iter_spatial_map_points(Map(), 10),
        frame_id='zed_3d_map',
        stamp=Time(),
    )

    assert [field.name for field in msg.fields] == ['x', 'y', 'z', 'rgb']
    assert msg.point_step == 16
    assert msg.width == 2
    assert struct.unpack('<fffIfffI', bytes(msg.data)) == (
        1.0, 2.0, 3.0, 0xFF0000, 4.0, 5.0, 6.0, 0x0080FF,
    )


def test_preview_publish_respects_rate_and_max_points():
    class Clock:
        def now(self):
            return self

        def to_msg(self):
            return Time()

    class Sl:
        class ERROR_CODE:
            SUCCESS = 'SUCCESS'

        class FusedPointCloud:
            vertices = [(1, 2, 3), (4, 5, 6), (7, 8, 9)]

    class Camera:
        def __init__(self):
            self.requests = 0

        def request_spatial_map_async(self):
            self.requests += 1

        def get_spatial_map_request_status_async(self):
            return 'SUCCESS'

        def retrieve_spatial_map_async(self, spatial_map):
            return 'SUCCESS'

    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.camera = Camera()
    node.sl = Sl
    node.map_type = 'fused_point_cloud'
    node.preview_period = 1.0
    node.preview_request_pending = False
    node.last_preview_publish = 0.0
    node.preview_max_points = 2
    node.preview_frame_id = 'zed_3d_map'
    node.preview_pub = RawPublisher()
    node.get_clock = lambda: Clock()

    node.maybe_publish_preview(1.0)
    node.maybe_publish_preview(1.1)
    node.maybe_publish_preview(1.2)
    assert node.camera.requests == 1
    assert len(node.preview_pub.messages) == 1
    assert node.preview_pub.messages[-1].width == 2

    node.maybe_publish_preview(1.5)
    assert node.camera.requests == 1
    node.maybe_publish_preview(2.3)
    assert node.camera.requests == 2


def test_package_declares_sensor_msgs_dependency():
    package_xml = (REPO_ROOT / 'src/ylhb_3d_mapping/package.xml').read_text(encoding='utf-8')

    assert '<exec_depend>sensor_msgs</exec_depend>' in package_xml


def test_export_writes_exporting_status_before_metadata(tmp_path):
    class Sl:
        class ERROR_CODE:
            SUCCESS = 'SUCCESS'

        class MESH_FILE_FORMAT:
            PLY = 'PLY'
            OBJ = 'OBJ'

        class FusedPointCloud:
            def save(self, output_file, file_format):
                Path(output_file).write_text('ply', encoding='utf-8')
                return 'SUCCESS'

    class Camera:
        def extract_whole_spatial_map(self, spatial_map):
            status_path = tmp_path / 'status.json'
            assert json.loads(status_path.read_text(encoding='utf-8'))['state'] == 'exporting'

    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.camera = Camera()
    node.sl = Sl
    node.mapping_started = True
    node.success_frames = 30
    node.min_success_frames_before_export = 30
    node.current_output_dir = str(tmp_path)
    node.output_file = ''
    node.map_type = 'fused_point_cloud'
    node.started_at = 10.0
    node.status_pub = FakePublisher()
    node.result_pub = FakePublisher()
    node.resolution_preset = 'medium'
    node.range_preset = 'near'
    node.save_texture = False
    node.camera_resolution = 'HD720'
    node.camera_fps = 15
    node.depth_mode = 'NEURAL'
    node.preview_topic = '/inspection_ai/mapping3d_pointcloud'
    node.preview_frame_id = 'zed_3d_map'
    node.preview_max_points = 200000
    node.preview_point_count = 0
    node.export_point_count = 0
    node.depth_minimum_distance = 0.25
    node.depth_maximum_distance = 4.0
    node.confidence_threshold = 60
    node.texture_confidence_threshold = 60
    node.spatial_mapping_max_memory_mb = 1024
    node.mesh_filter_preset = 'high'

    node.export_current_map_in_worker()

    assert (tmp_path / 'metadata.json').exists()
    status = json.loads((tmp_path / 'status.json').read_text(encoding='utf-8'))
    assert status['state'] == 'succeeded'
    assert status['export_point_count'] == 0
    assert status['parameters']['depth_minimum_distance'] == 0.25
    assert node.result_pub.messages[-1]['output_file'].endswith('pointcloud.ply')


def test_mesh_export_filters_and_textures_before_save(tmp_path):
    class Sl:
        class ERROR_CODE:
            SUCCESS = 'SUCCESS'

        class MESH_FILE_FORMAT:
            PLY = 'PLY'
            OBJ = 'OBJ'

        class Mesh:
            vertices = [(1, 2, 3)]

            def __init__(self):
                self.calls = []

            def filter(self):
                self.calls.append('filter')
                return 'SUCCESS'

            def apply_texture(self):
                self.calls.append('apply_texture')
                return 'SUCCESS'

            def save(self, output_file, file_format):
                self.calls.append(('save', file_format))
                Path(output_file).write_text('obj', encoding='utf-8')
                return 'SUCCESS'

    class Camera:
        def extract_whole_spatial_map(self, spatial_map):
            self.spatial_map = spatial_map

    node = ZedSpatialMappingNode.__new__(ZedSpatialMappingNode)
    node.camera = Camera()
    node.sl = Sl
    node.mapping_started = True
    node.success_frames = 30
    node.min_success_frames_before_export = 30
    node.current_output_dir = str(tmp_path)
    node.output_file = ''
    node.map_type = 'mesh'
    node.started_at = 10.0
    node.status_pub = FakePublisher()
    node.result_pub = FakePublisher()
    node.resolution_preset = 'high'
    node.range_preset = 'near'
    node.save_texture = True
    node.camera_resolution = 'HD720'
    node.camera_fps = 15
    node.depth_mode = 'NEURAL_PLUS'
    node.preview_topic = '/inspection_ai/mapping3d_pointcloud'
    node.preview_frame_id = 'zed_3d_map'
    node.preview_max_points = 1000000
    node.preview_point_count = 0
    node.export_point_count = 0
    node.depth_minimum_distance = 0.25
    node.depth_maximum_distance = 4.0
    node.confidence_threshold = 60
    node.texture_confidence_threshold = 60
    node.spatial_mapping_max_memory_mb = 1024
    node.mesh_filter_preset = 'high'

    node.export_current_map_in_worker()

    assert node.camera.spatial_map.calls == ['filter', 'apply_texture', ('save', 'OBJ')]
    assert node.export_point_count == 1


def test_3d_mapping_does_not_touch_nav2_or_perception_launch():
    nav2 = (REPO_ROOT / 'src/ylhb_base/config/nav2_params.yaml').read_text(encoding='utf-8')
    perception_launch = (
        REPO_ROOT / 'src/ylhb_perception/launch/perception.launch.py'
    ).read_text(encoding='utf-8')

    assert '3d_mapping' not in nav2
    assert 'zed_spatial_mapping' not in nav2
    assert 'ylhb_3d_mapping' not in perception_launch
    assert 'zed_spatial_mapping' not in perception_launch
