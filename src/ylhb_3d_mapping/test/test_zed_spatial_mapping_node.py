import json
from pathlib import Path

import pytest
import yaml
from rclpy.validate_topic_name import validate_topic_name

from ylhb_3d_mapping.zed_spatial_mapping_node import (
    ZedSpatialMappingNode,
    build_metadata,
    parse_command_json,
)


REPO_ROOT = Path(__file__).resolve().parents[3]


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def test_yaml_defaults_to_fused_point_cloud():
    config = yaml.safe_load(
        (REPO_ROOT / 'src/ylhb_3d_mapping/config/zed_spatial_mapping.yaml').read_text(encoding='utf-8')
    )

    params = config['zed_spatial_mapping_node']['ros__parameters']
    assert params['map_type'] == 'fused_point_cloud'
    assert params['output_root'].endswith('/runs/3d_mapping')


def test_yaml_topics_are_valid_ros_names():
    config_text = (REPO_ROOT / 'src/ylhb_3d_mapping/config/zed_spatial_mapping.yaml').read_text(
        encoding='utf-8'
    )
    config = yaml.safe_load(config_text)
    params = config['zed_spatial_mapping_node']['ros__parameters']

    for key in ('command_topic', 'status_topic', 'result_topic'):
        validate_topic_name(params[key])

    old_prefix = '/inspection_ai/' + '3d' + '_mapping_'
    assert old_prefix not in config_text


def test_command_json_accepts_known_commands_and_rejects_bad_input():
    assert parse_command_json('{"command":"start"}')['command'] == 'start'
    assert parse_command_json('{"command":"stop_and_export"}')['command'] == 'stop_and_export'

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


def test_3d_mapping_does_not_touch_nav2_or_perception_launch():
    nav2 = (REPO_ROOT / 'src/ylhb_base/config/nav2_params.yaml').read_text(encoding='utf-8')
    perception_launch = (
        REPO_ROOT / 'src/ylhb_perception/launch/perception.launch.py'
    ).read_text(encoding='utf-8')

    assert '3d_mapping' not in nav2
    assert 'zed_spatial_mapping' not in nav2
    assert 'ylhb_3d_mapping' not in perception_launch
    assert 'zed_spatial_mapping' not in perception_launch
