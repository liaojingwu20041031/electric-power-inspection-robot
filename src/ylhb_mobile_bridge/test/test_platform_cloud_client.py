import hashlib
import io
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from PIL import Image

from ylhb_mobile_bridge.platform_cloud_client import PlatformCloudClient
from ylhb_mobile_bridge.platform_store import DeploymentStore, canonical_json
from ylhb_mobile_bridge.map_upload import (
    MapUploadWorker,
    content_identity_sha256,
    normalized_map_identity,
)
from ylhb_mobile_bridge.patrol_route_store import (
    validate_route_file,
    validate_route_map_binding,
)


class FakeStore:
    def __init__(self):
        self.values = {}

    def cloud_state(self, key, default=''):
        return self.values.get(key, default)

    def set_cloud_state(self, key, value):
        self.values[key] = str(value)

    def pending_event_count(self):
        return 0

    def pending_command_count(self):
        return 0

    def latest_event_sequence(self):
        return 7


class FakeNetworkStatusProvider:
    def __init__(self):
        self.calls = []

    def route_to_host(self, hostname):
        self.calls.append(hostname)
        return {
            'interface': 'eth0',
            'type': 'ethernet',
            'label': '5G 有线网络',
            'sourceAddress': '192.168.8.20',
            'gateway': '192.168.8.1',
            'metric': 100,
            'alternateCloudRoutes': [
                {
                    'interface': 'wlan0',
                    'type': 'wifi',
                    'label': 'Wi-Fi 网络',
                    'sourceAddress': '192.168.137.100',
                    'gateway': '192.168.137.1',
                    'metric': 600,
                }
            ],
            'failoverAvailable': True,
        }


class FakeResponse:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return json.dumps({'ok': True}).encode('utf-8')


def make_client(monkeypatch):
    monkeypatch.setenv('YLHB_CLOUD_BASE_URL', 'https://cloud.example')
    monkeypatch.setenv('YLHB_CLOUD_ROBOT_TOKEN', 'secret')
    bridge = SimpleNamespace(
        network_status=FakeNetworkStatusProvider(),
        cloud_status_snapshot=lambda: {
            'state': 'idle',
            'platformContext': {},
            'mapPose': None,
            'odomPose': None,
            'health': {'ok': True},
        },
    )
    return PlatformCloudClient(FakeStore(), bridge, 'robot-1', 'boot-1')


def test_status_adds_system_routing_diagnostics(monkeypatch):
    client = make_client(monkeypatch)

    status = client.status()

    assert status['networkMode'] == 'system-routing'
    assert status['cloudEgress'] == {
        'interface': 'eth0',
        'type': 'ethernet',
        'label': '5G 有线网络',
        'sourceAddress': '192.168.8.20',
        'gateway': '192.168.8.1',
        'metric': 100,
    }
    assert status['alternateCloudRoutes'][0]['interface'] == 'wlan0'
    assert status['failoverAvailable'] is True
    assert status['lastSuccessfulEgress'] == {}


def test_heartbeat_payload_does_not_include_local_network_diagnostics(monkeypatch):
    client = make_client(monkeypatch)

    payload = client._heartbeat_payload()

    assert payload == {
        'protocolVersion': '1.0',
        'robotId': 'robot-1',
        'bootId': 'boot-1',
        'softwareVersion': 'unknown',
        'state': 'idle',
        'activeExecutionId': None,
        'activeDeploymentId': None,
        'lastReceivedCommandId': None,
        'latestLocalEventSequence': 7,
        'mapPose': None,
        'odomPose': None,
        'health': {'ok': True},
    }
    assert 'cloudEgress' not in payload
    assert 'networkMode' not in payload


def test_successful_cloud_request_records_egress_without_changing_request(monkeypatch):
    client = make_client(monkeypatch)
    monkeypatch.setattr(
        'ylhb_mobile_bridge.platform_cloud_client.urllib.request.urlopen',
        lambda *_args, **_kwargs: FakeResponse(),
    )

    assert client._request('GET', '/health') == {'ok': True}

    status = client.status()
    assert status['lastSuccessfulEgress']['interface'] == 'eth0'
    assert client.bridge.network_status.calls == [
        'cloud.example',
        'cloud.example',
    ]


def test_map_upload_identity_snapshot_and_persistent_states(tmp_path, monkeypatch):
    monkeypatch.setenv('YLHB_MAP_UPLOAD_ENABLED', 'false')
    maps = tmp_path / 'maps'
    maps.mkdir()
    yaml_path = maps / 'my_map.yaml'
    pgm_path = maps / 'my_map.pgm'
    pgm = b'P5\n2 1\n255\n\x00\xff'
    pgm_hash = hashlib.sha256(pgm).hexdigest()
    yaml_path.write_text(
        'image: my_map.pgm\nresolution: 0.0500\norigin: [-0, 0.00, 0]\n',
        encoding='utf-8',
    )
    pgm_path.write_bytes(pgm)
    first_identity = normalized_map_identity(yaml_path.read_bytes(), pgm_hash)
    second_identity = normalized_map_identity(
        b'mode: trinary\nfree_thresh: 0.250\nimage: my_map.pgm\n'
        b'origin: [0.0, -0, 0.000]\noccupied_thresh: 0.650\n'
        b'negate: 0\nresolution: 0.05\n',
        pgm_hash,
    )
    assert first_identity == second_identity
    assert content_identity_sha256(first_identity) == content_identity_sha256(second_identity)

    store = DeploymentStore(tmp_path / 'platform')
    worker = MapUploadWorker(store, 'robot-1')
    created = worker.enqueue(yaml_path, pgm_path)
    duplicate = worker.enqueue(yaml_path, pgm_path)
    assert created['task_created'] is True
    assert duplicate['task_created'] is False
    assert duplicate['task_id'] == created['task_id']
    record = store.map_upload(created['task_id'])
    assert record and record['status'] == 'PENDING'
    snapshot = Path(record['pgm_path'])
    pgm_path.write_bytes(b'P5\n2 1\n255\n\x01\xfe')
    assert snapshot.read_bytes() == pgm

    retryable = store.finish_map_upload(
        record['task_id'], 'FAILED_RETRYABLE', error='network',
        next_retry_at=time.time() - 1,
    )
    assert retryable['status'] == 'FAILED_RETRYABLE'
    assert store.next_due_map_upload()['task_id'] == record['task_id']
    final = store.finish_map_upload(record['task_id'], 'FAILED_FINAL', error='format')
    assert final['status'] == 'FAILED_FINAL'
    requeued = store.requeue_map_upload(record['task_id'])
    assert requeued['status'] == 'PENDING'
    succeeded = store.finish_map_upload(
        record['task_id'], 'SUCCEEDED', map_asset_id='map-1'
    )
    assert succeeded['status'] == 'SUCCEEDED'
    assert worker.enqueue(yaml_path, snapshot)['task_id'] == record['task_id']
    assert store.map_upload(record['task_id'])['map_asset_id'] == 'map-1'

    monkeypatch.setenv('YLHB_MAP_UPLOAD_SNAPSHOT_MAX_BYTES', '1')
    limited = MapUploadWorker(DeploymentStore(tmp_path / 'limited'), 'robot-1')
    with pytest.raises(ValueError, match='disk limit'):
        limited.enqueue(yaml_path, snapshot)


def test_start_command_task_id_can_be_restored_without_changing_start_contract(tmp_path):
    store = DeploymentStore(tmp_path / 'platform')
    command = {
        'commandId': 'command-1', 'requestId': 'request-1', 'type': 'START',
        'executionId': 'execution-1', 'deploymentId': 'deployment-1',
        'executorRouteId': 'route-1',
    }

    store.receive_cloud_command(command)
    assert store.task_id_for_execution('execution-1') == ''

    second = {
        **command, 'commandId': 'command-2', 'requestId': 'request-2',
        'executionId': 'execution-2', 'taskId': 'task-2',
    }
    store.receive_cloud_command(second)
    assert store.task_id_for_execution('execution-2') == 'task-2'


def test_inspection_image_is_persisted_deduplicated_and_removed_after_success(tmp_path, monkeypatch):
    from ylhb_mobile_bridge.inspection_image_upload import InspectionImageUploadWorker

    monkeypatch.setenv('YLHB_INSPECTION_IMAGE_UPLOAD_ENABLED', 'true')
    monkeypatch.setenv('YLHB_CLOUD_BASE_URL', 'https://cloud.example')
    monkeypatch.setenv('YLHB_CLOUD_ROBOT_TOKEN', 'secret')
    store = DeploymentStore(tmp_path / 'platform')
    worker = InspectionImageUploadWorker(store, 'robot-1')
    request = {
        'capture_identity': 'execution-1:1:0:checkpoint-1:MOVING:123',
        'task_id': 'task-1', 'execution_id': 'execution-1',
        'checkpoint_id': 'checkpoint-1', 'kind': 'MOVING',
        'captured_at': '2026-07-20T00:00:00+00:00',
    }
    created = worker.enqueue(request, b'jpeg-bytes')
    duplicate = worker.enqueue(request, b'different-bytes')
    record = store.inspection_image_upload(created['capture_task_id'])

    assert created['task_created'] is True
    assert duplicate['task_created'] is False
    assert duplicate['capture_task_id'] == created['capture_task_id']
    assert Path(record['file_path']).read_bytes() == b'jpeg-bytes'
    idempotency_key = record['idempotency_key']
    monkeypatch.setattr(worker, '_upload', lambda _record: {
        'taskId': 'task-1', 'executionId': 'execution-1',
        'checkpointId': 'checkpoint-1', 'imageId': 'image-1',
    })

    worker._process(record)

    succeeded = store.inspection_image_upload(created['capture_task_id'])
    assert succeeded['status'] == 'SUCCEEDED'
    assert succeeded['idempotency_key'] == idempotency_key
    assert succeeded['image_id'] == 'image-1'
    assert not Path(record['file_path']).exists()


def test_inspection_image_retry_reuses_file_and_idempotency_key(tmp_path, monkeypatch):
    from ylhb_mobile_bridge.inspection_image_upload import (
        InspectionImageUploadError,
        InspectionImageUploadWorker,
    )

    monkeypatch.setenv('YLHB_INSPECTION_IMAGE_UPLOAD_ENABLED', 'true')
    monkeypatch.setenv('YLHB_CLOUD_BASE_URL', 'https://cloud.example')
    monkeypatch.setenv('YLHB_CLOUD_ROBOT_TOKEN', 'secret')
    store = DeploymentStore(tmp_path / 'platform')
    worker = InspectionImageUploadWorker(store, 'robot-1')
    created = worker.enqueue({
        'capture_identity': 'execution-1:1:0:checkpoint-1:MOVING:123',
        'task_id': 'task-1', 'execution_id': 'execution-1',
        'checkpoint_id': 'checkpoint-1', 'kind': 'MOVING',
        'captured_at': '2026-07-20T00:00:00+00:00',
    }, b'jpeg-bytes')
    record = store.inspection_image_upload(created['capture_task_id'])
    path = record['file_path']
    key = record['idempotency_key']
    monkeypatch.setattr(worker, '_upload', lambda _record: (_ for _ in ()).throw(
        InspectionImageUploadError('HTTP 503', retryable=True)
    ))

    worker._process(record)

    retry = store.inspection_image_upload(created['capture_task_id'])
    assert retry['status'] == 'FAILED_RETRYABLE'
    assert retry['idempotency_key'] == key
    assert retry['file_path'] == path
    assert Path(path).read_bytes() == b'jpeg-bytes'


def test_moving_image_longest_edge_is_resized_but_arrival_jpeg_is_unchanged():
    from ylhb_mobile_bridge.inspection_image_upload import prepare_inspection_image

    source = io.BytesIO()
    Image.new('RGB', (600, 1200), color=(1, 2, 3)).save(source, format='JPEG', quality=90)
    original = source.getvalue()

    moving = prepare_inspection_image(original, 'jpeg', 'MOVING', 640, 70)
    arrival = prepare_inspection_image(original, 'jpeg', 'ARRIVAL', 640, 70)

    with Image.open(io.BytesIO(moving)) as image:
        assert image.size == (320, 640)
    assert arrival == original


def test_deployment_hashes_raw_route_and_publishes_next_local_route(tmp_path):
    maps_dir = tmp_path / 'maps'
    maps_dir.mkdir()
    pgm = b'P5\n2 1\n255\n\x00\xff'
    pgm_hash = hashlib.sha256(pgm).hexdigest()
    (maps_dir / 'my_map.pgm').write_bytes(pgm)
    (maps_dir / 'my_map.yaml').write_text(
        'image: my_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        encoding='utf-8',
    )
    (maps_dir / 'route_patrol_001.json').write_text('{}', encoding='utf-8')
    route = {
        'version': 3,
        'frame_id': 'map',
        'map': {
            'yaml': 'cloud_map.yaml',
            'image': 'cloud_map.pgm',
            'resolution': 0.05,
            'origin': [0, 0, 0],
            'width': 2,
            'height': 1,
            'image_sha256': pgm_hash,
        },
        'active_route_id': 'cloud_route',
        'start_pose': {
            'frame_id': 'map',
            'pose': {'x': 0, 'y': 0, 'yaw': 0},
            'location': {
                'type': 'map_pose', 'frame_id': 'map',
                'x': 0, 'y': 0, 'yaw': 0,
            },
        },
        'targets': [{
            'id': 'target_001',
            'pose': {'x': 5, 'y': 0, 'yaw': 0},
            'location': {
                'type': 'map_pose', 'frame_id': 'map',
                'x': 5, 'y': 0, 'yaw': 0,
            },
            'task_duration_sec': 5,
        }],
        'routes': [{
            'id': 'cloud_route',
            'target_ids': ['target_001'],
            'goal_timeout_sec': 120,
        }],
        'keepout_zones': [],
    }
    route_bytes = json.dumps(route, indent=2).encode('utf-8')
    route_hash = hashlib.sha256(canonical_json(route)).hexdigest()
    assert hashlib.sha256(canonical_json(validate_route_file(route))).hexdigest() != route_hash
    manifest = {
        'schemaVersion': '1.0',
        'robotId': 'robot-1',
        'routeRevisionId': 'revision-1',
        'routeRevisionContentSha256': route_hash,
        'routePayloadSha256': route_hash,
        'mapAssetId': 'map-1',
        'mapImageSha256': pgm_hash,
        'yamlName': 'cloud_map.yaml',
        'pgmName': 'cloud_map.pgm',
    }
    store = DeploymentStore(tmp_path / 'platform', maps_dir / 'my_map')

    result = store.install(
        'deployment-1', manifest, route_bytes,
        b'image: cloud_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        pgm,
    )

    deployment_route = Path(result['routePath'])
    assert hashlib.sha256(deployment_route.read_bytes()).hexdigest() == route_hash
    assert hashlib.sha256(Path(result['mapPgmPath']).read_bytes()).hexdigest() == pgm_hash
    local_route_path = maps_dir / 'route_patrol_002.json'
    local_route = json.loads(local_route_path.read_text(encoding='utf-8'))
    assert local_route['map']['yaml'] == 'my_map.yaml'
    assert local_route['map']['image'] == 'my_map.pgm'
    validate_route_map_binding(local_route, maps_dir / 'my_map.yaml')

    assert store.install(
        'deployment-1', manifest, route_bytes,
        b'image: cloud_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        pgm,
    )['idempotent'] is True
    assert not (maps_dir / 'route_patrol_003.json').exists()
