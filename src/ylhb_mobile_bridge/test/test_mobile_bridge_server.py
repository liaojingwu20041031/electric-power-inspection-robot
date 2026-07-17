import importlib.util
import asyncio
import json
from pathlib import Path

import pytest

HAS_FASTAPI = importlib.util.find_spec("fastapi") is not None

if HAS_FASTAPI:
    import fastapi.routing

    from ylhb_mobile_bridge.mobile_bridge_server import make_app
    from ylhb_mobile_bridge.platform_store import DeploymentStore

pytestmark = pytest.mark.skipif(
    not HAS_FASTAPI,
    reason="fastapi is not installed",
)


@pytest.fixture(autouse=True)
def run_sync_endpoints_inline(monkeypatch):
    async def run_inline(function, *args, **kwargs):
        return function(*args, **kwargs)

    monkeypatch.setattr(fastapi.routing, "run_in_threadpool", run_inline)


class FakeLogger:
    def error(self, *_args, **_kwargs):
        return None


class FakeBridge:
    require_token = False
    api_token = ""
    map_stream_rate_hz = 1.0
    map_max_size_px = 64

    def __init__(self):
        self.stopped = False
        self.stop_motion_calls = 0
        self.robot_id = "robot-test"
        self.mapping_map_reset_count = 0
        self.mapping_map_available = False
        self.system_commands = []
        self.patrol_commands = []
        self.system_status_payload = {}
        self.patrol_status_payload = {}
        self.local_app_enabled = True
        self.local_app_clients = {'status': 0, 'map': 0}

    def get_logger(self):
        return FakeLogger()

    def robot_status(self):
        return {
            "online": True,
            "pose": None,
            "velocity": None,
            "network": {
                "candidateEndpoints": [
                    {
                        "url": "http://192.168.8.20:8000",
                        "interface": "eth0",
                        "type": "ethernet",
                        "linkUp": True,
                    }
                ]
            },
        }

    def debug_status(self):
        return {"online": True, "map_meta": None}

    def mapping_status(self, process=None):
        return {
            "mapping_status": "not_running",
            "process": process,
            "map_meta": None,
        }

    def map_snapshot(self, downsample=1):
        return None

    def reset_mapping_map(self):
        self.mapping_map_reset_count += 1

    def has_mapping_map(self):
        return self.mapping_map_available

    def publish_velocity(self, *_args):
        return None

    def publish_text_command(self, _text):
        return None

    def stop_all(self):
        self.stopped = True

    def stop_motion(self):
        self.stop_motion_calls += 1
        self.stopped = True

    def publish_system_command(self, command, **extra):
        self.system_commands.append((command, extra))

    def has_system_supervisor(self):
        return bool(self.system_status_payload)

    def system_status(self):
        return self.system_status_payload

    def publish_patrol_command(self, command):
        self.patrol_commands.append(command)

    def is_local_app_enabled(self):
        return self.local_app_enabled

    def local_app_client_connected(self, kind):
        self.local_app_clients[kind] += 1

    def local_app_client_disconnected(self, kind):
        self.local_app_clients[kind] -= 1

    def patrol_status(self):
        return self.patrol_status_payload


class FakeProcessManager:
    def process_status(self, mode):
        return {
            "mode": mode,
            "running": False,
            "managed_by_bridge": False,
        }

    def start(self, mode):
        return f"{mode} started"

    def start_mapping(self):
        return self.start("mapping")

    def stop(self, mode):
        return f"{mode} stopped"

    def save_map(self, map_name):
        return {
            "yaml_path": f"/tmp/{map_name}.yaml",
            "pgm_path": f"/tmp/{map_name}.pgm",
            "output": "saved",
        }


class AsgiResponse:
    def __init__(self, status_code, body):
        self.status_code = status_code
        self._body = body

    def json(self):
        return json.loads(self._body.decode("utf-8"))


class AsgiClient:
    def __init__(self, app):
        self.app = app

    def get(self, path, headers=None):
        return self.request("GET", path, headers=headers)

    def post(self, path, json=None, headers=None):
        return self.request("POST", path, json_body=json, headers=headers)

    def delete(self, path, json=None, headers=None):
        return self.request("DELETE", path, headers=headers)

    def request(self, method, path, json_body=None, headers=None):
        return asyncio.run(
            self._request(method, path, json_body=json_body, headers=headers)
        )

    async def _request(self, method, path, json_body=None, headers=None):
        body = b""
        request_headers = []
        if json_body is not None:
            body = json.dumps(json_body).encode("utf-8")
            request_headers.append((b"content-type", b"application/json"))
        for key, value in (headers or {}).items():
            request_headers.append((key.lower().encode(), value.encode()))

        scope = {
            "type": "http",
            "asgi": {"version": "3.0"},
            "http_version": "1.1",
            "method": method,
            "scheme": "http",
            "path": path,
            "raw_path": path.encode(),
            "query_string": b"",
            "headers": request_headers,
            "client": ("testclient", 50000),
            "server": ("testserver", 80),
        }
        messages = []
        received = False
        disconnect = asyncio.Event()

        async def receive():
            nonlocal received
            if received:
                await disconnect.wait()
                return {"type": "http.disconnect"}
            received = True
            return {"type": "http.request", "body": body, "more_body": False}

        async def send(message):
            messages.append(message)

        await self.app(scope, receive, send)
        status_code = next(
            message["status"]
            for message in messages
            if message["type"] == "http.response.start"
        )
        response_body = b"".join(
            message.get("body", b"")
            for message in messages
            if message["type"] == "http.response.body"
        )
        return AsgiResponse(status_code, response_body)


def make_client(bridge=None, process_manager=None, default_map_path=None):
    return AsgiClient(
        make_app(
            bridge or FakeBridge(),
            process_manager or FakeProcessManager(),
            default_map_path=default_map_path,
        )
    )


def test_status_uses_unified_response_envelope():
    bridge = FakeBridge()
    client = make_client(bridge)
    response = client.get("/api/status")
    second = client.get("/api/status")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["message"] == "status"
    assert body["data"]["online"] is True
    assert body["data"]["apiVersion"] == "1.1"
    assert body["data"]["robotId"] == "robot-test"
    assert body["data"]["bridgeInstanceId"] == second.json()["data"]["bridgeInstanceId"]
    assert body["data"]["network"]["candidateEndpoints"][0]["linkUp"] is True
    assert "timestamp" in body


def run_failing_websocket(path):
    bridge = FakeBridge()
    app = make_app(bridge, FakeProcessManager())
    received_connect = False

    async def run_websocket():
        nonlocal received_connect

        async def receive():
            nonlocal received_connect
            if not received_connect:
                received_connect = True
                return {"type": "websocket.connect"}
            await asyncio.sleep(10)

        async def send(message):
            if message["type"] == "websocket.send":
                raise RuntimeError("client disconnected")

        await app({
            "type": "websocket",
            "asgi": {"version": "3.0"},
            "http_version": "1.1",
            "scheme": "ws",
            "path": path,
            "raw_path": path.encode(),
            "query_string": b"",
            "headers": [],
            "client": ("testclient", 50000),
            "server": ("testserver", 80),
            "subprotocols": [],
        }, receive, send)

    asyncio.run(asyncio.wait_for(run_websocket(), timeout=1.0))
    return bridge


def test_status_websocket_disconnect_does_not_stop_motion():
    assert run_failing_websocket("/ws/status").stop_motion_calls == 0


def test_map_websocket_disconnect_does_not_stop_motion():
    assert run_failing_websocket("/ws/map").stop_motion_calls == 0


def test_validation_errors_use_unified_response_envelope():
    response = make_client().post("/api/cmd_vel", json={"duration_ms": 1})

    assert response.status_code == 422
    body = response.json()
    assert body["ok"] is False
    assert body["error"] == "validation_error"
    assert "timestamp" in body


def test_map_snapshot_without_map_returns_no_map_error():
    response = make_client().get("/api/debug/mapping/map_snapshot")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is False
    assert body["error"] == "no_map"


def test_mapping_save_without_current_slam_map_returns_no_map():
    response = make_client().post(
        "/api/debug/mapping/save",
        json={"map_name": "my_map"},
    )

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is False
    assert body["error"] == "no_map"


def test_mapping_save_with_current_slam_map_runs_map_saver():
    bridge = FakeBridge()
    bridge.mapping_map_available = True

    response = make_client(bridge).post(
        "/api/debug/mapping/save",
        json={"map_name": "my_map"},
    )

    assert response.status_code == 200
    assert response.json()["ok"] is True


def write_map_pair(directory: Path, name: str) -> None:
    (directory / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    (directory / f"{name}.pgm").write_bytes(b"P5\n1 1\n255\n\x00")


def test_map_management_api_lists_renames_and_deletes_maps(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    client = make_client(default_map_path=str(tmp_path / "my_map"))

    listed = client.get("/api/debug/maps")
    renamed = client.post(
        "/api/debug/maps/factory/rename",
        json={"new_name": "factory_floor_1"},
    )
    deleted = client.delete("/api/debug/maps/factory_floor_1")

    assert listed.status_code == 200
    assert listed.json()["data"]["count"] == 2
    assert renamed.status_code == 200
    assert renamed.json()["data"]["name"] == "factory_floor_1"
    assert deleted.status_code == 200
    assert sorted(deleted.json()["data"]["deleted"]) == [
        "factory_floor_1.pgm",
        "factory_floor_1.yaml",
    ]


def test_map_management_api_previews_map(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    client = make_client(default_map_path=str(tmp_path / "my_map"))

    response = client.get("/api/debug/maps/factory/preview")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["data"]["map_meta"]["yaml_file"] == "factory.yaml"
    assert body["data"]["png_base64"]


def test_map_management_api_confirms_default_and_archives_routes(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    (tmp_path / "route_patrol_001.json").write_text("{}", encoding="utf-8")
    client = make_client(default_map_path=str(tmp_path / "my_map"))

    response = client.post("/api/debug/maps/factory/confirm_default")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["data"]["default"]["name"] == "my_map"
    assert body["data"]["archived_routes"][0]["to"].startswith(
        "deprecated_route_patrol_001_"
    )


def test_default_map_stays_applied_when_upload_task_creation_fails(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    bridge = FakeBridge()
    bridge.map_upload_worker = type(
        'FailingUploadWorker',
        (),
        {'enqueue': lambda *_args: (_ for _ in ()).throw(RuntimeError('cloud unavailable'))},
    )()

    response = make_client(
        bridge=bridge,
        default_map_path=str(tmp_path / "my_map"),
    ).post("/api/debug/maps/factory/confirm_default")

    assert response.status_code == 200
    data = response.json()["data"]
    assert data["default_applied"] is True
    assert data["default"]["name"] == "my_map"
    assert data["upload"]["status"] == "FAILED_TO_CREATE"
    assert (tmp_path / "my_map.yaml").read_text(encoding="utf-8").startswith(
        "image: my_map.pgm"
    )


def test_current_default_can_be_uploaded_while_map_server_is_running(tmp_path):
    write_map_pair(tmp_path, "my_map")
    bridge = FakeBridge()
    bridge.debug_status = lambda: {
        "online": True,
        "nodes": {"map_server": True},
    }
    bridge.map_upload_worker = type(
        'RecordingUploadWorker',
        (),
        {'enqueue': lambda _self, yaml_path, pgm_path: {
            'task_created': True,
            'task_id': 'upload-1',
            'status': 'PENDING',
            'map_asset_id': '',
            'error': '',
            'content_identity_sha256': 'a' * 64,
        }},
    )()

    response = make_client(
        bridge=bridge,
        default_map_path=str(tmp_path / "my_map"),
    ).post("/api/debug/maps/my_map/confirm_default")

    assert response.status_code == 200
    assert response.json()["data"]["changed"] is False
    assert response.json()["data"]["upload"]["status"] == "PENDING"


@pytest.mark.parametrize(
    ("method", "path", "json", "status_code", "error"),
    [
        (
            "post",
            "/api/debug/maps/missing/rename",
            {"new_name": "new"},
            404,
            "map_not_found",
        ),
        (
            "delete",
            "/api/debug/maps/my_map",
            None,
            409,
            "default_map_protected",
        ),
        (
            "post",
            "/api/debug/maps/factory/rename",
            {"new_name": "../bad"},
            422,
            "validation_error",
        ),
    ],
)
def test_map_management_api_returns_specific_errors(
    tmp_path,
    method,
    path,
    json,
    status_code,
    error,
):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    response = getattr(
        make_client(default_map_path=str(tmp_path / "my_map")),
        method,
    )(path, json=json)

    assert response.status_code == status_code
    assert response.json()["error"] == error


@pytest.mark.parametrize("active_node", ["slam_toolbox", "map_server"])
def test_map_mutations_are_rejected_while_map_is_in_use(tmp_path, active_node):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    bridge = FakeBridge()
    bridge.debug_status = lambda: {
        "online": True,
        "nodes": {active_node: True},
    }
    client = make_client(
        bridge=bridge,
        default_map_path=str(tmp_path / "my_map"),
    )

    rename = client.post(
        "/api/debug/maps/factory/rename",
        json={"new_name": "new_factory"},
    )
    delete = client.delete("/api/debug/maps/factory")
    confirm = client.post("/api/debug/maps/factory/confirm_default")

    assert rename.status_code == 409
    assert rename.json()["error"] == "map_in_use"
    assert delete.status_code == 409
    assert delete.json()["error"] == "map_in_use"
    assert confirm.status_code == 409
    assert confirm.json()["error"] == "map_in_use"


def test_system_start_and_stop_routes_restrict_modes():
    bridge = FakeBridge()
    bridge.system_status_payload = {"success": True, "message": "ready"}
    client = make_client(bridge)

    start = client.post("/api/debug/system/start/bringup").json()
    stop = client.post("/api/debug/system/stop/mapping").json()

    assert start["ok"] is True
    assert stop["ok"] is True
    assert bridge.system_commands == [
        ("start_bringup", {}),
        ("stop_mapping", {}),
    ]
    response = client.post("/api/debug/system/start/localization")

    assert response.status_code == 404


@pytest.mark.parametrize("action", ["start", "stop"])
def test_system_process_routes_do_not_expose_navigation(action):
    response = make_client().post(
        f"/api/debug/system/{action}/navigation"
    )

    assert response.status_code == 404
    assert response.json()["error"] == "not_found"


@pytest.mark.parametrize(
    "path",
    [
        "/api/debug/system/start/mapping",
        "/api/debug/mapping/start",
    ],
)
def test_mapping_start_routes_clear_previous_map(path):
    bridge = FakeBridge()
    if path.startswith("/api/debug/system/"):
        bridge.system_status_payload = {"success": True, "message": "ready"}

    response = make_client(bridge).post(path)

    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert bridge.mapping_map_reset_count == 1


@pytest.mark.parametrize(
    "path",
    [
        "/api/debug/system/stop/mapping",
        "/api/debug/mapping/stop",
    ],
)
def test_mapping_stop_routes_clear_cached_map(path):
    bridge = FakeBridge()
    if path.startswith("/api/debug/system/"):
        bridge.system_status_payload = {"success": True, "message": "ready"}

    response = make_client(bridge).post(path)

    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert bridge.mapping_map_reset_count == 1


def test_system_status_returns_bringup_and_mapping_processes():
    bridge = FakeBridge()
    bridge.system_status_payload = {
        "bringup": "running",
        "mapping": "stopped",
        "success": True,
    }

    response = make_client(bridge).get("/api/debug/system/status")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["data"]["bringup"] == "running"
    assert body["data"]["mapping"] == "stopped"


@pytest.mark.parametrize("mode", ["bringup", "mapping"])
def test_system_start_routes_use_unified_response_envelope(mode):
    bridge = FakeBridge()
    bridge.system_status_payload = {"success": True, "message": "ready"}

    response = make_client(bridge).post(f"/api/debug/system/start/{mode}")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["message"] == f"system command sent: start_{mode}"
    assert "timestamp" in body


def test_system_start_requires_supervisor_online():
    response = make_client().post("/api/debug/system/start/bringup")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is False
    assert body["error"] == "supervisor_unavailable"


@pytest.mark.parametrize(
    ("endpoint", "command"),
    [
        ("/api/debug/patrol/start", "start"),
        ("/api/debug/patrol/pause", "pause"),
        ("/api/debug/patrol/resume", "resume"),
        ("/api/debug/patrol/cancel", "cancel"),
        ("/api/debug/patrol/reload", "reload"),
        ("/api/debug/patrol/initialize", "initialize"),
    ],
)
def test_patrol_command_routes_publish_to_patrol_topic(endpoint, command):
    bridge = FakeBridge()

    response = make_client(bridge).post(endpoint)

    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert bridge.patrol_commands == [command]


def test_patrol_status_route_reports_executor_missing_when_no_status():
    response = make_client().get("/api/debug/patrol/status")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["data"]["executor_running"] is False
    assert body["data"]["state"] == "unavailable"


def test_http_token_auth_allows_header_and_rejects_missing_token():
    bridge = FakeBridge()
    bridge.require_token = True
    bridge.api_token = "secret"
    client = make_client(bridge)

    assert client.get("/api/status").status_code == 401
    response = client.get("/api/status", headers={"X-API-Token": "secret"})
    assert response.status_code == 200
    assert response.json()["ok"] is True


def test_http_token_auth_allows_bearer_header():
    bridge = FakeBridge()
    bridge.require_token = True
    bridge.api_token = "secret"

    response = make_client(bridge).get(
        "/api/status",
        headers={"Authorization": "Bearer secret"},
    )

    assert response.status_code == 200
    assert response.json()["ok"] is True


def test_disabled_local_app_returns_503_without_blocking_platform_api():
    bridge = FakeBridge()
    bridge.local_app_enabled = False
    app = make_app(bridge, FakeProcessManager())

    @app.get('/api/platform/v1/health')
    def platform_health():
        return {'online': True}

    client = AsgiClient(app)
    response = client.get('/api/status')

    assert response.status_code == 503
    body = response.json()
    assert body['ok'] is False
    assert body['error'] == 'local_app_disabled'
    assert body['message'] == '本地 APP 服务已关闭'
    assert body['data'] is None
    assert 'timestamp' in body
    assert client.get('/api/platform/v1/health').status_code == 200


def test_local_app_and_cloud_overrides_use_separate_settings(tmp_path):
    store = DeploymentStore(tmp_path)

    store.set_bridge_setting('local_app_enabled_override', 'false')
    store.set_cloud_state('cloud_enabled_override', 'true')

    assert store.bridge_setting('local_app_enabled_override', '') == 'false'
    assert store.cloud_state('cloud_enabled_override', '') == 'true'
    assert store.cloud_state('local_app_enabled_override', '') == ''


@pytest.mark.parametrize(
    ('path', 'client_kind', 'close_raises'),
    [
        ('/ws/status', 'status', False),
        ('/ws/map', 'map', False),
        ('/ws/status', 'status', True),
        ('/ws/map', 'map', True),
    ],
)
def test_websockets_close_when_local_app_is_disabled(
    path, client_kind, close_raises
):
    bridge = FakeBridge()
    bridge.status_rate_hz = 0.1
    bridge.map_stream_rate_hz = 0.1
    app = make_app(bridge, FakeProcessManager())
    messages = []
    received_connect = False

    async def run_websocket():
        nonlocal received_connect

        async def receive():
            nonlocal received_connect
            if not received_connect:
                received_connect = True
                return {'type': 'websocket.connect'}
            await asyncio.sleep(10)

        async def send(message):
            messages.append(message)
            if message['type'] == 'websocket.send':
                bridge.local_app_enabled = False
            if message['type'] == 'websocket.close' and close_raises:
                raise RuntimeError('client disconnected during close')

        await app({
            'type': 'websocket',
            'asgi': {'version': '3.0'},
            'http_version': '1.1',
            'scheme': 'ws',
            'path': path,
            'raw_path': path.encode(),
            'query_string': b'',
            'headers': [],
            'client': ('testclient', 50000),
            'server': ('testserver', 80),
            'subprotocols': [],
        }, receive, send)

    asyncio.run(asyncio.wait_for(run_websocket(), timeout=1.0))

    closes = [message for message in messages if message['type'] == 'websocket.close']
    assert closes[-1]['code'] == 1012
    assert bridge.local_app_clients[client_kind] == 0
    assert bridge.stopped is False
