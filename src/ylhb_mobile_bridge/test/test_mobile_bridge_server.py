import importlib.util

import pytest

HAS_FASTAPI = importlib.util.find_spec("fastapi") is not None

if HAS_FASTAPI:
    from fastapi.testclient import TestClient
    from ylhb_mobile_bridge.mobile_bridge_server import make_app

pytestmark = pytest.mark.skipif(
    not HAS_FASTAPI,
    reason="fastapi is not installed",
)


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

    def get_logger(self):
        return FakeLogger()

    def robot_status(self):
        return {"online": True, "pose": None, "velocity": None}

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

    def publish_velocity(self, *_args):
        return None

    def publish_text_command(self, _text):
        return None

    def stop_all(self):
        self.stopped = True

    def stop_motion(self):
        self.stopped = True


class FakeProcessManager:
    def process_status(self, mode):
        return {
            "mode": mode,
            "running": False,
            "managed_by_bridge": False,
        }

    def start(self, mode):
        return f"{mode} started"

    def stop(self, mode):
        return f"{mode} stopped"


def make_client(bridge=None):
    return TestClient(make_app(bridge or FakeBridge(), FakeProcessManager()))


def test_status_uses_unified_response_envelope():
    response = make_client().get("/api/status")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["message"] == "status"
    assert body["data"]["online"] is True
    assert "timestamp" in body


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


def test_system_start_and_stop_routes_restrict_modes():
    client = make_client()

    start = client.post("/api/debug/system/start/bringup").json()
    stop = client.post("/api/debug/system/stop/navigation").json()

    assert start["ok"] is True
    assert stop["ok"] is True
    response = client.post("/api/debug/system/start/localization")

    assert response.status_code == 404


def test_system_status_returns_bringup_and_mapping_processes():
    response = make_client().get("/api/debug/system/status")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert set(body["data"]) == {"bringup", "mapping"}
    assert body["data"]["bringup"]["mode"] == "bringup"
    assert body["data"]["mapping"]["mode"] == "mapping"


@pytest.mark.parametrize("mode", ["bringup", "mapping"])
def test_system_start_routes_use_unified_response_envelope(mode):
    response = make_client().post(f"/api/debug/system/start/{mode}")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["message"] == f"{mode} started"
    assert "timestamp" in body


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
