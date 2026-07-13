#!/usr/bin/env python3
"""Offline checks for durable outbound Robot Platform Protocol state."""
import copy
import hashlib
import json
import tempfile
from pathlib import Path

import yaml

from ylhb_mobile_bridge.platform_cloud_client import PlatformCloudClient
from ylhb_mobile_bridge.patrol_route_store import load_route_file
from ylhb_mobile_bridge.platform_store import DeploymentStore, canonical_json, sha256


def deployment_payload() -> tuple[dict, bytes, bytes, bytes]:
    route = copy.deepcopy(load_route_file(Path(__file__).parents[1] / "src/ylhb_mobile_bridge/test/fixtures/patrol_routes.json"))
    pgm = b"P5\n4 3\n255\n" + bytes([254]) * 12
    route.update({"version": 3, "map": {"yaml": "site_map.yaml", "image": "site_map.pgm", "resolution": 0.025, "origin": [-7.07, -13.3, 0.0], "width": 4, "height": 3, "image_sha256": sha256(pgm)}})
    route["start_pose"].update({"frame_id": "map", "location": {"type": "map_pose", "frame_id": "map", **route["start_pose"]["pose"]}})
    for target in route["targets"]:
        target["location"] = {"type": "map_pose", "frame_id": "map", **target["pose"]}
    route["keepout_zones"] = []
    route_bytes = canonical_json(route)
    yaml_bytes = yaml.safe_dump({"image": "site_map.pgm", "resolution": 0.025, "origin": [-7.07, -13.3, 0.0]}, sort_keys=False).encode()
    manifest = {"schemaVersion": "1.0", "robotId": "robot-001", "routeRevisionId": "route-r1", "routeRevisionContentSha256": sha256(route_bytes), "routePayloadSha256": sha256(route_bytes), "routeContentSha256": sha256(route_bytes), "mapAssetId": "map-1", "mapImageSha256": sha256(pgm), "yamlName": "site_map.yaml", "pgmName": "site_map.pgm"}
    return manifest, route_bytes, yaml_bytes, pgm


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        store = DeploymentStore(Path(temporary))
        manifest, route, yaml_bytes, pgm = deployment_payload()
        deployment = store.install("deploy-1", manifest, route, yaml_bytes, pgm)
        assert Path(deployment["mapYamlPath"]).name == "site_map.yaml"
        assert Path(deployment["mapPgmPath"]).name == "site_map.pgm"
        assert Path(deployment["routePath"]).name == "route.json"
        command = {"commandId": "command-1", "requestId": "request-1", "type": "START", "executionId": "execution-1", "deploymentId": "deploy-1", "executorRouteId": "route-1"}
        assert store.receive_cloud_command(command)["state"] == "RECEIVED"
        assert store.receive_cloud_command(command)["state"] == "RECEIVED"
        store.set_command_state("command-1", "APPLIED", {"event": "route_started"})
        assert store.receive_cloud_command(command)["state"] == "APPLIED"
        assert store.append_event({"event": "route_started", "command_id": "command-1"})["sequence"] == 1
        store.set_cloud_state("last_uploaded_sequence", "0")
        assert store.cloud_state("last_uploaded_sequence") == "0"
        class Bridge:
            def enqueue_cloud_command(self, _command): pass
            def cloud_status_snapshot(self): return {"health": {}}
        downloader = PlatformCloudClient(store, Bridge(), "robot-001", "boot-1")
        assets = {"manifest": manifest, "route": route, "yaml": yaml_bytes, "pgm": pgm}
        downloader._request = lambda _method, path, binary=False, body=None: assets[path.rsplit("/", 1)[-1]]
        downloaded = downloader._download_deployment("deploy-2")
        assert Path(downloaded["mapYamlPath"]).name == "site_map.yaml"
        uploaded = []
        downloader._request = lambda _method, _path, body=None, binary=False: uploaded.append(body) or {"acceptedThroughSequence": 0}
        downloader._upload_events()
        downloader._upload_events()
        assert uploaded[0]["events"][0]["sequence"] == uploaded[1]["events"][0]["sequence"] == 1
    print("robot platform v1 smoke: ok")


if __name__ == "__main__":
    main()
