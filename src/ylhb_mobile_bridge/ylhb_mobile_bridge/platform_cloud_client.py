"""Outbound HTTPS client for the public Robot Bridge; it never accepts inbound control."""
import json
import logging
import os
import random
import ssl
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict

from .platform_store import PlatformStoreError


LOG = logging.getLogger(__name__)
BACKOFF_SECONDS = (1, 2, 4, 8, 15, 30)


class CloudRequestError(RuntimeError):
    def __init__(self, message: str, retry_after: float | None = None):
        super().__init__(message)
        self.retry_after = retry_after


class PlatformCloudClient:
    def __init__(self, store, bridge, robot_id: str, boot_id: str):
        self.store, self.bridge, self.robot_id, self.boot_id = store, bridge, robot_id, boot_id
        self.enabled = os.environ.get("YLHB_CLOUD_ENABLED", "false").strip().lower() in {"1", "true", "yes", "on"}
        self.base_url = os.environ.get("YLHB_CLOUD_BASE_URL", "").rstrip("/")
        self.token = os.environ.get("YLHB_CLOUD_ROBOT_TOKEN", "")
        self.timeout = float(os.environ.get("YLHB_CLOUD_REQUEST_TIMEOUT_SEC", "10"))
        self.idle_heartbeat = float(os.environ.get("YLHB_CLOUD_IDLE_HEARTBEAT_SEC", "3"))
        self.active_heartbeat = float(os.environ.get("YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC", "1"))
        self.max_backoff = float(os.environ.get("YLHB_CLOUD_MAX_BACKOFF_SEC", "30"))
        self.software_version = os.environ.get("YLHB_SOFTWARE_VERSION", "unknown")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._context = ssl.create_default_context(cafile=os.environ.get("YLHB_CLOUD_CA_FILE") or None)
        if self.enabled and (urllib.parse.urlparse(self.base_url).scheme != "https" or not self.token):
            raise ValueError("YLHB_CLOUD_ENABLED requires HTTPS base URL and robot token")

    def start(self) -> None:
        if self.enabled and not self._thread:
            self._thread = threading.Thread(target=self._run, name="platform-cloud-client", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=self.timeout + 1)

    def _request(self, method: str, path: str, body: Dict[str, Any] | None = None, binary: bool = False):
        data = None if body is None else json.dumps(body, ensure_ascii=False).encode("utf-8")
        headers = {"Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(self.base_url + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout, context=self._context) as response:
                content = response.read()
                return content if binary else json.loads(content.decode("utf-8"))
        except urllib.error.HTTPError as exc:
            retry_after = exc.headers.get("Retry-After", "")
            try:
                retry_after_value = min(self.max_backoff, max(0.0, float(retry_after)))
            except ValueError:
                retry_after_value = None
            raise CloudRequestError(f"HTTP {exc.code}", retry_after_value) from exc
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise CloudRequestError(str(exc)) from exc

    def _upload_events(self) -> None:
        cursor = int(self.store.cloud_state("last_uploaded_sequence", "0") or 0)
        events = self.store.events(cursor, 100)
        if not events:
            return
        reply = self._request("POST", "/robot-api/v1/events/batch", {"robotId": self.robot_id, "events": events})
        accepted = int(reply.get("acceptedThroughSequence", cursor))
        self.store.set_cloud_state("last_uploaded_sequence", str(max(0, accepted)))

    def _heartbeat_payload(self) -> Dict[str, Any]:
        snapshot = self.bridge.cloud_status_snapshot()
        context = snapshot.get("platformContext", {})
        return {
            "protocolVersion": "1.0", "robotId": self.robot_id, "bootId": self.boot_id,
            "softwareVersion": self.software_version, "state": snapshot.get("state", "idle"),
            "activeExecutionId": context.get("active_execution_id"), "activeDeploymentId": context.get("active_deployment_id"),
            "lastReceivedCommandId": self.store.cloud_state("last_received_command_id", "") or None,
            "latestLocalEventSequence": self.store.latest_event_sequence(),
            "mapPose": snapshot.get("mapPose"), "odomPose": snapshot.get("odomPose"),
            "health": snapshot.get("health", {}),
        }

    def _download_deployment(self, deployment_id: str) -> Dict[str, Any]:
        manifest = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/manifest")
        if manifest.get("robotId") != self.robot_id or manifest.get("deploymentId", deployment_id) != deployment_id:
            raise PlatformStoreError("INVALID_REQUEST", "downloaded deployment identity mismatch")
        route = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/route", binary=True)
        yaml_bytes = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/yaml", binary=True)
        pgm = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/pgm", binary=True)
        return self.store.install(deployment_id, manifest, route, yaml_bytes, pgm)

    def _enqueue(self, record: Dict[str, Any]) -> None:
        command = record["payload"]
        if record["command_type"] == "START":
            deployment = self.store.deployment(record["deployment_id"])
            if not deployment:
                deployment = self._download_deployment(record["deployment_id"])
            command = {**command, "routePath": deployment["routePath"], "mapYamlPath": deployment["mapYamlPath"]}
        self.bridge.enqueue_cloud_command(command)
        self.store.set_command_state(record["command_id"], "DISPATCHED")

    def _handle_command(self, command: Dict[str, Any]) -> None:
        try:
            record = self.store.receive_cloud_command(command)
            if record["state"] == "APPLIED":
                return
            if record["command_type"] == "START" and self.bridge.cloud_status_snapshot().get("state") in {"starting", "running", "paused", "manual_takeover", "returning_home", "waiting_loop"}:
                self.store.set_command_state(record["command_id"], "REJECTED", {"error": "ROBOT_BUSY"})
                raise PlatformStoreError("ROBOT_BUSY", "robot is busy", 409)
            if record["state"] == "RECEIVED":
                self._request("POST", f"/robot-api/v1/commands/{record['command_id']}/ack", {"robotId": self.robot_id, "leaseToken": command.get("leaseToken", ""), "status": "RECEIVED", "executionId": record["execution_id"]})
                self.store.set_command_state(record["command_id"], "ACKED")
                record = self.store.command(record["command_id"]) or record
            self._enqueue(record)
            self.store.set_cloud_state("last_received_command_id", record["command_id"])
        except (PlatformStoreError, CloudRequestError, ValueError) as exc:
            command_id = str(command.get("commandId") or "")
            if command_id:
                try:
                    self._request("POST", f"/robot-api/v1/commands/{command_id}/ack", {"robotId": self.robot_id, "leaseToken": command.get("leaseToken", ""), "status": "REJECTED", "executionId": command.get("executionId", ""), "errorCode": getattr(exc, "code", "COMMAND_REJECTED"), "errorMessage": str(exc)})
                except CloudRequestError:
                    pass
                if self.store.command(command_id):
                    self.store.set_command_state(command_id, "REJECTED", {"error": str(exc)})
            raise

    def run_once(self) -> float:
        self._upload_events()
        reply = self._request("POST", "/robot-api/v1/heartbeat", self._heartbeat_payload())
        accepted = int(reply.get("acceptedEventSequence", self.store.cloud_state("last_uploaded_sequence", "0") or 0))
        self.store.set_cloud_state("last_uploaded_sequence", str(max(0, accepted)))
        command = reply.get("command")
        if command:
            self._handle_command(command)
        return max(0.1, float(reply.get("nextHeartbeatSec", self.active_heartbeat if command else self.idle_heartbeat)))

    def _run(self) -> None:
        for record in self.store.pending_cloud_commands():
            if record["state"] in {"ACKED", "DISPATCHED"}:
                try:
                    self._enqueue(record)
                except (PlatformStoreError, CloudRequestError, ValueError):
                    pass
        failure = 0
        delay = 0.0
        while not self._stop.wait(delay):
            try:
                delay = self.run_once()
                failure = 0
            except CloudRequestError as exc:
                base = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                delay = exc.retry_after if exc.retry_after is not None else base * random.uniform(1.0, 1.2)
                failure += 1
                LOG.warning("cloud heartbeat failed: %s", exc)
            except (PlatformStoreError, ValueError) as exc:
                delay = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                failure += 1
                LOG.warning("cloud command rejected: %s", exc)
