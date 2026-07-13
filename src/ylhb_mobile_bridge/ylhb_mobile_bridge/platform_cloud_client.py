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
from datetime import datetime, timezone
from typing import Any, Dict

from .platform_store import PlatformStoreError


LOG = logging.getLogger(__name__)
BACKOFF_SECONDS = (1, 2, 4, 8, 15, 30)
TRUE_VALUES = {"1", "true", "yes", "on"}
ACTIVE_STATES = {"starting", "running", "paused", "manual_takeover", "returning_home", "waiting_loop"}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_server_url(value: str) -> str:
    try:
        parsed = urllib.parse.urlsplit(value)
        port = f":{parsed.port}" if parsed.port else ""
    except ValueError:
        return ""
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        return ""
    host = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
    return urllib.parse.urlunsplit((parsed.scheme, host + port, parsed.path.rstrip("/"), "", ""))


class CloudRequestError(RuntimeError):
    def __init__(self, message: str, retry_after: float | None = None):
        super().__init__(message)
        self.retry_after = retry_after


class PlatformCloudClient:
    def __init__(self, store, bridge, robot_id: str, boot_id: str):
        self.store, self.bridge, self.robot_id, self.boot_id = store, bridge, robot_id, boot_id
        self.base_url = os.environ.get("YLHB_CLOUD_BASE_URL", "").rstrip("/")
        self.token = os.environ.get("YLHB_CLOUD_ROBOT_TOKEN", "")
        self.configured = _safe_server_url(self.base_url).startswith("https://") and bool(self.token)
        default_enabled = os.environ.get("YLHB_CLOUD_ENABLED", "false").strip().lower() in TRUE_VALUES
        override = self.store.cloud_state("cloud_enabled_override", "").strip().lower()
        self.desired_enabled = self.configured and ((override == "true") if override in {"true", "false"} else default_enabled)
        self.timeout = float(os.environ.get("YLHB_CLOUD_REQUEST_TIMEOUT_SEC", "10"))
        self.idle_heartbeat = float(os.environ.get("YLHB_CLOUD_IDLE_HEARTBEAT_SEC", "3"))
        self.active_heartbeat = float(os.environ.get("YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC", "1"))
        self.max_backoff = float(os.environ.get("YLHB_CLOUD_MAX_BACKOFF_SEC", "30"))
        self.software_version = os.environ.get("YLHB_SOFTWARE_VERSION", "unknown")
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        config_error = ""
        try:
            self._context = ssl.create_default_context(cafile=os.environ.get("YLHB_CLOUD_CA_FILE") or None)
        except OSError as exc:
            self._context = ssl.create_default_context()
            self.configured = False
            self.desired_enabled = False
            config_error = f"invalid CA configuration: {type(exc).__name__}"
        self._status = {
            "connected": False,
            "state": "UNCONFIGURED" if not self.configured else ("CONNECTING" if self.desired_enabled else "DISABLED"),
            "lastAttemptAt": "", "lastSuccessAt": "", "lastError": config_error, "nextRetrySec": 0.0,
            "lastServerTime": "",
        }

    def start(self) -> None:
        if not self._thread:
            self._thread = threading.Thread(target=self._run, name="platform-cloud-client", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=self.timeout + 1)

    def set_enabled(self, enabled: bool) -> Dict[str, Any]:
        self.store.set_cloud_state("cloud_enabled_override", "true" if enabled else "false")
        with self._lock:
            self.desired_enabled = bool(enabled) and self.configured
            self._status.update({
                "connected": False,
                "state": "UNCONFIGURED" if not self.configured else ("CONNECTING" if enabled else "DISABLED"),
                "nextRetrySec": 0.0,
            })
        self._wake.set()
        return self.status()

    def status(self) -> Dict[str, Any]:
        snapshot = self.bridge.cloud_status_snapshot()
        context = snapshot.get("platformContext", {})
        with self._lock:
            status = dict(self._status)
            desired = self.desired_enabled
        return {
            "configured": self.configured,
            "desiredEnabled": desired,
            **status,
            "serverBaseUrl": _safe_server_url(self.base_url),
            "pendingEventCount": self.store.pending_event_count(),
            "pendingCommandCount": self.store.pending_command_count(),
            "lastReceivedCommandId": self.store.cloud_state("last_received_command_id", ""),
            "lastUploadedSequence": int(self.store.cloud_state("last_uploaded_sequence", "0") or 0),
            "latestLocalEventSequence": self.store.latest_event_sequence(),
            "activeExecutionId": context.get("active_execution_id") or "",
            "activeDeploymentId": context.get("active_deployment_id") or "",
        }

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
            raise CloudRequestError(f"request failed: {type(exc).__name__}") from exc

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

    def _record_failure(self, record: Dict[str, Any], state: str, event: str, exc: Exception) -> None:
        code = str(getattr(exc, "code", "COMMAND_FAILED" if state == "FAILED" else "COMMAND_REJECTED"))
        result = {
            "event": event,
            "command_id": record["command_id"], "request_id": record["request_id"],
            "execution_id": record["execution_id"], "deployment_id": record["deployment_id"],
            "error_code": code, "error_message": str(exc),
        }
        saved = self.store.append_event(result)
        self.store.set_command_state(record["command_id"], state, saved)

    def _handle_command(self, command: Dict[str, Any]) -> None:
        record = self.store.receive_cloud_command(command)
        if record["state"] in {"APPLIED", "REJECTED", "FAILED", "DISPATCHED"}:
            return
        if record["state"] == "RECEIVED":
            self._request("POST", f"/robot-api/v1/commands/{record['command_id']}/ack", {
                "robotId": self.robot_id, "leaseToken": command.get("leaseToken", ""),
                "status": "RECEIVED", "executionId": record["execution_id"],
            })
            self.store.set_command_state(record["command_id"], "ACKED")
            record = self.store.command(record["command_id"]) or record
        try:
            if record["command_type"] == "START" and self.bridge.cloud_status_snapshot().get("state") in ACTIVE_STATES:
                raise PlatformStoreError("ROBOT_BUSY", "robot is busy", 409)
            self._enqueue(record)
            self.store.set_cloud_state("last_received_command_id", record["command_id"])
        except PlatformStoreError as exc:
            deployment_error = record["command_type"] == "START" and not self.store.deployment(record["deployment_id"])
            self._record_failure(record, "FAILED" if deployment_error else "REJECTED", "command_failed" if deployment_error else "command_rejected", exc)
            raise
        except Exception as exc:
            self._record_failure(record, "FAILED", "command_failed", exc)
            raise

    def run_once(self) -> float:
        reply = self._request("POST", "/robot-api/v1/heartbeat", self._heartbeat_payload())
        accepted = int(reply.get("acceptedEventSequence", self.store.cloud_state("last_uploaded_sequence", "0") or 0))
        self.store.set_cloud_state("last_uploaded_sequence", str(max(0, accepted)))
        self._upload_events()
        command = reply.get("command")
        if command:
            self._handle_command(command)
        with self._lock:
            self._status["lastServerTime"] = str(reply.get("serverTime") or "")
        return max(0.1, float(reply.get("nextHeartbeatSec", self.active_heartbeat if command else self.idle_heartbeat)))

    def _recover_pending(self) -> bool:
        snapshot = self.bridge.cloud_status_snapshot()
        if not snapshot:
            return False
        patrol_state = str(snapshot.get("state") or "unknown")
        active_execution = str((snapshot.get("platformContext") or {}).get("active_execution_id") or "")
        for record in self.store.pending_cloud_commands():
            if record["state"] == "ACKED":
                try:
                    self._enqueue(record)
                except Exception as exc:
                    self._record_failure(record, "FAILED", "command_failed", exc)
                continue
            if record["state"] != "DISPATCHED":
                continue
            if record["command_type"] == "START":
                if active_execution == record["execution_id"] and patrol_state not in {"idle", "unknown", "unavailable"}:
                    self._record_recovered_result(record, "APPLIED", "route_started")
                elif patrol_state in {"idle", "unknown", "unavailable"}:
                    self._record_failure(record, "FAILED", "command_failed", PlatformStoreError("RECOVERY_NO_EXECUTION_EVIDENCE", "dispatched START has no active execution evidence"))
                continue
            targets = {
                "PAUSE": {"paused"}, "RESUME": {"running", "returning_home", "waiting_loop"},
                "TAKEOVER": {"manual_takeover"}, "CANCEL": {"canceled", "cancelled"},
            }
            compatible = {
                "PAUSE": {"running", "returning_home", "waiting_loop"},
                "RESUME": {"paused", "manual_takeover"},
                "TAKEOVER": {"running", "paused", "returning_home", "waiting_loop"},
                "CANCEL": {"running", "paused", "manual_takeover", "returning_home", "waiting_loop", "starting"},
            }
            events = {"PAUSE": "route_paused", "RESUME": "route_resumed", "TAKEOVER": "manual_takeover", "CANCEL": "route_canceled"}
            if active_execution == record["execution_id"] and patrol_state in targets[record["command_type"]]:
                self._record_recovered_result(record, "APPLIED", events[record["command_type"]])
            elif active_execution == record["execution_id"] and patrol_state in compatible[record["command_type"]]:
                try:
                    self._enqueue(record)
                except Exception as exc:
                    self._record_failure(record, "FAILED", "command_failed", exc)
            else:
                self._record_failure(record, "REJECTED", "command_rejected", PlatformStoreError("RECOVERY_INCOMPATIBLE_STATE", f"cannot recover {record['command_type']} while {patrol_state}"))
        return True

    def _record_recovered_result(self, record: Dict[str, Any], state: str, event: str) -> None:
        result = {
            "event": event, "recovered": True,
            "command_id": record["command_id"], "request_id": record["request_id"],
            "execution_id": record["execution_id"], "deployment_id": record["deployment_id"],
        }
        saved = self.store.append_event(result)
        self.store.set_command_state(record["command_id"], state, saved)

    def _run(self) -> None:
        recovered = False
        failure = 0
        delay = 0.0
        while not self._stop.is_set():
            with self._lock:
                enabled = self.desired_enabled
            if not enabled:
                self._wake.wait()
                self._wake.clear()
                continue
            if not recovered:
                recovered = self._recover_pending()
            if self._wake.wait(delay):
                self._wake.clear()
            if self._stop.is_set():
                break
            with self._lock:
                if not self.desired_enabled:
                    continue
            with self._lock:
                self._status.update({"state": "CONNECTING", "lastAttemptAt": _now(), "nextRetrySec": 0.0})
            try:
                delay = self.run_once()
                failure = 0
                with self._lock:
                    self._status.update({"connected": True, "state": "CONNECTED", "lastSuccessAt": _now(), "lastError": "", "nextRetrySec": delay})
            except CloudRequestError as exc:
                base = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                delay = exc.retry_after if exc.retry_after is not None else base * random.uniform(1.0, 1.2)
                failure += 1
                with self._lock:
                    self._status.update({"connected": False, "state": "BACKOFF", "lastError": str(exc), "nextRetrySec": delay})
                LOG.warning("cloud heartbeat failed: %s", exc)
            except (PlatformStoreError, ValueError) as exc:
                delay = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                failure += 1
                with self._lock:
                    self._status.update({"connected": True, "state": "CONNECTED", "lastError": str(exc), "nextRetrySec": delay})
                LOG.warning("cloud command handling failed: %s", exc)
            except Exception as exc:
                delay = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                failure += 1
                message = f"local cloud client error: {type(exc).__name__}"
                with self._lock:
                    self._status.update({"connected": False, "state": "BACKOFF", "lastError": message, "nextRetrySec": delay})
                LOG.warning("%s", message)
