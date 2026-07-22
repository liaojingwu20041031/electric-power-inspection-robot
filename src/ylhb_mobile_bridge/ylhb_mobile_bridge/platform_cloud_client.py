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

from .platform_store import (
    COMMAND_STATE_ACKED,
    COMMAND_STATE_ARMED,
    COMMAND_STATE_CONFIRMED,
    START_MODE_LOCAL_CONFIRM,
    START_MODE_REMOTE_IMMEDIATE,
    PlatformStoreError,
)


LOG = logging.getLogger(__name__)
BACKOFF_SECONDS = (1, 2, 4, 8, 15, 30)
TRUE_VALUES = {"1", "true", "yes", "on"}
ACTIVE_STATES = {"starting", "running", "paused", "manual_takeover", "returning_home", "waiting_loop"}
LOCAL_CONFIRM_PROTOCOL_VERSION = "1"
HEARTBEAT_STATE_MAP = {
    "waiting_schedule": "idle", "canceling": "running",
    "unavailable": "idle", "unknown": "idle",
}


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
    def __init__(
        self, message: str, retry_after: float | None = None,
        status_code: int = 0, code: str = "",
    ):
        super().__init__(message)
        self.retry_after = retry_after
        self.status_code = status_code
        self.code = code


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
        self.local_confirm_timeout = max(
            1.0,
            float(os.environ.get("YLHB_CLOUD_LOCAL_CONFIRM_TIMEOUT_SEC", "1800")),
        )
        self.software_version = os.environ.get("YLHB_SOFTWARE_VERSION", "unknown")
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self.network_status = getattr(bridge, 'network_status', None)
        self._last_successful_egress: Dict[str, Any] = {}
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
            "nextHeartbeatSec": 0.0, "heartbeatInFlight": False, "consecutiveFailures": 0,
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
                "nextRetrySec": 0.0, "nextHeartbeatSec": 0.0, "heartbeatInFlight": False,
            })
        self._wake.set()
        return self.status()

    def status(self) -> Dict[str, Any]:
        snapshot = self.bridge.cloud_status_snapshot()
        context = snapshot.get("platformContext", {})
        with self._lock:
            status = dict(self._status)
            desired = self.desired_enabled
            last_successful_egress = dict(self._last_successful_egress)
        route = self._route_diagnostics()
        cloud_egress = {
            key: route.get(key)
            for key in (
                'interface',
                'type',
                'label',
                'sourceAddress',
                'gateway',
                'metric',
            )
            if route.get(key) not in (None, '')
        }
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
            "pendingPlatformStart": self.store.pending_platform_start(),
            "networkMode": "system-routing",
            "cloudEgress": cloud_egress,
            "alternateCloudRoutes": list(
                route.get("alternateCloudRoutes") or []
            ),
            "failoverAvailable": bool(route.get("failoverAvailable")),
            "lastSuccessfulEgress": last_successful_egress,
        }

    def _cloud_hostname(self) -> str:
        try:
            return urllib.parse.urlsplit(self.base_url).hostname or ''
        except ValueError:
            return ''

    def _route_diagnostics(self) -> Dict[str, Any]:
        hostname = self._cloud_hostname()
        if not hostname or self.network_status is None:
            return {}
        try:
            route = self.network_status.route_to_host(hostname)
            return dict(route) if isinstance(route, dict) else {}
        except Exception:
            return {}

    def _record_successful_egress(self) -> None:
        route = self._route_diagnostics()
        if not route:
            return
        successful = {
            key: route.get(key)
            for key in (
                'interface',
                'type',
                'label',
                'sourceAddress',
                'gateway',
                'metric',
            )
            if route.get(key) not in (None, '')
        }
        with self._lock:
            self._last_successful_egress = successful

    def _request(self, method: str, path: str, body: Dict[str, Any] | None = None, binary: bool = False):
        data = None if body is None else json.dumps(body, ensure_ascii=False).encode("utf-8")
        headers = {"Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(self.base_url + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout, context=self._context) as response:
                content = response.read()
                result = content if binary else json.loads(content.decode("utf-8"))
                self._record_successful_egress()
                return result
        except urllib.error.HTTPError as exc:
            retry_after = exc.headers.get("Retry-After", "")
            try:
                retry_after_value = min(self.max_backoff, max(0.0, float(retry_after)))
            except ValueError:
                retry_after_value = None
            detail = ""
            error_code = ""
            try:
                error = json.loads(exc.read().decode("utf-8"))
                if isinstance(error, dict):
                    error_code = str(error.get("code") or "")
                    detail = ": ".join(str(error.get(key) or "") for key in ("code", "message")).strip(": ")
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            raise CloudRequestError(
                f"HTTP {exc.code}" + (f": {detail}" if detail else ""),
                retry_after_value,
                exc.code,
                error_code,
            ) from exc
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise CloudRequestError(f"request failed: {type(exc).__name__}") from exc

    def _upload_events(self) -> None:
        cursor = int(self.store.cloud_state("last_uploaded_sequence", "0") or 0)
        blocked = int(self.store.cloud_state("blocked_event_sequence", "0") or 0)
        if blocked > cursor:
            return
        if blocked:
            self.store.set_cloud_state("blocked_event_sequence", "0")
            self.store.set_cloud_state("blocked_event_error", "")
        # ponytail: one-at-a-time makes a 409 sequence-specific; batch again if event volume grows.
        events = self.store.events(cursor, 1)
        if not events:
            return
        required_identity = (
            "robot_id", "execution_id", "deployment_id", "request_id", "command_id",
        )
        invalid = next((event for event in events if any(
            not str(event.get(key) or "").strip() for key in required_identity
        )), None)
        if invalid:
            LOG.error(
                "refusing platform event upload with incomplete identity: sequence=%s event=%s",
                invalid.get("sequence"), invalid.get("event"),
            )
            return
        try:
            reply = self._request("POST", "/robot-api/v1/events/batch", {"robotId": self.robot_id, "events": events})
        except CloudRequestError as exc:
            if exc.status_code == 409 and exc.code.startswith("EVENT_"):
                sequence = int(events[0]["sequence"])
                self.store.set_cloud_state("blocked_event_sequence", str(sequence))
                self.store.set_cloud_state("blocked_event_error", f"{exc.code}: {exc}")
                LOG.error("isolated conflicting cloud event: sequence=%s error=%s", sequence, exc)
            else:
                LOG.warning("cloud event upload failed: %s", exc)
            return
        accepted = int(reply.get("acceptedThroughSequence", cursor))
        self.store.set_cloud_state("last_uploaded_sequence", str(max(0, accepted)))

    def _heartbeat_payload(self) -> Dict[str, Any]:
        snapshot = self.bridge.cloud_status_snapshot()
        context = snapshot.get("platformContext", {})
        readiness = self._local_confirm_readiness()
        return {
            "protocolVersion": "1.0", "robotId": self.robot_id, "bootId": self.boot_id,
            "softwareVersion": self.software_version,
            "state": HEARTBEAT_STATE_MAP.get(
                str(snapshot.get("state") or "idle"),
                str(snapshot.get("state") or "idle"),
            ),
            "activeExecutionId": context.get("active_execution_id"), "activeDeploymentId": context.get("active_deployment_id"),
            "lastReceivedCommandId": self.store.cloud_state("last_received_command_id", "") or None,
            "latestLocalEventSequence": self.store.latest_event_sequence(),
            "mapPose": snapshot.get("mapPose"), "odomPose": snapshot.get("odomPose"),
            "gnssFix": snapshot.get("gnssFix"),
            "capabilities": {
                "remoteImmediateStart": True,
                "localConfirmStart": True,
                "localConfirmProtocolVersion": LOCAL_CONFIRM_PROTOCOL_VERSION,
            },
            "health": {
                **snapshot.get("health", {}),
                "localConfirmStartReady": readiness["ready"],
                "localConfirmStartError": readiness["error"],
            },
        }

    def _local_confirm_readiness(self) -> Dict[str, Any]:
        provider = getattr(self.bridge, "local_confirm_start_readiness", None)
        if not callable(provider):
            return {"ready": False, "error": "LOCAL_CONFIRM_RUNTIME_UNAVAILABLE"}
        try:
            status = provider()
        except Exception:
            return {"ready": False, "error": "LOCAL_CONFIRM_RUNTIME_UNAVAILABLE"}
        if not isinstance(status, dict):
            return {"ready": False, "error": "LOCAL_CONFIRM_RUNTIME_UNAVAILABLE"}
        ready = bool(status.get("ready"))
        error = None if ready else str(
            status.get("error") or "LOCAL_CONFIRM_NOT_READY"
        )
        return {"ready": ready, "error": error}

    def _download_deployment(self, deployment_id: str) -> Dict[str, Any]:
        manifest = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/manifest")
        if manifest.get("robotId") != self.robot_id or manifest.get("deploymentId", deployment_id) != deployment_id:
            raise PlatformStoreError("INVALID_REQUEST", "downloaded deployment identity mismatch")
        route = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/route", binary=True)
        yaml_bytes = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/yaml", binary=True)
        pgm = self._request("GET", f"/robot-api/v1/deployments/{deployment_id}/pgm", binary=True)
        return self.store.install(deployment_id, manifest, route, yaml_bytes, pgm)

    def _prepared_command(self, record: Dict[str, Any]) -> Dict[str, Any]:
        command = record["payload"]
        if record["command_type"] == "START":
            deployment = self.store.deployment(record["deployment_id"])
            if not deployment:
                deployment = self._download_deployment(record["deployment_id"])
            command = {**command, "routePath": deployment["routePath"], "mapYamlPath": deployment["mapYamlPath"]}
        return command

    def _enqueue(self, record: Dict[str, Any]) -> None:
        command = self._prepared_command(record)
        self.bridge.enqueue_cloud_command(command)

    def _record_failure(self, record: Dict[str, Any], state: str, event: str, exc: Exception) -> None:
        code = str(getattr(exc, "code", "COMMAND_FAILED" if state == "FAILED" else "COMMAND_REJECTED"))
        result = {
            "schema_version": "1.0",
            "event": event,
            "robot_id": self.robot_id, "boot_id": self.boot_id,
            "command_id": record["command_id"], "request_id": record["request_id"],
            "execution_id": record["execution_id"], "deployment_id": record["deployment_id"],
            "reason": code, "error_code": code, "error_message": str(exc),
        }
        saved = self.store.append_event(result)
        self.store.set_command_state(record["command_id"], state, saved)

    def _handle_command(self, command: Dict[str, Any]) -> None:
        record = self.store.receive_cloud_command(command)
        self.store.set_cloud_state("last_received_command_id", record["command_id"])
        if record["state"] in {
            "APPLIED", "REJECTED", "FAILED", "DISPATCHED",
            COMMAND_STATE_ARMED, COMMAND_STATE_CONFIRMED,
        }:
            return
        if record["state"] == "RECEIVED":
            self._request("POST", f"/robot-api/v1/commands/{record['command_id']}/ack", {
                "robotId": self.robot_id, "leaseToken": command.get("leaseToken", ""),
                "status": "RECEIVED", "executionId": record["execution_id"],
            })
            self.store.set_command_state(record["command_id"], COMMAND_STATE_ACKED)
            record = self.store.command(record["command_id"]) or record
        try:
            if record["command_type"] == "START" and self.bridge.cloud_status_snapshot().get("state") in ACTIVE_STATES:
                raise PlatformStoreError("ROBOT_BUSY", "robot is busy", 409)
            if record["command_type"] == "CANCEL":
                canceled = self.store.cancel_armed_start(
                    record, self.robot_id, self.boot_id
                )
                if canceled:
                    return
            if record["command_type"] == "START" and str(
                record["payload"].get("startMode") or START_MODE_REMOTE_IMMEDIATE
            ) == START_MODE_LOCAL_CONFIRM:
                readiness = self._local_confirm_readiness()
                if not readiness["ready"]:
                    raise PlatformStoreError(
                        "LOCAL_CONFIRM_NOT_READY",
                        str(readiness["error"] or "local confirmation is not ready"),
                        409,
                    )
                self._prepared_command(record)
                self.store.arm_start(record["command_id"], self.robot_id, self.boot_id)
                return
            self._enqueue(record)
        except PlatformStoreError as exc:
            deployment_error = (
                exc.code != "LOCAL_CONFIRM_NOT_READY"
                and record["command_type"] == "START"
                and not self.store.deployment(record["deployment_id"])
            )
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
                    if (
                        record["command_type"] == "START"
                        and record["payload"].get("startMode") == START_MODE_LOCAL_CONFIRM
                    ):
                        readiness = self._local_confirm_readiness()
                        if not readiness["ready"]:
                            raise PlatformStoreError(
                                "LOCAL_CONFIRM_NOT_READY",
                                str(readiness["error"]), 409,
                            )
                        self._prepared_command(record)
                        self.store.arm_start(
                            record["command_id"], self.robot_id, self.boot_id
                        )
                    else:
                        self._enqueue(record)
                except Exception as exc:
                    rejected = getattr(exc, "code", "") == "LOCAL_CONFIRM_NOT_READY"
                    self._record_failure(
                        record,
                        "REJECTED" if rejected else "FAILED",
                        "command_rejected" if rejected else "command_failed",
                        exc,
                    )
                continue
            if record["state"] == COMMAND_STATE_CONFIRMED:
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

    def confirm_local_start(self) -> Dict[str, Any]:
        record = self.store.confirm_armed_start(self.robot_id, self.boot_id)
        self._enqueue(record)
        return record

    def expire_local_confirmations(self) -> int:
        return len(self.store.expire_armed_starts(
            self.local_confirm_timeout, self.robot_id, self.boot_id
        ))

    def _record_recovered_result(self, record: Dict[str, Any], state: str, event: str) -> None:
        result = {
            "schema_version": "1.0", "event": event, "recovered": True,
            "robot_id": self.robot_id, "boot_id": self.boot_id,
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
                with self._lock:
                    self._status.update({"connected": False, "state": "UNCONFIGURED" if not self.configured else "DISABLED", "heartbeatInFlight": False, "nextHeartbeatSec": 0.0, "nextRetrySec": 0.0})
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
                # Keep a proven connection visually stable while the next heartbeat is sent.
                self._status.update({"state": "CONNECTING" if not self._status["connected"] else "CONNECTED", "lastAttemptAt": _now(), "nextRetrySec": 0.0, "heartbeatInFlight": True})
            try:
                delay = self.run_once()
                failure = 0
                with self._lock:
                    self._status.update({"connected": True, "state": "CONNECTED", "heartbeatInFlight": False, "lastSuccessAt": _now(), "lastError": "", "nextRetrySec": 0.0, "nextHeartbeatSec": delay, "consecutiveFailures": 0})
            except CloudRequestError as exc:
                base = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                delay = exc.retry_after if exc.retry_after is not None else base * random.uniform(1.0, 1.2)
                failure += 1
                with self._lock:
                    self._status.update({"connected": False, "state": "BACKOFF", "heartbeatInFlight": False, "lastError": str(exc), "nextRetrySec": delay, "nextHeartbeatSec": 0.0, "consecutiveFailures": failure})
                LOG.warning("cloud heartbeat failed: %s", exc)
            except (PlatformStoreError, ValueError) as exc:
                delay = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                failure += 1
                with self._lock:
                    self._status.update({"connected": True, "state": "CONNECTED", "heartbeatInFlight": False, "lastError": str(exc), "nextRetrySec": 0.0, "nextHeartbeatSec": delay, "consecutiveFailures": failure})
                LOG.warning("cloud command handling failed: %s", exc)
            except Exception as exc:
                delay = min(self.max_backoff, BACKOFF_SECONDS[min(failure, len(BACKOFF_SECONDS) - 1)])
                failure += 1
                message = f"local cloud client error: {type(exc).__name__}"
                with self._lock:
                    self._status.update({"connected": False, "state": "BACKOFF", "heartbeatInFlight": False, "lastError": message, "nextRetrySec": delay, "nextHeartbeatSec": 0.0, "consecutiveFailures": failure})
                LOG.warning("%s", message)
