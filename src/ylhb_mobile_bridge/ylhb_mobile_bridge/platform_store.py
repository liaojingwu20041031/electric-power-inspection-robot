"""Small durable store for Robot Platform Protocol v1."""
import hashlib
import json
import os
import shutil
import sqlite3
import tempfile
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List

import yaml

from .patrol_route_store import (
    ROUTE_NUMBER_PATTERN,
    validate_route_file,
    validate_route_map_binding,
)


DELIVERY_FIELDS = {
    "leaseToken", "leaseUntil", "serverTime", "attemptCount",
    "deliveryAttempt", "deliveredAt", "nextHeartbeatSec",
}
START_MODE_REMOTE_IMMEDIATE = "REMOTE_IMMEDIATE"
START_MODE_LOCAL_CONFIRM = "LOCAL_CONFIRM"
START_MODES = {START_MODE_REMOTE_IMMEDIATE, START_MODE_LOCAL_CONFIRM}

COMMAND_STATE_RECEIVED = "RECEIVED"
COMMAND_STATE_ACKED = "ACKED"
COMMAND_STATE_ARMED = "ARMED"
COMMAND_STATE_CONFIRMED = "CONFIRMED"
COMMAND_STATE_DISPATCHED = "DISPATCHED"
COMMAND_STATE_APPLIED = "APPLIED"
COMMAND_STATE_REJECTED = "REJECTED"
COMMAND_STATE_FAILED = "FAILED"
COMMAND_TRANSITIONS = {
    COMMAND_STATE_RECEIVED: {COMMAND_STATE_ACKED},
    COMMAND_STATE_ACKED: {
        COMMAND_STATE_ARMED, COMMAND_STATE_DISPATCHED,
        COMMAND_STATE_REJECTED, COMMAND_STATE_FAILED,
    },
    COMMAND_STATE_ARMED: {
        COMMAND_STATE_CONFIRMED, COMMAND_STATE_REJECTED, COMMAND_STATE_FAILED,
    },
    COMMAND_STATE_CONFIRMED: {
        COMMAND_STATE_DISPATCHED, COMMAND_STATE_REJECTED, COMMAND_STATE_FAILED,
    },
    COMMAND_STATE_DISPATCHED: {
        COMMAND_STATE_APPLIED, COMMAND_STATE_REJECTED, COMMAND_STATE_FAILED,
    },
    COMMAND_STATE_APPLIED: set(),
    COMMAND_STATE_REJECTED: set(),
    COMMAND_STATE_FAILED: set(),
}


class PlatformStoreError(ValueError):
    def __init__(self, code: str, message: str, status_code: int = 400):
        super().__init__(message)
        self.code = code
        self.status_code = status_code


def canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def normalize_command_business_payload(command: Dict[str, Any]) -> Dict[str, Any]:
    return {key: value for key, value in command.items() if key not in DELIVERY_FIELDS}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_name(name: str, suffix: str) -> str:
    path = Path(name)
    allowed_suffixes = (".yaml", ".yml") if suffix == ".yaml" else (suffix,)
    if not name or path.name != name or path.is_absolute() or ".." in path.parts or not name.lower().endswith(allowed_suffixes):
        raise PlatformStoreError("INVALID_REQUEST", f"invalid {suffix} filename")
    return name


class DeploymentStore:
    def __init__(self, root: Path, default_map_path: Path | str | None = None):
        self.root = Path(root).expanduser()
        self.default_map_path = Path(default_map_path).expanduser() if default_map_path else None
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "staging").mkdir(exist_ok=True)
        (self.root / "deployments").mkdir(exist_ok=True)
        self.db_path = self.root / "platform.db"
        with self._connection() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS deployments (
                  deployment_id TEXT PRIMARY KEY, manifest_json TEXT NOT NULL,
                  route_sha256 TEXT NOT NULL, map_sha256 TEXT NOT NULL, installed_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS executions (
                  execution_id TEXT PRIMARY KEY, deployment_id TEXT NOT NULL,
                  request_id TEXT NOT NULL, state TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS events (
                  sequence INTEGER PRIMARY KEY AUTOINCREMENT, event_json TEXT NOT NULL, occurred_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS processed_commands (
                  command_id TEXT PRIMARY KEY, request_id TEXT NOT NULL UNIQUE,
                  execution_id TEXT NOT NULL, deployment_id TEXT NOT NULL,
                  command_type TEXT NOT NULL, payload_json TEXT NOT NULL,
                  state TEXT NOT NULL, result_json TEXT NOT NULL DEFAULT '{}',
                  created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS cloud_state (
                  state_key TEXT PRIMARY KEY, state_value TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS bridge_settings (
                  setting_key TEXT PRIMARY KEY, setting_value TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS map_uploads (
                  task_id TEXT PRIMARY KEY,
                  idempotency_key TEXT NOT NULL UNIQUE,
                  content_identity_sha256 TEXT NOT NULL UNIQUE,
                  identity_json TEXT NOT NULL,
                  yaml_sha256 TEXT NOT NULL,
                  pgm_sha256 TEXT NOT NULL,
                  yaml_path TEXT NOT NULL,
                  pgm_path TEXT NOT NULL,
                  status TEXT NOT NULL CHECK (
                    status IN ('PENDING','FAILED_RETRYABLE','FAILED_FINAL','SUCCEEDED')
                  ),
                  retry_count INTEGER NOT NULL DEFAULT 0,
                  next_retry_at REAL NOT NULL DEFAULT 0,
                  map_asset_id TEXT NOT NULL DEFAULT '',
                  last_error TEXT NOT NULL DEFAULT '',
                  created_at REAL NOT NULL,
                  updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS inspection_image_uploads (
                  capture_task_id TEXT PRIMARY KEY,
                  capture_identity TEXT NOT NULL UNIQUE,
                  idempotency_key TEXT NOT NULL UNIQUE,
                  task_id TEXT NOT NULL,
                  execution_id TEXT NOT NULL,
                  checkpoint_id TEXT NOT NULL,
                  capture_kind TEXT NOT NULL CHECK (capture_kind IN ('MOVING','ARRIVAL')),
                  captured_at TEXT NOT NULL,
                  image_sha256 TEXT NOT NULL,
                  file_path TEXT NOT NULL,
                  file_size INTEGER NOT NULL,
                  status TEXT NOT NULL CHECK (status IN (
                    'PENDING','FAILED_RETRYABLE','FAILED_FINAL',
                    'CREDENTIAL_BLOCKED','SUCCEEDED'
                  )),
                  retry_count INTEGER NOT NULL DEFAULT 0,
                  next_retry_at REAL NOT NULL DEFAULT 0,
                  image_id TEXT NOT NULL DEFAULT '',
                  last_error TEXT NOT NULL DEFAULT '',
                  created_at REAL NOT NULL,
                  updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS scene_uploads (
                  task_id TEXT PRIMARY KEY,
                  idempotency_key TEXT NOT NULL UNIQUE,
                  source_reconstruct_session_id TEXT NOT NULL,
                  source_capture_session_id TEXT NOT NULL DEFAULT '',
                  model_sha256 TEXT NOT NULL,
                  intent_number INTEGER NOT NULL DEFAULT 0,
                  model_path TEXT NOT NULL,
                  metadata_path TEXT NOT NULL,
                  metadata_json TEXT NOT NULL,
                  file_size INTEGER NOT NULL,
                  asset_kind TEXT NOT NULL,
                  format TEXT NOT NULL,
                  status TEXT NOT NULL CHECK (status IN (
                    'PENDING','FAILED_RETRYABLE','FAILED_FINAL',
                    'CREDENTIAL_BLOCKED','SUCCEEDED'
                  )),
                  retry_count INTEGER NOT NULL DEFAULT 0,
                  next_retry_at REAL NOT NULL DEFAULT 0,
                  scene_asset_id TEXT NOT NULL DEFAULT '',
                  last_error TEXT NOT NULL DEFAULT '',
                  supersedes_task_id TEXT NOT NULL DEFAULT '',
                  created_at REAL NOT NULL,
                  updated_at REAL NOT NULL,
                  UNIQUE(source_reconstruct_session_id, model_sha256, intent_number)
                );
                CREATE INDEX IF NOT EXISTS idx_scene_upload_status_retry
                ON scene_uploads(status, next_retry_at);
                CREATE INDEX IF NOT EXISTS idx_scene_upload_session
                ON scene_uploads(source_reconstruct_session_id);
            """)

    def _default_map_yaml_path(self) -> Path | None:
        if self.default_map_path is None:
            return None
        if self.default_map_path.suffix.lower() in {".yaml", ".yml"}:
            return self.default_map_path
        return self.default_map_path.with_suffix(".yaml")

    @staticmethod
    def _next_route_path(directory: Path) -> Path:
        numbers = [
            int(match.group(1))
            for path in directory.glob("route_patrol_*.json")
            if (match := ROUTE_NUMBER_PATTERN.fullmatch(path.name))
        ]
        return directory / f"route_patrol_{max(numbers, default=0) + 1:03d}.json"

    def _connection(self) -> sqlite3.Connection:
        db = sqlite3.connect(self.db_path, timeout=5)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA journal_mode=WAL")
        db.execute("PRAGMA foreign_keys=ON")
        db.execute("PRAGMA busy_timeout=5000")
        return db

    def deployment(self, deployment_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute("SELECT * FROM deployments WHERE deployment_id=?", (deployment_id,)).fetchone()
        if not row:
            return None
        data = dict(row)
        data["manifest"] = json.loads(data.pop("manifest_json"))
        data["directory"] = str(self.root / "deployments" / deployment_id)
        data["routePath"] = str(self.root / "deployments" / deployment_id / "route.json")
        data["mapYamlPath"] = str(self.root / "deployments" / deployment_id / data["manifest"]["yamlName"])
        data["mapPgmPath"] = str(self.root / "deployments" / deployment_id / data["manifest"]["pgmName"])
        return data

    def install(self, deployment_id: str, manifest: Dict[str, Any], route: bytes, yaml_bytes: bytes, pgm: bytes) -> Dict[str, Any]:
        if not deployment_id or "/" in deployment_id or ".." in deployment_id:
            raise PlatformStoreError("INVALID_REQUEST", "invalid deploymentId")
        if len(route) > 5 * 1024 * 1024 or len(yaml_bytes) > 1024 * 1024 or len(pgm) > 100 * 1024 * 1024:
            raise PlatformStoreError("INVALID_REQUEST", "deployment file too large")
        required = ("schemaVersion", "robotId", "routeRevisionId", "routeRevisionContentSha256", "routePayloadSha256", "mapAssetId", "mapImageSha256", "yamlName", "pgmName")
        if not isinstance(manifest, dict) or any(not str(manifest.get(key, "")).strip() for key in required):
            raise PlatformStoreError("INVALID_REQUEST", "manifest is incomplete")
        yaml_name = _safe_name(str(manifest["yamlName"]), ".yaml")
        pgm_name = _safe_name(str(manifest["pgmName"]), ".pgm")
        try:
            route_data = json.loads(route.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            raise PlatformStoreError("INVALID_ROUTE", str(exc)) from exc
        route_hash = sha256(canonical_json(route_data))
        if route_hash != manifest["routePayloadSha256"]:
            raise PlatformStoreError("ROUTE_HASH_MISMATCH", "routePayloadSha256 mismatch")
        if route_hash != manifest["routeRevisionContentSha256"]:
            raise PlatformStoreError("ROUTE_HASH_MISMATCH", "routeRevisionContentSha256 mismatch")
        if manifest.get("routeContentSha256") and manifest["routeContentSha256"] != manifest["routeRevisionContentSha256"]:
            raise PlatformStoreError("ROUTE_HASH_MISMATCH", "route revision hash mismatch")
        try:
            normalized_route = validate_route_file(route_data)
        except ValueError as exc:
            raise PlatformStoreError("INVALID_ROUTE", str(exc)) from exc
        if sha256(pgm) != manifest["mapImageSha256"]:
            raise PlatformStoreError("MAP_HASH_MISMATCH", "mapImageSha256 mismatch")
        try:
            yaml_data = yaml.safe_load(yaml_bytes.decode("utf-8"))
            if not isinstance(yaml_data, dict) or Path(str(yaml_data.get("image", ""))).name != pgm_name:
                raise ValueError("map yaml image does not match uploaded pgm")
        except (UnicodeDecodeError, yaml.YAMLError, ValueError) as exc:
            raise PlatformStoreError("INVALID_MAP", str(exc)) from exc
        existing = self.deployment(deployment_id)
        if existing:
            if existing["route_sha256"] == route_hash and existing["map_sha256"] == manifest["mapImageSha256"]:
                return {"deploymentId": deployment_id, "state": "DEPLOYED", "idempotent": True, **existing}
            raise PlatformStoreError("DEPLOYMENT_CONFLICT", "deploymentId already has different content", 409)
        staging = Path(tempfile.mkdtemp(prefix=f"{deployment_id}-", dir=self.root / "staging"))
        target = self.root / "deployments" / deployment_id
        installed = False
        local_route_path = None
        local_route_temp = None
        local_route_published = False
        try:
            (staging / "manifest.json").write_bytes(canonical_json(manifest))
            (staging / "route.json").write_bytes(canonical_json(route_data))
            # Validate against original names first; executor routes bind those names.
            (staging / yaml_name).write_bytes(yaml_bytes)
            (staging / pgm_name).write_bytes(pgm)
            validate_route_map_binding(normalized_route, staging / yaml_name)
            local_map_yaml = self._default_map_yaml_path()
            if local_map_yaml is not None:
                local_map_data = yaml.safe_load(local_map_yaml.read_text(encoding="utf-8"))
                if not isinstance(local_map_data, dict):
                    raise ValueError("local map yaml must be an object")
                local_image_name = Path(str(local_map_data.get("image", ""))).name
                local_route = {
                    **normalized_route,
                    "map": {
                        **normalized_route["map"],
                        "yaml": local_map_yaml.name,
                        "image": local_image_name,
                    },
                }
                validate_route_map_binding(local_route, local_map_yaml)
                local_route_path = self._next_route_path(local_map_yaml.parent)
                with tempfile.NamedTemporaryFile(
                    prefix=f".{local_route_path.name}.",
                    suffix=".tmp",
                    dir=local_map_yaml.parent,
                    delete=False,
                ) as route_file:
                    route_file.write(canonical_json(local_route))
                    local_route_temp = Path(route_file.name)
            os.replace(staging, target)
            installed = True
            if local_route_path is not None and local_route_temp is not None:
                os.replace(local_route_temp, local_route_path)
                local_route_published = True
            with self._connection() as db:
                db.execute("INSERT INTO deployments VALUES (?, ?, ?, ?, ?)", (deployment_id, json.dumps(manifest), route_hash, manifest["mapImageSha256"], _now()))
            return {"deploymentId": deployment_id, "state": "DEPLOYED", "idempotent": False, "routePayloadSha256": route_hash, "mapImageSha256": manifest["mapImageSha256"], "routePath": str(target / "route.json"), "mapYamlPath": str(target / yaml_name), "mapPgmPath": str(target / pgm_name)}
        except PlatformStoreError:
            if installed:
                shutil.rmtree(target, ignore_errors=True)
            if local_route_published and local_route_path is not None:
                local_route_path.unlink(missing_ok=True)
            raise
        except Exception as exc:
            if installed:
                shutil.rmtree(target, ignore_errors=True)
            if local_route_published and local_route_path is not None:
                local_route_path.unlink(missing_ok=True)
            raise PlatformStoreError("INVALID_MAP", str(exc)) from exc
        finally:
            shutil.rmtree(staging, ignore_errors=True)
            if local_route_temp is not None:
                local_route_temp.unlink(missing_ok=True)

    def execution(self, execution_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute("SELECT * FROM executions WHERE execution_id=?", (execution_id,)).fetchone()
        return dict(row) if row else None

    def upsert_execution(self, execution_id: str, deployment_id: str, request_id: str, state: str) -> Dict[str, Any]:
        with self._connection() as db:
            existing = db.execute("SELECT * FROM executions WHERE execution_id=?", (execution_id,)).fetchone()
            if existing and (existing["deployment_id"] != deployment_id or existing["request_id"] != request_id):
                raise PlatformStoreError("EXECUTION_CONFLICT", "executionId already belongs to another request", 409)
            db.execute("INSERT INTO executions VALUES (?, ?, ?, ?, ?) ON CONFLICT(execution_id) DO UPDATE SET state=excluded.state, updated_at=excluded.updated_at", (execution_id, deployment_id, request_id, state, _now()))
        return self.execution(execution_id) or {}

    def append_event(self, event: Dict[str, Any]) -> Dict[str, Any]:
        event = dict(event)
        event.setdefault("occurred_at", _now())
        with self._connection() as db:
            cursor = db.execute("INSERT INTO events(event_json, occurred_at) VALUES (?, ?)", (json.dumps(event, ensure_ascii=False), event["occurred_at"]))
            event["sequence"] = cursor.lastrowid
        return event

    def events(self, after_sequence: int, limit: int) -> List[Dict[str, Any]]:
        limit = max(1, min(int(limit), 1000))
        with self._connection() as db:
            rows = db.execute("SELECT sequence, event_json FROM events WHERE sequence>? ORDER BY sequence LIMIT ?", (max(0, int(after_sequence)), limit)).fetchall()
        return [{**json.loads(row["event_json"]), "sequence": row["sequence"]} for row in rows]

    def latest_event_sequence(self) -> int:
        with self._connection() as db:
            row = db.execute("SELECT COALESCE(MAX(sequence), 0) AS sequence FROM events").fetchone()
        return int(row["sequence"])

    def receive_cloud_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        command_id = str(command.get("commandId") or "")
        request_id = str(command.get("requestId") or "")
        command_type = str(command.get("type") or "").upper()
        execution_id = str(command.get("executionId") or "")
        deployment_id = str(command.get("deploymentId") or "")
        if not command_id or not request_id or command_type not in {"START", "PAUSE", "RESUME", "TAKEOVER", "CANCEL"} or not execution_id:
            raise PlatformStoreError("INVALID_COMMAND", "invalid cloud command")
        if command_type == "START" and (not deployment_id or not str(command.get("executorRouteId") or "")):
            raise PlatformStoreError("INVALID_COMMAND", "START requires deploymentId and executorRouteId")
        business_command = normalize_command_business_payload(command)
        if command_type == "START":
            start_mode = str(
                business_command.get("startMode") or START_MODE_REMOTE_IMMEDIATE
            ).upper()
            if start_mode not in START_MODES:
                raise PlatformStoreError(
                    "INVALID_START_MODE", f"unsupported startMode: {start_mode}"
                )
            business_command["startMode"] = start_mode
        payload = canonical_json(business_command).decode("utf-8")
        with self._connection() as db:
            db.execute("BEGIN IMMEDIATE")
            existing = db.execute("SELECT * FROM processed_commands WHERE command_id=?", (command_id,)).fetchone()
            if existing:
                if existing["payload_json"] != payload:
                    raise PlatformStoreError("COMMAND_CONFLICT", "commandId has different payload", 409)
                return self._command_dict(existing)
            duplicate = db.execute("SELECT * FROM processed_commands WHERE request_id=?", (request_id,)).fetchone()
            if duplicate:
                raise PlatformStoreError("COMMAND_CONFLICT", "requestId belongs to another command", 409)
            stamp = _now()
            db.execute("INSERT INTO processed_commands(command_id,request_id,execution_id,deployment_id,command_type,payload_json,state,result_json,created_at,updated_at) VALUES (?,?,?,?,?,?,?,?,?,?)", (command_id, request_id, execution_id, deployment_id, command_type, payload, COMMAND_STATE_RECEIVED, "{}", stamp, stamp))
            return {"command_id": command_id, "request_id": request_id, "execution_id": execution_id, "deployment_id": deployment_id, "command_type": command_type, "payload": business_command, "state": COMMAND_STATE_RECEIVED, "result": {}}

    @staticmethod
    def _command_dict(row: sqlite3.Row) -> Dict[str, Any]:
        return {"command_id": row["command_id"], "request_id": row["request_id"], "execution_id": row["execution_id"], "deployment_id": row["deployment_id"], "command_type": row["command_type"], "payload": json.loads(row["payload_json"]), "state": row["state"], "result": json.loads(row["result_json"])}

    def command(self, command_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute("SELECT * FROM processed_commands WHERE command_id=?", (command_id,)).fetchone()
        return self._command_dict(row) if row else None

    def set_command_state(self, command_id: str, state: str, result: Dict[str, Any] | None = None) -> None:
        if state not in COMMAND_TRANSITIONS:
            raise PlatformStoreError("INVALID_COMMAND", "invalid command state")
        with self._connection() as db:
            row = db.execute("SELECT state FROM processed_commands WHERE command_id=?", (command_id,)).fetchone()
            if not row:
                raise PlatformStoreError("COMMAND_NOT_FOUND", "command does not exist", 404)
            current = str(row["state"])
            if state != current and state not in COMMAND_TRANSITIONS[current]:
                raise PlatformStoreError("INVALID_COMMAND_TRANSITION", f"cannot move command from {current} to {state}", 409)
            db.execute("UPDATE processed_commands SET state=?,result_json=?,updated_at=? WHERE command_id=?", (state, json.dumps(result or {}, ensure_ascii=False), _now(), command_id))

    def pending_cloud_commands(self) -> List[Dict[str, Any]]:
        with self._connection() as db:
            rows = db.execute("SELECT * FROM processed_commands WHERE state IN ('RECEIVED','ACKED','CONFIRMED','DISPATCHED') ORDER BY created_at").fetchall()
        return [self._command_dict(row) for row in rows]

    def pending_command_count(self) -> int:
        with self._connection() as db:
            row = db.execute("SELECT COUNT(*) AS count FROM processed_commands WHERE state IN ('RECEIVED','ACKED','ARMED','CONFIRMED','DISPATCHED')").fetchone()
        return int(row["count"])

    @staticmethod
    def _event_from_record(
        record: Dict[str, Any], event: str, robot_id: str, boot_id: str,
        **extra: Any,
    ) -> Dict[str, Any]:
        return {
            "schema_version": "1.0", "event": event,
            "robot_id": robot_id, "boot_id": boot_id,
            "command_id": record["command_id"],
            "request_id": record["request_id"],
            "execution_id": record["execution_id"],
            "deployment_id": record["deployment_id"],
            "occurred_at": _now(), **extra,
        }

    @staticmethod
    def _insert_event(db: sqlite3.Connection, event: Dict[str, Any]) -> Dict[str, Any]:
        cursor = db.execute(
            "INSERT INTO events(event_json, occurred_at) VALUES (?, ?)",
            (json.dumps(event, ensure_ascii=False), event["occurred_at"]),
        )
        return {**event, "sequence": cursor.lastrowid}

    def arm_start(self, command_id: str, robot_id: str, boot_id: str) -> Dict[str, Any]:
        with self._connection() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute(
                "SELECT * FROM processed_commands WHERE command_id=?",
                (command_id,),
            ).fetchone()
            if not row:
                raise PlatformStoreError("COMMAND_NOT_FOUND", "command does not exist", 404)
            record = self._command_dict(row)
            if record["command_type"] != "START" or record["state"] != COMMAND_STATE_ACKED:
                raise PlatformStoreError(
                    "INVALID_COMMAND_TRANSITION", "only ACKED START can be armed", 409
                )
            other = db.execute(
                "SELECT command_id FROM processed_commands "
                "WHERE command_type='START' AND state='ARMED' AND command_id<>?",
                (command_id,),
            ).fetchone()
            if other:
                raise PlatformStoreError(
                    "LOCAL_CONFIRM_ALREADY_ARMED",
                    "another platform START is waiting for local confirmation", 409,
                )
            armed_at = _now()
            event = self._insert_event(db, self._event_from_record(
                record, "start_waiting_local_confirmation", robot_id, boot_id,
                armed_at=armed_at,
            ))
            db.execute(
                "UPDATE processed_commands SET state=?,result_json=?,updated_at=? "
                "WHERE command_id=?",
                (COMMAND_STATE_ARMED, json.dumps({**event, "armedAt": armed_at}, ensure_ascii=False), armed_at, command_id),
            )
        return self.command(command_id) or {}

    def armed_start(self, execution_id: str = "") -> Dict[str, Any] | None:
        query = "SELECT * FROM processed_commands WHERE command_type='START' AND state='ARMED'"
        params: tuple[Any, ...] = ()
        if execution_id:
            query += " AND execution_id=?"
            params = (execution_id,)
        query += " ORDER BY created_at"
        with self._connection() as db:
            rows = db.execute(query, params).fetchall()
        if len(rows) > 1:
            raise PlatformStoreError(
                "MULTIPLE_ARMED_STARTS", "multiple platform START commands are armed", 409
            )
        return self._command_dict(rows[0]) if rows else None

    def pending_platform_start(self) -> Dict[str, Any]:
        record = self.armed_start()
        if not record:
            return {}
        payload = record["payload"]
        return {
            "taskName": str(payload.get("taskName") or payload.get("taskId") or ""),
            "routeName": str(payload.get("routeName") or payload.get("executorRouteId") or ""),
            "executionId": record["execution_id"],
            "deploymentId": record["deployment_id"],
            "armedAt": str(record["result"].get("armedAt") or ""),
        }

    def confirm_armed_start(self, robot_id: str, boot_id: str) -> Dict[str, Any]:
        with self._connection() as db:
            db.execute("BEGIN IMMEDIATE")
            rows = db.execute(
                "SELECT * FROM processed_commands "
                "WHERE command_type='START' AND state='ARMED' ORDER BY created_at"
            ).fetchall()
            if not rows:
                raise PlatformStoreError(
                    "NO_ARMED_PLATFORM_START", "no platform START awaits confirmation", 409
                )
            if len(rows) != 1:
                raise PlatformStoreError(
                    "MULTIPLE_ARMED_STARTS", "multiple platform START commands are armed", 409
                )
            record = self._command_dict(rows[0])
            event = self._insert_event(db, self._event_from_record(
                record, "local_start_confirmed", robot_id, boot_id,
            ))
            updated = db.execute(
                "UPDATE processed_commands SET state=?,result_json=?,updated_at=? "
                "WHERE command_id=? AND state=?",
                (COMMAND_STATE_CONFIRMED, json.dumps(event, ensure_ascii=False), _now(),
                 record["command_id"], COMMAND_STATE_ARMED),
            )
            if updated.rowcount != 1:
                raise PlatformStoreError(
                    "LOCAL_CONFIRM_ALREADY_CLAIMED",
                    "platform START was already confirmed or canceled", 409,
                )
        return self.command(record["command_id"]) or record

    def cancel_armed_start(
        self, cancel_record: Dict[str, Any], robot_id: str, boot_id: str,
    ) -> Dict[str, Any] | None:
        with self._connection() as db:
            db.execute("BEGIN IMMEDIATE")
            start_row = db.execute(
                "SELECT * FROM processed_commands WHERE command_type='START' "
                "AND state IN ('ARMED','CONFIRMED') AND execution_id=?",
                (cancel_record["execution_id"],),
            ).fetchone()
            if not start_row:
                return None
            start_record = self._command_dict(start_row)
            event = self._insert_event(db, self._event_from_record(
                cancel_record, "route_canceled", robot_id, boot_id,
                canceled_before_local_confirmation=True,
            ))
            stamp = _now()
            db.execute(
                "UPDATE processed_commands SET state=?,result_json=?,updated_at=? "
                "WHERE command_id=?",
                (COMMAND_STATE_REJECTED, json.dumps({
                    "event": "command_rejected",
                    "error_code": "CANCELED_BEFORE_LOCAL_CONFIRMATION",
                    "canceled_by_command_id": cancel_record["command_id"],
                }, ensure_ascii=False), stamp, start_record["command_id"]),
            )
            db.execute(
                "UPDATE processed_commands SET state=?,result_json=?,updated_at=? "
                "WHERE command_id=?",
                (COMMAND_STATE_APPLIED, json.dumps(event, ensure_ascii=False), stamp,
                 cancel_record["command_id"]),
            )
        return event

    def expire_armed_starts(
        self, timeout_sec: float, robot_id: str, boot_id: str,
    ) -> List[Dict[str, Any]]:
        cutoff = (datetime.now(timezone.utc) - timedelta(seconds=timeout_sec)).isoformat()
        expired: List[Dict[str, Any]] = []
        with self._connection() as db:
            db.execute("BEGIN IMMEDIATE")
            rows = db.execute(
                "SELECT * FROM processed_commands WHERE command_type='START' "
                "AND state='ARMED' AND updated_at<=? ORDER BY created_at",
                (cutoff,),
            ).fetchall()
            for row in rows:
                record = self._command_dict(row)
                event = self._insert_event(db, self._event_from_record(
                    record, "command_failed", robot_id, boot_id,
                    error_code="LOCAL_CONFIRM_TIMEOUT",
                    error_message="local confirmation timed out",
                ))
                db.execute(
                    "UPDATE processed_commands SET state=?,result_json=?,updated_at=? "
                    "WHERE command_id=?",
                    (COMMAND_STATE_FAILED, json.dumps(event, ensure_ascii=False),
                     event["occurred_at"], record["command_id"]),
                )
                expired.append(event)
        return expired

    def task_id_for_execution(self, execution_id: str) -> str:
        if not execution_id:
            return ""
        with self._connection() as db:
            rows = db.execute(
                "SELECT payload_json FROM processed_commands "
                "WHERE execution_id=? AND command_type='START' "
                "ORDER BY created_at DESC",
                (execution_id,),
            ).fetchall()
        for row in rows:
            try:
                task_id = str(json.loads(row["payload_json"]).get("taskId") or "")
            except (TypeError, json.JSONDecodeError):
                continue
            if task_id:
                return task_id
        return ""

    def pending_event_count(self, after_sequence: int | None = None) -> int:
        cursor = int(self.cloud_state("last_uploaded_sequence", "0") or 0) if after_sequence is None else max(0, int(after_sequence))
        with self._connection() as db:
            row = db.execute("SELECT COUNT(*) AS count FROM events WHERE sequence>?", (cursor,)).fetchone()
        return int(row["count"])

    def set_cloud_state(self, key: str, value: str) -> None:
        with self._connection() as db:
            db.execute("INSERT INTO cloud_state(state_key,state_value,updated_at) VALUES (?,?,?) ON CONFLICT(state_key) DO UPDATE SET state_value=excluded.state_value,updated_at=excluded.updated_at", (key, str(value), _now()))

    def cloud_state(self, key: str, default: str = "") -> str:
        with self._connection() as db:
            row = db.execute("SELECT state_value FROM cloud_state WHERE state_key=?", (key,)).fetchone()
        return str(row["state_value"]) if row else default

    def set_bridge_setting(self, key: str, value: str) -> None:
        with self._connection() as db:
            db.execute(
                "INSERT INTO bridge_settings(setting_key,setting_value,updated_at) VALUES (?,?,?) "
                "ON CONFLICT(setting_key) DO UPDATE SET setting_value=excluded.setting_value,updated_at=excluded.updated_at",
                (key, str(value), _now()),
            )

    def bridge_setting(self, key: str, default: str = "") -> str:
        with self._connection() as db:
            row = db.execute(
                "SELECT setting_value FROM bridge_settings WHERE setting_key=?",
                (key,),
            ).fetchone()
        return str(row["setting_value"]) if row else default

    @staticmethod
    def _map_upload_dict(row: sqlite3.Row | None) -> Dict[str, Any] | None:
        if row is None:
            return None
        data = dict(row)
        data["identity"] = json.loads(data.pop("identity_json"))
        return data

    def map_upload_by_identity(self, identity: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM map_uploads WHERE content_identity_sha256=?",
                (identity,),
            ).fetchone()
        return self._map_upload_dict(row)

    def map_upload(self, task_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM map_uploads WHERE task_id=?", (task_id,)
            ).fetchone()
        return self._map_upload_dict(row)

    def create_map_upload(self, record: Dict[str, Any]) -> Dict[str, Any]:
        stamp = time.time()
        with self._connection() as db:
            db.execute(
                "INSERT INTO map_uploads ("
                "task_id,idempotency_key,content_identity_sha256,identity_json,"
                "yaml_sha256,pgm_sha256,yaml_path,pgm_path,status,retry_count,"
                "next_retry_at,map_asset_id,last_error,created_at,updated_at"
                ") VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    record["task_id"], record["idempotency_key"],
                    record["content_identity_sha256"],
                    json.dumps(record["identity"], sort_keys=True),
                    record["yaml_sha256"], record["pgm_sha256"],
                    record["yaml_path"], record["pgm_path"], "PENDING", 0,
                    0.0, "", "", stamp, stamp,
                ),
            )
        return self.map_upload(record["task_id"]) or {}

    def requeue_map_upload(self, task_id: str) -> Dict[str, Any]:
        with self._connection() as db:
            db.execute(
                "UPDATE map_uploads SET status='PENDING',next_retry_at=0,"
                "last_error='',updated_at=? WHERE task_id=?",
                (time.time(), task_id),
            )
        return self.map_upload(task_id) or {}

    def restart_map_upload(
        self, previous_task_id: str, record: Dict[str, Any]
    ) -> Dict[str, Any]:
        stamp = time.time()
        with self._connection() as db:
            db.execute(
                "UPDATE map_uploads SET task_id=?,idempotency_key=?,"
                "identity_json=?,yaml_sha256=?,pgm_sha256=?,yaml_path=?,"
                "pgm_path=?,status='PENDING',retry_count=0,next_retry_at=0,"
                "map_asset_id='',last_error='',created_at=?,updated_at=? "
                "WHERE task_id=?",
                (
                    record["task_id"], record["idempotency_key"],
                    json.dumps(record["identity"], sort_keys=True),
                    record["yaml_sha256"], record["pgm_sha256"],
                    record["yaml_path"], record["pgm_path"], stamp, stamp,
                    previous_task_id,
                ),
            )
        return self.map_upload(record["task_id"]) or {}

    def restore_map_upload_snapshot(
        self, task_id: str, yaml_path: str, pgm_path: str
    ) -> None:
        with self._connection() as db:
            db.execute(
                "UPDATE map_uploads SET yaml_path=?,pgm_path=?,updated_at=? "
                "WHERE task_id=?",
                (yaml_path, pgm_path, time.time(), task_id),
            )

    def next_due_map_upload(self) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM map_uploads WHERE status='PENDING' OR "
                "(status='FAILED_RETRYABLE' AND next_retry_at<=?) "
                "ORDER BY created_at LIMIT 1",
                (time.time(),),
            ).fetchone()
        return self._map_upload_dict(row)

    def finish_map_upload(
        self,
        task_id: str,
        status: str,
        *,
        map_asset_id: str = "",
        error: str = "",
        next_retry_at: float = 0.0,
    ) -> Dict[str, Any]:
        if status not in {"FAILED_RETRYABLE", "FAILED_FINAL", "SUCCEEDED"}:
            raise ValueError("invalid map upload status")
        with self._connection() as db:
            db.execute(
                "UPDATE map_uploads SET status=?,retry_count=retry_count+1,"
                "next_retry_at=?,map_asset_id=?,last_error=?,updated_at=? "
                "WHERE task_id=?",
                (
                    status, next_retry_at, map_asset_id, error[:300],
                    time.time(), task_id,
                ),
            )
        return self.map_upload(task_id) or {}

    def map_uploads_for_cleanup(self) -> List[Dict[str, Any]]:
        with self._connection() as db:
            rows = db.execute(
                "SELECT * FROM map_uploads ORDER BY created_at DESC"
            ).fetchall()
        return [self._map_upload_dict(row) or {} for row in rows]

    def clear_map_upload_snapshot(self, task_id: str) -> None:
        with self._connection() as db:
            db.execute(
                "UPDATE map_uploads SET yaml_path='',pgm_path='',updated_at=? "
                "WHERE task_id=?",
                (time.time(), task_id),
            )

    @staticmethod
    def _inspection_image_upload_dict(
        row: sqlite3.Row | None,
    ) -> Dict[str, Any] | None:
        return dict(row) if row is not None else None

    def inspection_image_upload_by_identity(
        self, capture_identity: str,
    ) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM inspection_image_uploads WHERE capture_identity=?",
                (capture_identity,),
            ).fetchone()
        return self._inspection_image_upload_dict(row)

    def inspection_image_upload(
        self, capture_task_id: str,
    ) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM inspection_image_uploads WHERE capture_task_id=?",
                (capture_task_id,),
            ).fetchone()
        return self._inspection_image_upload_dict(row)

    def create_inspection_image_upload(
        self, record: Dict[str, Any],
    ) -> Dict[str, Any]:
        stamp = time.time()
        with self._connection() as db:
            db.execute(
                "INSERT INTO inspection_image_uploads ("
                "capture_task_id,capture_identity,idempotency_key,task_id,"
                "execution_id,checkpoint_id,capture_kind,captured_at,"
                "image_sha256,file_path,file_size,status,retry_count,"
                "next_retry_at,image_id,last_error,created_at,updated_at"
                ") VALUES (?,?,?,?,?,?,?,?,?,?,?,'PENDING',0,0,'','',?,?)",
                (
                    record["capture_task_id"], record["capture_identity"],
                    record["idempotency_key"], record["task_id"],
                    record["execution_id"], record["checkpoint_id"],
                    record["capture_kind"], record["captured_at"],
                    record["image_sha256"], record["file_path"],
                    record["file_size"], stamp, stamp,
                ),
            )
        return self.inspection_image_upload(record["capture_task_id"]) or {}

    def next_due_inspection_image_upload(self) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM inspection_image_uploads WHERE status='PENDING' OR "
                "(status='FAILED_RETRYABLE' AND next_retry_at<=?) "
                "ORDER BY created_at LIMIT 1",
                (time.time(),),
            ).fetchone()
        return self._inspection_image_upload_dict(row)

    def finish_inspection_image_upload(
        self,
        capture_task_id: str,
        status: str,
        *,
        image_id: str = "",
        error: str = "",
        next_retry_at: float = 0.0,
    ) -> Dict[str, Any]:
        if status not in {
            "FAILED_RETRYABLE", "FAILED_FINAL", "CREDENTIAL_BLOCKED", "SUCCEEDED",
        }:
            raise ValueError("invalid inspection image upload status")
        with self._connection() as db:
            db.execute(
                "UPDATE inspection_image_uploads SET status=?,"
                "retry_count=retry_count+1,next_retry_at=?,image_id=?,"
                "last_error=?,updated_at=? WHERE capture_task_id=?",
                (
                    status, next_retry_at, image_id, error[:300], time.time(),
                    capture_task_id,
                ),
            )
        return self.inspection_image_upload(capture_task_id) or {}

    def inspection_image_upload_diagnostics(self) -> Dict[str, Any]:
        with self._connection() as db:
            counts = db.execute(
                "SELECT "
                "SUM(CASE WHEN status IN ('PENDING','FAILED_RETRYABLE') THEN 1 ELSE 0 END) pending_count, "
                "SUM(CASE WHEN status IN ('FAILED_FINAL','CREDENTIAL_BLOCKED') THEN 1 ELSE 0 END) failed_count, "
                "SUM(CASE WHEN status='CREDENTIAL_BLOCKED' THEN 1 ELSE 0 END) credential_blocked "
                "FROM inspection_image_uploads"
            ).fetchone()
            last_success = db.execute(
                "SELECT updated_at FROM inspection_image_uploads "
                "WHERE status='SUCCEEDED' ORDER BY updated_at DESC LIMIT 1"
            ).fetchone()
            last_error = db.execute(
                "SELECT last_error FROM inspection_image_uploads "
                "WHERE last_error<>'' ORDER BY updated_at DESC LIMIT 1"
            ).fetchone()
        return {
            "pending_count": int(counts["pending_count"] or 0),
            "failed_count": int(counts["failed_count"] or 0),
            "credential_blocked": bool(counts["credential_blocked"] or 0),
            "last_success_at": float(last_success["updated_at"]) if last_success else 0.0,
            "last_error": str(last_error["last_error"]) if last_error else "",
        }

    def inspection_image_uploads_for_cleanup(self) -> List[Dict[str, Any]]:
        with self._connection() as db:
            rows = db.execute(
                "SELECT * FROM inspection_image_uploads "
                "WHERE status='FAILED_FINAL' AND file_path<>'' "
                "ORDER BY created_at"
            ).fetchall()
        return [dict(row) for row in rows]

    def clear_inspection_image_snapshot(self, capture_task_id: str) -> None:
        with self._connection() as db:
            db.execute(
                "UPDATE inspection_image_uploads SET file_path='',file_size=0,"
                "updated_at=? WHERE capture_task_id=?",
                (time.time(), capture_task_id),
            )

    @staticmethod
    def _scene_upload_dict(row: sqlite3.Row | None) -> Dict[str, Any] | None:
        return dict(row) if row is not None else None

    def scene_upload(self, task_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM scene_uploads WHERE task_id=?", (task_id,)
            ).fetchone()
        return self._scene_upload_dict(row)

    def scene_upload_by_session(self, session_id: str) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM scene_uploads "
                "WHERE source_reconstruct_session_id=? "
                "ORDER BY intent_number DESC LIMIT 1",
                (session_id,),
            ).fetchone()
        return self._scene_upload_dict(row)

    def scene_upload_by_identity(
        self, session_id: str, model_sha256: str,
    ) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM scene_uploads "
                "WHERE source_reconstruct_session_id=? AND model_sha256=? "
                "ORDER BY intent_number DESC LIMIT 1",
                (session_id, model_sha256),
            ).fetchone()
        return self._scene_upload_dict(row)

    def create_scene_upload(self, record: Dict[str, Any]) -> Dict[str, Any]:
        stamp = time.time()
        with self._connection() as db:
            db.execute(
                "INSERT INTO scene_uploads ("
                "task_id,idempotency_key,source_reconstruct_session_id,"
                "source_capture_session_id,model_sha256,intent_number,"
                "model_path,metadata_path,metadata_json,file_size,asset_kind,"
                "format,status,retry_count,next_retry_at,scene_asset_id,"
                "last_error,supersedes_task_id,created_at,updated_at"
                ") VALUES (?,?,?,?,?,?,?,?,?,?,?,?, 'PENDING',0,0,'','',?,?,?)",
                (
                    record["task_id"], record["idempotency_key"],
                    record["source_reconstruct_session_id"],
                    record.get("source_capture_session_id", ""),
                    record["model_sha256"], int(record.get("intent_number", 0)),
                    record["model_path"], record["metadata_path"],
                    record["metadata_json"], int(record["file_size"]),
                    record["asset_kind"], record["format"],
                    record.get("supersedes_task_id", ""), stamp, stamp,
                ),
            )
        return self.scene_upload(record["task_id"]) or {}

    def next_due_scene_upload(self) -> Dict[str, Any] | None:
        with self._connection() as db:
            row = db.execute(
                "SELECT * FROM scene_uploads WHERE status='PENDING' OR "
                "(status='FAILED_RETRYABLE' AND next_retry_at<=?) "
                "ORDER BY created_at LIMIT 1",
                (time.time(),),
            ).fetchone()
        return self._scene_upload_dict(row)

    def finish_scene_upload(
        self,
        task_id: str,
        status: str,
        *,
        scene_asset_id: str = "",
        error: str = "",
        next_retry_at: float = 0.0,
    ) -> Dict[str, Any]:
        if status not in {
            "FAILED_RETRYABLE", "FAILED_FINAL", "CREDENTIAL_BLOCKED", "SUCCEEDED",
        }:
            raise ValueError("invalid scene upload status")
        with self._connection() as db:
            db.execute(
                "UPDATE scene_uploads SET status=?,retry_count=retry_count+"
                "CASE WHEN ?='FAILED_RETRYABLE' THEN 1 ELSE 0 END,"
                "next_retry_at=?,scene_asset_id=?,last_error=?,updated_at=? "
                "WHERE task_id=?",
                (
                    status, status, next_retry_at, scene_asset_id, error[:300],
                    time.time(), task_id,
                ),
            )
        return self.scene_upload(task_id) or {}

    def requeue_scene_upload(self, task_id: str) -> Dict[str, Any]:
        with self._connection() as db:
            db.execute(
                "UPDATE scene_uploads SET status='PENDING',next_retry_at=0,"
                "last_error='',updated_at=? WHERE task_id=?",
                (time.time(), task_id),
            )
        return self.scene_upload(task_id) or {}

    def scene_uploads(self, limit: int = 20) -> List[Dict[str, Any]]:
        limit = max(1, min(int(limit), 100))
        with self._connection() as db:
            rows = db.execute(
                "SELECT * FROM scene_uploads ORDER BY created_at DESC LIMIT ?",
                (limit,),
            ).fetchall()
        return [dict(row) for row in rows]

    def scene_uploads_for_cleanup(self) -> List[Dict[str, Any]]:
        with self._connection() as db:
            rows = db.execute(
                "SELECT * FROM scene_uploads WHERE status='SUCCEEDED' "
                "AND model_path<>'' ORDER BY created_at DESC"
            ).fetchall()
        return [dict(row) for row in rows]

    def clear_scene_upload_snapshot(self, task_id: str) -> None:
        with self._connection() as db:
            db.execute(
                "UPDATE scene_uploads SET model_path='',metadata_path='',"
                "updated_at=? WHERE task_id=?",
                (time.time(), task_id),
            )
