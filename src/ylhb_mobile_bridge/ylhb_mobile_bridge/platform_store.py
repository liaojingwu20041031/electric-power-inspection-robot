"""Small durable store for Robot Platform Protocol v1."""
import hashlib
import json
import os
import shutil
import sqlite3
import tempfile
from datetime import datetime, timezone
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
COMMAND_TRANSITIONS = {
    "RECEIVED": {"ACKED"},
    "ACKED": {"DISPATCHED", "REJECTED", "FAILED"},
    "DISPATCHED": {"APPLIED", "REJECTED", "FAILED"},
    "APPLIED": set(),
    "REJECTED": set(),
    "FAILED": set(),
}


class PlatformStoreError(ValueError):
    def __init__(self, code: str, message: str, status_code: int = 400):
        super().__init__(message)
        self.code = code
        self.status_code = status_code


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


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
            db.execute("INSERT INTO processed_commands(command_id,request_id,execution_id,deployment_id,command_type,payload_json,state,result_json,created_at,updated_at) VALUES (?,?,?,?,?,?,?,?,?,?)", (command_id, request_id, execution_id, deployment_id, command_type, payload, "RECEIVED", "{}", stamp, stamp))
            return {"command_id": command_id, "request_id": request_id, "execution_id": execution_id, "deployment_id": deployment_id, "command_type": command_type, "payload": business_command, "state": "RECEIVED", "result": {}}

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
            rows = db.execute("SELECT * FROM processed_commands WHERE state IN ('RECEIVED','ACKED','DISPATCHED') ORDER BY created_at").fetchall()
        return [self._command_dict(row) for row in rows]

    def pending_command_count(self) -> int:
        with self._connection() as db:
            row = db.execute("SELECT COUNT(*) AS count FROM processed_commands WHERE state IN ('RECEIVED','ACKED','DISPATCHED')").fetchone()
        return int(row["count"])

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
