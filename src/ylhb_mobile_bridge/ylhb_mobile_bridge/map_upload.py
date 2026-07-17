"""Durable, single-threaded upload of the current default map."""
import hashlib
import http.client
import json
import logging
import os
import shutil
import ssl
import tempfile
import threading
import time
import urllib.parse
import uuid
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any, Dict

import yaml

from .platform_store import canonical_json


LOG = logging.getLogger(__name__)
TRUE_VALUES = {"1", "true", "yes", "on"}
ALLOWED_YAML_FIELDS = {
    "image", "resolution", "origin", "negate",
    "occupied_thresh", "free_thresh", "mode",
}


class MapUploadError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        retryable: bool,
        retry_after: float | None = None,
    ):
        super().__init__(message)
        self.retryable = retryable
        self.retry_after = retry_after


def _decimal(value: Any, field: str) -> Decimal:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a number")
    try:
        number = Decimal(str(value))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{field} must be a number") from exc
    if not number.is_finite():
        raise ValueError(f"{field} must be finite")
    return number


def _decimal_string(value: Any, field: str) -> str:
    number = _decimal(value, field)
    if number == 0:
        return "0"
    return format(number.normalize(), "f")


def normalized_map_identity(yaml_bytes: bytes, pgm_sha256: str) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(yaml_bytes.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError("map YAML is invalid") from exc
    if not isinstance(document, dict):
        raise ValueError("map YAML must contain an object")
    extra = set(document) - ALLOWED_YAML_FIELDS
    if extra:
        raise ValueError(f"unsupported map YAML fields: {', '.join(sorted(extra))}")
    image = document.get("image")
    if not isinstance(image, str) or Path(image).name != image or not image.lower().endswith(".pgm"):
        raise ValueError("map YAML image must be a local PGM filename")
    resolution = _decimal(document.get("resolution"), "resolution")
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    origin = document.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("origin must contain three numbers")
    negate = document.get("negate", 0)
    if negate not in (0, 1) or isinstance(negate, bool):
        raise ValueError("negate must be 0 or 1")
    occupied = _decimal(document.get("occupied_thresh", "0.65"), "occupied_thresh")
    free = _decimal(document.get("free_thresh", "0.25"), "free_thresh")
    if not (Decimal("0") <= free < occupied <= Decimal("1")):
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")
    mode = str(document.get("mode", "trinary")).lower()
    if mode not in {"trinary", "scale", "raw"}:
        raise ValueError("mode must be trinary, scale or raw")
    return {
        "pgmSha256": pgm_sha256,
        "resolution": _decimal_string(resolution, "resolution"),
        "origin": [_decimal_string(value, "origin") for value in origin],
        "negate": int(negate),
        "occupiedThresh": _decimal_string(occupied, "occupied_thresh"),
        "freeThresh": _decimal_string(free, "free_thresh"),
        "mode": mode,
    }


def content_identity_sha256(identity: Dict[str, Any]) -> str:
    return hashlib.sha256(canonical_json(identity)).hexdigest()


def _env_int(name: str, default: int) -> int:
    try:
        return max(0, int(os.environ.get(name, str(default))))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return max(0.0, float(os.environ.get(name, str(default))))
    except ValueError:
        return default


class MapUploadWorker:
    def __init__(self, store, robot_id: str):
        self.store = store
        self.robot_id = robot_id
        self.base_url = os.environ.get("YLHB_CLOUD_BASE_URL", "").rstrip("/")
        self.token = os.environ.get("YLHB_CLOUD_ROBOT_TOKEN", "")
        self.enabled = os.environ.get("YLHB_MAP_UPLOAD_ENABLED", "true").lower() in TRUE_VALUES
        self.yaml_max = _env_int("YLHB_MAP_UPLOAD_YAML_MAX_BYTES", 1024 * 1024)
        self.pgm_max = _env_int("YLHB_MAP_UPLOAD_PGM_MAX_BYTES", 100 * 1024 * 1024)
        self.request_max = _env_int("YLHB_MAP_UPLOAD_REQUEST_MAX_BYTES", 110 * 1024 * 1024)
        self.connect_timeout = _env_float("YLHB_MAP_UPLOAD_CONNECT_TIMEOUT_SEC", 10.0)
        self.read_timeout = _env_float("YLHB_MAP_UPLOAD_READ_TIMEOUT_SEC", 60.0)
        self.retry_base = _env_float("YLHB_MAP_UPLOAD_RETRY_BASE_SEC", 2.0)
        self.retry_max = _env_float("YLHB_MAP_UPLOAD_RETRY_MAX_SEC", 300.0)
        self.retention = _env_float("YLHB_MAP_UPLOAD_SUCCESS_RETENTION_SEC", 86400.0)
        self.success_keep = _env_int("YLHB_MAP_UPLOAD_SUCCESS_KEEP", 2)
        self.disk_max = _env_int("YLHB_MAP_UPLOAD_SNAPSHOT_MAX_BYTES", 1024 * 1024 * 1024)
        self.snapshot_root = self.store.root / "map_upload_snapshots"
        self.snapshot_root.mkdir(parents=True, exist_ok=True)
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self.current_task_id = ""
        try:
            self._context = ssl.create_default_context(
                cafile=os.environ.get("YLHB_CLOUD_CA_FILE") or None
            )
        except OSError:
            self._context = ssl.create_default_context()
            self.enabled = False

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="map-upload-worker", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=self.read_timeout + 1)

    @staticmethod
    def _response(record: Dict[str, Any], created: bool) -> Dict[str, Any]:
        return {
            "task_created": created,
            "task_id": record.get("task_id", ""),
            "status": record.get("status", ""),
            "map_asset_id": record.get("map_asset_id", ""),
            "error": record.get("last_error", ""),
            "content_identity_sha256": record.get("content_identity_sha256", ""),
        }

    def enqueue(self, yaml_path: Path, pgm_path: Path) -> Dict[str, Any]:
        yaml_path, pgm_path = Path(yaml_path), Path(pgm_path)
        yaml_size, pgm_size = yaml_path.stat().st_size, pgm_path.stat().st_size
        if yaml_size > self.yaml_max or pgm_size > self.pgm_max or yaml_size + pgm_size > self.request_max:
            raise ValueError("map files exceed configured upload size limit")
        self.cleanup_snapshots()
        if self._snapshot_bytes() + yaml_size + pgm_size > self.disk_max:
            raise ValueError("map upload snapshot disk limit exceeded")
        yaml_bytes = yaml_path.read_bytes()
        with pgm_path.open("rb") as pgm_source:
            if pgm_source.read(2) != b"P5":
                raise ValueError("map image must be an 8-bit P5 PGM")

        pgm_hash = hashlib.sha256()
        task_id = str(uuid.uuid4())
        temp_dir = Path(tempfile.mkdtemp(prefix=".map-upload-", dir=self.snapshot_root))
        target_dir = self.snapshot_root / task_id
        snapshot_yaml = temp_dir / yaml_path.name
        snapshot_pgm = temp_dir / pgm_path.name
        try:
            snapshot_yaml.write_bytes(yaml_bytes)
            with pgm_path.open("rb") as source, snapshot_pgm.open("wb") as target:
                while chunk := source.read(1024 * 1024):
                    target.write(chunk)
                    pgm_hash.update(chunk)
            pgm_sha = pgm_hash.hexdigest()
            identity = normalized_map_identity(yaml_bytes, pgm_sha)
            identity_sha = content_identity_sha256(identity)
            existing = self.store.map_upload_by_identity(identity_sha)
            if existing:
                if existing["status"] == "FAILED_FINAL":
                    existing_yaml = Path(existing.get("yaml_path", "")) if existing.get("yaml_path") else None
                    existing_pgm = Path(existing.get("pgm_path", "")) if existing.get("pgm_path") else None
                    if not existing_yaml or not existing_pgm or not existing_yaml.is_file() or not existing_pgm.is_file():
                        target_dir = self.snapshot_root / existing["task_id"]
                        shutil.rmtree(target_dir, ignore_errors=True)
                        os.replace(temp_dir, target_dir)
                        self.store.restore_map_upload_snapshot(
                            existing["task_id"],
                            str(target_dir / yaml_path.name),
                            str(target_dir / pgm_path.name),
                        )
                    else:
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    existing = self.store.requeue_map_upload(existing["task_id"])
                elif existing["status"] == "FAILED_RETRYABLE":
                    shutil.rmtree(temp_dir, ignore_errors=True)
                    existing = self.store.requeue_map_upload(existing["task_id"])
                else:
                    shutil.rmtree(temp_dir, ignore_errors=True)
                self._wake.set()
                self.cleanup_snapshots()
                return self._response(existing, False)

            os.replace(temp_dir, target_dir)
            record = self.store.create_map_upload({
                "task_id": task_id,
                "idempotency_key": str(uuid.uuid4()),
                "content_identity_sha256": identity_sha,
                "identity": identity,
                "yaml_sha256": hashlib.sha256(yaml_bytes).hexdigest(),
                "pgm_sha256": pgm_sha,
                "yaml_path": str(target_dir / yaml_path.name),
                "pgm_path": str(target_dir / pgm_path.name),
            })
        except Exception:
            shutil.rmtree(temp_dir, ignore_errors=True)
            if target_dir.exists() and self.store.map_upload(task_id) is None:
                shutil.rmtree(target_dir, ignore_errors=True)
            raise
        self._wake.set()
        self.cleanup_snapshots()
        return self._response(record, True)

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.enabled or not self.base_url or not self.token:
                self._wake.wait(60)
                self._wake.clear()
                continue
            record = self.store.next_due_map_upload()
            if record is None:
                self._wake.wait(30)
                self._wake.clear()
                continue
            self._process(record)

    def _process(self, record: Dict[str, Any]) -> None:
        self.current_task_id = record["task_id"]
        try:
            reply = self._upload(record)
            returned_identity = str(reply.get("contentIdentitySha256", ""))
            if returned_identity != record["content_identity_sha256"]:
                raise MapUploadError("platform returned a different content identity", retryable=False)
            map_asset_id = str(reply.get("mapAssetId", ""))
            if not map_asset_id:
                raise MapUploadError("platform response has no mapAssetId", retryable=False)
            self.store.finish_map_upload(
                record["task_id"], "SUCCEEDED", map_asset_id=map_asset_id
            )
        except MapUploadError as exc:
            if exc.retryable:
                retry_count = int(record.get("retry_count", 0)) + 1
                delay = exc.retry_after
                if delay is None:
                    delay = min(self.retry_max, self.retry_base * (2 ** min(retry_count - 1, 10)))
                self.store.finish_map_upload(
                    record["task_id"], "FAILED_RETRYABLE",
                    error=str(exc), next_retry_at=time.time() + delay,
                )
            else:
                self.store.finish_map_upload(
                    record["task_id"], "FAILED_FINAL", error=str(exc)
                )
        except Exception as exc:
            delay = min(self.retry_max, self.retry_base * (2 ** min(int(record.get("retry_count", 0)), 10)))
            self.store.finish_map_upload(
                record["task_id"], "FAILED_RETRYABLE",
                error=f"upload failed: {type(exc).__name__}",
                next_retry_at=time.time() + delay,
            )
            LOG.warning("map upload failed: %s", type(exc).__name__)
        finally:
            self.current_task_id = ""
            self.cleanup_snapshots()

    @staticmethod
    def _field(name: str, value: str, boundary: str) -> bytes:
        return (
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n"
            f"{value}\r\n"
        ).encode("utf-8")

    @staticmethod
    def _file_header(name: str, filename: str, content_type: str, boundary: str) -> bytes:
        return (
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"; "
            f"filename=\"{filename}\"\r\nContent-Type: {content_type}\r\n\r\n"
        ).encode("utf-8")

    def _upload(self, record: Dict[str, Any]) -> Dict[str, Any]:
        parsed = urllib.parse.urlsplit(self.base_url)
        if parsed.scheme != "https" or not parsed.hostname:
            raise MapUploadError("cloud URL must use HTTPS", retryable=False)
        yaml_path, pgm_path = Path(record["yaml_path"]), Path(record["pgm_path"])
        if not yaml_path.is_file() or not pgm_path.is_file():
            raise MapUploadError("map upload snapshot is missing", retryable=False)
        boundary = f"ylhb-{uuid.uuid4().hex}"
        parts = [
            self._field(
                "capturedAt",
                datetime.fromtimestamp(
                    float(record["created_at"]), timezone.utc
                ).isoformat(),
                boundary,
            ),
            self._field("contentIdentitySha256", record["content_identity_sha256"], boundary),
            self._field("yamlSha256", record["yaml_sha256"], boundary),
            self._field("pgmSha256", record["pgm_sha256"], boundary),
        ]
        yaml_header = self._file_header("yaml", yaml_path.name, "application/x-yaml", boundary)
        pgm_header = self._file_header("pgm", pgm_path.name, "image/x-portable-graymap", boundary)
        closing = f"\r\n--{boundary}--\r\n".encode("ascii")
        content_length = sum(map(len, parts)) + len(yaml_header) + yaml_path.stat().st_size + 2 + len(pgm_header) + pgm_path.stat().st_size + len(closing)
        if content_length > self.request_max:
            raise MapUploadError("multipart request exceeds configured size limit", retryable=False)
        connection_class = http.client.HTTPSConnection if parsed.scheme == "https" else http.client.HTTPConnection
        kwargs = {"timeout": self.connect_timeout}
        if parsed.scheme == "https":
            kwargs["context"] = self._context
        connection = connection_class(parsed.hostname, parsed.port, **kwargs)
        path = f"{parsed.path.rstrip('/')}/robot-api/v1/map-assets"
        try:
            connection.putrequest("POST", path)
            connection.putheader("Authorization", f"Bearer {self.token}")
            connection.putheader("Idempotency-Key", record["idempotency_key"])
            connection.putheader("Content-Type", f"multipart/form-data; boundary={boundary}")
            connection.putheader("Content-Length", str(content_length))
            connection.endheaders()
            for part in parts:
                connection.send(part)
            connection.send(yaml_header)
            self._send_file(connection, yaml_path)
            connection.send(b"\r\n")
            connection.send(pgm_header)
            self._send_file(connection, pgm_path)
            connection.send(closing)
            if connection.sock:
                connection.sock.settimeout(self.read_timeout)
            response = connection.getresponse()
            body = response.read(1024 * 1024 + 1)
        except (OSError, TimeoutError, http.client.HTTPException) as exc:
            raise MapUploadError(f"network upload failed: {type(exc).__name__}", retryable=True) from exc
        finally:
            connection.close()
        if len(body) > 1024 * 1024:
            raise MapUploadError("platform response is too large", retryable=False)
        retry_after = None
        try:
            retry_after = min(self.retry_max, max(0.0, float(response.getheader("Retry-After", ""))))
        except ValueError:
            pass
        if response.status == 429 or response.status >= 500:
            raise MapUploadError(f"platform HTTP {response.status}", retryable=True, retry_after=retry_after)
        if response.status >= 400:
            raise MapUploadError(f"platform HTTP {response.status}", retryable=False)
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MapUploadError("platform response is invalid JSON", retryable=True) from exc

    @staticmethod
    def _send_file(connection, path: Path) -> None:
        with path.open("rb") as source:
            while chunk := source.read(1024 * 1024):
                connection.send(chunk)

    def cleanup_snapshots(self) -> None:
        records = self.store.map_uploads_for_cleanup()
        now = time.time()
        succeeded = [record for record in records if record["status"] == "SUCCEEDED"]
        for index, record in enumerate(succeeded):
            if index >= self.success_keep or now - float(record["updated_at"]) > self.retention:
                self._delete_snapshot(record)
        if self._snapshot_bytes() <= self.disk_max:
            return
        candidates = sorted(
            (
                record for record in self.store.map_uploads_for_cleanup()
                if record["status"] in {"SUCCEEDED", "FAILED_FINAL"}
                and record.get("yaml_path")
            ),
            key=lambda item: (item["status"] != "SUCCEEDED", item["created_at"]),
        )
        for record in candidates:
            self._delete_snapshot(record)
            if self._snapshot_bytes() <= self.disk_max:
                break

    def _delete_snapshot(self, record: Dict[str, Any]) -> None:
        yaml_path = record.get("yaml_path", "")
        if not yaml_path:
            return
        directory = Path(yaml_path).parent
        try:
            directory.relative_to(self.snapshot_root)
        except ValueError:
            return
        shutil.rmtree(directory, ignore_errors=True)
        self.store.clear_map_upload_snapshot(record["task_id"])

    def _snapshot_bytes(self) -> int:
        return sum(
            path.stat().st_size
            for path in self.snapshot_root.rglob("*")
            if path.is_file()
        )
