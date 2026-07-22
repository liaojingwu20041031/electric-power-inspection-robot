"""Durable, single-threaded upload of completed 3D point-cloud assets."""
import hashlib
import http.client
import json
import logging
import os
import random
import shutil
import ssl
import tempfile
import threading
import time
import urllib.parse
import uuid
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
from pathlib import Path
from typing import Any, Dict


LOG = logging.getLogger(__name__)
TRUE_VALUES = {"1", "true", "yes", "on"}
CHUNK_SIZE = 1024 * 1024
RESPONSE_MAX_BYTES = 1024 * 1024


class SceneUploadError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        retryable: bool,
        credential_blocked: bool = False,
        retry_after: float | None = None,
    ):
        super().__init__(message)
        self.retryable = retryable
        self.credential_blocked = credential_blocked
        self.retry_after = retry_after


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


def _utc_text(value: Any) -> str:
    if value in (None, ""):
        return ""
    if isinstance(value, (int, float)):
        return datetime.fromtimestamp(float(value), timezone.utc).isoformat()
    return str(value)


class SceneUploadWorker:
    def __init__(
        self,
        store,
        robot_id: str,
        *,
        allowed_root: Path | str | None = None,
    ):
        self.store = store
        self.robot_id = robot_id
        self.base_url = os.environ.get("YLHB_CLOUD_BASE_URL", "").rstrip("/")
        self.token = os.environ.get("YLHB_CLOUD_ROBOT_TOKEN", "")
        self.enabled = (
            os.environ.get("YLHB_SCENE_UPLOAD_ENABLED", "true").strip().lower()
            in TRUE_VALUES
        )
        configured_root = (
            allowed_root
            if allowed_root is not None
            else os.environ.get("YLHB_SCENE_UPLOAD_ALLOWED_ROOT")
            or "/home/nvidia/ros2_DL/runs/3d_reconstruct"
        )
        self.allowed_root = Path(configured_root).expanduser().resolve()
        self.model_max = _env_int(
            "YLHB_SCENE_UPLOAD_MODEL_MAX_BYTES", 1024 * 1024 * 1024
        )
        self.metadata_max = _env_int(
            "YLHB_SCENE_UPLOAD_METADATA_MAX_BYTES", 1024 * 1024
        )
        self.request_max = _env_int(
            "YLHB_SCENE_UPLOAD_REQUEST_MAX_BYTES", 1080000000
        )
        self.connect_timeout = _env_float(
            "YLHB_SCENE_UPLOAD_CONNECT_TIMEOUT_SEC", 10.0
        )
        self.read_timeout = _env_float(
            "YLHB_SCENE_UPLOAD_READ_TIMEOUT_SEC", 1800.0
        )
        self.retry_base = _env_float("YLHB_SCENE_UPLOAD_RETRY_BASE_SEC", 5.0)
        self.retry_max = _env_float("YLHB_SCENE_UPLOAD_RETRY_MAX_SEC", 1800.0)
        self.disk_max = _env_int(
            "YLHB_SCENE_UPLOAD_SNAPSHOT_MAX_BYTES", 5 * 1024 * 1024 * 1024
        )
        self.retention = _env_float(
            "YLHB_SCENE_UPLOAD_SUCCESS_RETENTION_SEC", 7 * 86400.0
        )
        self.success_keep = _env_int("YLHB_SCENE_UPLOAD_SUCCESS_KEEP", 2)
        self.snapshot_root = self.store.root / "scene_upload_snapshots"
        self.snapshot_root.mkdir(parents=True, exist_ok=True)
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._enqueue_lock = threading.Lock()
        self.current_task_id = ""
        self._configuration_error = ""
        try:
            self._context = ssl.create_default_context(
                cafile=os.environ.get("YLHB_CLOUD_CA_FILE") or None
            )
        except OSError as exc:
            self._context = ssl.create_default_context()
            self._configuration_error = f"cloud CA file is invalid: {exc}"
        self._cleanup_startup_orphans()

    def _cleanup_startup_orphans(self) -> None:
        for directory in self.snapshot_root.iterdir():
            if not directory.is_dir():
                continue
            if (
                directory.name.startswith(".scene-upload-")
                or self.store.scene_upload(directory.name) is None
            ):
                shutil.rmtree(directory, ignore_errors=True)
                if directory.exists():
                    LOG.warning(
                        "failed to remove orphan scene snapshot directory=%s",
                        directory,
                    )

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="scene-upload-worker", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=self.read_timeout + 1)

    @staticmethod
    def _response(record: Dict[str, Any], created: bool = False) -> Dict[str, Any]:
        return {
            "task_created": created,
            "task_id": record.get("task_id", ""),
            "source_reconstruct_session_id": record.get(
                "source_reconstruct_session_id", ""
            ),
            "status": record.get("status", ""),
            "retry_count": int(record.get("retry_count", 0)),
            "next_retry_at": float(record.get("next_retry_at", 0.0)),
            "scene_asset_id": record.get("scene_asset_id", ""),
            "last_error": record.get("last_error", ""),
            "model_sha256": record.get("model_sha256", ""),
        }

    def status(self, task_id: str) -> Dict[str, Any]:
        record = self.store.scene_upload(str(task_id))
        return self._response(record) if record else {}

    def status_for_session(self, session_id: str) -> Dict[str, Any]:
        record = self.store.scene_upload_by_session(str(session_id))
        return self._response(record) if record else {}

    def list_status(self, limit: int = 20) -> list[Dict[str, Any]]:
        return [self._response(record) for record in self.store.scene_uploads(limit)]

    def _source_path(self, value: Path, label: str) -> Path:
        try:
            path = Path(value).expanduser().resolve(strict=True)
        except OSError as exc:
            raise ValueError(f"{label} does not exist") from exc
        if not path.is_file():
            raise ValueError(f"{label} must be a regular file")
        try:
            path.relative_to(self.allowed_root)
        except ValueError as exc:
            raise ValueError(f"{label} is outside the allowed reconstruction root") from exc
        return path

    def enqueue(self, model_path: Path, metadata_path: Path) -> Dict[str, Any]:
        if not self.enabled:
            raise ValueError("scene upload is disabled")
        model = self._source_path(model_path, "model file")
        metadata_file = self._source_path(metadata_path, "metadata file")
        if model.suffix.lower() != ".ply":
            raise ValueError("scene model must be a PLY file")
        model_size, metadata_size = model.stat().st_size, metadata_file.stat().st_size
        if (
            model_size <= 0 or model_size > self.model_max
            or metadata_size <= 0 or metadata_size > self.metadata_max
            or model_size + metadata_size > self.request_max
        ):
            raise ValueError("scene files exceed configured upload size limit")
        try:
            metadata_bytes = metadata_file.read_bytes()
            metadata = json.loads(metadata_bytes.decode("utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("scene metadata is invalid JSON") from exc
        if not isinstance(metadata, dict) or metadata.get("state") != "succeeded":
            raise ValueError("scene metadata state must be succeeded")
        try:
            declared_output = Path(str(metadata.get("output_file") or "")).expanduser().resolve(
                strict=True
            )
        except OSError as exc:
            raise ValueError("scene metadata output_file is invalid") from exc
        if declared_output != model:
            raise ValueError("scene metadata output_file does not match the model")
        if metadata.get("file_size_bytes") not in (None, model_size):
            raise ValueError("scene model size does not match metadata")
        asset_kind = str(metadata.get("asset_kind") or "POINT_CLOUD").upper()
        model_format = str(metadata.get("format") or "PLY").upper()
        if asset_kind != "POINT_CLOUD" or model_format != "PLY":
            raise ValueError("scene metadata must describe a PLY point cloud")
        session_id = str(metadata.get("session_id") or "").strip()
        if not session_id:
            raise ValueError("scene metadata has no reconstruction session_id")
        with self._enqueue_lock:
            return self._create_snapshot_task(
                model, metadata_bytes, metadata, session_id,
                asset_kind, model_format, intent_number=0,
            )

    def _create_snapshot_task(
        self,
        model: Path,
        metadata_bytes: bytes,
        metadata: Dict[str, Any],
        session_id: str,
        asset_kind: str,
        model_format: str,
        *,
        intent_number: int,
        supersedes_task_id: str = "",
    ) -> Dict[str, Any]:
        source_stat = model.stat()
        required_bytes = source_stat.st_size + len(metadata_bytes)
        self.cleanup_snapshots(required_bytes)
        if self._snapshot_bytes() + required_bytes > self.disk_max:
            raise ValueError("scene upload snapshot disk limit exceeded")
        task_id = str(uuid.uuid4())
        temp_dir = Path(tempfile.mkdtemp(prefix=".scene-upload-", dir=self.snapshot_root))
        target_dir = self.snapshot_root / task_id
        snapshot_model = temp_dir / "pointcloud.ply"
        snapshot_metadata = temp_dir / "metadata.json"
        digest = hashlib.sha256()
        try:
            with model.open("rb") as source, snapshot_model.open("wb") as target:
                while chunk := source.read(CHUNK_SIZE):
                    target.write(chunk)
                    digest.update(chunk)
                target.flush()
                os.fsync(target.fileno())
            with snapshot_metadata.open("wb") as target:
                target.write(metadata_bytes)
                target.flush()
                os.fsync(target.fileno())
            final_source_stat = model.stat()
            if (
                final_source_stat.st_size != source_stat.st_size
                or final_source_stat.st_mtime_ns != source_stat.st_mtime_ns
            ):
                raise ValueError("scene model changed while creating the upload snapshot")
            model_sha256 = digest.hexdigest()
            declared_hash = str(metadata.get("model_sha256") or "").lower()
            if declared_hash and declared_hash != model_sha256:
                raise ValueError("scene model SHA-256 does not match metadata")
            existing_session = self.store.scene_upload_by_session(session_id)
            if existing_session and existing_session["model_sha256"] != model_sha256:
                raise ValueError("reconstruction session already has a different model hash")
            existing = self.store.scene_upload_by_identity(session_id, model_sha256)
            if existing and not supersedes_task_id:
                shutil.rmtree(temp_dir, ignore_errors=True)
                return self._response(existing, False)
            os.replace(temp_dir, target_dir)
            record = self.store.create_scene_upload({
                "task_id": task_id,
                "idempotency_key": str(uuid.uuid4()),
                "source_reconstruct_session_id": session_id,
                "source_capture_session_id": str(
                    metadata.get("source_capture_session_id") or ""
                ),
                "model_sha256": model_sha256,
                "intent_number": intent_number,
                "model_path": str(target_dir / snapshot_model.name),
                "metadata_path": str(target_dir / snapshot_metadata.name),
                "metadata_json": json.dumps(
                    metadata, ensure_ascii=False, sort_keys=True, allow_nan=False
                ),
                "file_size": (target_dir / snapshot_model.name).stat().st_size,
                "asset_kind": asset_kind,
                "format": model_format,
                "supersedes_task_id": supersedes_task_id,
            })
        except Exception:
            shutil.rmtree(temp_dir, ignore_errors=True)
            if target_dir.exists() and self.store.scene_upload(task_id) is None:
                shutil.rmtree(target_dir, ignore_errors=True)
            raise
        blocked_status, blocked_error = self._configuration_block()
        if blocked_status:
            record = self.store.finish_scene_upload(
                record["task_id"], blocked_status, error=blocked_error,
            )
        self._wake.set()
        self.cleanup_snapshots()
        return self._response(record, True)

    def retry(self, task_id: str) -> Dict[str, Any]:
        with self._enqueue_lock:
            record = self.store.scene_upload(str(task_id))
            if record is None:
                raise ValueError("scene upload task does not exist")
            if record["status"] in {"FAILED_RETRYABLE", "CREDENTIAL_BLOCKED"}:
                retried = self.store.requeue_scene_upload(record["task_id"])
                self._wake.set()
                return self._response(retried, False)
            if record["status"] != "FAILED_FINAL":
                return self._response(record, False)
            latest = self.store.scene_upload_by_identity(
                record["source_reconstruct_session_id"], record["model_sha256"]
            )
            if latest and latest["task_id"] != record["task_id"]:
                return self._response(latest, False)
            model = Path(record["model_path"])
            metadata_file = Path(record["metadata_path"])
            if not model.is_file() or not metadata_file.is_file():
                raise ValueError("scene upload snapshot is missing")
            metadata_bytes = metadata_file.read_bytes()
            metadata = json.loads(record["metadata_json"])
            return self._create_snapshot_task(
                model, metadata_bytes, metadata,
                record["source_reconstruct_session_id"], record["asset_kind"],
                record["format"], intent_number=int(record["intent_number"]) + 1,
                supersedes_task_id=record["task_id"],
            )

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.enabled:
                self._wake.wait(60)
                self._wake.clear()
                continue
            record = self.store.next_due_scene_upload()
            if record is None:
                self._wake.wait(30)
                self._wake.clear()
                continue
            blocked_status, blocked_error = self._configuration_block()
            if blocked_status:
                self.store.finish_scene_upload(
                    record["task_id"], blocked_status, error=blocked_error,
                )
                continue
            self._process(record)

    def _configuration_block(self) -> tuple[str, str]:
        if self._configuration_error:
            return "FAILED_FINAL", self._configuration_error
        if not self.token:
            return "CREDENTIAL_BLOCKED", "cloud robot token is not configured"
        parsed = urllib.parse.urlsplit(self.base_url)
        if parsed.scheme != "https" or not parsed.hostname:
            return "FAILED_FINAL", "cloud URL must use HTTPS"
        return "", ""

    def _process(self, record: Dict[str, Any]) -> None:
        self.current_task_id = record["task_id"]
        try:
            reply = self._upload(record)
            returned_hash = str(reply.get("modelSha256") or "").lower()
            if returned_hash and returned_hash != record["model_sha256"]:
                raise SceneUploadError(
                    "platform returned a different modelSha256", retryable=False
                )
            scene_asset_id = str(reply.get("sceneAssetId") or "")
            if not scene_asset_id:
                raise SceneUploadError(
                    "platform response has no sceneAssetId", retryable=False
                )
            self.store.finish_scene_upload(
                record["task_id"], "SUCCEEDED", scene_asset_id=scene_asset_id
            )
        except SceneUploadError as exc:
            if exc.credential_blocked:
                status, next_retry = "CREDENTIAL_BLOCKED", 0.0
            elif exc.retryable:
                retry_count = int(record.get("retry_count", 0)) + 1
                delay = exc.retry_after
                if delay is None:
                    delay = min(
                        self.retry_max,
                        self.retry_base * (2 ** min(retry_count - 1, 10))
                        * random.uniform(1.0, 1.2),
                    )
                status, next_retry = "FAILED_RETRYABLE", time.time() + delay
            else:
                status, next_retry = "FAILED_FINAL", 0.0
            self.store.finish_scene_upload(
                record["task_id"], status, error=str(exc),
                next_retry_at=next_retry,
            )
        except Exception as exc:
            retry_count = int(record.get("retry_count", 0)) + 1
            delay = min(
                self.retry_max,
                self.retry_base * (2 ** min(retry_count - 1, 10))
                * random.uniform(1.0, 1.2),
            )
            self.store.finish_scene_upload(
                record["task_id"], "FAILED_RETRYABLE",
                error=f"upload failed: {type(exc).__name__}",
                next_retry_at=time.time() + delay,
            )
            LOG.warning(
                "scene upload failed task_id=%s error=%s",
                record.get("task_id", ""), type(exc).__name__,
            )
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
            raise SceneUploadError("cloud URL must use HTTPS", retryable=False)
        model_path, metadata_path = Path(record["model_path"]), Path(record["metadata_path"])
        if not model_path.is_file() or not metadata_path.is_file():
            raise SceneUploadError("scene upload snapshot is missing", retryable=False)
        metadata = json.loads(record["metadata_json"])
        boundary = f"ylhb-{uuid.uuid4().hex}"
        parts = [
            self._field("modelSha256", record["model_sha256"], boundary),
            self._field("assetKind", record["asset_kind"], boundary),
            self._field("format", record["format"], boundary),
            self._field(
                "sourceSessionId", record["source_reconstruct_session_id"], boundary
            ),
            self._field(
                "capturedAt",
                _utc_text(metadata.get("captured_at") or metadata.get("created_at")),
                boundary,
            ),
            self._field(
                "reconstructedAt", _utc_text(metadata.get("finished_at")), boundary
            ),
            self._field(
                "coordinateSystem", str(metadata.get("coordinate_system") or ""), boundary
            ),
            self._field("unit", str(metadata.get("unit") or ""), boundary),
            self._field(
                "pointCount", str(metadata.get("export_point_count") or ""), boundary
            ),
        ]
        model_header = self._file_header(
            "model", model_path.name, "application/octet-stream", boundary
        )
        metadata_header = self._file_header(
            "metadata", metadata_path.name, "application/json", boundary
        )
        closing = f"\r\n--{boundary}--\r\n".encode("ascii")
        content_length = (
            sum(map(len, parts)) + len(model_header) + model_path.stat().st_size + 2
            + len(metadata_header) + metadata_path.stat().st_size + len(closing)
        )
        if content_length > self.request_max:
            raise SceneUploadError(
                "multipart request exceeds configured size limit", retryable=False
            )
        connection = http.client.HTTPSConnection(
            parsed.hostname, parsed.port, timeout=self.connect_timeout,
            context=self._context,
        )
        path = f"{parsed.path.rstrip('/')}/robot-api/v1/scene-assets"
        try:
            connection.putrequest("POST", path)
            connection.putheader("Authorization", f"Bearer {self.token}")
            connection.putheader("Idempotency-Key", record["idempotency_key"])
            connection.putheader(
                "Content-Type", f"multipart/form-data; boundary={boundary}"
            )
            connection.putheader("Content-Length", str(content_length))
            connection.endheaders()
            for part in parts:
                connection.send(part)
            connection.send(model_header)
            self._send_file(connection, model_path)
            connection.send(b"\r\n")
            connection.send(metadata_header)
            self._send_file(connection, metadata_path)
            connection.send(closing)
            if connection.sock:
                connection.sock.settimeout(self.read_timeout)
            response = connection.getresponse()
            body = response.read(RESPONSE_MAX_BYTES + 1)
        except ssl.SSLCertVerificationError as exc:
            raise SceneUploadError(
                "cloud TLS certificate verification failed", retryable=False
            ) from exc
        except (OSError, TimeoutError, http.client.HTTPException) as exc:
            raise SceneUploadError(
                f"network upload failed: {type(exc).__name__}", retryable=True
            ) from exc
        finally:
            connection.close()
        if len(body) > RESPONSE_MAX_BYTES:
            raise SceneUploadError("platform response is too large", retryable=False)
        retry_after = None
        try:
            retry_after = min(
                self.retry_max,
                max(0.0, float(response.getheader("Retry-After", ""))),
            )
        except (TypeError, ValueError):
            try:
                retry_at = parsedate_to_datetime(
                    response.getheader("Retry-After", "")
                )
                retry_after = min(
                    self.retry_max,
                    max(0.0, retry_at.timestamp() - time.time()),
                )
            except (TypeError, ValueError, OverflowError):
                pass
        if response.status == 401:
            raise SceneUploadError(
                "platform HTTP 401", retryable=False, credential_blocked=True
            )
        if response.status == 429 or response.status >= 500:
            raise SceneUploadError(
                f"platform HTTP {response.status}", retryable=True,
                retry_after=retry_after,
            )
        if response.status not in {200, 201}:
            raise SceneUploadError(
                f"platform HTTP {response.status}", retryable=False
            )
        try:
            reply = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise SceneUploadError(
                "platform response is invalid JSON", retryable=True
            ) from exc
        if not isinstance(reply, dict):
            raise SceneUploadError("platform response is invalid JSON", retryable=True)
        return reply

    @staticmethod
    def _send_file(connection, path: Path) -> None:
        with path.open("rb") as source:
            while chunk := source.read(CHUNK_SIZE):
                connection.send(chunk)

    def cleanup_snapshots(self, required_bytes: int = 0) -> None:
        records = self.store.scene_uploads_for_cleanup()
        now = time.time()
        for index, record in enumerate(records):
            if index >= self.success_keep or now - float(record["updated_at"]) > self.retention:
                self._delete_snapshot(record)
        if self._snapshot_bytes() + required_bytes <= self.disk_max:
            return
        for record in reversed(self.store.scene_uploads_for_cleanup()):
            self._delete_snapshot(record)
            if self._snapshot_bytes() + required_bytes <= self.disk_max:
                break

    def _delete_snapshot(self, record: Dict[str, Any]) -> None:
        model_path = record.get("model_path", "")
        if not model_path:
            return
        directory = Path(model_path).parent
        try:
            directory.relative_to(self.snapshot_root)
        except ValueError:
            return
        shutil.rmtree(directory, ignore_errors=True)
        if directory.exists():
            LOG.warning(
                "failed to remove scene upload snapshot task_id=%s directory=%s",
                record.get("task_id", ""), directory,
            )
            return
        self.store.clear_scene_upload_snapshot(record["task_id"])

    def _snapshot_bytes(self) -> int:
        return sum(
            path.stat().st_size
            for path in self.snapshot_root.rglob("*")
            if path.is_file()
        )
