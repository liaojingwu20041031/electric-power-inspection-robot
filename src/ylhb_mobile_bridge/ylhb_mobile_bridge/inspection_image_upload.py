"""On-demand patrol image encoding and durable cloud upload."""
import hashlib
import http.client
import io
import json
import logging
import os
import ssl
import tempfile
import threading
import time
import urllib.parse
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

from PIL import Image


LOG = logging.getLogger(__name__)
TRUE_VALUES = {"1", "true", "yes", "on"}


class InspectionImageUploadError(RuntimeError):
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


def prepare_inspection_image(
    data: bytes,
    format_hint: str,
    kind: str,
    moving_max_edge: int,
    moving_jpeg_quality: int,
) -> bytes:
    with Image.open(io.BytesIO(data)) as source:
        source.load()
        if kind == "ARRIVAL" and any(
            marker in str(format_hint).lower() for marker in ("jpeg", "jpg")
        ):
            return data
        image = source.convert("RGB")
        if kind == "MOVING":
            limit = max(1, int(moving_max_edge))
            image.thumbnail(
                (limit, limit),
                getattr(getattr(Image, "Resampling", Image), "LANCZOS"),
            )
            quality = max(1, min(95, int(moving_jpeg_quality)))
        else:
            quality = 95
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=quality)
        return output.getvalue()


class InspectionImageUploadWorker:
    def __init__(self, store, robot_id: str):
        self.store = store
        self.robot_id = robot_id
        self.base_url = os.environ.get("YLHB_CLOUD_BASE_URL", "").rstrip("/")
        self.token = os.environ.get("YLHB_CLOUD_ROBOT_TOKEN", "")
        self.enabled = (
            os.environ.get("YLHB_INSPECTION_IMAGE_UPLOAD_ENABLED", "false")
            .strip().lower() in TRUE_VALUES
        )
        self.image_max = _env_int(
            "YLHB_INSPECTION_IMAGE_UPLOAD_MAX_BYTES", 20 * 1024 * 1024
        )
        self.disk_max = _env_int(
            "YLHB_INSPECTION_IMAGE_UPLOAD_SNAPSHOT_MAX_BYTES",
            2 * 1024 * 1024 * 1024,
        )
        self.timeout = _env_float("YLHB_CLOUD_REQUEST_TIMEOUT_SEC", 10.0)
        self.retry_max = _env_float("YLHB_CLOUD_MAX_BACKOFF_SEC", 30.0)
        self.snapshot_root = self.store.root / "inspection_image_upload_snapshots"
        self.snapshot_root.mkdir(parents=True, exist_ok=True)
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self.current_capture_kind = ""
        self._capture_error = ""
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
                target=self._run,
                name="inspection-image-upload-worker",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=self.timeout + 1)

    def capture_allowed(self) -> bool:
        diagnostics = self.store.inspection_image_upload_diagnostics()
        return bool(
            self.enabled and self.base_url and self.token
            and not diagnostics["credential_blocked"]
        )

    def status(self) -> Dict[str, Any]:
        diagnostics = self.store.inspection_image_upload_diagnostics()
        if not self.enabled:
            state = "DISABLED"
        elif not self.base_url or not self.token:
            state = "UNCONFIGURED"
        elif diagnostics["credential_blocked"]:
            state = "CREDENTIAL_BLOCKED"
        elif self.current_capture_kind:
            state = "UPLOADING"
        elif diagnostics["pending_count"]:
            state = "PENDING"
        else:
            state = "IDLE"
        last_success = diagnostics["last_success_at"]
        return {
            "enabled": self.enabled,
            "state": state,
            "pendingCount": diagnostics["pending_count"],
            "failedCount": diagnostics["failed_count"],
            "currentCaptureKind": self.current_capture_kind,
            "lastSuccessAt": (
                datetime.fromtimestamp(last_success, timezone.utc).isoformat()
                if last_success else ""
            ),
            "lastError": self._capture_error or diagnostics["last_error"],
        }

    def note_capture_error(self, error: str) -> None:
        self._capture_error = str(error or "")[:300]

    def clear_capture_error(self) -> None:
        self._capture_error = ""

    @staticmethod
    def _response(record: Dict[str, Any], created: bool) -> Dict[str, Any]:
        return {
            "task_created": created,
            "capture_task_id": record.get("capture_task_id", ""),
            "status": record.get("status", ""),
            "image_id": record.get("image_id", ""),
            "error": record.get("last_error", ""),
        }

    def enqueue(self, request: Dict[str, Any], image_bytes: bytes) -> Dict[str, Any]:
        required = (
            "capture_identity", "task_id", "execution_id", "checkpoint_id",
            "kind", "captured_at",
        )
        if any(not str(request.get(field) or "") for field in required):
            raise ValueError("inspection image capture context is incomplete")
        if request["kind"] not in {"MOVING", "ARRIVAL"}:
            raise ValueError("invalid inspection image capture kind")
        existing = self.store.inspection_image_upload_by_identity(
            str(request["capture_identity"])
        )
        if existing:
            return self._response(existing, False)
        if not self.capture_allowed():
            raise ValueError("inspection image capture is disabled or credential blocked")
        if not image_bytes or len(image_bytes) > self.image_max:
            raise ValueError("inspection image exceeds configured upload size limit")
        self.cleanup_snapshots(len(image_bytes))
        if self._snapshot_bytes() + len(image_bytes) > self.disk_max:
            raise ValueError("inspection image snapshot disk limit exceeded")

        capture_task_id = str(uuid.uuid4())
        target = self.snapshot_root / f"{capture_task_id}.jpg"
        handle = tempfile.NamedTemporaryFile(
            prefix=".inspection-image-", suffix=".tmp",
            dir=self.snapshot_root, delete=False,
        )
        temp_path = Path(handle.name)
        try:
            with handle:
                handle.write(image_bytes)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_path, target)
            record = self.store.create_inspection_image_upload({
                "capture_task_id": capture_task_id,
                "capture_identity": str(request["capture_identity"]),
                "idempotency_key": str(uuid.uuid4()),
                "task_id": str(request["task_id"]),
                "execution_id": str(request["execution_id"]),
                "checkpoint_id": str(request["checkpoint_id"]),
                "capture_kind": str(request["kind"]),
                "captured_at": str(request["captured_at"]),
                "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
                "file_path": str(target),
                "file_size": len(image_bytes),
            })
        except Exception:
            temp_path.unlink(missing_ok=True)
            target.unlink(missing_ok=True)
            raise
        self._wake.set()
        return self._response(record, True)

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.enabled or not self.base_url or not self.token:
                self._wake.wait(60)
                self._wake.clear()
                continue
            record = self.store.next_due_inspection_image_upload()
            if record is None:
                self._wake.wait(30)
                self._wake.clear()
                continue
            self._process(record)

    def _process(self, record: Dict[str, Any]) -> None:
        self.current_capture_kind = str(record.get("capture_kind") or "")
        try:
            reply = self._upload(record)
            for response_field, record_field in (
                ("taskId", "task_id"),
                ("executionId", "execution_id"),
                ("checkpointId", "checkpoint_id"),
            ):
                if str(reply.get(response_field) or "") != str(record[record_field]):
                    raise InspectionImageUploadError(
                        f"platform returned a different {response_field}",
                        retryable=False,
                    )
            image_id = str(reply.get("imageId") or "")
            if not image_id:
                raise InspectionImageUploadError(
                    "platform response has no imageId", retryable=False
                )
            self.store.finish_inspection_image_upload(
                record["capture_task_id"], "SUCCEEDED", image_id=image_id
            )
            Path(record["file_path"]).unlink(missing_ok=True)
            self.store.clear_inspection_image_snapshot(record["capture_task_id"])
        except InspectionImageUploadError as exc:
            if exc.credential_blocked:
                status, next_retry = "CREDENTIAL_BLOCKED", 0.0
            elif exc.retryable:
                retry_count = int(record.get("retry_count", 0)) + 1
                delay = exc.retry_after
                if delay is None:
                    delay = min(self.retry_max, 2 ** min(retry_count - 1, 10))
                status, next_retry = "FAILED_RETRYABLE", time.time() + delay
            else:
                status, next_retry = "FAILED_FINAL", 0.0
            self.store.finish_inspection_image_upload(
                record["capture_task_id"], status,
                error=str(exc), next_retry_at=next_retry,
            )
        except Exception as exc:
            retry_count = int(record.get("retry_count", 0)) + 1
            delay = min(self.retry_max, 2 ** min(retry_count - 1, 10))
            self.store.finish_inspection_image_upload(
                record["capture_task_id"], "FAILED_RETRYABLE",
                error=f"upload failed: {type(exc).__name__}",
                next_retry_at=time.time() + delay,
            )
            LOG.warning(
                "inspection image upload failed capture=%s error=%s",
                record.get("capture_task_id", ""), type(exc).__name__,
            )
        finally:
            self.current_capture_kind = ""

    @staticmethod
    def _field(name: str, value: str, boundary: str) -> bytes:
        return (
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n"
            f"{value}\r\n"
        ).encode("utf-8")

    def _upload(self, record: Dict[str, Any]) -> Dict[str, Any]:
        parsed = urllib.parse.urlsplit(self.base_url)
        if parsed.scheme != "https" or not parsed.hostname:
            raise InspectionImageUploadError(
                "cloud URL must use HTTPS", retryable=False
            )
        image_path = Path(record["file_path"])
        if not image_path.is_file():
            raise InspectionImageUploadError(
                "inspection image snapshot is missing", retryable=False
            )
        boundary = f"ylhb-{uuid.uuid4().hex}"
        parts = [
            self._field("taskId", record["task_id"], boundary),
            self._field("executionId", record["execution_id"], boundary),
            self._field("checkpointId", record["checkpoint_id"], boundary),
            self._field("capturedAt", record["captured_at"], boundary),
            self._field("imageSha256", record["image_sha256"], boundary),
        ]
        file_header = (
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"image\"; "
            f"filename=\"{image_path.name}\"\r\nContent-Type: image/jpeg\r\n\r\n"
        ).encode("utf-8")
        closing = f"\r\n--{boundary}--\r\n".encode("ascii")
        content_length = (
            sum(map(len, parts)) + len(file_header)
            + image_path.stat().st_size + len(closing)
        )
        if content_length > self.image_max + 16 * 1024:
            raise InspectionImageUploadError(
                "multipart request exceeds configured size limit", retryable=False
            )
        connection = http.client.HTTPSConnection(
            parsed.hostname, parsed.port, timeout=self.timeout, context=self._context
        )
        path = f"{parsed.path.rstrip('/')}/robot-api/v1/inspection-images"
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
            connection.send(file_header)
            with image_path.open("rb") as source:
                while chunk := source.read(1024 * 1024):
                    connection.send(chunk)
            connection.send(closing)
            response = connection.getresponse()
            body = response.read(1024 * 1024 + 1)
        except (OSError, TimeoutError, http.client.HTTPException) as exc:
            raise InspectionImageUploadError(
                f"network upload failed: {type(exc).__name__}", retryable=True
            ) from exc
        finally:
            connection.close()
        if len(body) > 1024 * 1024:
            raise InspectionImageUploadError(
                "platform response is too large", retryable=False
            )
        retry_after = None
        try:
            retry_after = min(
                self.retry_max,
                max(0.0, float(response.getheader("Retry-After", ""))),
            )
        except ValueError:
            pass
        if response.status in {401, 403}:
            raise InspectionImageUploadError(
                f"platform HTTP {response.status}", retryable=False,
                credential_blocked=True,
            )
        if response.status == 429 or response.status >= 500:
            raise InspectionImageUploadError(
                f"platform HTTP {response.status}", retryable=True,
                retry_after=retry_after,
            )
        if response.status not in {200, 201}:
            raise InspectionImageUploadError(
                f"platform HTTP {response.status}", retryable=False
            )
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise InspectionImageUploadError(
                "platform response is invalid JSON", retryable=True
            ) from exc

    def cleanup_snapshots(self, required_bytes: int = 0) -> None:
        if self._snapshot_bytes() + required_bytes <= self.disk_max:
            return
        for record in self.store.inspection_image_uploads_for_cleanup():
            path = Path(record.get("file_path") or "")
            try:
                path.relative_to(self.snapshot_root)
            except ValueError:
                continue
            path.unlink(missing_ok=True)
            self.store.clear_inspection_image_snapshot(record["capture_task_id"])
            if self._snapshot_bytes() + required_bytes <= self.disk_max:
                break

    def _snapshot_bytes(self) -> int:
        return sum(
            path.stat().st_size
            for path in self.snapshot_root.iterdir()
            if path.is_file() and not path.name.startswith(".")
        )
