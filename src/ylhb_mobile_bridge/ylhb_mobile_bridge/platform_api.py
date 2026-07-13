"""Authenticated Robot Platform Protocol v1 endpoints."""
import json
import os
import uuid
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse

from .platform_store import DeploymentStore, PlatformStoreError


STATE_MAP = {"idle": "idle", "starting": "starting", "running": "running", "paused": "paused", "manual_takeover": "manual_takeover", "returning_home": "returning_home", "waiting_loop": "waiting_loop", "succeeded": "succeeded", "failed": "failed", "canceled": "canceled", "cancelled": "canceled"}


def _error(code: str, message: str, request_id: str = "", details: Any = None, status_code: int = 400) -> JSONResponse:
    body = {"code": code, "message": message, "requestId": request_id}
    if details is not None:
        body["details"] = details
    return JSONResponse(status_code=status_code, content=body)


def attach_platform_api(app: FastAPI, bridge) -> DeploymentStore:
    root = Path(os.environ.get("YLHB_PLATFORM_STORAGE_DIR") or bridge.get_parameter("platform_storage_dir").value or "~/.local/share/ylhb/platform").expanduser()
    store = DeploymentStore(root)
    robot_id = os.environ.get("YLHB_ROBOT_ID") or str(bridge.get_parameter("robot_id").value)
    token = os.environ.get("YLHB_PLATFORM_API_TOKEN") or str(bridge.get_parameter("platform_api_token").value)
    boot_id = str(uuid.uuid4())

    @app.middleware("http")
    async def platform_auth(request: Request, call_next):
        if request.url.path.startswith("/api/platform/v1"):
            value = request.headers.get("authorization", "")
            if not token or value != f"Bearer {token}":
                return _error("AUTH_FAILED", "Bearer token required", status_code=401)
        return await call_next(request)

    @app.exception_handler(PlatformStoreError)
    async def platform_store_error(_request: Request, exc: PlatformStoreError):
        return _error(exc.code, str(exc), status_code=exc.status_code)

    def context() -> Dict[str, Any]:
        status = bridge.patrol_status()
        return status if isinstance(status, dict) else {}

    @app.get("/api/platform/v1/health")
    def health():
        status = bridge.robot_status()
        return {"robotId": robot_id, "bootId": boot_id, "state": STATE_MAP.get(str(context().get("state", "idle")), "idle"), "mapPose": status.get("mapPose"), "odomPose": status.get("odomPose"), "online": True}

    @app.put("/api/platform/v1/deployments/{deployment_id}")
    async def deploy(deployment_id: str, request: Request):
        form = await request.form()
        try:
            manifest = json.loads((await form["manifest"].read()).decode("utf-8"))
            route = await form["route"].read()
            yaml_bytes = await form["yaml"].read()
            pgm = await form["pgm"].read()
        except (KeyError, AttributeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise PlatformStoreError("INVALID_REQUEST", "manifest, route, yaml and pgm are required") from exc
        if manifest.get("robotId") != robot_id:
            raise PlatformStoreError("INVALID_REQUEST", "manifest robotId does not match this robot")
        result = store.install(deployment_id, manifest, route, yaml_bytes, pgm)
        return result

    def execution_response(execution_id: str) -> Dict[str, Any]:
        execution = store.execution(execution_id)
        if not execution:
            raise HTTPException(status_code=404, detail="execution not found")
        state = STATE_MAP.get(str(context().get("state") or execution["state"]), execution["state"])
        return {"executionId": execution_id, "deploymentId": execution["deployment_id"], "requestId": execution["request_id"], "state": state, "robotId": robot_id, "bootId": boot_id, "mapPose": bridge.robot_status().get("mapPose"), "odomPose": bridge.robot_status().get("odomPose")}

    @app.post("/api/platform/v1/executions/{execution_id}/start", status_code=202)
    async def start(execution_id: str, request: Request):
        body = await request.json()
        deployment_id, request_id = str(body.get("deploymentId", "")), str(body.get("requestId", ""))
        executor_route_id = str(body.get("executorRouteId", ""))
        if not deployment_id or not request_id or not executor_route_id:
            raise PlatformStoreError("INVALID_REQUEST", "deploymentId, executorRouteId and requestId are required")
        deployment = store.deployment(deployment_id)
        if not deployment:
            raise PlatformStoreError("DEPLOYMENT_NOT_FOUND", "deployment not found", 404)
        execution = store.upsert_execution(execution_id, deployment_id, request_id, "starting")
        directory = Path(deployment["directory"])
        platform_context = {"active_execution_id": execution_id, "active_deployment_id": deployment_id, "active_request_id": request_id, "active_route_revision_id": deployment["manifest"]["routeRevisionId"], "active_route_path": str(directory / "route.json"), "active_map_yaml_path": str(directory / "map.yaml"), "executor_route_id": executor_route_id}
        bridge.set_platform_context(platform_context)
        bridge.publish_system_command("start_platform_patrol", profile=str(body.get("profile") or "inspection"), **platform_context)
        store.append_event({"schema_version": "1.0", "robot_id": robot_id, "boot_id": boot_id, "execution_id": execution_id, "deployment_id": deployment_id, "request_id": request_id, "event": "command_accepted", "state": "starting"})
        return {"accepted": True, "state": "STARTING", "executionId": execution["execution_id"]}

    @app.post("/api/platform/v1/executions/{execution_id}/{action}")
    async def control(execution_id: str, action: str, request: Request):
        commands = {"pause": "pause_patrol", "resume": "resume_patrol", "takeover": "takeover_patrol", "cancel": "cancel_patrol"}
        if action not in commands:
            raise HTTPException(status_code=404, detail="control not found")
        body = await request.json()
        request_id = str(body.get("requestId", ""))
        execution = store.execution(execution_id)
        if not execution:
            raise PlatformStoreError("EXECUTION_NOT_FOUND", "execution not found", 404)
        if not request_id:
            raise PlatformStoreError("INVALID_REQUEST", "requestId is required")
        state = {"pause": "paused", "resume": "running", "takeover": "manual_takeover", "cancel": "canceled"}[action]
        store.upsert_execution(execution_id, execution["deployment_id"], execution["request_id"], state)
        bridge.publish_system_command(commands[action], execution_id=execution_id, request_id=request_id)
        event_name = {"pause": "route_paused", "resume": "route_resumed", "takeover": "manual_takeover", "cancel": "route_canceled"}[action]
        store.append_event({"schema_version": "1.0", "robot_id": robot_id, "boot_id": boot_id, "execution_id": execution_id, "deployment_id": execution["deployment_id"], "request_id": request_id, "event": event_name, "state": state})
        return {"accepted": True, "state": state.upper(), "executionId": execution_id}

    @app.get("/api/platform/v1/executions/{execution_id}")
    def execution_status(execution_id: str):
        return execution_response(execution_id)

    @app.get("/api/platform/v1/events")
    def events(afterSequence: int = 0, limit: int = 100):
        return {"events": store.events(afterSequence, limit)}

    bridge.platform_store = store
    bridge.platform_robot_id = robot_id
    bridge.platform_boot_id = boot_id
    return store
