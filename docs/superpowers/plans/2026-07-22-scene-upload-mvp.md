# 3D Scene Upload MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Persist and reliably upload completed ZED PLY reconstruction assets without changing the existing 2D map upload or navigation behavior.

**Architecture:** Keep reconstruction, orchestration, and cloud upload separate. ZED writes a finalized, hashed asset; Supervisor publishes a ready event; Mobile Bridge validates it, snapshots it, persists one deduplicated task, streams it to the platform, and publishes status consumed by the existing UI status path.

**Tech Stack:** Python stdlib, ROS2 `std_msgs/String`, SQLite, existing PyQt/QML UI, pytest only for deterministic contracts.

---

### Task 1: Final reconstruction metadata

**Files:**
- Modify: `src/ylhb_3d_mapping/ylhb_3d_mapping/zed_svo_tools.py`
- Test: `src/ylhb_3d_mapping/test/test_zed_svo_tools.py`

- [ ] Add a small streaming SHA-256 helper and populate schema `1.1`, asset kind, format, coordinate system, unit, final size, final hash, frame placeholders, and existing point count only after `spatial_map.save()` completes.
- [ ] Extend the existing reconstruction test with one assertion group covering the final file metadata and hash.
- [ ] Run only `pytest -q src/ylhb_3d_mapping/test/test_zed_svo_tools.py`.

### Task 2: Durable scene upload store

**Files:**
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/platform_store.py`
- Test: `src/ylhb_mobile_bridge/test/test_platform_cloud_client.py`

- [ ] Add `scene_uploads` and its two indexes in the existing schema initialization.
- [ ] Add focused CRUD methods for identity lookup, due-task selection, requeue, finalization, status listing, and snapshot cleanup using the existing connection/transaction pattern.
- [ ] Extend the existing store test with one lifecycle covering persistence, dedup identity lookup, retry, and success.
- [ ] Run only the affected store test file.

### Task 3: Independent streaming worker

**Files:**
- Create: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/scene_upload.py`
- Modify: `src/ylhb_mobile_bridge/test/test_platform_cloud_client.py`

- [ ] Implement allowed-root validation, metadata validation, immutable copied snapshot, session/hash deduplication, stable idempotency key, disk quota, retry/restart/status methods, and single-thread lifecycle.
- [ ] Implement HTTPS multipart streaming to `/robot-api/v1/scene-assets`, bounded response reading, response hash/asset-ID validation, credential blocking, permanent error classification, retry-after, and jittered exponential backoff.
- [ ] Add a small deterministic worker test group reusing current HTTP/store test helpers; do not add a new test framework or ROS fake integration suite.
- [ ] Run only the selected worker tests.

### Task 4: ROS lifecycle and automatic trigger

**Files:**
- Modify: `src/ylhb_llm/ylhb_llm/system_supervisor_node.py`
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/platform_api.py`
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/mobile_bridge_server.py`
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/ros_bridge.py`
- Modify: `src/ylhb_mobile_bridge/config/mobile_bridge.yaml`

- [ ] Publish `/inspection_ai/scene_asset_ready` only for a complete successful reconstruction.
- [ ] Initialize/start/stop the scene worker beside existing workers.
- [ ] Subscribe to ready and command topics, validate their JSON contracts, call only worker APIs, and publish `/inspection_ai/scene_upload_status` without credentials.
- [ ] Subscribe Supervisor to upload status and expose it through the existing system-status payload.
- [ ] Add only direct callback contract assertions to an existing ROS bridge test if needed.

### Task 5: Controlled UI actions and status

**Files:**
- Modify: `src/ylhb_llm/ylhb_llm/ui_backend.py`
- Modify: `src/ylhb_llm/qml/pages/Mapping3DPage.qml`

- [ ] Add backend status properties plus enqueue/retry slots that publish controlled session/task commands rather than arbitrary paths.
- [ ] Add compact upload state, retry, and scene asset ID controls to each reconstruction row while preserving current uncommitted visual changes.
- [ ] Avoid brittle layout/text tests; rely on Python syntax/QML loading checks already present.

### Task 6: Focused verification

**Files:**
- Review all files above.

- [ ] Run `python3 -m compileall` only for the three affected Python packages.
- [ ] Run only directly affected pytest files; do not run full `colcon test` or full-repository pytest.
- [ ] Build only `ylhb_3d_mapping`, `ylhb_mobile_bridge`, and `ylhb_llm` with testing disabled if the environment is ready.
- [ ] Inspect `git diff` to confirm no 2D map, Nav2, route, token logging, or unrelated user changes were overwritten.
