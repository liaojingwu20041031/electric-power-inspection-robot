# Mapping3D Upload Card Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the crowded reconstruction upload row with a readable touch-friendly card while preserving the existing upload behavior.

**Architecture:** Keep all status and command flow unchanged. QML derives presentation from the existing `sceneUploadStatuses`; the backend only gains a clipboard helper if required.

**Tech Stack:** Qt Quick Controls 2, existing PyQt backend and Theme singleton.

---

### Task 1: Reconstruction cards

**Files:**
- Modify: `src/ylhb_llm/qml/pages/Mapping3DPage.qml`

- [ ] Replace each reconstruction `RowLayout` delegate with a white card and nested upload status panel.
- [ ] Show model name, session, point count, formatted file size, reconstruction state, upload state, error, and asset ID.
- [ ] Keep upload/retry/delete enable rules unchanged and make button labels status-specific.

### Task 2: Clipboard action

**Files:**
- Modify only if needed: `src/ylhb_llm/ylhb_llm/ui_backend.py`

- [ ] Reuse an existing clipboard helper if present; otherwise add one `copyText(text)` slot using Qt clipboard APIs.
- [ ] Do not add a clipboard service abstraction or dialog framework.

### Task 3: Focused verification

**Files:**
- Verify the files above.

- [ ] Run `qmllint` for `Mapping3DPage.qml`.
- [ ] Run the existing Mapping3D QML contract test and UI backend test only if backend changes.
- [ ] Build only `ylhb_llm` with testing disabled.
