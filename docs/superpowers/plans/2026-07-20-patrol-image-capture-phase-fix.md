# Patrol Image Capture Phase Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish `navigation_phase=target` immediately after Nav2 accepts a patrol checkpoint goal so existing patrol image capture triggers work.

**Architecture:** Keep all existing waiting, sending, retry and upload behavior. Change only the accepted-goal transition in `PatrolExecutorNode`, then protect it with one existing-file regression test.

**Tech Stack:** Python 3, ROS2 Humble, rclpy, pytest

---

### Task 1: Accepted Nav2 Goal Phase

**Files:**
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/patrol_executor_node.py`
- Test: `src/ylhb_mobile_bridge/test/test_patrol_executor_logic.py`

- [ ] **Step 1: Write the failing test**

Add a test using the existing `FakeActionClient`, `FakeGoalHandle`, and `make_timer_node` helpers:

```python
def test_accepted_navigation_goal_publishes_target_phase():
    node = make_timer_node(
        FakeActionClient(ready=True, goal_handles=[FakeGoalHandle()])
    )

    node._request_navigation(
        {"x": 1.0, "y": 2.0, "yaw": 0.0}, 9.0, lambda _ok: None
    )

    assert node._active_navigation["navigation_phase"] == "target"
    assert node.statuses[-1]["navigation_phase"] == "target"
```

- [ ] **Step 2: Run the regression test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_DL/install/setup.bash
pytest -q src/ylhb_mobile_bridge/test/test_patrol_executor_logic.py::test_accepted_navigation_goal_publishes_target_phase
```

Expected: FAIL because the current phase remains `sending_goal`.

- [ ] **Step 3: Implement the minimal transition**

After an accepted goal passes the cancellation check in `_on_goal_response`, publish the semantic target-navigation phase:

```python
context["navigation_phase"] = "target"
self._publish_current_status()
```

- [ ] **Step 4: Verify GREEN and related patrol tests**

Run:

```bash
pytest -q src/ylhb_mobile_bridge/test/test_patrol_executor_logic.py
python3 -m py_compile src/ylhb_mobile_bridge/ylhb_mobile_bridge/patrol_executor_node.py
git diff --check
```

Expected: the patrol executor test file passes, syntax compilation succeeds, and the diff has no whitespace errors.

- [ ] **Step 5: Review the final diff**

Confirm only the accepted-goal phase transition, one regression test, and this plan are changed. Do not start Nav2, patrol, or robot motion.
