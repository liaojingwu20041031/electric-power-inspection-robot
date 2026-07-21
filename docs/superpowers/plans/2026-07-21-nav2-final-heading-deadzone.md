# Nav2 Final Heading Deadzone Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保持 `0.55 rad/s` 起步旋转基线，并让 DWB 使用可克服底盘死区的纯旋转速度完成 `0.05 rad` 末端朝向。

**Architecture:** Rotation Shim 仅处理新路径起步对齐，DWB `RotateToGoal` 处理目标点末端朝向。普通导航与禁行区导航保持完全相同的控制器参数，现有配置测试保护两套配置一致、纯旋转死区过滤和目标容差范围。

**Tech Stack:** ROS 2 Humble、Nav2 1.1.20、DWB、YAML、pytest

---

### Task 1: 同步末端朝向控制参数

**Files:**
- Modify: `src/ylhb_base/config/nav2_params.yaml:148-226`
- Modify: `src/ylhb_base/config/nav2_params_keepout.yaml:148-221`
- Test: `src/ylhb_base/test/test_nav2_localization_config.py:214-367`

- [ ] **Step 1: 修改现有配置测试以表达死区与末端朝向契约**

在 `test_rotation_shim_handles_large_initial_heading_changes()` 中将末端职责断言改为：

```python
assert follow_path["rotate_to_goal_heading"] is False
```

在 `test_dwb_low_speed_limits_match_velocity_smoother()` 中使用参数关系保护纯旋转速度样本：

```python
assert follow_path["min_speed_xy"] > 0.0
assert 0.50 <= follow_path["min_speed_theta"] <= follow_path["rotate_to_heading_angular_vel"]
assert follow_path["vtheta_samples"] >= 14

theta_step = 2 * follow_path["max_vel_theta"] / (follow_path["vtheta_samples"] - 1)
theta_samples = [
    -follow_path["max_vel_theta"] + index * theta_step
    for index in range(follow_path["vtheta_samples"])
]
assert any(
    follow_path["min_speed_theta"] <= sample <= follow_path["rotate_to_heading_angular_vel"]
    for sample in theta_samples
)
```

在 `test_navigation_goal_tolerances_are_conservative_and_consistent()` 中同步检查两套 goal checker，并允许收紧后的角度范围：

```python
keepout_goal_checker = load_keepout_nav2_params()["controller_server"]["ros__parameters"]["goal_checker"]

assert goal_checker == keepout_goal_checker
assert 0.05 <= goal_checker["yaw_goal_tolerance"] <= 0.08
```

- [ ] **Step 2: 运行定向测试并确认旧配置失败**

Run:

```bash
python3 -m pytest src/ylhb_base/test/test_nav2_localization_config.py::test_rotation_shim_handles_large_initial_heading_changes src/ylhb_base/test/test_nav2_localization_config.py::test_dwb_low_speed_limits_match_velocity_smoother src/ylhb_base/test/test_nav2_localization_config.py::test_navigation_goal_tolerances_are_conservative_and_consistent -q
```

Expected: 三个测试至少有一个失败，分别指出旧配置仍由 Rotation Shim 处理末端朝向、`min_speed_xy` 为零或 `yaw_goal_tolerance` 仍为 `0.08`。

- [ ] **Step 3: 最小修改普通导航和禁行区导航配置**

在两份 YAML 的 `FollowPath` 下同步设置：

```yaml
rotate_to_heading_angular_vel: 0.55
max_angular_accel: 0.80
rotate_to_goal_heading: False
min_speed_xy: 0.01
min_speed_theta: 0.50
vtheta_samples: 14
RotateToGoal.scale: 20.0
RotateToGoal.slowing_factor: 5.0
RotateToGoal.lookahead_time: 0.4
```

在两份 YAML 的 `goal_checker` 下同步设置：

```yaml
xy_goal_tolerance: 0.08
yaw_goal_tolerance: 0.05
stateful: True
```

保留 `max_vel_theta: 0.65`、加减速度、速度平滑器和规划器参数不变，并更新相邻注释，使其说明 Rotation Shim 仅负责起步、DWB 纯旋转过滤无效小速度。

- [ ] **Step 4: 运行现有 Nav2 配置测试**

Run:

```bash
python3 -m pytest src/ylhb_base/test/test_nav2_localization_config.py -q
```

Expected: PASS；不新增测试文件，不运行全仓库 pytest 或 `colcon test`。

- [ ] **Step 5: 定向构建受影响包**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ylhb_base --cmake-args -DBUILD_TESTING=OFF
```

Expected: `ylhb_base` 构建成功。

- [ ] **Step 6: 检查差异并提交**

Run:

```bash
git diff --check
git diff -- src/ylhb_base/config/nav2_params.yaml src/ylhb_base/config/nav2_params_keepout.yaml src/ylhb_base/test/test_nav2_localization_config.py
git add src/ylhb_base/config/nav2_params.yaml src/ylhb_base/config/nav2_params_keepout.yaml src/ylhb_base/test/test_nav2_localization_config.py docs/superpowers/plans/2026-07-21-nav2-final-heading-deadzone.md
git commit -m "fix(nav2): separate startup and final heading rotation"
```

Expected: 仅提交两份配置、一个现有测试文件和本实施计划；不包含 `maps/route_patrol_002.json`。
