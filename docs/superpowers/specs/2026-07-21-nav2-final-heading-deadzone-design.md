# Nav2 末端朝向死区优化设计

## 目标

保持实车已验证的 `0.55 rad/s` 起步及大角度原地旋转能力，同时将目标点末端朝向交给 DWB `RotateToGoal`，避免 Rotation Shim 共用同一速度造成末端对正粗糙。

## 修改范围

同步调整：

- `src/ylhb_base/config/nav2_params.yaml`
- `src/ylhb_base/config/nav2_params_keepout.yaml`
- `src/ylhb_base/test/test_nav2_localization_config.py`

不修改底盘驱动、速度平滑器、规划器、位置容差或恢复行为。

## 参数设计

Rotation Shim 只负责起步：

```yaml
rotate_to_heading_angular_vel: 0.55
max_angular_accel: 0.80
rotate_to_goal_heading: False
```

DWB 负责末端纯旋转，并避开已知无效的小角速度：

```yaml
min_speed_xy: 0.01
min_speed_theta: 0.50
vtheta_samples: 14
RotateToGoal.scale: 20.0
RotateToGoal.slowing_factor: 5.0
RotateToGoal.lookahead_time: 0.4
```

在 `max_vel_theta: 0.65` 下，14 个均匀样本包含约 `0.55 rad/s`。`min_speed_xy: 0.01` 使 Nav2 1.1.20 的联合最小速度检查能在零平移时过滤低于 `0.50 rad/s` 的角速度；正常带平移路径跟踪仍可使用小角速度修正。

目标检查器只收紧朝向：

```yaml
xy_goal_tolerance: 0.08
yaw_goal_tolerance: 0.05
stateful: True
```

## 验证

静态验证只运行现有 Nav2 配置测试和 `ylhb_base` 定向构建，不新增测试文件、不运行全仓库测试。

实机验收由用户现场执行，依次确认：

1. 大角度起步仍能可靠原地旋转。
2. 普通导航和禁行区导航均能完成最终朝向。
3. 最终朝向误差进入 `0.05 rad`，且无低速空转、往复振荡或超时。
4. 若失败，回传 `/cmd_vel`、`/odom`、机器人 pose 和 `controller_server` 日志。
