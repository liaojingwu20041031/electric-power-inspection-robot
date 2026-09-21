# 辅助传感器传输优先模式

本模式先完成“RDK 采集 → ROS 2 → Jetson 订阅/调用”，不要求外参标定，也不修改 `/cmd_vel`。

## RDK 端

事件相机桥发布：

- `/dg/event_camera/features`：`sensor_msgs/PointCloud2`，字段 `pixel_x`、`pixel_y`、`event_count`、`positive_fraction`；
- `/dg/event_camera/motion_mask`：`sensor_msgs/Image`，`mono8`；
- `/dg/event_camera/quality`：`diagnostic_msgs/DiagnosticArray`；
- `/dg/event_camera/camera_info`：`sensor_msgs/CameraInfo`；
- `/dg/event_camera/aps/image_raw/compressed` 和 `/dg/event_camera/fused/image/compressed`：预览。

完整 EVS 原始流、APS 原图和融合帧继续留在 RDK 本地记录。

毫米波驱动完成后发布：

- `/dg/mmwave/points`：`sensor_msgs/PointCloud2`，`frame_id=mmwave_link`，字段 `x`、`y`、`z`、`radial_velocity`、`snr_db`、`confidence`、`track_id`；
- `/dg/mmwave/quality`：`diagnostic_msgs/DiagnosticArray`。

## Jetson 端

两台设备连接同一局域网后保持：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

启动现有三模型侧链时先使用观察模式：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_hybrid_vision_assist:=true \
  enable_mmwave_observation:=true \
  enable_mmwave_dynamic_mask:=false
```

其中：

- `enable_hybrid_vision_assist:=true`：让 MODEL-2 读取事件相机的质量、特征和运动掩码，并发布 `/dg/event_camera/assist_quality`；
- `enable_mmwave_observation:=true`：订阅毫米波目标并在 `/dg/lidar/quality` 记录收到的数据；
- `enable_mmwave_dynamic_mask:=false`：只调用和记录毫米波，不修改激光扫描。

验证：

```bash
ros2 topic echo /dg/event_camera/quality --once
ros2 topic echo /dg/event_camera/assist_quality --once
ros2 topic echo /dg/mmwave/quality --once
ros2 topic echo /dg/lidar/quality --once
ros2 topic info /dg/event_camera/features -v
ros2 topic info /dg/mmwave/points -v
```

完成外参测量和动态场景验证后，才把 `enable_mmwave_dynamic_mask` 改为 `true`；事件相机也只有在内参、外参和运动补偿条件满足后才允许修改激光结果。


## 传输证据监视器

在英伟纳端可额外开启只读监视器：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_hybrid_vision_assist:=true \
  enable_mmwave_observation:=true \
  enable_mmwave_dynamic_mask:=false \
  enable_auxiliary_transport_monitor:=true
```

监视器发布 `/dg/auxiliary/transport_quality`，用于确认 RDK 的话题已经到达英伟纳，并记录 `GOOD/STALE/UNSEEN`、消息数量、检测数量和 MODEL-2 是否具备观察调用条件。该节点只读传感器话题，不发布 `/cmd_vel`，不修改 `/scan` 或 `/dg/lidar/filtered_scan`。
