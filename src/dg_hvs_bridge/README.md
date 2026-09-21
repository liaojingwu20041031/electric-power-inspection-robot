# dg_hvs_bridge

RDK 运行账号、板端工作区和记录目录通过 `DG_RDK_USER`、`DG_RDK_HOST`、`DG_RDK_WORKSPACE`、`DG_RDK_RECORD_DIR` 环境变量指定；默认账号为 `root`，默认记录目录为 `$HOME/dg202611_rdk_records`。脚本不会保存或读取 SSH 密码。

RDK X5 上的 APX003CC HVS 双流 ROS 2 桥。它只发布相机数据和诊断，不发布运动命令。

构建时必须把 `HV_TOOLKIT_ROOT` 指向官方 `shimetapi_hybrid_vision_toolkit` 根目录。RDK X5 当前没有 `colcon` 时，使用标准 CMake/ament 流程：

```bash
source /opt/tros/humble/setup.bash
export HV_TOOLKIT_ROOT=$HOME/shimetapi_hybrid_vision_toolkit
cmake -S src/dg_hvs_bridge -B build/dg_hvs_bridge \
  -DCMAKE_BUILD_TYPE=Release \
  -DHV_TOOLKIT_ROOT="$HV_TOOLKIT_ROOT"
cmake --build build/dg_hvs_bridge -j2
cmake --install build/dg_hvs_bridge --prefix "$HOME/dg202611_hvs_ws/install"

# 标准 CMake 安装不会自动生成工作区 setup.bash；运行时显式加入安装前缀和 Toolkit 库
export AMENT_PREFIX_PATH="$HOME/dg202611_hvs_ws/install:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:${LD_LIBRARY_PATH:-}"
```

有完整 ROS 2 工作区时也可以用 `colcon build --packages-select dg_hvs_bridge --cmake-args -DHV_TOOLKIT_ROOT=$HV_TOOLKIT_ROOT`；两种方式得到的是同一个桥接目标。

默认参数保留当前板端已验证的 HVS 配置：`sensor_index=52`、`i2c_bus=6`、`evs_fps=240`。这里的索引是板端传感器配置索引，不等同于 EVS/APS 的 VC 编号；EVS/APS 的 VC1 配对由官方 `Backend::MipiHvs` 完成。官方文档中的 `0/1` 是通用默认值，不能在本板上未经验证直接替换已工作的配置。

桥接会记录 APS 的实际格式、尺寸和字节数，并输出 `aps_packets`、`aps_rate_per_sec`、`aps_decode_failures`、`get_frame_timeouts` 等诊断字段。这样可以区分“后端没有取到 APS”“取到了但格式未支持”和“已经有 APS 但融合配对未通过”。目前支持 NV12、Gray8、RAW8、BayerRG8、RGB888，以及明确的 16 位 unpacked RAW10；没有猜测 packed RAW10 的布局。

`recording_enabled:=true` 时在 RDK 本地保存 RAW8、EVT2、事件 CSV、APS PGM、融合 JPEG、配置和运行统计。结束后使用 `run_hvs_capture_session.sh` 生成 SHA-256 清单。



## 三路画面采集

若只需要查看和回放 EVS、APS 以及 EVS+APS 融合画面，可使用 `run_three_view_capture.sh`。脚本启动同一 HVS 桥，并用 rosbag 保存：

- `/dg/event_camera/motion_mask`：EVS 事件活动画面；
- `/dg/event_camera/aps/image_raw/compressed`：APS 压缩画面；
- `/dg/event_camera/fused/image/compressed`：EVS+APS 融合画面。

默认采集 30 秒，输出到 `$HOME/dg202611_hvs_bags/three_views_<UTC>`，并记录环境、命令和 SHA-256 清单。设置 `DG_HVS_RECORD_RAW=true` 会额外保存 RAW8、EVT2、事件 CSV、APS PGM 和融合 JPEG。脚本不会启动底盘、Nav2 或发布 `/cmd_vel`。
## features 话题

参数文件默认启用桥接内置的 `/dg/event_camera/features` 发布器。它使用与现有 `event_feature_node` 相同的 PointCloud2 字段：`pixel_x`、`pixel_y`、`event_count`、`positive_fraction`。内置聚合同时按 `feature_min_events_per_cell` 和 `feature_min_unique_pixels` 过滤，避免单个热像素重复触发被当作特征。桥接还会为每个特征包发布 `/dg/event_camera/feature_quality`（`DiagnosticArray`），其中包含 `feature_count`、`event_count`、按候选特征聚合的 `unique_pixel_count`、`repeatability_status`、`repeatability`、`frame_id`、`observation_only=true` 和 `publishes_cmd_vel=false`。重复性使用相邻包的一对一像素距离匹配，距离阈值由 `feature_match_distance_px` 控制；首包为 `NO_PREVIOUS_PACKET`，已有前一包但任一包无特征时为 `ZERO_FEATURE_PAIR`，只有两包都有特征才为 `COMPARED`。如果仍运行 `run_rdk_event_feature_sidecar.sh`，请把桥接参数 `publish_features` 设为 `false`，避免两个节点同时发布 `/dg/event_camera/features` 和 `/dg/event_camera/feature_quality`；sidecar 仅用于桥接内置特征关闭时的替代路径。


## 当前 RDK 实测状态（2026-09-12）

厂商底层 `dual_vc_vin -c 1 -d 0` 已在 RDK 上持续输出 EVS 约 208.5–209.0 FPS、APS 约 29.79–29.85 FPS。DG 桥接已切换到官方 HBN 双 VC 采集：EVS 与 APS 独立取帧，APS 实际为 RDK `MEM_PIX_FMT_NV12`（1632×1224，两 plane），质量诊断可达到 `GOOD`，`pair_valid=true`，APS 源约 30 Hz，融合图约 10 Hz，EVS/APS 取帧错误为 0。

旧的 Toolkit `Backend::MipiHvs` APS 失败记录仍作为故障排查证据保留；当前默认 RDK 配置使用 `backend: hbn`。这只证明 RDK 本地 ROS 双流桥通过，尚未证明机器人 Jetson 已跨板消费事件数据。机器人端联调仍需单独完成。
