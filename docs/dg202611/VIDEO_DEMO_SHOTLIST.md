# DG-202611 实机视频拍摄清单

视频用于展示真实可见功能；性能数字以同一场景生成的 ROS 2 采集目录为准。每段视频
同时拍到机器人、手机 App 或机器人 UI，并在开头口述场景编号和日期。

## 拍摄前

1. 确认机器人上层电池、底盘驱动电池均有电，急停可立即按下。
2. 确认手机和机器人在同一热点，App 显示本机已连接。
3. 手机或相机固定拍摄机器人运动；另一台设备录机器人屏幕或 UI。不要只录黑屏终端。
4. 在机器人上先启动基础系统和观察模式；采集器不会发布 `/cmd_vel`。

```bash
cd /home/nvidia/dg202611_ws
./scripts/run_real_robot_capture.sh \
  --scenario VIDEO_00_IDLE \
  --duration 120 \
  --operator peng \
  --hardware-notes "视频拍摄前静止基线；上下电池供电；急停可用"
```

看到 `CAPTURE_READY` 后再开始录屏。采集目录在
`results/real_robot/<时间>_VIDEO_00_IDLE/`，保留其中的 rosbag、日志和清单。

## 片段一：底盘和手机控制

视频中依次展示：App 连接、前进约 0.3 m、停止、后退约 0.3 m、左转、右转、再次停止。
每次动作前先说动作名称，速度保持低速，画面始终能看到车轮方向。出现异常立即按急停。

```bash
./scripts/run_real_robot_capture.sh \
  --scenario VIDEO_01_MOBILE_DRIVE \
  --duration 180 \
  --operator peng \
  --hardware-notes "手机控制方向核对；低速短距离；现场看护；急停可用"
```

验收时对照 `/cmd_vel`、`/odom` 和 `/zlac8015d/status`，不要仅凭视频判断方向已修复。

## 片段二：建图、初始位姿和巡逻

视频中展示地图页面、机器人位姿、路线目标点，然后设置初始位姿并点击“启动本地巡检”。
先完成第一个目标点，再继续第二个目标点；到点停留时拍到 UI 的目标编号和状态变化。

```bash
./scripts/run_real_robot_capture.sh \
  --scenario VIDEO_02_PATROL \
  --duration 900 \
  --wait-for-patrol-start \
  --stop-on-patrol-terminal \
  --post-roll-sec 5 \
  --startup-timeout-sec 120 \
  --operator peng \
  --hardware-notes "route_patrol_002；已设置map初始位姿；用户现场启动巡逻"
```

必须保留失败视频和失败目录。成功视频要能看见 `/patrol/status` 从 `running` 到
`succeeded`；失败时保留 Nav2 日志，不能剪掉失败过程后再宣称成功。

## 片段三：三模型和 UWB 观察链

机器人静止拍摄 UI 的模型状态面板，同时在终端记录状态话题。UWB 在画面和口述中标为
“已有数据回放”，不要把回放说成机器人实时 UWB 定位。

```bash
./scripts/run_real_robot_capture.sh \
  --scenario VIDEO_03_THREE_MODEL_UWB \
  --duration 180 \
  --operator peng \
  --hardware-notes "三模型观察链；UWB为2026-09-05实测数据回放；不启用主动运动恢复"
```

视频应展示这些状态有持续更新：`/dg/lidar/quality`、`/dg/fusion/status`、
`/dg/relocalization/status`、`/dg/uwb/quality`。若事件相机只提供辅助质量，也明确说出
“事件相机当前为观察输入，未修改 LiDAR 结果”。

## 片段四：事件相机三视图

在 RDK 上启动 EVS、APS 和融合预览，只展示画面和质量状态，不把预览误称为定位精度。
视频中依次拍 EVS 原始画面、APS 画面、EVS+APS 融合画面，以及 `pair_valid`、
`fused_frames` 和错误计数。

```bash
# RDK 终端：按 RDK 事件相机启动文档启动桥和预览
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# NVIDIA 终端：记录跨板接收和融合状态
cd /home/nvidia/dg202611_ws
./scripts/run_real_robot_capture.sh \
  --scenario VIDEO_04_EVENT_CAMERA \
  --duration 120 \
  --operator peng \
  --hardware-notes "RDK EVS+APS；跨板质量与features接收；DIAGNOSTIC_ONLY"
```

完整事件原始流留在 RDK 本地；跨板只传低带宽特征、质量和必要的压缩预览。

## 文件命名和交付

视频文件使用 `YYYYMMDD_VIDEO_<编号>_<场景>.mp4` 命名。每段视频旁边保存对应采集目录，
并把视频文件复制到该目录后重新生成 SHA-256 清单：

```bash
cd /home/nvidia/dg202611_ws/results/real_robot/<场景目录>
sha256sum video/*.mp4 > video/SHA256SUMS
sha256sum -c manifest.sha256
```

最终交付按“视频 + rosbag + 结构化 CSV + 运行日志 + 配置 + 哈希清单”成套归档。视频
展示功能，日志证明数据来源；两者缺一不可。
