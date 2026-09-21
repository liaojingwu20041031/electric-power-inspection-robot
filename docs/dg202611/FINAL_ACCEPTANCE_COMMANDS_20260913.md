# DG-202611 最终融合验收命令清单

**用途**：给现场人员按顺序执行的命令模板。命令分为只读检查、观察模式和用户实机动作；不能把前两类结果当作实机运动或精度指标。

**当前代码基线**：本地开发分支 HEAD 为`0a5ad6b`；`4f3ffb4`是阶段性证据提交。审计记录中的机器人部署快照仍为旧版本`c75d48e`，不代表当前代码已上板。最终发布包按最后本地提交的短 SHA 命名并与旁侧哈希绑定。RDK HBN目标架构已编译通过并已形成真实事件相机采集证据，NVIDIA跨板业务接收已通过；`/dg/auxiliary/transport_quality`仍待新版传输监视器部署。部署新包前只做检查和回放，不打开底盘电源、不发送`/cmd_vel`。

## 1. 两板环境记录（只读）

在 NVIDIA 和 RDK 各开一个终端，执行并保存完整输出：

```bash
whoami
hostname
pwd
ip -br addr
printf 'ROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nRMW_IMPLEMENTATION=%s\n' \
  "${ROS_DOMAIN_ID:-UNSET}" "${ROS_LOCALHOST_ONLY:-UNSET}" "${RMW_IMPLEMENTATION:-UNSET}"
date -Is
timedatectl status --no-pager
```

两板联调前统一设置：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

RDK 的系统时间若仍显示 `2000-01-01`，先由现场按既定网络策略完成校时，再重新记录 `date -Is`；校时前不要填写跨板延迟数值。

## 2. RDK 相机设备检查（只读）

```bash
ls -ld /app/multimedia_samples/sample_apx003cc 2>/dev/null || true
ls -l /dev/video* /dev/media* /dev/v4l-subdev* 2>&1 || true
ps -ef | grep -E '[e]vs|[h]vs|[d]g_hvs|[a]px003cc' || true
```

通过条件：有设备节点、厂商 EVS/APS 采集进程可见，并且不存在持续取帧错误。没有设备节点时停止本项，记录为 `PENDING`，不要轮换试不同 CAM overlay。

## 3. DDS 基础链路（无运动）

两板都设置 ROS 环境后，先在接收端启动命令，再在发布端发送一次诊断消息，避免`--once`在发布前结束。

NVIDIA先接收：

```bash
timeout 15 ros2 topic echo /dg/link_test --once
```

随后在RDK发布：

```bash
ros2 topic pub --once /dg/link_test std_msgs/msg/String \
  "{data: 'rdk_to_jetson_20260913'}"
```

反向测试同样先在RDK启动接收，再由NVIDIA发布`jetson_to_rdk_20260913`。两向都收到才记录`DDS_LINK_TEST_PASS`；这只证明发现和网络，不证明事件业务数据已经到达。

## 4. NVIDIA 观察模式安全检查（只读）

先确认底盘没有订阅端，控制输出保持禁用：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_recovery -v
ros2 topic info /scan -v
ros2 topic info /odom -v
ros2 topic info /imu/data -v
```

观察模式启动参数固定为：

```text
enable_nav2:=false
enable_cmd_vel_arbiter:=false
enable_uwb_replay:=false
enable_hybrid_vision_assist:=true
enable_auxiliary_transport_monitor:=true
enable_mmwave_observation:=true
enable_mmwave_dynamic_mask:=false
```

第一轮只启动观察侧链；不启用主动恢复运动、不启用毫米波动态掩码，也不让新节点发布`/cmd_vel`。
启动后只检查状态，不发送控制：

```bash
ros2 node list | grep -E 'dg_|inspection' || true
timeout 15 ros2 topic echo /dg/navigation/status --once
timeout 15 ros2 topic echo /dg/relocalization/status --once
```

底盘电源关闭时 `/scan`、`/odom`、`/imu/data` 没有发布者是预期现象；不得据此判定硬件损坏。

## 5. 事件相机业务话题

RDK HBN 桥确认运行后，先在RDK运行时显式加载Toolkit库，再在NVIDIA端执行：

RDK运行环境：

```bash
export HV_TOOLKIT_ROOT=$HOME/shimetapi_hybrid_vision_toolkit
export AMENT_PREFIX_PATH=$HOME/dg202611_hvs_ws/install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:${LD_LIBRARY_PATH:-}"
```

NVIDIA接收检查：

```bash
timeout 15 ros2 topic echo /dg/event_camera/quality --once
timeout 15 ros2 topic echo /dg/event_camera/features --once
timeout 15 ros2 topic echo /dg/event_camera/feature_quality --once
timeout 15 ros2 topic echo /dg/event_camera/assist_quality --once
timeout 15 ros2 topic echo /dg/event_camera/camera_info --once
timeout 15 ros2 topic echo /dg/auxiliary/transport_quality --once
```

通过条件：quality不是`UNSEEN`，features、feature_quality、assist_quality和CameraInfo有消息，且NVIDIA端持续收到业务话题。2026-09-13已形成`CROSS_BOARD_RECEIVE_PASS`：quality=`GOOD`、feature_quality=`COMPARED`、features约9.2–9.9 Hz，详见`evidence/cross_board/20260913/nvidia_receive.txt`及其SHA-256清单。`/dg/auxiliary/transport_quality`属于新版传输监视器验收项，监视器尚未部署前保持`PENDING`，不填写其新鲜度或延迟结论。`timeout`退出码`124`表示在限定时间内没有收到消息，应记录为`PENDING`，不能填写PASS。

## 6. UWB 回放（无底盘运动）

在已构建的 Linux ROS 2 工作区中执行，使用默认已采集静态文件：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_uwb_replay:=true \
  enable_mmwave_dynamic_mask:=false
```

记录以下一次性输出：

```bash
timeout 15 ros2 topic echo /dg/uwb/quality --once
timeout 15 ros2 topic echo /dg/fusion/status --once
timeout 15 ros2 topic echo /dg/fusion/pose --once
```

通过条件：质量状态可见、物理源去重生效、MODEL-1 给出 `UWB_AIDED` 或明确拒绝原因。没有真实 `/scan`、`/odom` 时，只能记录回放消费证据，不能宣称地图坐标精度。

## 7. 毫米波规范化回放（无底盘运动）

取得真实厂家数据并转换为规范化 CSV 后才执行：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_mmwave_replay:=true \
  mmwave_input_file:=/absolute/path/normalized_mmwave.csv \
  enable_mmwave_observation:=true \
  enable_mmwave_dynamic_mask:=false
```

通过条件：`/dg/mmwave/points` 的 `frame_id` 为 `mmwave_link`，点云字段完整，quality 新鲜，MODEL-2 能观察到；动态掩码必须仍为关闭。

## 8. 现场实机动作（仅用户执行）

只有静态检查、事件/UWB/毫米波回放证据均保存后，现场人员才可以：

1. 确认底盘电池、急停、周围空间和人工停止方式；
2. 启动基础 bringup，确认真实 `/scan`、`/odom`、`/imu/data`、TF 和底盘订阅者；
3. 低速、短距离执行前后左右动作；
4. 保存 `/cmd_vel`、`/odom`、TF、模型状态、控制器日志和现场照片。

代理不代替现场人员打开底盘电源、启动巡检、发送导航目标或发布 `/cmd_vel`。

## 9. 证据归档

每次通过或失败都保存：命令输出、启动参数、ROS 环境、设备时间、Git 提交号、原始数据/rosbag、日志、SHA-256 和现场照片。报告中分别标记 `SOFTWARE_PRECHECK`、`ROS2_REPLAY`、`RDK_LIVE_CAPTURE`、`CROSS_BOARD_RECEIVE` 和 `REAL_ROBOT`，不得用回放或静态结果替代实机指标。
