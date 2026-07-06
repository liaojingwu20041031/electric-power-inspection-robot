<div align="center">

# 电力巡检机器人

面向电力设施巡检场景的 ROS 2 移动机器人系统

<p>
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white">
  <img alt="Ubuntu 22.04" src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white">
  <img alt="Jetson Orin Nano" src="https://img.shields.io/badge/Jetson-Orin%20Nano%20Super-76B900?logo=nvidia&logoColor=white">
  <img alt="Nav2" src="https://img.shields.io/badge/Navigation-Nav2-2563EB">
  <img alt="CANopen" src="https://img.shields.io/badge/Drive-CANopen-0F766E">
  <img alt="React Native" src="https://img.shields.io/badge/APP-Expo%20React%20Native-000020?logo=expo&logoColor=white">
</p>

集成底盘控制、激光雷达、IMU、RTK/GNSS 数据接入、建图定位、自主导航、双目视觉、TensorRT 感知、
语音交互、巡检控制界面和移动端调试 APP，提供从硬件接入到上层任务编排的一体化开发工作空间。

[实机展示](#实机展示) · [核心能力](#核心能力) · [移动端-app](#移动端-app) · [系统架构](#系统架构) · [快速开始](#快速开始) · [项目文档](#项目文档)

</div>

---

## 实机展示

<table>
  <tr>
    <td width="50%" align="center">
      <img src="记录照片/MVIMG_20260625_161859..jpg" width="100%" alt="电力巡检机器人整机侧视图">
    </td>
    <td width="50%" align="center">
      <img src="记录照片/MVIMG_20260625_161907..jpg" width="100%" alt="电力巡检机器人整机正视图">
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <img src="记录照片/MVIMG_20260625_161920..jpg" width="100%" alt="电力巡检机器人移动底盘与控制硬件">
    </td>
  </tr>
</table>

## 核心能力

| 能力 | 实现 |
|---|---|
| 移动底盘 | ZLAC8015D V4 双轮差速底盘，PEAK PCAN-USB、SocketCAN、CANopen |
| 传感器接入 | RPLidar、HiPNUC IMU、WTRTK980 RTK_4G、ZED 2i，提供稳定设备别名和统一坐标系 |
| 状态估计 | 轮速里程计与 IMU 经 `robot_localization` EKF 融合 |
| 建图定位 | SLAM Toolbox 在线建图，AMCL 定位与 Scan-to-Map 重定位 |
| 三维建模 | ZED SDK Spatial Mapping 导出巡检用 PLY 点云或 OBJ 网格，不作为 Nav2 地图 |
| 导航巡逻 | Nav2 单点导航、本地路线巡逻、到点任务触发、返航与暂停/恢复/取消 |
| 视觉感知 | ZED 深度图像、YOLO、TensorRT 推理和目标空间定位入口 |
| 任务交互 | 中文控制界面、任务事件、系统状态管理、ASR/TTS 和语音指令 |
| 移动端调试 | Expo React Native APP、HTTP/WebSocket 实时状态、低速底盘控制和建图管理 |
| 调试运维 | Jetson 安装/构建/启动脚本、CAN 诊断、ROS 2 回归测试 |

## 移动端 APP

配套调试端 [ylhb-robot-mobile](https://github.com/liaojingwu20041031/ylhb-robot-mobile)
通过局域网连接 Jetson 上的 `ylhb_mobile_bridge`，用于现场联调，不替代正式巡检任务流程。

| APP 功能 | 说明 |
|---|---|
| 连接与状态 | HTTP 连通性检测，实时查看底盘、雷达、IMU、里程计、建图进程和数据新鲜度 |
| 实时推送 | WebSocket 推送机器人状态与当前 SLAM 地图快照 |
| 底盘调试 | 低速方向控制、短时速度指令、自动归零和急停 |
| 建图管理 | 启动/停止底层与建图进程，预览地图并保存到工作空间 |
| 诊断信息 | 展示进程状态、PID、退出码和日志尾部，辅助现场排查 |

启动机器人端桥接：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

服务默认监听 `0.0.0.0:8000`。手机与 Jetson 需处于同一可信局域网，并在 APP
中将地址设为 `http://<Jetson_IP>:8000`、关闭 `Mock Mode`。详细接口和安全限制见
[Mobile Bridge APP 调试接口](docs/mobile_debug_api.md)。

## 系统架构

```mermaid
flowchart LR
    UI[巡检界面 / 语音指令] --> TASK[任务与系统管理]
    APP[移动端调试 APP] -->|HTTP / WebSocket| BRIDGE[Mobile Bridge]
    ROUTE[路线文件 / 巡逻命令] --> PATROL[本地巡逻执行器]
    PATROL --> NAV[Nav2 导航]
    PATROL --> TASK
    TASK --> PERCEPTION[视觉感知]
    BRIDGE --> TASK
    BRIDGE --> CMD

    NAV --> CMD[/cmd_vel/]
    CMD --> BASE[ZLAC8015D 底盘]
    BASE --> ODOM[轮速里程计]

    LIDAR[RPLidar] --> SLAM[SLAM / AMCL]
    IMU[HiPNUC IMU] --> EKF[EKF 状态估计]
    RTK[WTRTK980 RTK] --> GPS["/gps/fix + /gps/rtk_status"]
    ODOM --> EKF
    EKF --> NAV
    SLAM --> NAV

    ZED[ZED 2i] --> PERCEPTION
    PERCEPTION --> TASK
```

图中连线表示功能数据关系，不代表所有节点必须同时启动。实际启动依赖以主调试
手册的“实机启动组合”为准。

核心 TF 链：

```text
map -> odom -> base_footprint -> base_link -> laser_link
                         `-----> imu_link
                         `-----> gps_link
```

## 功能模块

| ROS 2 包 | 职责 |
|---|---|
| `ylhb_base` | CAN/串口底盘、URDF/TF、EKF、RTK NMEA 接入、SLAM、AMCL、Nav2 和重定位辅助 |
| `ylhb_perception` | ZED 图像输入、YOLO/TensorRT 推理、深度目标定位 |
| `ylhb_llm` | 文本任务解析、语音输入输出、系统管理、基础运动命令和显示界面 |
| `ylhb_mobile_bridge` | HTTP/WebSocket 调试桥接，以及独立的本地 Nav2 巡逻执行器 |
| `ylhb_interfaces` | 巡检任务、状态、语音等自定义消息 |
| `hipnuc_imu` | HiPNUC IMU 串口驱动 |
| `rplidar_ros-ros2` | Slamtec RPLidar ROS 2 驱动 |
| `zed-ros2-wrapper` | ZED 2i 图像、深度和相机信息发布 |

运行入口与包职责不是同一概念。`bringup`、`mapping`、`navigation`、`zed`、
`perception`、`inspection` 是运行组合；ROS 包表描述代码归属。完整启动依赖和
数据流见 [重点使用与调试文档](src/PROJECT_DOC_zh.md)。

## 快速开始

### 1. 获取工作空间

推荐将仓库放在 `~/ros2_DL`：

```bash
cd ~
git clone https://github.com/liaojingwu20041031/electric-power-inspection-robot.git ros2_DL
cd ~/ros2_DL
```

运行环境为 Ubuntu 22.04 与 ROS 2 Humble。Jetson 首次部署可执行：

```bash
./scripts/install_jetson_dependencies.sh
./scripts/build_on_jetson.sh
```

常规增量构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 准备底盘 CAN

ZLAC 底盘默认使用 `can1`，波特率为 `500000`：

```bash
./scripts/setup_zlac_can.sh can1 500000
ip -details link show can1
```

设备连接和权限配置请参考 [重点使用与调试文档](src/PROJECT_DOC_zh.md)。

### 3. 启动机器人

以下模式应根据任务在独立终端中启动：

```bash
# 底盘、IMU、雷达、robot_state_publisher 与 EKF
./scripts/run_on_jetson.sh bringup

# 可选：同时启动 WTRTK980 RTK NMEA 数据发布
ros2 launch ylhb_base bringup.launch.py enable_rtk:=true

# 在线建图
./scripts/run_on_jetson.sh mapping

# 使用 maps/my_map.yaml 定位与导航
./scripts/run_on_jetson.sh navigation

# ZED 2i 与 TensorRT 感知
./scripts/run_on_jetson.sh zed
./scripts/run_on_jetson.sh perception

# 可选：ZED SDK 三维建模导出，不要与 zed/zed_wrapper 同时运行
./scripts/run_on_jetson.sh zed_3d_mapping

# 巡检界面、任务管理与语音交互
./scripts/run_on_jetson.sh inspection
```

准备好 `maps/route_patrol_*.json` 后，本地 Nav2 巡逻执行器可用以下入口启动：

```bash
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py \
  auto_start:=false \
  publish_initial_pose_on_startup:=true
```

`auto_start:=false` 表示只加载路线并发布初始位姿，随后停在 `idle` 等待人工确认。
定位稳定、现场安全后，再发送开始命令：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String "{data: start}"
```

路线格式、状态观察、暂停/恢复/取消和验收步骤见
[重点使用与调试文档](src/PROJECT_DOC_zh.md#12-本地巡逻-patrol-调试)。

键盘遥控：

```bash
./scripts/run_on_jetson.sh teleop
```

## 导航与机器人模型

- `base_footprint` 保持在差速运动学中心，作为导航与里程计基准。
- URDF 使用与实车外廓一致的偏心圆柱 visual/collision 模型。
- local/global costmap 使用同一组 16 点多边形 footprint。
- 全局地图结合静态层、激光障碍层和膨胀层；局部地图持续处理动态障碍。
- 导航失败时采用受约束的小幅后退和局部代价地图清理，不执行激进恢复动作。

## 构建与验证

针对底盘、几何模型、SLAM、导航配置和重定位逻辑的测试均注册在
`ylhb_base` 包中：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ylhb_base
source install/setup.bash
colcon test --packages-select ylhb_base --event-handlers console_direct+
colcon test-result --verbose
```

硬件诊断入口：

```bash
./scripts/diagnose_pcan.sh
ros2 topic list -t
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 topic echo /gps/rtk_status --once
```

WTRTK980 当前属于第一阶段接入：发布 `/gps/fix`、`/gps/nmea_sentence` 和
`/gps/rtk_status` 供验证和显示使用，不参与 AMCL、Nav2、`map -> odom` 或巡逻路线计算。
RTK 冒烟测试可运行：

```bash
./scripts/rtk_smoke_test.sh
```

## 项目文档

- [重点使用与调试文档](src/PROJECT_DOC_zh.md)：项目定位、硬件接线、启动流程、ROS 话题、接口约定和故障排查
- [Mobile Bridge APP 调试接口](docs/mobile_debug_api.md)：移动端状态、底盘控制、建图流程和接口约定
- [移动端 APP 仓库](https://github.com/liaojingwu20041031/ylhb-robot-mobile)：Expo React Native 局域网调试端
- [官方通信协议](官方通信协议/)：ZLAC8015D V4 手册、CANopen 示例和 RTK 接入资料
- [CAD 机械模型](CAD/Retail-Cart-3D-Model/)：底盘、支架和结构件模型

## 使用说明

本仓库用于机器人研发、联调与实验验证。启动底盘前应架空驱动轮或确保周围无人员和障碍物；
修改轮径、轮距、CAN 映射、URDF 或 Nav2 footprint 后，应重新执行包测试并进行低速实车验证。
