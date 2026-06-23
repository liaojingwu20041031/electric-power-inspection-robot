# 电力巡检机器人 ROS 2 项目使用与调试手册

平台规划：Jetson Orin Nano Super、Ubuntu 22.04、ROS 2 Humble
统一工作区：`~/ros2_DL`
当前定位：电力巡检机器人集成开发工作空间，覆盖底盘、雷达、IMU、建图、导航、ZED 感知、语音/LLM 任务层、显示 UI 和 mobile bridge。

本文按实机调试顺序组织：先看系统能力边界，再按底层、控制、建图、导航、
巡逻、感知和任务层逐步联调。

## 项目定位与交付

本项目面向电力实训和变电站巡检方向，当前阶段完成的是 PC 主机端仓库初始化整理和 ROS 2 智能机器人可复用框架保留。项目尚未在 Jetson/开发板上进行完整实机运行，因此不能视为已经完成的电力巡检业务系统。

当前交付：

- 仓库已迁移到 `liaojingwu20041031/electric-power-inspection-robot`。
- 文档入口已整理为中文命名，方便实训汇报和后续开发。
- LLM 层已从旧业务语义清理为巡检任务初始化框架。
- 默认任务话题切换为 `/inspection_ai/*`。
- 保留底盘、导航、感知、语音、移动端桥接等可复用 ROS 2 框架。
- 未实现正式巡检状态机、检查点执行闭环、检测服务、告警数据库或巡检报告导出。

### 技术栈概览

| 类型 | 技术/模块 |
|---|---|
| 系统 | Ubuntu 22.04、ROS 2 Humble |
| 计算平台 | Jetson Orin Nano Super，后续实机验证 |
| 底盘 | ZLAC8015D V4、SocketCAN/CANopen |
| 导航 | RPLidar、IMU、SLAM Toolbox、Nav2、AMCL、EKF |
| 感知 | ZED 2i、YOLO/TensorRT、OpenCV |
| LLM/语音 | DashScope/Qwen、ASR、TTS、语音路由 |
| UI/桥接 | PyQt 控制台、HTTP/WebSocket mobile bridge |
| 后续规划 | LocateAnything-3B、LingBot-Map、巡检业务协议 |

### 初始化框架边界

当前只提供"可以继续开发"的机器人框架：

```text
底盘/传感器框架
  -> 建图与导航框架
  -> 感知输入框架
  -> LLM/语音/任务事件框架
  -> UI 与移动端桥接占位
```

正式电力巡检流程仍属于后续开发：

```text
站点/区域/路线/检查点建模
  -> 任务下发、暂停、恢复、取消、人工接管
  -> 路线级安全检测
  -> 检查点级设备检测
  -> 告警记录和巡检报告
```

### 后续扩展方向

- 开发板实机验证底盘、雷达、IMU、ZED、Nav2 和语音链路。
- 设计正式 `/inspection/*` 任务协议和巡检消息。
- 增加站点、区域、路线和检查点配置。
- 接入人员、安全帽、火源、烟雾、障碍物等路线级检测。
- 评估 LocateAnything-3B 用于开关状态、漏油、异物等检查点级定位。
- 评估 LingBot-Map 用于三维空间重建、巡检点位对齐和后台展示。

### 文档入口

- [项目封面](PROJECT_COVER.md)
- [项目概览](../docs/项目概览.md)
- [快速使用](../docs/快速使用.md)
- [接口约定](../docs/接口约定.md)
- [迁移清理记录](../docs/迁移清理记录.md)

## 0. 系统能力地图

当前系统按运行职责分为六层。阅读和排障时先确认自己正在处理哪一层，不要把
ROS 包名、launch 模式和业务功能混在一起看。

| 层级 | 当前能力 | 主要运行入口 | 关键输出 |
|---|---|---|---|
| 硬件与底盘 | USB 设备绑定、PCAN/SocketCAN、ZLAC8015D 或 STM32 底盘、IMU、RPLidar、URDF、EKF | `./scripts/run_on_jetson.sh bringup` | `/odom`、`/scan`、`/imu/data`、TF、`/cmd_vel` 执行 |
| 建图定位 | SLAM Toolbox 建图、地图保存、AMCL 定位、Scan-to-Map 修正 | `mapping`、`navigation`、`scan_map_relocalization_node` | `/map`、`map -> odom`、`/initialpose` 修正 |
| 导航与巡逻 | Nav2 单点导航、本地路线巡逻、到点任务触发、返航、暂停/恢复/取消 | `navigation`、`patrol_executor.launch.py` | Nav2 action、`/patrol/status`、`/patrol/event` |
| 视觉感知 | ZED 2i 图像/深度、YOLO/TensorRT 检测、目标空间定位 | `zed`、`perception` | `/perception/detections`、`/perception/localized_objects` |
| 任务与交互 | 文本/语音命令、任务事件、TTS、中文显示 UI、系统进程 supervisor | `llm`、`inspection` | `/inspection_ai/*`、短时 `/cmd_vel`、UI |
| 外部调试接口 | HTTP/WebSocket 调试、移动端状态查询、低速控制、系统启动/停止入口 | `mobile_bridge.launch.py` | Web API、WebSocket、ROS 话题转发 |

当前仓库已经包含机器人底层、导航、感知和交互框架；完整巡检业务闭环、检测服务 API、告警数据库、报告导出、LocateAnything-3B 推理和 LingBot-Map 实际接入仍属于后续扩展。

## 1. 实机启动组合

根目录脚本是实机使用入口。`scripts/run_on_jetson.sh` 会自动进入工作空间、
source ROS 环境和 install 环境，并把常用运行模式统一成固定命令。

| 要做什么 | 必须先启动 | 再启动 | 备注 |
|---|---|---|---|
| 台架底盘/传感器验收 | 无 | `bringup` | 先确认 CAN、IMU、雷达、TF 和 `/odom` |
| 手动遥控 | `bringup` | `teleop` | 低速确认方向、轮速、里程计和急停习惯 |
| 在线建图 | `bringup` | `mapping` | `mapping.launch.py` 不启动底盘、雷达或 EKF |
| 单点导航 | `bringup` | `navigation` | 启动后先发布 `map` frame 的 `/initialpose` |
| 本地巡逻 | `bringup`、`navigation` | `patrol_executor.launch.py` | 默认 `auto_start:=false`，先定位再发送 `start` |
| 视觉检测 | `zed` | `perception` | ZED wrapper 和 TensorRT 检测分开启动 |
| 任务/语音/UI | 视任务需要启动底层或导航 | `inspection` 或 `llm` | `inspection` 是带 UI、语音和 supervisor 的组合模式 |
| 移动端调试 | 视接口需要启动底层或导航 | `mobile_bridge.launch.py` | 只做外部协议到 ROS 的转换 |

常用顺序：

```text
bringup -> teleop 验收
bringup -> mapping -> 保存地图
bringup -> navigation -> initialpose -> 单点导航
bringup -> navigation -> patrol_executor -> /patrol/command start
zed -> perception -> inspection
```

## 2. 项目结构总览

仓库按 ROS 2 工作空间组织。根目录负责部署、地图、文档和硬件资料；`src/` 下放 ROS 2 包和第三方驱动。

```text
~/ros2_DL
├── scripts/                  # Jetson 安装、构建、启动、CAN 和 PCAN 诊断脚本
├── maps/                     # 默认地图 my_map.yaml / my_map.pgm
├── docs/                     # 项目概览、快速使用、接口约定等说明
├── CAD/                      # 底盘、支架、结构件 CAD/STL/3MF 文件
├── 官方通信协议/              # ZLAC8015D V4 官方手册和 CANopen/RS485 示例
└── src/
    ├── ylhb_base/            # 底盘、URDF、EKF、SLAM、Nav2、重定位
    ├── ylhb_interfaces/      # 巡检任务、状态和语音消息定义
    ├── ylhb_llm/             # 任务层、语音、显示 UI、系统 supervisor
    ├── ylhb_perception/      # ZED 图像、YOLO/TensorRT、深度定位
    ├── ylhb_mobile_bridge/   # HTTP/WebSocket 与 ROS 2 桥接
    ├── hipnuc_imu/           # N300WP PRO/HiPNUC IMU 串口驱动
    ├── rplidar_ros-ros2/     # RPLidar ROS 2 驱动
    └── zed-ros2-wrapper/     # ZED 2i 官方 ROS 2 wrapper
```

容易混淆的边界：

- `scripts/` 是现场启动和诊断入口，不是 ROS 包。
- `maps/` 存放运行时地图和路线，`maps/my_map.*` 给 Nav2 使用，
  `maps/route_patrol_*.json` 给本地巡逻执行器使用。
- `ylhb_mobile_bridge` 同时包含 HTTP/WebSocket bridge 和本地巡逻执行器。
  前者服务手机/Web 调试，后者直接调用 Nav2；两者共享包但职责不同。
- `ylhb_llm` 是任务/语音/UI 层，不是底盘控制器。它可以发布短时 `/cmd_vel`
  或系统命令，但不替代 Nav2 和底盘驱动。

## 3. ROS 包职责

| ROS 包 | 运行类型 | 主要职责 | 不负责 |
|---|---|---|---|
| `ylhb_base` | C++/Python 节点、launch、配置 | 底盘后端、URDF/TF、EKF、SLAM、AMCL、Nav2、重定位辅助 | 业务巡检流程、语音/UI、Web API |
| `hipnuc_imu` | 传感器驱动包 | HiPNUC IMU 串口数据解析与发布 | EKF 参数、底盘控制 |
| `rplidar_ros-ros2` | 第三方雷达驱动 | Slamtec RPLidar `/scan` 发布 | 地图、导航策略 |
| `zed-ros2-wrapper` | 第三方相机 wrapper | ZED 2i 图像、深度、相机信息 | YOLO 业务检测 |
| `ylhb_perception` | 感知节点、launch、模型配置 | YOLO/TensorRT 检测、深度融合、目标定位输出 | 导航控制、任务调度 |
| `ylhb_interfaces` | 消息定义 | 任务事件、任务状态、语音输出状态等自定义消息 | 运行节点 |
| `ylhb_llm` | Python 任务/语音/UI 节点 | 文本任务解析、语音输入输出、显示 UI、system supervisor、短时基础运动命令 | 底盘闭环、Nav2 算法、本地路线巡逻状态机 |
| `ylhb_mobile_bridge` | Python bridge 与巡逻执行器 | HTTP/WebSocket 调试入口、本地 Nav2 巡逻执行器、路线文件校验 | 感知算法、底盘驱动、Nav2 参数调优 |

常见 launch 与节点归属：

| 入口 | 来自 | 启动内容 |
|---|---|---|
| `ylhb_base bringup.launch.py` | `ylhb_base` | 底盘后端、IMU、RPLidar、robot_state_publisher、EKF |
| `ylhb_base mapping.launch.py` | `ylhb_base` | SLAM Toolbox 建图 |
| `ylhb_base navigation.launch.py` | `ylhb_base` | Nav2、map_server、AMCL |
| `ylhb_mobile_bridge patrol_executor.launch.py` | `ylhb_mobile_bridge` | 本地路线巡逻执行器 |
| `ylhb_mobile_bridge mobile_bridge.launch.py` | `ylhb_mobile_bridge` | HTTP/WebSocket mobile bridge |
| `ylhb_perception perception.launch.py` | `ylhb_perception` | YOLO 检测与深度目标定位 |
| `ylhb_llm llm.launch.py` | `ylhb_llm` | 任务、语音、UI、system supervisor 的可选组合 |

## 4. 数据流与修改入口

### 4.1 底盘闭环

```text
Nav2 / teleop / UI / mobile bridge / patrol_executor
  -> /cmd_vel
  -> zlac8015d_canopen_controller 或 base_controller
  -> /odom
  -> robot_localization EKF
  -> odom -> base_footprint TF
  -> Nav2 controller / planner
```

`ylhb_base` 中两个底盘后端互斥启动：默认是 `zlac8015d_canopen_controller`，通过 SocketCAN `can1` 控制 ZLAC8015D；`base_controller` 是 STM32 串口回退方案。两者都不发布底盘 TF，`odom -> base_footprint` 统一交给 EKF，避免 TF 冲突。

### 4.2 建图链路

```text
/scan + /odom + TF
  -> slam_toolbox async_slam_toolbox_node
  -> /map
  -> nav2_map_server map_saver_cli
  -> maps/my_map.yaml + maps/my_map.pgm
```

建图必须先启动底层 bringup，因为 `mapping.launch.py` 只启动 `slam_toolbox`，不启动雷达、底盘或 EKF。保存地图时统一写入 `~/ros2_DL/maps/`，避免旧版本把地图散落到 `src/`。

### 4.3 定位与单点导航链路

```text
maps/my_map.yaml + /scan + /odom + TF
  -> Nav2 map_server + AMCL
  -> planner_server / controller_server
  -> /cmd_vel
  -> 底盘控制器
```

`navigation.launch.py` 只启动 Nav2，不启动 bringup。AMCL 不强制使用建图原点，现场需要先发布 `map` frame 的 `/initialpose`。`scan_map_relocalization_node` 在粗位姿附近做二维激光到地图匹配，质量达标后再发布修正位姿；`amcl_swing_relocalization_node` 只做小幅原地摆头帮助 AMCL 收敛。

### 4.4 本地巡逻链路

```text
maps/route_patrol_*.json
  -> patrol_executor_node
  -> Nav2 NavigateToPose action
  -> /patrol/status + /patrol/event
  -> 到点后 /inspection_ai/text_command
```

本地巡逻执行器属于 `ylhb_mobile_bridge` 包，但它不是 Web bridge。
它读取地图坐标路线文件，按目标点顺序调用 Nav2，处理暂停、恢复、取消、返航、
循环和失败策略。它不修改 Nav2 参数、不清 costmap、不接管底盘驱动。

### 4.5 视觉感知链路

```text
ZED 2i RGB 图像
  -> yolo_detector_node
  -> /perception/detections
  -> object_localizer_node + ZED 深度/内参
  -> /perception/target_pose
  -> /perception/localized_objects
```

`ylhb_perception` 默认使用 Jetson 上编译好的 TensorRT engine：`src/ylhb_perception/models/yolo26.engine`。C++ TensorRT 节点负责实时检测，Python `object_localizer_node.py` 根据深度图和相机内参估算目标 3D 坐标。调试图默认关闭，避免 DDS 图像链路拖慢推理。

### 4.6 任务、语音与 UI 链路

```text
文字 / 语音 / UI / mobile bridge
  -> /inspection_ai/text_command 或 /inspection_ai/system_command
  -> inspection_task_node / voice_command_router_node / system_supervisor_node
  -> /inspection_ai/task_event、/inspection_ai/say_text、/cmd_vel 或系统进程动作
```

`ylhb_llm` 不是底盘控制器本体，而是上层任务编排和交互层。`basic_motion_command_node` 可以把“前进、后退、左转、右转、停止”等短命令转换为短时 `/cmd_vel`；`inspection_task_node` 把巡检文本解析成 `TaskEvent`；`system_supervisor_node` 负责启动/停止底层、建图、导航、感知等系统进程；`inspection_display_ui_node` 提供本机中文显示界面。

### 4.7 修改功能时看哪里

| 修改目标 | 优先查看 | 注意事项 |
|---|---|---|
| CAN 接口、轮径、轮距、速度限幅 | `src/ylhb_base/config/zlac8015d.yaml`、`base_kinematics.yaml` | 修改后先台架低速验收 `/cmd_vel`、`/odom`、方向 |
| 机器人外形、TF、雷达/IMU 安装位 | `src/ylhb_base/urdf/ylhb.urdf.xacro` | 改 URDF 后运行 `xacro`、`check_urdf` 和几何测试 |
| EKF 融合和 TF 发布 | `src/ylhb_base/config/ekf.yaml` | 保持 EKF 独占 `odom -> base_footprint` |
| SLAM 参数 | `src/ylhb_base/config/slam_toolbox_params.yaml` | 先确认 `/scan`、`/odom`、TF 正常再调参数 |
| Nav2 行为和 footprint | `src/ylhb_base/config/nav2_params.yaml` | 改 footprint、速度或恢复行为后运行导航配置测试 |
| 本地巡逻路线 | `maps/route_patrol_*.json` | 坐标必须是 `map` frame，`start_pose` 应是机器人真实起点 |
| 本地巡逻状态机 | `src/ylhb_mobile_bridge/ylhb_mobile_bridge/patrol_executor_node.py`、`patrol_route_store.py` | 改后运行两个 patrol pytest |
| 语音、UI、任务话题 | `src/ylhb_llm/config/llm.yaml` | `/inspection_ai/*` 是任务层主命名空间 |
| 感知模型和检测阈值 | `src/ylhb_perception/config/detector.yaml` | TensorRT engine 输入尺寸需与 `imgsz` 一致 |
| 手机/Web 调试入口 | `src/ylhb_mobile_bridge/config/mobile_bridge.yaml` | bridge 只做外部协议到 ROS 的转换 |
| 测试与回归 | `src/ylhb_base/test/`、`src/ylhb_mobile_bridge/test/`、`src/ylhb_llm/test/` | 配置或行为变更应同步更新对应测试 |

## 5. 工作空间规范

推荐工作空间固定为：

```bash
cd ~
git clone https://github.com/liaojingwu20041031/electric-power-inspection-robot.git ros2_DL
cd ~/ros2_DL
```

ROS 包应直接位于：

```text
~/ros2_DL/src/ylhb_base
~/ros2_DL/src/ylhb_interfaces
~/ros2_DL/src/ylhb_llm
~/ros2_DL/src/ylhb_mobile_bridge
~/ros2_DL/src/ylhb_perception
~/ros2_DL/src/hipnuc_imu
~/ros2_DL/src/rplidar_ros-ros2
~/ros2_DL/src/zed-ros2-wrapper
```

如果临时放在其他目录，先设置：

```bash
export WS_DIR=/path/to/ros2_DL
```

后续地图统一保存到：

```text
~/ros2_DL/maps/my_map.yaml
~/ros2_DL/maps/my_map.pgm
```

`~/ros2_DL/src/my_map.yaml` 仅作为旧版本兼容地图，不作为推荐路径。

## 6. 依赖安装与编译

开发板首次部署：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
./scripts/install_jetson_dependencies.sh
colcon build --symlink-install
source ~/ros2_DL/install/setup.bash
```

日常重新编译：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译自研包可用于快速检查：

```bash
colcon build --symlink-install \
  --packages-select ylhb_interfaces ylhb_base ylhb_llm ylhb_perception ylhb_mobile_bridge
```

当前工作空间包含 11 个 ROS 2 包：7 个本项目/驱动包和 4 个 ZED wrapper 子包。`colcon test` 中 ZED 第三方包可能触发上游 lint/copyright 问题，且受限网络下 `xmllint` 可能无法下载 ROS schema；自研包问题应优先用 `--packages-select ylhb_base ylhb_llm ylhb_perception ylhb_mobile_bridge ylhb_interfaces` 单独验证。

## 7. 硬件绑定与 CAN 检查

Jetson 实机建议先执行一次 USB/CAN 绑定：

```bash
cd ~/ros2_DL
sudo ./src/bind_usb.sh
```

目标设备约定：

```text
/dev/robot_lidar -> RPLidar A2M8，CP210x，10c4:ea60
/dev/robot_imu   -> N300WP PRO / HiPNUC IMU，CH9102，1a86:55d4
can1             -> PEAK PCAN-USB，0c72:000c，500000 bit/s
```

验收命令：

```bash
systemctl status robot-hardware-guard.service --no-pager
tail -n 120 /var/log/robot-hardware-guard.log
lsusb
ls -l /dev/robot_lidar /dev/robot_imu /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB*
ip -details link show can1
```

如果只想手动配置 CAN：

```bash
cd ~/ros2_DL
./scripts/setup_zlac_can.sh can1 500000
ip -br link show can1
candump -tz can1
```

只有 `can1` 存在且 `candump -tz can1` 能看到或至少不报接口错误时，再继续底盘 bringup。

## 8. 底层 Bringup

终端 1 启动底层系统：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base bringup.launch.py
```

等价脚本：

```bash
./scripts/run_on_jetson.sh bringup
```

默认底盘后端是 ZLAC8015D SocketCAN：

```bash
ros2 launch ylhb_base bringup.launch.py base_backend:=zlac
```

无 USB-CAN 或需要旧底盘板时使用 STM32 串口回退：

```bash
ros2 launch ylhb_base bringup.launch.py base_backend:=stm32 base_port:=/dev/ttyS1
```

台架调试时可临时关闭 IMU，但实机建图和导航不推荐：

```bash
ros2 launch ylhb_base bringup.launch.py enable_imu:=false
```

Bringup 会启动：

```text
zlac8015d_canopen_controller 或 base_controller
imu_driver
rplidar_node
robot_state_publisher
ekf_filter_node
```

底层验收：

```bash
ros2 node list
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /scan --once
ros2 topic echo /zlac8015d/status --once
ros2 topic echo /zlac8015d/fault --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

RPLidar A2M8 默认 `115200` 波特率，frame 为 `laser_link`。如果 `/dev/robot_lidar` 不存在，launch 会尝试按 USB ID 查找 CP2102 `/dev/ttyUSB*`；长期修复仍应使用 `sudo ./src/bind_usb.sh`。

IMU 默认 `/dev/robot_imu`、`115200` 波特率。N300WP PRO 枚举但没有串口时，先检查：

```bash
lsusb | grep -i '1a86:55d4'
systemctl status ModemManager --no-pager
sudo ./src/bind_usb.sh
ls -l /dev/robot_imu /dev/ttyACM* /dev/ttyCH343USB* /dev/ttyUSB*
```

## 9. 底盘控制

底层 bringup 正常后，在终端 2 做键盘控制：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

等价脚本：

```bash
./scripts/run_on_jetson.sh teleop
```

### 9.1 键盘速度参数调节

`teleop_twist_keyboard` 发布 `/cmd_vel` 时，`speed` 控制线速度 `linear.x`，单位 `m/s`；`turn` 控制角速度 `angular.z`，单位 `rad/s`。可以启动时直接指定：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.30
```

使用脚本时，把参数接在 `teleop` 后面：

```bash
./scripts/run_on_jetson.sh teleop --ros-args -p speed:=0.10 -p turn:=0.30
```

键盘控制运行后也可以临时调节当前速度：

```text
q / z：线速度和角速度同时增加 / 减小 10%
w / x：只增加 / 减小线速度 speed
e / c：只增加 / 减小角速度 turn
k 或其他非运动按键：停止
```

屏幕上的 `currently: speed ... turn ...` 会显示当前线速度和角速度。建议实机先用 `speed:=0.10`、`turn:=0.30` 低速测试，再逐步增加。

注意：键盘发布的速度仍受底盘最终限幅限制。实机底盘限幅在 `src/ylhb_base/config/base_kinematics.yaml`：

```yaml
zlac8015d_canopen_controller:
  ros__parameters:
    # 最大线速度，单位 m/s。调大前先确认场地、电机和急停安全。
    max_linear_speed: 0.35
    # 最大角速度，单位 rad/s。数值越大，原地转向越快。
    max_angular_speed: 0.55
```

这两个参数会限制 `/cmd_vel` 输入，超过后由底盘控制器自动限幅。修改后重新启动 bringup 生效：

```bash
ros2 launch ylhb_base bringup.launch.py
```

如果调的是 Nav2 自动导航速度，而不是键盘控制速度，还需要同步修改 `src/ylhb_base/config/nav2_params.yaml`：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # Nav2 输出线速度上限，单位 m/s。
      max_vel_x: 0.15
      max_speed_xy: 0.15
      # Nav2 输出角速度上限，单位 rad/s。
      max_vel_theta: 0.35

velocity_smoother:
  ros__parameters:
    # 平滑器最终输出上限：[x, y, theta]。
    max_velocity: [0.15, 0.0, 0.35]
    min_velocity: [-0.05, 0.0, -0.35]
```

建议先小幅调节：线速度每次增加 `0.05 m/s`，角速度每次增加 `0.05 rad/s` 到 `0.10 rad/s`。Nav2 的 `max_vel_x`、`max_speed_xy`、`velocity_smoother.max_velocity[0]` 应保持一致；`max_vel_theta` 与 `velocity_smoother.max_velocity[2]` 应保持一致。

也可以直接发布速度指令：

```bash
# 前进 0.10 m/s，持续发布时机器人会运动
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"

# 停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

检查订阅链路：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /odom
ros2 topic echo /zlac8015d/status
```

方向验收：

```text
linear.x > 0：机器人应向前。
angular.z > 0：机器人应按 ROS 约定左转。
/odom 中 x、yaw 应与实际运动方向一致。
松开键盘或停止发布后，cmd_vel 超时保护应停车。
```

语音/文字基础动作入口由 `basic_motion_command_node` 处理：

```bash
ros2 launch ylhb_llm llm.launch.py enable_voice:=false enable_tts:=false enable_display_ui:=false
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String "{data: '前进'}"
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String "{data: '停止'}"
```

默认要求底盘状态在线；如果只是离线调试任务层，需要在配置里临时关闭 `require_chassis_online`，不要在实机运行时关闭安全检查。

## 10. 建图 Mapping

建图需要先保持终端 1 的 bringup 运行，再开终端 2：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base mapping.launch.py
```

等价脚本：

```bash
./scripts/run_on_jetson.sh mapping
```

终端 3 用键盘慢速遥控机器人覆盖场地：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

建图过程检查：

```bash
ros2 node list | grep slam
ros2 topic echo /map --once --field info
ros2 topic hz /scan
ros2 topic echo /odom --once
```

保存地图到推荐路径：

```bash
mkdir -p ~/ros2_DL/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_DL/maps/my_map \
  --ros-args -p save_map_timeout:=10.0
```

保存后检查：

```bash
ls -lh ~/ros2_DL/maps/my_map.yaml ~/ros2_DL/maps/my_map.pgm
```

地图保存失败时优先检查 `/map` 是否存在、`slam_toolbox` 是否运行，以及当前终端是否已 source 工作空间环境。

## 11. 导航 Navigation

导航需要已有地图，并且必须先启动底层 bringup。`navigation.launch.py` 只启动
Nav2，不启动雷达、底盘、IMU 或 EKF。推荐终端顺序：

```bash
# 终端 1：底层
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base bringup.launch.py

# 终端 2：导航
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base navigation.launch.py map:=$HOME/ros2_DL/maps/my_map.yaml

# 终端 3：等待人工粗定位并做局部 Scan-to-Map 修正
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ylhb_base scan_map_relocalization_node
```

等价脚本：

```bash
./scripts/run_on_jetson.sh navigation map:=$HOME/ros2_DL/maps/my_map.yaml
```

`navigation.launch.py` 默认优先读取 `~/ros2_DL/maps/my_map.yaml`；如果缺失，会回退到旧兼容路径 `~/ros2_DL/src/my_map.yaml` 并打印 warning。

先启动 Scan-to-Map 节点，再从 RViz/Foxglove 发布 2D Pose Estimate。因为
`/initialpose` 通常不是持久化话题，先点击再启动节点可能收不到粗位姿。
粗位姿的 `header.frame_id` 必须是 `map`；`odom`、`base_link` 或空 frame
会被拒绝。该节点只在粗位姿附近做二维局部搜索，不是全局自动定位，也不使用
3D 点云。

导航与定位启动检查：

```bash
ros2 node list | grep -E 'amcl|planner|controller|bt_navigator|map_server'
ros2 lifecycle nodes
ros2 topic echo /amcl_pose --once
ros2 topic echo /scan_match_pose --once
ros2 topic echo /cmd_vel
ros2 action list | grep navigate
```

AMCL 不再默认强制使用建图原点。应在 RViz/Foxglove 中把 Fixed Frame 设为
`map`，发布粗略的 2D Pose Estimate。Scan-to-Map 质量达标时会发布修正后的
`/initialpose` 和 `/scan_match_pose`；质量不达标时只打印
`score`、`mean_distance`、`inlier_ratio`，不会发布错误位姿。
命令行粗位姿示例：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}"
```

如果粗定位后激光与地图仍有小幅偏差，可在周围无人员、线缆和低矮障碍时运行：

```bash
ros2 run ylhb_base amcl_swing_relocalization_node
```

该节点仅按 `/odom` 闭环小幅左右摆头，默认执行两轮“左 10°、右 20°、
左 10°”，`linear.x/y` 始终为 0，不前进后退、不做 360° 旋转。超时、
异常、Ctrl+C 和正常退出都会发布零速度。需要调整时使用 ROS 参数，例如：

```bash
ros2 run ylhb_base amcl_swing_relocalization_node --ros-args \
  -p cycles:=1 -p angular_speed:=0.15
```

只有在 RViz/Foxglove 中 `/scan` 与静态地图轮廓贴合、`map -> odom ->
base_footprint` 连续稳定后再发送导航目标。

发送一个简单导航目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

导航不动时按顺序检查：

```text
1. /scan 是否有频率。
2. /odom 是否更新。
3. TF 是否有 map -> odom -> base_footprint/base_link -> laser_link。
4. /initialpose 是否已设置到地图中的正确位置。
5. Scan-to-Map 是否报告质量达标，/scan 与地图是否贴合。
6. local/global costmap 是否都能看到 /scan 动态障碍。
7. controller_server 是否在输出 /cmd_vel。
8. 底盘节点是否订阅 /cmd_vel 且 ZLAC 状态在线。
```

动态避障验收时先使用低速短距离目标，在 local/global costmap 中观察人员或
纸箱等障碍是否被标记。机器人应减速、绕行或停止；如果路径仍穿过障碍、
障碍层不清除、定位跳变或持续原地转向，应立即取消目标并停车，不要通过提高
速度或关闭障碍层绕过问题。

## 12. 本地巡逻 Patrol 调试

本地巡逻功能位于 `ylhb_mobile_bridge` 包内，只通过标准 ROS 接口调用现有
Nav2，不修改 `ylhb_base` 的 Nav2 参数、DWB、costmap、footprint、机器人模型
或底盘控制代码。它的核心输入是地图坐标路线文件，核心输出是
`/patrol/status`、`/patrol/event` 和到点后的
`/inspection_ai/text_command`。

### 12.1 路线准备

路线生成工具的正式输出目录是 `~/ros2_DL/maps/`，文件名使用
`route_patrol_*.json`。当前实测默认路线为：

```bash
ls -lh ~/ros2_DL/maps/route_patrol_001.json
```

`route_file_path:=auto` 会扫描该目录。存在多个文件时优先选择文件名中编号
最大的路线，例如 `route_patrol_010.json` 优先于 `route_patrol_002.json`；
文件名编号都无法解析时，按修改时间选择最新文件。临时测试可用绝对路径覆盖：

```bash
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py \
  route_file_path:=/home/nvidia/ros2_DL/maps/route_patrol_001.json
```

路线文件必须满足以下约束：

- 顶层 `version` 为 `2`，`frame_id` 为 `map`。
- `active_route_id` 指定默认路线。
- `start_pose.pose` 使用机器人真实起点的 `x`、`y`、`yaw`，也是返航位姿。
- `start_pose.publish_initial_pose=true` 时，执行器启动后发布 `/initialpose`。
- `targets` 定义巡检点和 `task_duration_sec`，`routes[].target_ids` 定义执行顺序。
- `return_to_start` 控制完成后是否返航；`failure_policy` 支持 `abort` 和
  `abort_and_return_home`。
- 路线内部循环由 `loop` 控制；自动定时当前只支持 `schedules` 中的 `interval`。

修改后可以用纯 Python 校验路线文件：

```bash
python3 - <<'PY'
import sys
sys.path.insert(0, "src/ylhb_mobile_bridge")
from ylhb_mobile_bridge.patrol_route_store import load_route_file

route = load_route_file("/home/nvidia/ros2_DL/maps/route_patrol_001.json")
print(route["active_route_id"], len(route["targets"]), len(route["routes"]))
PY
```

### 12.2 实机启动顺序

终端 1 启动底层：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base bringup.launch.py
```

终端 2 启动导航：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base navigation.launch.py map:=$HOME/ros2_DL/maps/my_map.yaml
```

终端 3 可选启动 scan-to-map 修正：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ylhb_base scan_map_relocalization_node
```

终端 4 启动巡逻执行器：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py \
  auto_start:=false \
  publish_initial_pose_on_startup:=true
```

注意：上面的命令只启动巡逻执行器、加载路线并发布初始位姿。因为显式设置了
`auto_start:=false`，节点会保持运行并停在 `idle` 状态等待人工确认，机器人
不会自动开始移动。这是正常行为，不是节点卡死。

如果路线文件里 `start_pose.publish_initial_pose=true`，节点会向 `/initialpose`
发布 3 次粗位姿。日志应显示实际加载的
`/home/nvidia/ros2_DL/maps/route_patrol_*.json`。可以在另一个终端检查状态：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_DL/install/setup.bash
ros2 topic echo /patrol/status --once
ros2 topic echo /amcl_pose --once
```

确认 AMCL 定位稳定、机器人周围安全后，发送开始命令：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String "{data: start}"
```

此后 `/patrol/status` 应从 `idle` 进入 `running`，Nav2 开始执行路线。如果希望
巡逻执行器发布完初始位姿后直接开始巡逻，可以把启动参数改为
`auto_start:=true`；实机首次调试仍建议使用 `false`，确认定位后再手动启动。

### 12.3 状态观察

开一个观察终端：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /patrol/status
ros2 topic echo /patrol/event
ros2 topic echo /inspection_ai/text_command
```

另一个终端检查 Nav2、TF 和动作接口：

```bash
ros2 action list | grep navigate
ros2 topic echo /amcl_pose --once
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo map base_footprint
```

`/patrol/status` 常见状态：

```text
idle                空闲
waiting_schedule    等待 interval 定时触发
running             正在前往巡检点
paused              已暂停，当前 Nav2 goal 已取消
waiting_loop        已回到起点，等待下一轮循环
canceling           正在取消
canceled            已取消，不返航
returning_home      正在返回 start_pose.pose
succeeded           巡逻成功
failed              巡逻失败
```

`/patrol/event` 常见事件：

```text
initial_pose_published
route_started
target_reached
target_task_finished
return_home_started
route_finished
route_failed
```

### 12.4 开始、暂停、恢复和取消

命令既可使用纯文本，也可使用 JSON。启动当前 `active_route_id`：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String "{data: start}"
```

指定路线启动：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"start\",\"route_id\":\"route_patrol_001\"}'}"
```

暂停、恢复、取消：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"pause\"}'}"

ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"resume\"}'}"

ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"cancel\"}'}"
```

重新加载路线文件：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"reload\"}'}"
```

重新发布初始位姿：

```bash
ros2 topic pub --once /patrol/command std_msgs/msg/String \
  "{data: '{\"command\":\"initialize\"}'}"
```

注意：`initialize` 和 `relocalize` 只允许在 `idle`、`waiting_schedule`、
`failed`、`succeeded`、`canceled` 状态执行；巡逻运行、暂停、返航或取消中会
被拒绝，避免重置定位导致 Nav2 跳变。

### 12.5 验收

按顺序验收：

```text
1. route 文件校验通过。
2. patrol_executor 启动后发布 initial_pose_published。
3. /scan、/odom、map -> odom -> base_footprint TF 稳定。
4. 发送 start 后，/patrol/status 进入 running。
5. Nav2 action /navigate_to_pose 有目标执行。
6. 到达每个 target 后发布 target_reached。
7. /inspection_ai/text_command 收到“已到达xxx，开始执行任务”。
8. 等待 task_duration_sec 后发布 target_task_finished。
9. return_to_start=true 时最后进入 returning_home 并返回 start_pose.pose。
10. 全部完成后发布 route_finished，状态进入 succeeded。
11. pause 会取消当前 Nav2 goal 并发布零速度。
12. resume 会从当前 target 或循环等待阶段恢复。
13. cancel 会取消当前 goal、发布零速度，不返航。
14. 单点失败会按 max_retries_per_checkpoint 重试。
15. abort_and_return_home 失败策略会尝试返回 start_pose.pose，最终状态为 failed。
```

发布前的软件回归命令：

```bash
python3 -m pytest src/ylhb_mobile_bridge/test/test_patrol_route_store.py -q
python3 -m pytest src/ylhb_mobile_bridge/test/test_patrol_executor_logic.py -q
colcon build --symlink-install --packages-select ylhb_base ylhb_mobile_bridge
```

### 12.6 常见问题

启动后直接 `failed`：

```text
多半是 route 文件无效，或 start_pose 与机器人实际放置位置不一致。
不要把 start_pose 写成默认 0,0,0；必须按真实地图坐标设置起点。
```

只发布了 `/initialpose`，没有自动巡逻：

```text
auto_start 默认为 false。手动发送 start，或启动时设置 auto_start:=true。
auto_start:=true 时会等 start_pose 初始位姿发布序列完成后再启动路线。
```

发送 `initialize` 被拒绝：

```text
说明当前处于 running、paused、returning_home 或 canceling。
先 cancel 或等待任务结束，再重新发布 start_pose 初始位姿。
```

到点没有触发文字任务：

```bash
ros2 topic echo /inspection_ai/text_command
ros2 topic echo /patrol/event
```

如果有 `target_reached` 但没有文字，检查 `text_command_topic` 参数是否仍为
`/inspection_ai/text_command`。

机器人导航行为异常：

```text
先按第 11 章检查 Nav2、定位、costmap 和底盘链路。
本地巡逻节点不清 costmap、不 reset Nav2 lifecycle、不修改 AMCL 或 DWB 参数。
```

## 13. ZED 与感知

启动 ZED 2i：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

等价脚本：

```bash
./scripts/run_on_jetson.sh zed
```

启动 YOLO/TensorRT 感知：

```bash
ros2 launch ylhb_perception perception.launch.py \
  model_path:=$HOME/ros2_DL/src/ylhb_perception/models/yolo26.engine \
  backend:=tensorrt \
  confidence_threshold:=0.35 \
  imgsz:=960 \
  max_det:=20 \
  half:=true \
  publish_debug_image:=false \
  show_debug_window:=false
```

调试时打开本地 OpenCV 窗口：

```bash
ros2 launch ylhb_perception perception.launch.py \
  model_path:=$HOME/ros2_DL/src/ylhb_perception/models/yolo26.engine \
  backend:=tensorrt \
  show_debug_window:=true \
  debug_window_max_hz:=15.0
```

检查话题：

```bash
ros2 topic list | grep zed
ros2 topic echo /perception/detections
ros2 topic echo /perception/localized_objects
```

模型推荐路径：

```text
~/ros2_DL/src/ylhb_perception/models/yolo26.onnx
~/ros2_DL/src/ylhb_perception/models/yolo26.engine
```

## 14. AI 任务层与显示 UI

文字调试优先关闭语音和 TTS：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_llm llm.launch.py \
  enable_voice:=false \
  enable_tts:=false \
  enable_display_ui:=false
```

发送巡检文字命令：

```bash
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String \
  "{data: '{\"source\":\"cli\",\"text\":\"开始巡检任务\"}'}"
```

查看任务链路：

```bash
ros2 topic echo /inspection_ai/task_event
ros2 topic echo /inspection_ai/task_status
ros2 topic echo /inspection_ai/say_text
ros2 topic echo /inspection_ai/task_context_status
```

启动本机显示 UI 和 supervisor：

```bash
export DISPLAY=:0
./scripts/run_on_jetson.sh inspection
```

`inspection` 模式会启动显示 UI、系统 supervisor 和内嵌 AI 任务层。UI 可以发出启动/停止底层、建图、导航、感知等系统命令；核心 ROS 节点仍以各自 launch 管理。

如需 DashScope：

```bash
export DASHSCOPE_API_KEY=你的DashScope_API_Key
ros2 launch ylhb_llm llm.launch.py enable_llm_parse:=true enable_voice:=false enable_tts:=false
```

## 15. Mobile Bridge

`ylhb_mobile_bridge` 是 ROS 2 与移动端调试 APP 之间的 HTTP/WebSocket 桥接，定位为**低速底盘控制、建图、导航的现场调试入口**，不承载正式巡检任务流程。调试端 APP 仓库为 `liaojingwu20041031/ylhb-robot-mobile`（Expo React Native），仅用于局域网调试。

### 15.1 启动桥接

终端 N（在 bringup/导航等底层已启动后）：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

默认监听 `0.0.0.0:8000`，默认地图前缀 `/home/nvidia/ros2_DL/maps/my_map`。配置文件在 `src/ylhb_mobile_bridge/config/mobile_bridge.yaml`，可改 `host`、`port`、话题名和速度限幅。

启动后先在 Jetson 本机验证服务是否起来：

```bash
curl http://localhost:8000/api/status
```

返回 JSON 含 `online: true` 即正常。

本地巡逻执行器也在 `ylhb_mobile_bridge` 包中，但它不是 HTTP/WebSocket 服务，两者共享包但职责不同：

```bash
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py
```

### 15.2 调试端 APP 连接

APP 默认连接地址为 `http://192.168.1.100:8000`，端口与 bridge 一致。连接步骤：

1. 确认 Jetson 与手机处于同一局域网（建议使用 Jetson 的有线/无线 LAN IP，不要用 USB 网络共享地址）。
2. 在 Jetson 上查看实际 IP：

   ```bash
   ip -br addr show
   ```

3. 在手机上安装调试端 APP（Expo Go 扫码或构建 APK），打开设置页。
4. 将 `baseUrl` 改为 `http://<Jetson_IP>:8000`（例如 `http://192.168.1.50:8000`）。
5. 关闭 `Mock Mode`（默认开启，开启时 APP 不发任何网络请求，仅用内置模拟数据）。
6. 首页点"测试 HTTP 连接"，成功后 `connectionState` 显示 `connected`。

### 15.3 WebSocket 实时状态

APP 设置页打开 `WebSocket` 开关后，dashboard 页会连接 `ws://<baseUrl>/ws/status`，服务端每 500ms 推送一次 `RobotStatus` JSON。WebSocket 打开后无需手动刷新；关闭时回退到手动"刷新状态"按钮。

WebSocket 仅服务端→客户端单向推送，APP 不向服务端发任何帧，也不做自动重连。断连后需手动重新打开开关。

### 15.4 调试 API 概览

完整接口契约见 [接口约定](../docs/接口约定.md)。APP 当前使用的端点：

| 分类 | 方法 | 路径 | 用途 |
|---|---|---|---|
| 状态 | GET | `/api/status` | 机器人基础状态、健康检查 |
| 控制 | POST | `/api/cmd_vel` | 短时 `/cmd_vel`，限速限时长 |
| 控制 | POST | `/api/stop` | 急停：零速度 + 停止任务文本 |
| 任务 | POST | `/api/text_command` | 文本命令到 `/inspection_ai/text_command` |
| 底盘测试 | POST | `/api/debug/chassis/test` | 低速底盘动作测试 |
| 底盘测试 | POST | `/api/debug/chassis/stop` | 停止底盘测试 |
| 建图 | GET | `/api/debug/mapping/status` | 建图依赖检查 |
| 建图 | POST | `/api/debug/mapping/start` | 启动建图进程 |
| 建图 | POST | `/api/debug/mapping/save` | 保存地图 |
| 建图 | POST | `/api/debug/mapping/stop` | 停止建图进程 |
| 导航 | GET | `/api/debug/navigation/status` | 导航依赖检查 |
| 导航 | POST | `/api/debug/navigation/start` | 启动 Nav2 |
| 导航 | POST | `/api/debug/navigation/set_initial_pose` | 发布 `/initialpose` |
| 导航 | POST | `/api/debug/navigation/goal` | 发送单点目标 |
| 导航 | POST | `/api/debug/navigation/cancel` | 取消当前目标 |
| 实时 | WS | `/ws/status` | 推送状态，约 2Hz |

统一响应信封：

```json
{ "ok": true, "message": "...", "data": {...} }
```

失败时 `ok: false`，`error` 取值 `invalid_request`/`ros_unavailable`/`process_error`/`not_allowed`/`internal_error`。

### 15.5 状态字段说明

`/api/status` 与 `/api/debug/status` 返回的字段对接调试端 APP：

- `system_mode`：来自 `/inspection_ai/system_mode`，取 `ready`/`mapping`/`fault`/`sleep` 等，TRANSIENT_LOCAL QoS。
- `last_map_age_sec`：`/map` 话题最近一次发布距今秒数，建图调试时判断地图是否在持续生成。
- `scan_range_min` / `scan_range_max`：`/scan` 最近一帧测距范围（米），确认雷达数据质量。
- `last_odom_age_sec` / `last_scan_age_sec`：里程计/雷达数据新鲜度，APP 用 >3s 判定为 stale。
- `task_status`：回显最后一次通过 bridge 下发的文本命令，不订阅正式任务状态。
- `zlac_status`：底盘状态，正常为 `online`，故障为 `fault: <详情>`。
- `mapping_status` / `nav2_status`：`running` 或 `not_running`，按节点存在性判定。
- `battery_percent`：电量，当前由底盘侧提供；未提供时为空，APP 解析但不显示。

`DebugStatus.nodes` 检测键：`zlac8015d_canopen_controller`、`slam_toolbox`、`bt_navigator`、`controller_server`、`planner_server`、`amcl`、`map_server`、`bringup`（检测 `robot_state_publisher`）、`rplidar_node`、`tf`（检测 `/tf` 话题是否有发布者）。

> 零售遗留字段 `salesDialogueStatus`、`cart` 在电力巡检机器人上不实现，APP 对缺失值显示"未知"，属预期行为。

### 15.6 调试流程

底盘测试（需先启动 bringup）：

```text
APP 控制页 -> 方向按钮 -> POST /api/cmd_vel (300ms, ±0.03 m/s, ±0.15 rad/s)
APP 急停按钮 -> POST /api/stop
```

建图（需先启动 bringup）：

```text
APP 建图面板 -> 开始建图 -> POST /api/debug/mapping/start
遥控机器人覆盖场地
APP 建图面板 -> 保存地图 -> POST /api/debug/mapping/save { map_name: "my_map" }
APP 建图面板 -> 停止建图 -> POST /api/debug/mapping/stop
```

导航（需先启动 bringup）：

```text
APP 导航面板 -> 开始导航 -> POST /api/debug/navigation/start
APP 导航面板 -> 设置初始位姿 -> POST /api/debug/navigation/set_initial_pose
APP 导航面板 -> 发送目标点 -> POST /api/debug/navigation/goal
APP 导航面板 -> 取消 -> POST /api/debug/navigation/cancel
```

### 15.7 安全限制

- `/api/cmd_vel` 服务端强制限幅：线速度 ≤ 0.15 m/s，角速度 ≤ 0.5 rad/s，`duration_ms` 范围 50–3000ms，超时自动发布零速度。
- 进程管理只允许白名单命令（`mapping`、`navigation`），通过 `scripts/run_on_jetson.sh` 执行。
- bridge 默认允许局域网跨域（CORS `*`），**仅用于局域网，不暴露公网**。
- 无鉴权：依赖网络层隔离，不要把 8000 端口映射到公网或不可信网络。
- APP 侧额外节流：底盘命令 200ms 最小间隔，控制面板 10s 无操作自动锁定。

### 15.8 故障排查

APP 显示 `network_error`：

```text
1. 确认手机与 Jetson 同局域网。
2. 确认 baseUrl 的 IP 与 Jetson 实际 IP 一致，端口 8000。
3. Jetson 上 curl http://localhost:8000/api/status 验证服务已启动。
4. 确认 APP 的 Mock Mode 已关闭。
```

APP 连接成功但状态全灰：

```text
1. 确认 bringup 已启动（/odom、/scan、TF）。
2. APP 调试页查看 nodes/topics 哪些为 false。
3. last_odom_age_sec / last_scan_age_sec > 3s 表示传感器无数据。
```

WebSocket 连不上：

```text
1. 确认 APP 设置页 WebSocket 开关已打开。
2. 确认 baseUrl 协议与 WS 一致（http -> ws，https -> wss）。
3. 服务端每 500ms 推送，无消息说明服务端未启动或已退出。
```

建图/导航启动失败返回 `process_error`：

```text
1. 确认 scripts/run_on_jetson.sh 存在且可执行。
2. 确认 workspace_dir 参数指向正确的工作空间。
3. 在 Jetson 上手动执行 ./scripts/run_on_jetson.sh mapping 验证脚本本身。
```

## 16. 常用调试命令

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic info /cmd_vel -v
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /map --once --field info
ros2 lifecycle nodes
ros2 run tf2_tools view_frames
```

查看 launch 参数：

```bash
ros2 launch ylhb_base bringup.launch.py --show-args
ros2 launch ylhb_base navigation.launch.py --show-args
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py --show-args
ros2 launch ylhb_llm llm.launch.py --show-args
ros2 launch ylhb_perception perception.launch.py --show-args
```

脚本模式：

```bash
./scripts/run_on_jetson.sh bringup
./scripts/run_on_jetson.sh mapping
./scripts/run_on_jetson.sh navigation
./scripts/run_on_jetson.sh zed
./scripts/run_on_jetson.sh perception
./scripts/run_on_jetson.sh llm
./scripts/run_on_jetson.sh inspection
./scripts/run_on_jetson.sh teleop
```

## 17. 常见问题

### 编译通过但测试不全绿

当前 ZED wrapper 第三方包存在 lint/copyright 测试失败，受限网络下 `xmllint` 也会因无法加载 `download.ros.org` schema 失败。这不影响 `colcon build` 生成运行文件。自研包问题应优先用 `--packages-select ylhb_base ylhb_llm ylhb_perception ylhb_mobile_bridge ylhb_interfaces` 单独验证。

### CAN 不在线

- 检查 `ip -br link show can1`。
- 检查 `candump -tz can1`。
- 检查 `src/ylhb_base/config/zlac8015d.yaml` 中 `can_interface` 是否为实际接口。
- 不要在没有 SocketCAN 接口时修改 ROS 2 控制逻辑。

### 雷达无数据

- 检查 `/dev/robot_lidar`。
- 检查 launch 日志中的 `serial_baudrate` 是否为 `115200`。
- 执行 `ros2 topic hz /scan`。
- 不要把 N300WP PRO IMU 串口当作雷达端口。

### IMU 无数据

- 检查 `/dev/robot_imu`、`/dev/ttyACM*`、`/dev/ttyCH343USB*` 和 `/dev/ttyUSB*`。
- 检查 `lsusb | grep -i '1a86:55d4'`，确认 N300WP PRO 已枚举。
- 检查 ModemManager 是否抢占串口，必要时重新执行 `sudo ./src/bind_usb.sh`。
- 检查 `ros2 topic echo /imu/data --once`。
- 实机建图和导航不要长期使用 `enable_imu:=false`。

### 地图保存到了错误目录

推荐只使用：

```bash
~/ros2_DL/maps/my_map.yaml
~/ros2_DL/maps/my_map.pgm
```

如果发现 `~/ros2_DL/src/maps`，说明旧配置或旧进程仍在运行；停止相关节点，重新 source 并启动当前工作空间。

### Nav2 未启动或无法导航

- 先启动 bringup。
- 确认 `/odom`、`/scan`、TF 和地图存在。
- 先运行 `scan_map_relocalization_node`，再发布 `map` frame 的 `/initialpose`。
- 确认 `/scan_match_pose` 合理且 `/scan` 与地图轮廓贴合。
- 查看 `planner_server`、`controller_server`、`bt_navigator`、`amcl`。
- 查看 local/global costmap 的 `/scan` 障碍层是否更新。
- 查看 `/cmd_vel` 是否输出，以及底盘节点是否订阅。

### UI 不显示

- Jetson 物理屏幕使用 `DISPLAY=:0`。
- SSH X11 的 `localhost:10.0` 会显示到远程电脑，不适合现场屏幕。
- `./scripts/run_on_jetson.sh inspection` 会把 SSH 转发 DISPLAY 自动改成本机显示。
