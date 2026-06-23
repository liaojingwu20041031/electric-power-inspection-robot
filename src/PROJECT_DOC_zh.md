# 电力巡检机器人 ROS 2 项目使用与调试手册

平台规划：Jetson Orin Nano Super、Ubuntu 22.04、ROS 2 Humble
统一工作区：`~/ros2_DL`
当前定位：电力巡检机器人集成开发工作空间，覆盖底盘、雷达、IMU、建图、导航、ZED 感知、语音/LLM 任务层、显示 UI 和 mobile bridge。

本文按实机调试顺序组织：先底层，再控制，再建图，再导航，最后接入感知和任务层。

## 0. 项目主流程

```text
1. 底层 bringup
   ZLAC8015D SocketCAN 底盘 或 STM32 串口回退 + IMU + RPLidar + URDF + EKF

2. 手动控制验收
   /cmd_vel -> 底盘动作，确认方向、速度、里程计、TF

3. 建图
   /scan + /odom + TF -> slam_toolbox -> 保存 maps/my_map.yaml 和 maps/my_map.pgm

4. 导航
   已知地图 + AMCL + Nav2 -> /cmd_vel -> 底盘闭环运动

5. 感知和任务层
   ZED 2i -> YOLO/TensorRT -> /perception/localized_objects
   文字/语音/UI -> /inspection_ai/* -> 系统控制或巡检任务事件
```

当前仓库已经包含机器人底层、导航、感知和交互框架；完整巡检业务闭环、检测服务 API、告警数据库、报告导出、LocateAnything-3B 推理和 LingBot-Map 实际接入仍属于后续扩展。

## 1. 项目结构总览

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

根目录脚本是实机使用入口。`scripts/run_on_jetson.sh` 会自动进入工作空间、source ROS 环境和 install 环境，并把常用运行模式统一成 `bringup`、`mapping`、`navigation`、`zed`、`perception`、`llm`、`inspection`、`teleop`。日常调试优先使用这些脚本，只有需要查看 launch 细节时再直接运行 `ros2 launch`。

## 2. ROS 包结构分析

| 层级 | 包/目录 | 主要职责 | 关键输入 | 关键输出 |
|---|---|---|---|---|
| 硬件接入 | `scripts/`、`src/bind_usb.sh` | 绑定 USB 设备、配置 `can1`、安装/检查驱动 | USB 设备、PCAN、系统权限 | `/dev/robot_lidar`、`/dev/robot_imu`、`can1` |
| 传感器驱动 | `hipnuc_imu`、`rplidar_ros-ros2`、`zed-ros2-wrapper` | 发布 IMU、激光雷达和 ZED 图像/深度 | 串口、USB 相机 | `/imu/data`、`/scan`、`/zed/...` |
| 底盘与导航 | `ylhb_base` | 底盘控制、里程计、URDF、EKF、SLAM、AMCL、Nav2 | `/cmd_vel`、`/scan`、`/imu/data`、地图 | `/odom`、TF、`/map`、Nav2 action |
| 共享接口 | `ylhb_interfaces` | 定义任务事件、任务状态、语音输出状态等消息 | 无运行节点 | `TaskEvent`、`TaskStatus`、`SayText`、`VoiceStatus` |
| 视觉感知 | `ylhb_perception` | 2D 检测、深度融合、目标位置输出 | ZED RGB、深度、相机内参 | `/perception/detections`、`/perception/localized_objects` |
| 任务交互 | `ylhb_llm` | 文本/语音命令、任务事件、显示 UI、系统进程管理 | `/inspection_ai/text_command`、语音、感知结果 | `/inspection_ai/*`、`/cmd_vel`、系统启动/停止 |
| 外部桥接 | `ylhb_mobile_bridge` | 对手机端或 Web 端提供 HTTP/WebSocket 调试入口 | HTTP/WebSocket 请求、ROS 状态 | `/cmd_vel`、`/inspection_ai/text_command`、状态 JSON |

这个分层的核心原则是：底层只处理实时运动和传感器，任务层通过 ROS 话题和服务调用底层能力，不直接改写硬件驱动；感知层只输出检测和定位结果，不直接控制底盘；外部桥接只做协议转换，不承载 Nav2 或底盘算法。

## 3. 运行链路解释

### 3.1 底盘与导航闭环

```text
Nav2 / teleop / UI / mobile bridge
  -> /cmd_vel
  -> zlac8015d_canopen_controller 或 base_controller
  -> /odom
  -> robot_localization EKF
  -> odom -> base_footprint TF
  -> Nav2 controller / planner
```

`ylhb_base` 中两个底盘后端互斥启动：默认是 `zlac8015d_canopen_controller`，通过 SocketCAN `can1` 控制 ZLAC8015D；`base_controller` 是 STM32 串口回退方案。两者都不发布底盘 TF，`odom -> base_footprint` 统一交给 EKF，避免 TF 冲突。

### 3.2 建图链路

```text
/scan + /odom + TF
  -> slam_toolbox async_slam_toolbox_node
  -> /map
  -> nav2_map_server map_saver_cli
  -> maps/my_map.yaml + maps/my_map.pgm
```

建图必须先启动底层 bringup，因为 `mapping.launch.py` 只启动 `slam_toolbox`，不启动雷达、底盘或 EKF。保存地图时统一写入 `~/ros2_DL/maps/`，避免旧版本把地图散落到 `src/`。

### 3.3 定位与导航链路

```text
maps/my_map.yaml + /scan + /odom + TF
  -> Nav2 map_server + AMCL
  -> planner_server / controller_server
  -> /cmd_vel
  -> 底盘控制器
```

`navigation.launch.py` 只启动 Nav2，不启动 bringup。AMCL 不强制使用建图原点，现场需要先发布 `map` frame 的 `/initialpose`。`scan_map_relocalization_node` 在粗位姿附近做二维激光到地图匹配，质量达标后再发布修正位姿；`amcl_swing_relocalization_node` 只做小幅原地摆头帮助 AMCL 收敛。

### 3.4 视觉感知链路

```text
ZED 2i RGB 图像
  -> yolo_detector_node
  -> /perception/detections
  -> object_localizer_node + ZED 深度/内参
  -> /perception/target_pose
  -> /perception/localized_objects
```

`ylhb_perception` 默认使用 Jetson 上编译好的 TensorRT engine：`src/ylhb_perception/models/yolo26.engine`。C++ TensorRT 节点负责实时检测，Python `object_localizer_node.py` 根据深度图和相机内参估算目标 3D 坐标。调试图默认关闭，避免 DDS 图像链路拖慢推理。

### 3.5 任务、语音与 UI 链路

```text
文字 / 语音 / UI / mobile bridge
  -> /inspection_ai/text_command 或 /inspection_ai/system_command
  -> inspection_task_node / voice_command_router_node / system_supervisor_node
  -> /inspection_ai/task_event、/inspection_ai/say_text、/cmd_vel 或系统进程动作
```

`ylhb_llm` 不是底盘控制器本体，而是上层任务编排和交互层。`basic_motion_command_node` 可以把“前进、后退、左转、右转、停止”等短命令转换为短时 `/cmd_vel`；`inspection_task_node` 把巡检文本解析成 `TaskEvent`；`system_supervisor_node` 负责启动/停止底层、建图、导航、感知等系统进程；`inspection_display_ui_node` 提供本机中文显示界面。

## 4. 修改功能时看哪里

| 修改目标 | 优先查看 | 注意事项 |
|---|---|---|
| CAN 接口、轮径、轮距、速度限幅 | `src/ylhb_base/config/zlac8015d.yaml`、`base_kinematics.yaml` | 修改后先台架低速验收 `/cmd_vel`、`/odom`、方向 |
| 机器人外形、TF、雷达/IMU 安装位 | `src/ylhb_base/urdf/ylhb.urdf.xacro` | 改 URDF 后运行 `xacro`、`check_urdf` 和几何测试 |
| EKF 融合和 TF 发布 | `src/ylhb_base/config/ekf.yaml` | 保持 EKF 独占 `odom -> base_footprint` |
| SLAM 参数 | `src/ylhb_base/config/slam_toolbox_params.yaml` | 先确认 `/scan`、`/odom`、TF 正常再调参数 |
| Nav2 行为和 footprint | `src/ylhb_base/config/nav2_params.yaml` | 改 footprint、速度或恢复行为后运行导航配置测试 |
| 语音、UI、任务话题 | `src/ylhb_llm/config/llm.yaml` | `/inspection_ai/*` 是任务层主命名空间 |
| 感知模型和检测阈值 | `src/ylhb_perception/config/detector.yaml` | TensorRT engine 输入尺寸需与 `imgsz` 一致 |
| 手机/Web 调试入口 | `src/ylhb_mobile_bridge/config/mobile_bridge.yaml` | bridge 只做外部协议到 ROS 的转换 |
| 测试与回归 | `src/ylhb_base/test/`、`src/ylhb_llm/test/` | 配置变更应同步更新对应测试 |

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

启动移动端桥接：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

本地巡逻执行器也在 `ylhb_mobile_bridge` 包中，但它不是 HTTP/WebSocket 服务。
巡逻调试使用：

```bash
ros2 launch ylhb_mobile_bridge patrol_executor.launch.py
```

默认监听：

```text
0.0.0.0:8000
```

默认地图前缀已统一为：

```text
/home/nvidia/ros2_DL/maps/my_map
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
