# 电力巡检机器人 ROS 2 项目使用与调试手册

平台规划：Jetson Orin Nano Super、Ubuntu 22.04、ROS 2 Humble
统一工作区：`~/ros2_DL`
当前定位：电力巡检机器人初始化框架，保留底盘、雷达、IMU、建图、导航、ZED 感知、语音/LLM 任务层、显示 UI 和 mobile bridge。

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

初始化框架尚未实现完整巡检业务闭环、检测服务 API、告警数据库、报告导出、LocateAnything-3B 推理和 LingBot-Map 实际接入。

## 1. 工作空间规范

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

## 2. 依赖安装与编译

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

当前工作空间实测 `colcon build --symlink-install` 可完成 10 个包构建。`colcon test` 中 ZED 第三方包会触发上游 lint/copyright 问题，且受限网络下 `xmllint` 无法下载 ROS schema；自研底盘映射 gtest 和自研包 lint 已通过。

## 3. 硬件绑定与 CAN 检查

Jetson 实机建议先执行一次 USB/CAN 绑定：

```bash
cd ~/ros2_DL
sudo ./src/bind_usb.sh
```

目标设备约定：

```text
/dev/robot_lidar -> RPLidar A2M8，CP210x，10c4:ea60
/dev/robot_imu   -> WIT IMU，CH340，1a86:7523
can1             -> PEAK PCAN-USB，0c72:000c，500000 bit/s
```

验收命令：

```bash
systemctl status robot-hardware-guard.service --no-pager
tail -n 120 /var/log/robot-hardware-guard.log
lsusb
ls -l /dev/robot_lidar /dev/robot_imu /dev/ttyUSB* /dev/ttyCH341USB*
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

## 4. 底层 Bringup

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

IMU 默认 `/dev/robot_imu`、`9600` 波特率。CH340 枚举但没有串口时，先检查：

```bash
./scripts/install_ch341_safe.sh --precheck
./scripts/install_ch341_safe.sh --test-load
sudo ./src/bind_usb.sh
ls -l /dev/robot_imu /dev/ttyCH341USB* /dev/ttyUSB*
```

## 5. 底盘控制

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

### 5.1 键盘速度参数调节

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

## 6. 建图 Mapping

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

## 7. 导航 Navigation

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

## 8. ZED 与感知

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

## 9. AI 任务层与显示 UI

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

## 10. Mobile Bridge

启动移动端桥接：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

默认监听：

```text
0.0.0.0:8000
```

默认地图前缀已统一为：

```text
/home/nvidia/ros2_DL/maps/my_map
```

## 11. 常用调试命令

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

## 12. 常见问题

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
- 不要把 CH340 IMU 串口当作雷达端口。

### IMU 无数据

- 检查 `/dev/robot_imu` 和 `/dev/ttyCH341USB*`。
- 检查 `lsmod | grep -E 'ch341|ch34x'`。
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
