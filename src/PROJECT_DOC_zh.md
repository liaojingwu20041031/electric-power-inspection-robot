# 电力巡检机器人初始化框架使用与调试手册

平台规划：Jetson Orin Nano Super、Ubuntu 22.04、ROS 2 Humble
推荐工作区：`~/ros2_DL`
当前状态：PC 主机端仓库初始化整理完成，开发板实机运行仍待验证。

## 1. 当前阶段说明

当前仓库不是完整电力巡检系统，而是电力巡检机器人方向的初始化框架。已保留 ROS 2 机器人通用能力，包括底盘、雷达、IMU、建图、导航、ZED 感知、语音、LLM 任务事件、控制台和 mobile bridge。

当前没有实现：

- 正式巡检状态机。
- 检查点导航执行闭环。
- 检测服务 API。
- 告警数据库或巡检报告导出。
- LocateAnything-3B 推理代码。
- LingBot-Map 实际接入代码。

相关文档入口：

- [项目概览](../docs/项目概览.md)
- [快速使用](../docs/快速使用.md)
- [接口约定](../docs/接口约定.md)
- [迁移清理记录](../docs/迁移清理记录.md)

## 2. 工作空间规范

后续 PC 和开发板统一使用 `~/ros2_DL`：

```bash
cd ~
git clone https://github.com/liaojingwu20041031/electric-power-inspection-robot.git ros2_DL
cd ~/ros2_DL
```

如需放到其他目录，显式设置：

```bash
export WS_DIR=/path/to/ros2_DL
```

## 3. 编译流程

PC 或开发板具备 ROS 2 Humble 时执行：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ros2_DL/install/setup.bash
```

开发板上也可使用脚本：

```bash
cd ~/ros2_DL
./scripts/install_jetson_dependencies.sh
./scripts/build_on_jetson.sh
source ~/ros2_DL/install/setup.bash
```

如果当前 PC 没有 ROS 2 环境，只做 Python 文件语法检查即可，不要把未运行的 ROS 2 结果写成实测通过。

## 4. CAN 配置说明

本节用于后续开发板调试。当前 PC 初始化阶段不验证 CAN 实机通信。

```bash
cd ~/ros2_DL
./scripts/setup_zlac_can.sh can1 500000
ip -br link show can1
candump -tz can1
```

PEAK PCAN-USB 排查建议：

- 先确认 `lsusb` 能看到 PEAK USB 设备。
- 确认系统出现 SocketCAN 网络接口，例如 `can1`。
- 若只出现 PEAK 字符设备而没有 `can1`，不要修改 ROS 2 CAN 配置。
- 临时验证驱动模块成功后，再考虑安装；不要直接替换内核或修改启动配置。
- 只有 `candump -tz can1` 能看到 ZLAC8015D 启动报文后，再继续底盘 bringup。

## 5. Bringup 启动与检查

后续开发板使用：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source ~/ros2_DL/install/setup.bash
./scripts/run_on_jetson.sh bringup
```

常用检查：

```bash
ros2 node list
ros2 topic echo /odom
ros2 topic echo /zlac8015d/status
ros2 topic echo /zlac8015d/fault
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

## 6. Mapping 建图与地图保存

后续开发板使用：

```bash
cd ~/ros2_DL
./scripts/run_on_jetson.sh mapping
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_DL/maps/my_map \
  --ros-args -p save_map_timeout:=10.0
```

检查输出：

```bash
ls ~/ros2_DL/maps/my_map.yaml
ls ~/ros2_DL/maps/my_map.pgm
```

## 7. Navigation 导航与初始位姿

后续开发板使用：

```bash
cd ~/ros2_DL
./scripts/run_on_jetson.sh navigation
```

如果机器人不在建图原点附近，先从 RViz/Foxglove 发布 `/initialpose`，再发送导航目标。

命令行调试：

```bash
ros2 topic echo /amcl_pose
ros2 action list | grep navigate
ros2 topic echo /cmd_vel
```

## 8. Perception 感知框架

后续开发板使用：

```bash
cd ~/ros2_DL
./scripts/run_on_jetson.sh zed
./scripts/run_on_jetson.sh perception
```

模型路径统一放在：

```text
~/ros2_DL/src/ylhb_perception/models/yolo26.onnx
~/ros2_DL/src/ylhb_perception/models/yolo26.engine
```

检查话题：

```bash
ros2 topic list | grep zed
ros2 topic echo /perception/detections
ros2 topic echo /perception/localized_objects
```

## 9. Inspection/LLM 框架启动

当前初始化阶段可重点检查 LLM 任务事件链路。默认关闭语音和 TTS：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source ~/ros2_DL/install/setup.bash
ros2 launch ylhb_llm llm.launch.py enable_voice:=false enable_tts:=false
```

开发板 UI 入口：

```bash
./scripts/run_on_jetson.sh inspection
```

## 10. /inspection_ai/text_command 测试

```bash
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String \
  "{data: '{\"source\":\"cli\",\"text\":\"开始巡检任务\"}'}"
```

查看任务事件：

```bash
ros2 topic echo /inspection_ai/task_event
```

查看播报请求：

```bash
ros2 topic echo /inspection_ai/say_text
```

查看任务层上下文：

```bash
ros2 topic echo /inspection_ai/task_context_status
```

## 11. 常用 ROS 2 调试命令

```bash
ros2 node list
ros2 node info /inspection_task_node
ros2 topic list
ros2 topic info /inspection_ai/task_event
ros2 topic echo /inspection_ai/task_event
ros2 topic hz /scan
ros2 service list
ros2 action list
ros2 launch ylhb_llm llm.launch.py enable_voice:=false enable_tts:=false
```

## 12. 常见问题排查

### CAN 不在线

- 检查 `ip -br link show can1`。
- 检查 `candump -tz can1` 是否有 ZLAC8015D 报文。
- 检查 `src/ylhb_base/config/zlac8015d.yaml` 中 `can_interface` 是否与实际接口一致。
- 不要在没有 SocketCAN 接口时修改 ROS 2 控制逻辑。

### 雷达无数据

- 检查雷达 USB 权限和串口绑定。
- 执行 `ros2 topic hz /scan`。
- 确认 `rplidar_ros` 节点是否启动。

### IMU 无数据

- 检查串口权限和设备名。
- 确认 bringup 参数和硬件连接。
- 检查 TF 中 `base_footprint`、`base_link`、`imu_link` 是否正常。

### Nav2 未启动或无法导航

- 先启动 bringup，确认 `/odom`、`/scan` 和 TF。
- 确认地图文件存在。
- 检查 `/initialpose` 是否已设置。
- 查看 `planner_server`、`controller_server`、`bt_navigator` 节点状态。

### UI 无法显示

- 检查开发板本地 `DISPLAY`，通常为 `:0`。
- 远程 SSH 时不要依赖 `localhost:10.0` 显示 PyQt UI。
- 可先关闭 UI，只运行 LLM 任务层命令行检查。

### 语音默认关闭

- 初始化阶段默认 `enable_voice:=false`、`enable_tts:=false`。
- 开启语音前检查麦克风、扬声器、DashScope Key 和网络。

### DashScope Key 未配置

- 未配置 Key 时不要开启需要云端 ASR/TTS/LLM 的功能。
- 仍可用 `/inspection_ai/text_command` 做本地任务事件链路检查。

### 话题没有输出

- 先确认节点是否存在：`ros2 node list`。
- 确认话题名是否为 `/inspection_ai/*`。
- 检查 launch 参数是否关闭了对应节点。

### colcon build 失败

- 确认已 source `/opt/ros/humble/setup.bash`。
- 确认依赖安装完整。
- 先构建单包定位问题：`colcon build --packages-select ylhb_llm`。
- 第三方 ZED/RPLidar 包的依赖问题和本项目 LLM 初始化清理分开处理。

## 13. 后续开发板待验证清单

- CAN 设备识别与 ZLAC8015D 通信。
- 底盘 bringup。
- 雷达 `/scan`。
- IMU 数据。
- EKF/TF。
- SLAM Toolbox 建图。
- Nav2 导航。
- ZED 相机。
- 感知节点。
- UI 显示。
- 语音输入/输出。
- `/inspection_ai/*` 任务事件链路。
