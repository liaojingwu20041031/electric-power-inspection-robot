# ROS2 Smart Car on Jetson

当前 ROS 2 工作区以 Jetson Orin Nano Super 本机开发、构建、运行为主，不再使用旧的 RDK X5 / PC 传板流程。

默认工作区路径是 `~/ros2_ws`。如果克隆到其他目录，运行脚本前设置：

```bash
export WS_DIR=/path/to/ros2_ws
```

## 主要目录

- `ylhb_base/`：底盘、IMU、URDF、EKF、RPLidar bringup、SLAM、Nav2
- `ylhb_perception/`：ZED 图像订阅、YOLO 优化模型推理、深度融合入口
- `zed-ros2-wrapper/`：第三方 ZED 2i ROS 2 wrapper
- `rplidar_ros-ros2/`：第三方 RPLidar ROS 2 驱动
- `PROJECT_DOC_zh.md`：当前 Jetson 本机运行文档
- `bind_usb.sh`：IMU / RPLidar 串口别名绑定脚本
- `my_map.yaml`、`my_map.pgm`：默认 Nav2 地图

## 快速开始

```bash
cd ~/ros2_ws
./scripts/install_jetson_dependencies.sh
./scripts/build_on_jetson.sh
```

`scripts/run_on_jetson.sh` 会自动加载 `/opt/ros/$ROS_DISTRO/setup.bash` 和 `install/setup.bash`。日常启动不需要手动 `source`；如果当前终端开了 `set -u`，手动 source ROS 环境可能触发 `AMENT_TRACE_SETUP_FILES: unbound variable`。

构建脚本会在 `colcon build` 阶段设置 `PYTHONNOUSERSITE=1`，避免用户级 `pip` 包覆盖 ROS/Ubuntu 构建工具。自研包验证命令：

```bash
PYTHONNOUSERSITE=1 colcon test \
  --packages-select ylhb_base ylhb_perception ylhb_llm ylhb_interfaces \
  --event-handlers console_direct+
```

ZED/RPLidar 第三方源码用于比赛部署构建；完整 `colcon test` 可能因 vendor lint 或离线 schema 校验失败，不作为自研项目质量门槛。

启动底盘、IMU、雷达、URDF、EKF：

```bash
./scripts/run_on_jetson.sh bringup
```

启动 ZED 2i：

```bash
./scripts/run_on_jetson.sh zed
```

启动视觉检测：

```bash
./scripts/run_on_jetson.sh perception \
  model_path:=/home/nvidia/ros2_ws/src/ylhb_perception/models/yolo26.engine \
  backend:=tensorrt \
  imgsz:=960 \
  confidence_threshold:=0.35 \
  max_det:=20 \
  half:=true \
  publish_debug_image:=false \
  log_interval_sec:=2.0 \
  device:=cuda:0
```

启动大模型任务层：

```bash
export DASHSCOPE_API_KEY=你的DashScopeKey
./scripts/run_on_jetson.sh llm enable_voice:=false enable_tts:=false
```

eMeet Luna 语音输入/播报：

```bash
./scripts/run_on_jetson.sh llm \
  enable_voice:=true \
  enable_tts:=true \
  audio_input_device:=plughw:CARD=Luna,DEV=0 \
  audio_output_device:=plughw:CARD=Luna,DEV=0
```

语音输入采用按钮或服务触发：

```bash
ros2 service call /retail_ai/capture_voice std_srvs/srv/Trigger "{}"
```

启动比赛现场显示屏 UI / 总控台：

```bash
./scripts/run_on_jetson.sh competition
```

比赛现场物理屏幕使用 `DISPLAY=:0`，`competition` 脚本会自动覆盖 SSH X11 转发的 `DISPLAY=localhost:10.0`。远程 X11 调试时使用：

```bash
./scripts/run_on_jetson.sh competition force_local_display:=false fullscreen:=false
```

`competition` 启动时会自动关闭 X11 屏保、空白屏和 DPMS 省电息屏，避免比赛过程中显示屏自动熄灭。若需要手动确认：

```bash
DISPLAY=:0 xset q
```

`competition` 也会默认启动 IBus 拼音输入法。物理屏幕上的系统虚拟键盘仍按屏幕手势调出；项目不再额外启动 Onboard。若需要确认中文输入法：

```bash
ibus engine
ibus list-engine | grep -i pinyin
```

也可以单独启动 UI：

```bash
ros2 run ylhb_llm retail_display_ui_node
```

UI 支持任务 A/B/C/D 启动、B-1 图片预览、建图启动/停止、保存地图、导航/感知/AI 任务层启动、识别结果显示、播报文本、购物车和总价显示。

小尺寸显示屏会自动启用紧凑布局：顶部状态栏换行显示，字号和间距会缩小，主内容支持滚动，避免无边框全屏时界面被固定最小尺寸撑到屏幕外。若现场仍有显示器过扫描导致边缘被裁切，先用窗口化确认：

```bash
./scripts/run_on_jetson.sh competition fullscreen:=false
```

B-1 按钮依赖 `retail_task_node` 提供 `/retail_ai/start_b1_task`。比赛启动后等顶部 `B1服务: 就绪` 再点击；如果只单独运行 UI，不会自动拥有 B-1 服务。

任务书图片分析服务：

```bash
ros2 service call /retail_ai/start_b1_task std_srvs/srv/Trigger "{}"
```

文字任务输入：

```bash
ros2 topic pub --once /retail_ai/text_command std_msgs/msg/String "{data: '来瓶可乐'}"
ros2 topic echo /retail_ai/task_event
ros2 topic echo /retail_ai/say_text
```

启动建图：

```bash
./scripts/run_on_jetson.sh mapping
```

启动导航：

```bash
./scripts/run_on_jetson.sh navigation
```

默认导航地图：

```text
~/ros2_ws/src/my_map.yaml
```

详细说明见 `PROJECT_DOC_zh.md` 和 `../MIGRATION_JETSON.md`。
