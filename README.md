# YLHB Smart Retail Robot

基于 ROS 2 和 Jetson Orin Nano Super 的智能零售机器人项目，面向比赛场景构建，覆盖底盘控制、激光雷达建图、Nav2 自主导航、ZED 2i 视觉感知、YOLO 商品识别、大模型任务理解和现场显示屏总控台。

本仓库是一个完整 ROS 2 工作区源码仓库。`build/`、`install/`、`log/` 和模型二进制文件不随仓库提交，克隆后需要在目标 Jetson 上本机构建。

默认工作区路径是 `~/ros2_ws`。如果克隆到其他目录，运行脚本前设置：

```bash
export WS_DIR=/path/to/ros2_ws
```

## 项目能力

- 底盘与传感器：底盘串口控制、IMU 驱动、RPLidar 激光雷达、URDF 机器人模型、EKF 融合。
- 建图与导航：SLAM Toolbox 建图、地图保存、AMCL 定位、Nav2 路径规划与控制。
- 视觉感知：ZED 2i 图像订阅、YOLO26 商品检测、TensorRT 推理、调试窗口和检测结果发布。
- AI 任务层：DashScope/Qwen 图片理解、文字或语音任务解析、商品推荐、结算播报和任务事件流。
- 比赛 UI：任务 A/B/C/D 启动入口、B-1 图片预览、建图/导航/感知/AI 控制、识别结果、购物车和总价显示。

## 目录结构

```text
.
├── scripts/
│   ├── install_jetson_dependencies.sh
│   ├── build_on_jetson.sh
│   └── run_on_jetson.sh
├── src/
│   ├── ylhb_base/             # 底盘、IMU、URDF、EKF、SLAM、Nav2
│   ├── ylhb_perception/       # ZED 图像订阅、YOLO/TensorRT 感知、深度定位
│   ├── ylhb_llm/              # 大模型任务层、语音输入输出、比赛显示屏 UI
│   ├── ylhb_interfaces/       # 项目自定义 ROS 2 消息
│   ├── rplidar_ros-ros2/      # 第三方 RPLidar ROS 2 驱动源码
│   ├── zed-ros2-wrapper/      # 第三方 ZED ROS 2 wrapper 源码
│   ├── PROJECT_DOC_zh.md      # 详细中文开发和比赛调试文档
│   ├── my_map.yaml
│   └── my_map.pgm
└── MIGRATION_JETSON.md        # Jetson 本机化迁移说明
```

## 硬件与软件环境

- 主控：Jetson Orin Nano Super
- 系统：Ubuntu + ROS 2 Humble
- 相机：ZED 2i
- 雷达：RPLidar
- 推理：CUDA / TensorRT
- AI 服务：DashScope API

> DashScope API Key 不写入代码。运行大模型任务层前在终端设置 `DASHSCOPE_API_KEY`。

## 快速开始

在 Jetson 上克隆仓库：

```bash
cd ~
git clone https://github.com/liaojingwu20041031/ylhb-smart-retail-robot.git ros2_ws
cd ~/ros2_ws
```

安装依赖并构建：

```bash
./scripts/install_jetson_dependencies.sh
./scripts/build_on_jetson.sh
```

`scripts/run_on_jetson.sh` 会自动加载 `/opt/ros/$ROS_DISTRO/setup.bash` 和 `install/setup.bash`。日常启动不需要手动 `source`。

构建脚本会在 `colcon build` 阶段设置 `PYTHONNOUSERSITE=1`，避免用户目录中通过 `pip --user` 安装的 Python 包覆盖 ROS/Ubuntu 自带构建工具。

自研包验证命令：

```bash
PYTHONNOUSERSITE=1 colcon test \
  --packages-select ylhb_base ylhb_perception ylhb_llm ylhb_interfaces \
  --event-handlers console_direct+
```

仓库包含 ZED/RPLidar 第三方源码。第三方包用于部署构建，默认不把 vendor lint 作为项目质量门槛；完整 `colcon test` 可能因第三方包格式规则或离线 schema 校验失败。

## 常用启动命令

启动底盘、IMU、雷达、URDF 和 EKF：

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

接入 eMeet Luna 一体式音箱/麦克风后，先确认系统能看到声卡：

```bash
lsusb | grep -i emeet
arecord -l
aplay -l
```

当前项目推荐使用唤醒式连续语音模式。UI 中点击“开启语音模式”后，机器人本地监听人声，听到“小零小零”或“小玲小玲”等唤醒词后进入会话；后续语音会先发布结构化事件，再由路由器过滤后进入 `/retail_ai/text_command`。eMeet Luna 枚举为 `CARD=Luna` 时启动：

```bash
export DASHSCOPE_API_KEY=你的DashScopeKey
./scripts/run_on_jetson.sh llm \
  enable_voice:=true \
  enable_voice_session:=true \
  enable_tts:=true \
  audio_input_device:=plughw:CARD=Luna,DEV=0 \
  audio_output_device:=plughw:CARD=Luna,DEV=0 \
  tts_voice:=Serena
```

也可以开启按键式单次录音服务，作为 ASR 调试入口：

```bash
./scripts/run_on_jetson.sh llm \
  enable_voice:=false \
  enable_capture_voice:=true \
  enable_tts:=false \
  audio_input_device:=plughw:CARD=Luna,DEV=0
```

```bash
ros2 service call /retail_ai/capture_voice std_srvs/srv/Trigger "{}"
```

连续语音模式服务和调试话题：

```bash
ros2 service call /retail_ai/start_voice_session std_srvs/srv/Trigger "{}"
ros2 service call /retail_ai/stop_voice_session std_srvs/srv/Trigger "{}"
ros2 topic echo /retail_ai/voice_session_status
ros2 topic echo /retail_ai/voice_command_event
```

启动比赛现场显示屏 UI / 总控台：

```bash
./scripts/run_on_jetson.sh competition
```

`competition` 默认启用 eMeet Luna 连续语音会话和 TTS 播报，播报默认使用 `Serena` 女声音色。启动后在 UI 的“系统控制”页点击“一键启动比赛节点”，由总控台启动底盘/雷达、ZED、感知和导航；不需要再手动开其它终端。若现场需要临时静音：

`competition` 的实际默认参数是：`enable_voice:=true`、`enable_voice_session:=true`、`enable_capture_voice:=false`、`enable_tts:=true`、`audio_input_device:=plughw:CARD=Luna,DEV=0`、`audio_output_device:=default`。按键式 `/retail_ai/capture_voice` 默认关闭，比赛交互走唤醒式连续语音。

```bash
./scripts/run_on_jetson.sh competition enable_tts:=false
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

UI 的“保存地图”默认写入 `~/ros2_ws/src/maps/<map_name>.yaml/.pgm`；导航默认仍读取 `~/ros2_ws/src/my_map.yaml`。要使用新保存的地图，需要复制成默认地图或启动导航时覆盖 `map:=...`。

## 比赛任务流程

项目按比赛任务 A/B/C/D 组织运行：

- 任务 A：语音、文字或键盘命令转换为 `/cmd_vel`，完成前进、后退、转向、停止等基础动作。
- 任务 B-1：导入任务书图片，大模型理解任务，导航到货架 A，识别真实商品，推荐商品，抓取后前往结算区 B。
- 任务 B-2：接收购物需求或商品指令，大模型像销售员一样给出主推和备选商品；连续语音模式需要用户确认后才发布取货事件，然后导航到货架 A，抓取商品，前往结算区 B 并返回起点 S。
- 任务 C：识别结算区商品，播报商品清单，根据 `products.yaml` 计算总价并返回起点 S。
- 任务 D：通过现场显示屏 UI 展示任务状态、识别结果、播报文本、购物车和结算信息。

任务书图片分析服务：

任务书图片放在 `/home/nvidia/ros2_ws/src/ylhb_llm/test_images`，目录内只保留一张 `.jpg/.jpeg/.png`：

```bash
ros2 service call /retail_ai/start_b1_task std_srvs/srv/Trigger "{}"
```

主要 AI/语音/UI 话题和服务：

```text
/retail_ai/task_event              # TaskEvent，inspect_shelf_for_recommendation / pick_item / checkout / return_start
/retail_ai/task_status             # TaskStatus，执行层回传 started/succeeded/failed/rejected
/retail_ai/sales_dialogue_status   # B-2 销售对话状态 JSON
/retail_ai/voice_command_event     # 连续语音 ASR 事件 JSON
/retail_ai/voice_session_status    # 连续语音会话状态 JSON
/retail_ai/capture_voice           # 单次录音 ASR service，competition 默认关闭
/retail_ai/start_voice_session     # 开启连续语音 service
/retail_ai/stop_voice_session      # 关闭连续语音 service
```

文字入口仍兼容纯文本；连续语音模式会先生成结构化事件，再由路由器发布结构化 `/retail_ai/text_command`：

```bash
ros2 topic pub --once /retail_ai/text_command std_msgs/msg/String "{data: '来瓶可乐'}"
ros2 topic pub --once /retail_ai/text_command std_msgs/msg/String "{data: '我口渴了'}"
ros2 topic pub --once /retail_ai/text_command std_msgs/msg/String "{data: '确认'}"
ros2 topic echo /retail_ai/task_event
ros2 topic echo /retail_ai/sales_dialogue_status
ros2 topic echo /retail_ai/say_text
```

## 模型文件说明

模型二进制文件不提交到 GitHub。需要在 Jetson 上准备：

```text
/home/nvidia/ros2_ws/src/ylhb_perception/models/yolo26.onnx
/home/nvidia/ros2_ws/src/ylhb_perception/models/yolo26.engine
```

PC 端导出 ONNX 后，将 `yolo26.onnx` 放入 `src/ylhb_perception/models/`，再在 Jetson 上编译 TensorRT engine：

```bash
ros2 run ylhb_perception export_yolo_trt.py \
  --onnx /home/nvidia/ros2_ws/src/ylhb_perception/models/yolo26.onnx \
  --output /home/nvidia/ros2_ws/src/ylhb_perception/models/yolo26.engine \
  --workspace 2048
```

## 显示屏注意事项

比赛现场物理屏幕使用：

```bash
export DISPLAY=:0
```

`competition` 启动脚本会自动将 SSH X11 转发的 `DISPLAY=localhost:10.0` 切换为本机显示 `:0`，并关闭 X11 屏保、空白屏和 DPMS 省电息屏。

远程 X11 调试时使用：

```bash
./scripts/run_on_jetson.sh competition force_local_display:=false fullscreen:=false
```

## 第三方组件

本仓库包含以下第三方 ROS 2 包源码，便于比赛部署时直接构建：

- `src/zed-ros2-wrapper/`：Stereolabs ZED ROS 2 wrapper
- `src/rplidar_ros-ros2/`：Slamtec RPLidar ROS 2 driver

第三方组件的许可证文件保留在各自目录中。业务逻辑主要放在 `ylhb_base`、`ylhb_perception`、`ylhb_llm` 和 `ylhb_interfaces`。

## 安全与规范

- 不提交 `DASHSCOPE_API_KEY`、`.env`、SSH key、证书或其他本机密钥。
- 不提交 `.onnx`、`.engine`、`.pt` 等模型二进制文件。
- 串口权限优先使用 udev 规则或用户组，不建议长期使用 `chmod 777`。
- 安全和部署注意事项见 [SECURITY.md](SECURITY.md)。

## 详细文档

- [src/PROJECT_DOC_zh.md](src/PROJECT_DOC_zh.md)：比赛调试顺序、节点关系、话题流向、启动命令和常见问题。
- [MIGRATION_JETSON.md](MIGRATION_JETSON.md)：从旧 PC 推流识别流程迁移到 Jetson 本机开发、本机构建、本机运行的说明。
