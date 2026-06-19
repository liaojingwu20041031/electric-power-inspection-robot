<div align="center">

# 电力行业巡检机器人

### ROS 2 + Jetson Orin Nano Super 驱动的变电站智能巡检机器人

**面向电力实训场景的移动巡检机器人系统，覆盖建图导航、巡检任务、视觉检测、语音提示、后台告警和巡检记录。**

<p>
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white">
  <img alt="Ubuntu 22.04" src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white">
  <img alt="Jetson Orin Nano" src="https://img.shields.io/badge/Jetson-Orin%20Nano%20Super-76B900?logo=nvidia&logoColor=white">
  <img alt="SocketCAN" src="https://img.shields.io/badge/SocketCAN-ZLAC8015D-1F7A8C">
  <img alt="Nav2" src="https://img.shields.io/badge/Nav2-SLAM%20%2B%20Navigation-5E35B1">
  <img alt="Inspection AI" src="https://img.shields.io/badge/AI-Inspection%20Task%20Layer-0F766E">
  <img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue">
</p>

巡检主线：建图定位 -> 路线规划 -> 任务下发 -> 自主巡航 -> 路线级安全检测 -> 检查点采集 -> 检查点级识别 -> 告警与记录

</div>

---

## 项目目标

本项目面向电力行业巡检实训，目标是实现一个可运行、可扩展、便于继续改造的机器人系统框架。

| 模块 | 目标 |
|---|---|
| 底盘与导航 | 使用 ZLAC8015D、RPLidar、IMU、SLAM Toolbox、Nav2 完成建图、定位、导航和避障 |
| 巡检任务层 | 保留 LLM/语音/任务事件框架，提供通用巡检任务能力，提供巡检任务脚手架 |
| 路线级检测 | 后续接入人员、安全帽、火源、烟雾、障碍物等实时检测 |
| 检查点检测 | 后续接入开关/刀闸状态、表计/指示灯、漏油、异物、烟火等检测 |
| 语音交互 | 支持唤醒、ASR、TTS，到点提示和异常提示可继续扩展 |
| 后台/UI | 提供巡检节点启停、文本/语音指令、任务事件、感知输出和状态展示 |
| 三维空间 | 预留 LingBot-Map 用于三维重建、点位对齐和后台三维可视化 |

---

## 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| `ylhb_base` | 可复用 | 底盘、IMU、URDF、EKF、SLAM、Nav2 |
| `ylhb_perception` | 可复用 | ZED 图像输入、YOLO/TensorRT 检测、深度定位入口 |
| `ylhb_llm` | 已清理业务 | 提供巡检任务事件、语音和控制台能力，只保留巡检 LLM 框架 |
| 语音链路 | 可复用 | `/inspection_ai/*` 话题下的语音输入、语音路由和 TTS 输出 |
| UI 控制台 | 已切换 | 新入口为 `inspection_display_ui_node` |
| 任务节点 | 已切换 | 新入口为 `inspection_task_node`，输出通用巡检 `TaskEvent` |

---

## LLM 层框架

LLM 层现在只保留通用能力，面向巡检任务扩展：

```text
src/ylhb_llm/
├── ylhb_llm/
│   ├── inspection_task_node.py          # 通用巡检任务事件脚手架
│   ├── inspection_display_ui_node.py    # 巡检控制台 UI
│   ├── qwen_client.py                   # 通用 LLM/ASR/TTS 客户端
│   ├── voice_input_node.py              # 单次录音 ASR
│   ├── voice_session_node.py            # 唤醒式连续语音会话
│   ├── voice_command_router_node.py     # 语音命令路由
│   ├── voice_output_node.py             # TTS 播报队列
│   ├── basic_motion_command_node.py     # 基础运动文字/语音指令
│   └── system_supervisor_node.py        # 机器人节点启停与地图保存
├── config/llm.yaml                      # 巡检任务层配置
└── launch/llm.launch.py                 # LLM 层启动入口
```

默认话题统一切换为 `/inspection_ai/*`：

```text
/inspection_ai/text_command
/inspection_ai/task_event
/inspection_ai/task_status
/inspection_ai/say_text
/inspection_ai/task_context_status
/inspection_ai/system_command
/inspection_ai/system_status
/inspection_ai/system_mode
/inspection_ai/voice_command_event
/inspection_ai/voice_session_status
/inspection_ai/voice_status
```

---

## 快速开始

默认工作区路径建议使用 `~/ros2_ws`。

```bash
cd ~
git clone https://github.com/liaojingwu20041031/electric-power-inspection-robot.git ros2_ws
cd ~/ros2_ws
./scripts/install_jetson_dependencies.sh
./scripts/build_on_jetson.sh
```

配置 CAN：

```bash
./scripts/setup_zlac_can.sh can1 500000
ip -br link show can1
```

启动模式：

```text
bringup       启动底盘、IMU、雷达、URDF、EKF
mapping       启动 SLAM Toolbox 建图
navigation    启动 Nav2 定位和导航
zed           启动 ZED 2i wrapper
perception    启动视觉感知节点
llm           启动巡检 LLM/语音/UI 相关节点
inspection    启动巡检控制台、system supervisor 和内嵌任务层
teleop        启动键盘遥控
```

现场入口：

```bash
./scripts/run_on_jetson.sh inspection
```

发送巡检指令示例：

```bash
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String \
  "{data: '{\"source\":\"cli\",\"text\":\"开始巡检任务\"}'}"

ros2 topic echo /inspection_ai/task_event
ros2 topic echo /inspection_ai/task_context_status
ros2 topic echo /inspection_ai/say_text
```

---

## 后续开发建议

1. 在 `inspection_task_node.py` 中把通用 intent 扩展为正式巡检状态机。
2. 新增站点、区域、路线、检查点配置文件。
3. 接入安全帽、人员、火源、烟雾、障碍物等路线级检测。
4. 接入 LocateAnything-3B 或专项模型做检查点级检测。
5. 将 `TaskEvent.raw_json` 中的巡检 JSON 固化成专用 ROS 消息。
6. 逐步接入 LingBot-Map 做三维重建和点位对齐。

---

## 参考

- [项目封面文档](PROJECT_COVER.md)
- [详细开发文档](src/PROJECT_DOC_zh.md)
- [电力巡检迁移计划](docs/electric_power_inspection_plan.md)
- [LocateAnything-3B](https://huggingface.co/nvidia/LocateAnything-3B)
- [LingBot-Map](https://github.com/robbyant/lingbot-map)
