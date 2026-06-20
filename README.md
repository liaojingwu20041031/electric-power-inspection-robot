<div align="center">

# 电力行业巡检机器人

### 电力巡检机器人初始化框架

**当前仓库处于 PC 主机端初始化整理阶段，只保留 ROS 2 智能机器人可复用框架，尚未在 Jetson/开发板上完成实机运行验证。**

<p>
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white">
  <img alt="Ubuntu 22.04" src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white">
  <img alt="Jetson Orin Nano" src="https://img.shields.io/badge/Jetson-Orin%20Nano%20Super-76B900?logo=nvidia&logoColor=white">
  <img alt="SocketCAN" src="https://img.shields.io/badge/SocketCAN-ZLAC8015D-1F7A8C">
  <img alt="Nav2" src="https://img.shields.io/badge/Nav2-SLAM%20%2B%20Navigation-5E35B1">
  <img alt="Inspection AI" src="https://img.shields.io/badge/AI-Inspection%20Scaffold-0F766E">
</p>

</div>

---

## 当前定位

本项目由原智能机器人项目迁移为“电力行业巡检机器人”方向。当前交付目标不是完整电力巡检系统，而是完成仓库初始化、文档中文化、旧业务残留清理，并保留后续可继续开发的 ROS 2 机器人框架。

当前阶段明确：

- 已完成 PC 主机端仓库初始化整理。
- 尚未在 Jetson/开发板上实机运行验证。
- 保留底盘、导航、感知、语音、LLM 任务层、移动端桥接等可复用框架。
- 暂不实现正式巡检状态机、检查点导航闭环、检测服务 API、告警数据库或巡检报告导出。
- LocateAnything-3B 与 LingBot-Map 仅作为后续规划方向，当前未接入实际代码。

## 技术栈

| 模块 | 当前保留内容 |
|---|---|
| 机器人底层 | ROS 2 Humble、ZLAC8015D、SocketCAN、RPLidar、IMU、URDF、EKF |
| 建图导航 | SLAM Toolbox、Nav2、AMCL、地图保存入口 |
| 视觉感知 | ZED 2i、YOLO/TensorRT 检测框架、深度定位入口 |
| LLM/语音 | DashScope/Qwen 配置、ASR、TTS、语音路由、任务事件框架 |
| 交互入口 | PyQt 控制台、HTTP/WebSocket mobile bridge |
| 后续规划 | 巡检路线/检查点、LocateAnything-3B、LingBot-Map、告警与记录 |

## 快速开始

推荐工作空间统一使用 `~/ros2_DL`。

```bash
cd ~
git clone https://github.com/liaojingwu20041031/electric-power-inspection-robot.git ros2_DL
cd ~/ros2_DL
```

有 ROS 2 Humble 环境时，可做编译检查：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ros2_DL/install/setup.bash
```

LLM/巡检任务层的初始化检查示例：

```bash
ros2 launch ylhb_llm llm.launch.py enable_voice:=false enable_tts:=false

ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String \
  "{data: '{\"source\":\"cli\",\"text\":\"开始巡检任务\"}'}"
```

Jetson/开发板上的底盘、雷达、IMU、ZED、Nav2、感知和语音链路需要后续实机验证。

## 文档入口

- [重点使用调试文档](src/PROJECT_DOC_zh.md)
- [项目概览](docs/项目概览.md)
- [快速使用](docs/快速使用.md)
- [接口约定](docs/接口约定.md)
- [迁移清理记录](docs/迁移清理记录.md)

## 后续方向

- 在开发板上验证 CAN、底盘、雷达、IMU、EKF、SLAM、Nav2、ZED、感知和语音链路。
- 在现有 `TaskEvent` 框架上逐步设计正式巡检任务协议。
- 后续再扩展站点、区域、路线、检查点和任务下发能力。
- 后续评估 LocateAnything-3B 用于检查点级复杂目标定位。
- 后续评估 LingBot-Map 用于三维空间重建、点位对齐和可视化后台。
