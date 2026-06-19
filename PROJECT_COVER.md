# 电力行业巡检机器人项目封面

## 项目名称

电力行业巡检机器人

## 项目定位

面向电力实训和变电站巡检场景的移动机器人系统。项目以 ROS 2 和 Jetson Orin Nano Super 为核心，集成底盘控制、建图导航、视觉感知、语音交互、LLM 任务解析、巡检任务下发、告警展示和巡检记录生成能力。

## 核心目标

- 实现机器人自主建图、定位、导航和避障。
- 支持巡检路线和检查点任务框架。
- 支持路线级安全检测和检查点级设备检测的后续扩展。
- 支持语音提示、任务状态展示和异常告警。
- 预留 LocateAnything-3B 和 LingBot-Map 接入位置。

## 技术栈

| 类型 | 技术 |
|---|---|
| 系统 | Ubuntu 22.04, ROS 2 Humble |
| 主控 | Jetson Orin Nano Super |
| 底盘 | ZLAC8015D V4, SocketCAN/CANopen |
| 导航 | RPLidar, IMU, SLAM Toolbox, Nav2, AMCL, EKF |
| 视觉 | ZED 2i, YOLO/TensorRT, OpenCV |
| LLM/语音 | DashScope/Qwen, ASR, TTS, 唤醒式连续语音 |
| UI | PyQt 巡检控制台 |
| 三维预留 | LingBot-Map |

## 巡检流程

```text
建图定位 -> 路线规划 -> 任务下发 -> 自主巡航 -> 路线级检测 -> 到达检查点 -> 语音提示 -> 图像采集 -> 检查点检测 -> 告警/记录
```

## 当前交付

- 已完成项目仓库切换和文档重写。
- 已整理 LLM 层为巡检任务框架，包含任务事件、语音、控制台和系统管理。
- 已保留 LLM 层框架，包括任务事件、语音输入、语音路由、TTS、UI 和 system supervisor。
- 已统一新话题命名为 `/inspection_ai/*`。

## 后续扩展方向

- 巡检路线和检查点配置。
- 检查点状态机。
- 检测服务 API。
- 告警等级和巡检记录。
- 三维地图展示。
