# 电力行业巡检机器人项目封面

## 项目名称

电力行业巡检机器人初始化框架

## 项目定位

本项目面向电力实训和变电站巡检方向，当前阶段完成的是 PC 主机端仓库初始化整理和 ROS 2 智能机器人可复用框架保留。项目尚未在 Jetson/开发板上进行完整实机运行，因此不能视为已经完成的电力巡检业务系统。

## 当前交付

- 仓库已迁移到 `liaojingwu20041031/electric-power-inspection-robot`。
- 文档入口已整理为中文命名，方便实训汇报和后续开发。
- LLM 层已从旧业务语义清理为巡检任务初始化框架。
- 默认任务话题切换为 `/inspection_ai/*`。
- 保留底盘、导航、感知、语音、移动端桥接等可复用 ROS 2 框架。
- 未实现正式巡检状态机、检查点执行闭环、检测服务、告警数据库或巡检报告导出。

## 技术栈概览

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

## 初始化框架边界

当前只提供“可以继续开发”的机器人框架：

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

## 后续扩展方向

- 开发板实机验证底盘、雷达、IMU、ZED、Nav2 和语音链路。
- 设计正式 `/inspection/*` 任务协议和巡检消息。
- 增加站点、区域、路线和检查点配置。
- 接入人员、安全帽、火源、烟雾、障碍物等路线级检测。
- 评估 LocateAnything-3B 用于开关状态、漏油、异物等检查点级定位。
- 评估 LingBot-Map 用于三维空间重建、巡检点位对齐和后台展示。

## 文档入口

- [重点使用调试文档](src/PROJECT_DOC_zh.md)
- [项目概览](docs/项目概览.md)
- [快速使用](docs/快速使用.md)
- [接口约定](docs/接口约定.md)
- [迁移清理记录](docs/迁移清理记录.md)
