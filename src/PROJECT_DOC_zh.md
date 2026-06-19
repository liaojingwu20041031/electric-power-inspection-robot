# 电力行业巡检机器人项目开发文档

平台：Jetson Orin Nano Super  
工作区：`~/ros2_ws`  
核心任务：**建图、导航、巡检检测、语音提示、告警记录**

## 1. 项目概览

本项目是一个电力行业巡检机器人实训工程。当前重点是保留底层机器人能力和 LLM 层框架，保留通用机器人任务框架，为后续实现电力巡检任务状态机做准备。

当前 LLM 层聚焦巡检指令解析、任务事件发布、语音交互和系统控制。

## 2. 主流程

```text
1. 启动底盘、雷达、IMU、URDF、EKF
2. 使用 SLAM Toolbox 建图，或加载已有地图
3. 使用 Nav2 完成定位、路径规划和避障
4. 启动 ZED 与感知节点
5. 启动巡检 LLM 层和控制台
6. 下发巡检指令，例如“开始巡检任务”
7. LLM 层发布通用 TaskEvent
8. 后续执行层根据 TaskEvent 完成路线导航、检查点采集、检测和告警
```

## 3. LLM 层节点

| 节点 | 作用 |
|---|---|
| `inspection_task_node` | 接收文本/语音指令，输出通用巡检任务事件 |
| `inspection_display_ui_node` | 巡检控制台，展示系统状态、任务上下文、感知输出和事件时间线 |
| `voice_input_node` | 单次录音 ASR |
| `voice_session_node` | 唤醒式连续语音会话 |
| `voice_command_router_node` | 将语音事件路由为系统命令、运动指令或巡检任务指令 |
| `voice_output_node` | TTS 播报队列 |
| `basic_motion_command_node` | 前进、后退、左转、右转、停止等基础运动指令 |
| `system_supervisor_node` | 启停底盘、导航、ZED、感知、LLM，保存地图，软件急停 |

## 4. 核心话题

```text
/inspection_ai/text_command          # 文本/语音转写后的任务指令
/inspection_ai/task_event            # 通用巡检任务事件
/inspection_ai/task_status           # 执行层回传任务状态
/inspection_ai/task_context_status   # LLM 任务层上下文状态
/inspection_ai/say_text              # 播报请求
/inspection_ai/voice_status          # 播报状态
/inspection_ai/system_command        # 系统控制命令
/inspection_ai/system_status         # 系统控制状态
/inspection_ai/system_mode           # ready/mapping/running/fault 等模式
/inspection_ai/voice_command_event   # 连续语音 ASR 事件
/inspection_ai/voice_session_status  # 连续语音会话状态
```

## 5. 启动命令

```bash
# 底盘、IMU、雷达、URDF、EKF
./scripts/run_on_jetson.sh bringup

# 建图
./scripts/run_on_jetson.sh mapping

# 导航
./scripts/run_on_jetson.sh navigation

# ZED
./scripts/run_on_jetson.sh zed

# 感知
./scripts/run_on_jetson.sh perception

# 巡检控制台 + LLM/语音/system supervisor
./scripts/run_on_jetson.sh inspection
```

## 6. 指令测试

```bash
ros2 topic pub --once /inspection_ai/text_command std_msgs/msg/String \
  "{data: '{\"source\":\"cli\",\"text\":\"开始巡检任务\"}'}"

ros2 topic echo /inspection_ai/task_event
ros2 topic echo /inspection_ai/task_context_status
ros2 topic echo /inspection_ai/say_text
```

## 7. TaskEvent 使用约定

当前暂时复用 `ylhb_interfaces/msg/TaskEvent`。字段含义在巡检框架中解释为：

| 字段 | 巡检含义 |
|---|---|
| `task_id` | 巡检任务事件 ID |
| `intent` | `start_inspection`, `pause_inspection`, `resume_inspection`, `cancel_inspection`, `manual_takeover`, `inspect_checkpoint`, `emergency_stop`, `inspection_query` |
| `item_id` | 临时作为 route/checkpoint/target ID |
| `item_name` | 临时作为 route/checkpoint/target 名称 |
| `destination` | `route`, `checkpoint` 或后续自定义目的地 |
| `confidence` | 规则或 LLM 解析置信度 |
| `source` | `ui`, `voice`, `cli`, `service` 等 |
| `requires_ack` | 是否需要执行层回传状态 |
| `raw_json` | 巡检任务 JSON，后续可演进为专用消息 |

## 8. 后续开发点

1. 新增站点、区域、路线、检查点配置。
2. 将 `inspection_task_node` 扩展为正式状态机。
3. 增加检查点到达后的语音提示：“已到达指定位置，开始检查”。
4. 增加图像/视频采集与检测服务调用。
5. 增加告警 JSON 和巡检记录 JSON。
6. 接入 LocateAnything-3B 和安全帽/火源等实时检测模型。
7. 接入 LingBot-Map 做三维重建和后台可视化。
