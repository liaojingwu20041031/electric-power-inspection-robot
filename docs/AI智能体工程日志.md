# AI Agent 工程日志

更新时间：2026-07-11

## 2026-07-11：实机运动测试职责边界

- 实地路线、巡逻、导航目标、遥控和任何可能驱动真实机器人运动的操作，必须由用户在现场执行和监护。
- Agent 可执行静态验证、构建、日志读取、ROS 图/TF/参数/costmap 等只读诊断；仅在确认无执行器副作用后，才可做诊断 topic 通信检查。
- 不确定某 ROS 命令是否会改变实机状态时，默认按高风险处理：Agent 只提供命令、观察项和通过标准，等待用户回传结果。
- 该边界由 `skills/robot-motion-test-boundary/SKILL.md` 定义，并由根目录 `AGENTS.md` 强制触发。

`ylhb_llm` 当前使用本地 mini-agent-core 风格内核，不依赖外部运行时。核心链路是：

```text
UI / voice_session_node
-> /inspection_ai/agent_request
-> inspection_agent_node
-> InspectionAgentRuntime
-> Qwen OpenAI-compatible tool calling
-> agent_schema.validate_decision()
-> agent_policy.authorize()
-> agent_tools.execute()
-> /inspection_ai/agent_chat
-> /inspection_ai/agent_event
-> /inspection_ai/agent_status
-> /inspection_ai/say_text
```

## 与 mini-agent-core 的关系

`mini-agent-core` 是作者独立维护的轻量 Agent Core / SDK 模板，面向
OpenAI-compatible tool calling、AgentSpec、ToolPack、工具安全分级和 ROS2-ready
任务编排。本项目没有把它作为 pip 包或外部运行时依赖，而是在 `ylhb_llm` 中实现了
本地化的 mini-agent-core 风格运行时。

这里的 Agent 只负责语音/UI 意图解析、任务级工具选择和受控执行编排。ROS 2 控制、
急停、导航、巡逻、路线解析和基础运动技能仍保留在机器人项目内部，由
system supervisor、patrol executor、RouteToolPack、SkillToolPack 和本地安全策略承接。

## 关键文件

- `src/ylhb_llm/ylhb_llm/inspection_agent_node.py`
- `src/ylhb_llm/ylhb_llm/inspection_agent_runtime.py`
- `src/ylhb_llm/ylhb_llm/inspection_agent_spec.py`
- `src/ylhb_llm/ylhb_llm/agent_chat_schema.py`
- `src/ylhb_llm/ylhb_llm/agent_schema.py`
- `src/ylhb_llm/ylhb_llm/agent_policy.py`
- `src/ylhb_llm/ylhb_llm/agent_tools.py`
- `src/ylhb_llm/config/robot_capabilities.yaml`
- `src/ylhb_llm/qml/pages/VoiceAiPage.qml`

## Topics

- `/inspection_ai/agent_request`：输入，`std_msgs/String` JSON。
- `/inspection_ai/agent_chat`：聊天记录，`schema_version, turn_id, client_msg_id, role, text, intent, tool_name, status, timestamp, source, raw`。
- `/inspection_ai/agent_status`：latched 状态 JSON，包含 `agent_spec_summary`。
- `/inspection_ai/agent_event`：工具执行结果 `ToolResult`。
- `/inspection_ai/say_text`：TTS 播报。

## 参数

```yaml
inspection_agent_node:
  ros__parameters:
    enable_llm_planner: true
    offline_safe_mode: true
    agent_chat_topic: /inspection_ai/agent_chat
```

`enable_llm_fallback` 已删除。Planner 不可用时只执行本地急停/停止安全反射；其它自然语言会发布 system chat：`LLM Planner 不可用，未执行动作。`

## 安全默认值

- Planner 工具不暴露 `send_motion_command`。
- 直接拒绝 `/cmd_vel`、`cmd_vel`、`nav2_goal`、`delete_map`、`edit_route`。
- 急停词直接走本地 `emergency_stop`，不调用 LLM。
- 普通停止词直接走 `stop_motion`。
- 相对旋转和移动只走 `rotate_relative`、`move_relative`，参数范围由 schema/policy 双层限制。
- `go_to_checkpoint.target_id` 必须能从 `RouteCatalog` 解析到真实 target。

## 调试

重复播报或重复执行时先确认没有启动多个 agent：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_DL/install/setup.bash
ros2 node list --no-daemon
```

核心测试：

```bash
python3 -m pytest src/ylhb_llm/test/test_inspection_agent_spec.py src/ylhb_llm/test/test_inspection_agent_runtime.py src/ylhb_llm/test/test_agent_chat_schema.py -q
python3 -m pytest src/ylhb_llm/test/test_inspection_agent_node.py src/ylhb_llm/test/test_agent_schema.py src/ylhb_llm/test/test_agent_policy.py src/ylhb_llm/test/test_agent_tools.py src/ylhb_llm/test/test_ui_backend.py src/ylhb_llm/test/test_qml_navigation.py -q
```

## 智能运维助手升级（2026-07-20）

保留 `InspectionAgentRuntime`、`AgentOperationManager`、策略授权和现有 ROS 通信协议，新增四个受控工具：

- `search_robot_help`：只读取工作区白名单 Markdown，返回相对来源路径。
- `get_connection_info`：复用 `NetworkStatusProvider`，读取实时接口、APP 地址、Bridge、本地 APP 和云状态。
- `run_self_check`：用确定性规则区分观测证据、可能原因、人工动作和可恢复性，不调用 LLM。
- `recover_component`：只向 Supervisor 发布白名单组件恢复命令，并等待真实 Operation 终态。

恢复配置位于 `config/agent_recovery.yaml`，首版仅允许 `perception` 和 Supervisor 管理的 `mobile_bridge`。底盘、CAN、Nav2、bringup、电源和线路故障禁止自动恢复。健康监控发布 `/inspection_ai/health_status` 与 `/inspection_ai/diagnostic_event`，默认开启只读监控、关闭自动恢复。
