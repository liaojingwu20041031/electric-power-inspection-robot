# AI Agent 工程日志

更新时间：2026-07-05

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
