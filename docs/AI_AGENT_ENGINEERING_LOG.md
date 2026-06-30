# AI Agent 工程日志

更新时间：2026-06-30

本文记录 `ylhb_llm` 里 AI Agent 工程层的当前实现，方便后续调试和调整。它不是产品说明书，重点是能快速找到链路、配置、风险点和验证命令。

## 当前目标

第一阶段只做本地 Agent 编排层：

- 语音/UI 文本先进入 Agent。
- Agent 产出结构化 `AgentDecision`。
- `agent_schema` 校验字段和工具白名单。
- `agent_policy` 做机器人安全门控。
- `agent_tools` 只调用已有 ROS2 topic。
- 不引入 LangChain、MCP server、数据库或长期记忆。

保留不动的稳定层：

- Nav2、底盘控制、巡逻执行器主状态机。
- `PatrolPage` 主流程。
- 地图和路线保存逻辑。
- `voice_output_node.py` 的 TTS 缓存、队列、分段和中断。

## 当前链路

```text
voice_session_node / voice_input_node / UI text
-> /inspection_ai/voice_command_event 或 /inspection_ai/text_command
-> voice_command_router_node
-> /inspection_ai/agent_request
-> inspection_agent_node
-> agent_schema.validate_decision()
-> agent_policy.authorize()
-> agent_tools.execute()
-> /inspection_ai/system_command 或 /inspection_ai/text_command
-> /inspection_ai/agent_event
-> /inspection_ai/agent_status
-> /inspection_ai/say_text
-> voice_output_node
```

急停是例外：

```text
voice_command_router_node
-> /inspection_ai/system_command command=emergency_stop
-> /inspection_ai/agent_event
```

急停仍直达 `system_supervisor_node`，不等 Agent 推理。

## 关键文件

Agent 核心：

- `src/ylhb_llm/ylhb_llm/inspection_agent_node.py`
- `src/ylhb_llm/ylhb_llm/agent_schema.py`
- `src/ylhb_llm/ylhb_llm/agent_policy.py`
- `src/ylhb_llm/ylhb_llm/agent_tools.py`
- `src/ylhb_llm/ylhb_llm/agent_state.py`

接入点：

- `src/ylhb_llm/ylhb_llm/voice_command_router_node.py`
- `src/ylhb_llm/ylhb_llm/voice_input_node.py`
- `src/ylhb_llm/ylhb_llm/ui_ros_bridge.py`
- `src/ylhb_llm/ylhb_llm/ui_backend.py`
- `src/ylhb_llm/qml/pages/VoiceAiPage.qml`
- `src/ylhb_llm/launch/llm.launch.py`
- `src/ylhb_llm/setup.py`

配置：

- `src/ylhb_llm/config/llm.yaml`
- `src/ylhb_llm/config/agent_tools.yaml`
- `src/ylhb_llm/config/agent_prompts.yaml`
- `src/ylhb_llm/config/voice_intents.yaml`
- `src/ylhb_llm/config/voice_replies.yaml`

测试：

- `src/ylhb_llm/test/test_agent_schema.py`
- `src/ylhb_llm/test/test_agent_policy.py`
- `src/ylhb_llm/test/test_agent_tools.py`
- `src/ylhb_llm/test/test_inspection_agent_node.py`

## Topic 契约

Agent 输入：

- `/inspection_ai/agent_request`，`std_msgs/String`，JSON object。

Agent 输出：

- `/inspection_ai/agent_status`，`std_msgs/String`，latched JSON。
- `/inspection_ai/agent_event`，`std_msgs/String`，JSON `ToolResult`。
- `/inspection_ai/say_text`，`ylhb_interfaces/SayText`。

工具实际调用：

- `/inspection_ai/system_command`：巡逻和系统命令。
- `/inspection_ai/text_command`：短时运动文本，继续交给 `basic_motion_command_node`。

## AgentDecision

必填字段：

```json
{
  "schema_version": "1.0",
  "decision_id": "xxx",
  "response_type": "tool",
  "intent": "start_patrol",
  "safety_level": "normal",
  "tool_call": {
    "name": "start_patrol_mode",
    "arguments": {}
  },
  "speak": "准备开始巡逻。",
  "final_answer": "",
  "need_confirm": false,
  "reason_cn": "用户要求开始巡逻"
}
```

`tool_call.name` 只能是：

- `get_system_status`
- `get_patrol_status`
- `get_voice_status`
- `start_patrol_mode`
- `pause_patrol`
- `resume_patrol`
- `cancel_patrol`
- `emergency_stop`
- `reload_patrol_route`
- `return_ready`
- `send_text_motion`
- `generate_local_status_reply`

`send_text_motion.command` 只能是：

- `前进`
- `后退`
- `左转`
- `右转`
- `停止`

## ToolResult

所有执行、拒绝、异常都应该发布 `ToolResult` 到 `/inspection_ai/agent_event`：

```json
{
  "schema_version": "1.0",
  "tool_name": "pause_patrol",
  "ok": true,
  "status": "executed",
  "message": "已发送系统命令: pause_patrol",
  "data": {},
  "error_code": "",
  "timestamp": 1782810000.0
}
```

## 安全策略

当前策略在 `agent_policy.py`：

- `emergency_stop` 永远允许，`priority=10`，`interrupt=true`。
- `start_patrol_mode` 允许。
- `pause_patrol` 只允许在 `running`、`returning_home`、`waiting_loop`。
- `resume_patrol` 优先要求 `paused`；`unknown` 时允许并提示等待执行器确认。
- `cancel_patrol` 允许在 `running`、`paused`、`returning_home`、`waiting_loop`、`canceling`。
- 普通“停止”走 `send_text_motion(command="停止")`，不会取消巡逻。
- 拒绝 `/cmd_vel`、`cmd_vel`、`nav2_goal`、`delete_map`、`edit_route`。

注意：`agent_tools.py` 会固定 system command 为白名单工具名，即使 arguments 里带了 `command: stop_robot_stack` 也不会覆盖。

## 本地意图规则

当前规则在 `inspection_agent_node.py::decide_local()`：

- `开始巡逻/开始巡检/启动巡逻/启动巡检/执行巡逻/执行巡检` -> `start_patrol_mode`
- `暂停巡逻/暂停巡检` -> `pause_patrol`
- `继续巡逻/继续巡检/恢复巡逻/恢复巡检` -> `resume_patrol`
- `取消巡逻/取消巡检/终止巡逻/终止巡检` -> `cancel_patrol`
- `急停/紧急停止` -> `emergency_stop`
- 单独 `停止/停下/停下来/刹车/停` -> `send_text_motion(command="停止")`
- `前进/后退/左转/右转` -> `send_text_motion`
- `状态/怎么样/情况/进度` -> 本地状态回答
- `你能做什么/有什么功能/功能/怎么用` -> 本地能力回答

复杂巡检知识问答才进入 LLM fallback。当前 `llm.yaml` 默认：

```yaml
enable_llm_fallback: false
```

## 启动和回退

默认启动 Agent：

```bash
ros2 launch ylhb_llm llm.launch.py
```

关闭 Agent 回退旧链路：

```bash
ros2 launch ylhb_llm llm.launch.py enable_inspection_agent:=false
```

语音路由兼容开关：

```yaml
voice_command_router_node:
  ros__parameters:
    enable_inspection_agent: true
    publish_legacy_text_command: false
```

`publish_legacy_text_command: true` 会让 router 同时发旧 `/inspection_ai/text_command`。调试重复执行时优先检查这里。

## 常用调试命令

看 Agent 状态：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_DL/install/setup.bash
ros2 topic echo /inspection_ai/agent_status
```

看工具事件：

```bash
ros2 topic echo /inspection_ai/agent_event
```

手动发开始巡逻：

```bash
ros2 topic pub --once /inspection_ai/agent_request std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"text\":\"开始巡逻\"}'}"
```

手动发普通停止：

```bash
ros2 topic pub --once /inspection_ai/agent_request std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"text\":\"停止\"}'}"
```

看 supervisor 命令：

```bash
ros2 topic echo /inspection_ai/system_command
```

看短时运动文本：

```bash
ros2 topic echo /inspection_ai/text_command
```

## 验证命令

只跑 Agent 相关测试：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/home/nvidia/ros2_DL/src/ylhb_llm:/home/nvidia/ros2_DL/src/ylhb_mobile_bridge:$PYTHONPATH \
python3 -m pytest -q \
  src/ylhb_llm/test/test_agent_schema.py \
  src/ylhb_llm/test/test_agent_policy.py \
  src/ylhb_llm/test/test_agent_tools.py \
  src/ylhb_llm/test/test_inspection_agent_node.py
```

跑 `ylhb_llm` 全量测试：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/home/nvidia/ros2_DL/src/ylhb_llm:/home/nvidia/ros2_DL/src/ylhb_mobile_bridge:$PYTHONPATH \
python3 -m pytest -q src/ylhb_llm/test
```

当前最后一次验证结果：

```text
108 passed
```

## 调整建议

增加一个新语音控制命令时：

1. 在 `agent_schema.py` 加白名单工具或参数约束。
2. 在 `agent_policy.py` 加安全门控。
3. 在 `agent_tools.py` 映射到已有 ROS topic。
4. 在 `inspection_agent_node.py::decide_local()` 加本地意图。
5. 加一条最小测试，优先测真实事故风险。

调整巡逻状态门控时：

- 优先改 `agent_policy.py`。
- 不要在多个 caller 里散落状态判断。
- 测试覆盖允许状态和拒绝状态即可，不锁死完整中文文案。

启用 LLM fallback 前：

- 确认 `DASHSCOPE_API_KEY` 已设置。
- 把 `enable_llm_fallback` 改为 `true`。
- 先用非控制类问题测试。
- 控制类命令仍应走本地规则，不依赖网络。

## 已知边界

- 第一阶段没有多步工具计划。
- 没有长期记忆。
- 没有 MCP server。
- 没有数据库。
- 复杂问答默认不联网。
- UI 只显示简洁 Agent 状态，详细信息看 `/inspection_ai/agent_event`。

