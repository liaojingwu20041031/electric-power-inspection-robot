# AI Agent 工程调试手册

适用工作区：`~/ros2_DL`  
适用项目：`electric-power-inspection-robot`  
目标模块：`src/ylhb_llm` 的语音、任务、Agent、工具调用、播报与 UI 诊断层。

> 本手册只描述 AI Agent 工程层的调试方法。导航、巡逻、底盘、Nav2 参数、地图和感知模型调试继续参考 `src/PROJECT_DOC_zh.md`。

---

## 1. 设计目标

本项目的 AI Agent 不直接控制底盘，也不直接发布连续 `/cmd_vel`。它负责：

1. 理解用户语音/文本输入。
2. 输出严格 JSON 格式的 `AgentDecision`。
3. 由本地 validator 校验 JSON 结构。
4. 由 `agent_policy` 做安全门控。
5. 由 `agent_tools` 调用白名单 ROS 工具。
6. 根据 `ToolResult` 播报、解释、生成状态回答或报告草稿。

核心原则：

```text
LLM 负责理解、规划、解释；
ROS 工具负责执行；
Safety Governor 负责拦截；
JSON Schema 负责让输出可验证、可回放、可调试。
```

---

## 2. 推荐运行链路

```text
voice_session_node / voice_input_node / UI 文本输入
        ↓
voice_command_event 或 agent_request
        ↓
inspection_agent_node
        ↓
AgentDecision JSON
        ↓
agent_schema.validate()
        ↓
agent_policy.authorize()
        ↓
agent_tools.execute()
        ↓
ToolResult JSON
        ↓
SayText / agent_status / agent_event
```

---

## 3. 为什么必须严格 JSON

不要让 LLM 输出“我准备帮你启动巡逻”这种自然语言再由代码猜测含义。Agent 必须输出机器可处理的 JSON：

```json
{
  "schema_version": "1.0",
  "decision_id": "dec_20260630_001",
  "response_type": "tool_call",
  "intent": "patrol_start",
  "safety_level": "normal",
  "tool_call": {
    "name": "start_patrol_mode",
    "arguments": {
      "profile": "navigation"
    }
  },
  "speak": {
    "reply_key": "command.patrol_start",
    "text": "已发送开始巡逻命令。",
    "priority": 7,
    "interrupt": false
  },
  "need_confirm": false,
  "reason_cn": "用户明确要求开始巡逻。"
}
```

执行层只看结构化字段，不解析自然语言。

---

## 4. AgentDecision 协议

建议文件：

```text
src/ylhb_llm/ylhb_llm/agent_schema.py
```

### 4.1 顶层字段

| 字段 | 类型 | 必填 | 说明 |
|---|---|---:|---|
| `schema_version` | string | 是 | 当前固定为 `1.0` |
| `decision_id` | string | 是 | 一次决策唯一 ID |
| `response_type` | enum | 是 | `tool_call` / `final_answer` / `status_reply` / `need_confirm` / `reject` / `ignore` |
| `intent` | string | 是 | 归一化意图，如 `patrol_start` |
| `safety_level` | enum | 是 | `emergency` / `normal` / `requires_confirm` / `blocked` |
| `tool_call` | object/null | 是 | 工具调用 |
| `speak` | object/null | 是 | 播报指令 |
| `final_answer` | string | 是 | 复杂问答文本，非问答时为空 |
| `need_confirm` | bool | 是 | 是否等待二次确认 |
| `reason_cn` | string | 是 | 简短中文原因，不能放推理链 |

### 4.2 response_type 规则

| response_type | 允许动作 |
|---|---|
| `tool_call` | 执行白名单工具 |
| `status_reply` | 只读取状态并播报 |
| `final_answer` | 只回答，不执行工具 |
| `need_confirm` | 不执行工具，进入等待确认 |
| `reject` | 不执行工具，说明拒绝原因 |
| `ignore` | 不执行工具，不播报或低优先级提示 |

---

## 5. ToolCall 协议

```json
{
  "name": "pause_patrol",
  "arguments": {}
}
```

### 5.1 白名单工具

第一阶段只允许这些工具：

| 工具 | 参数 | ROS 行为 |
|---|---|---|
| `get_system_status` | `{}` | 读取 `/inspection_ai/system_status` 缓存 |
| `get_patrol_status` | `{}` | 读取 `/patrol/status` 缓存 |
| `get_voice_status` | `{}` | 读取 `/inspection_ai/voice_session_status` 缓存 |
| `start_patrol_mode` | `{"profile":"navigation"|"inspection"}` | 发布 `/inspection_ai/system_command` |
| `pause_patrol` | `{}` | 发布 `pause_patrol` |
| `resume_patrol` | `{}` | 发布 `resume_patrol` |
| `cancel_patrol` | `{}` | 发布 `cancel_patrol` |
| `emergency_stop` | `{}` | 发布 `emergency_stop`，高优先级 |
| `reload_patrol_route` | `{}` | 发布 `reload_patrol_route` |
| `return_ready` | `{}` | 发布 `return_ready` |
| `send_text_motion` | `{"command":"前进"|"后退"|"左转"|"右转"|"停止"}` | 发文本运动命令 |
| `generate_local_status_reply` | `{"query":"..."}` | 本地模板生成状态回答 |

禁止第一阶段实现：

```text
LLM 直接发布 /cmd_vel
LLM 直接生成 Nav2 goal
LLM 修改路线文件
LLM 删除地图
LLM 绕过 system_supervisor
LLM 直接调用底盘驱动
```

---

## 6. ToolResult 协议

每个工具都返回结构化结果：

```json
{
  "schema_version": "1.0",
  "tool_name": "start_patrol_mode",
  "ok": true,
  "status": "sent",
  "message": "已发送开始巡逻命令。",
  "data": {
    "command": "start_patrol_mode",
    "profile": "navigation"
  },
  "error_code": "",
  "timestamp": 1782816000.0
}
```

字段要求：

| 字段 | 说明 |
|---|---|
| `ok` | 是否执行成功或已发送 |
| `status` | `sent` / `done` / `rejected` / `failed` / `timeout` |
| `message` | 给 UI/日志看的短文本 |
| `data` | 工具返回数据 |
| `error_code` | 失败码，成功时为空 |

---

## 7. 本地规则和 LLM 的边界

### 7.1 永远走本地规则

```text
急停
暂停巡逻
继续巡逻
取消巡逻
开始巡逻
停止短时运动
前进/后退/左转/右转
状态查询
“你能做什么”
“怎么用”
```

### 7.2 可以走 LLM

```text
解释机器人功能
解释电力巡检流程
解释检测项含义
解释异常检测结果
根据检测结果生成报告草稿
根据当前状态给下一步建议
```

### 7.3 LLM 输出限制

即使是复杂问答，LLM 也只能输出：

```text
final_answer
status_reply
tool_call 结构化建议
need_confirm
reject
```

代码不能执行未通过 schema 和 policy 的工具。

---

## 8. Safety Governor 调试

建议文件：

```text
src/ylhb_llm/ylhb_llm/agent_policy.py
```

### 8.1 必须允许

```text
emergency_stop 永远允许
```

### 8.2 条件允许

```text
start_patrol_mode：允许发送，但如果状态未知，要提示 warning
pause_patrol：巡逻 running/returning_home/waiting_loop 时允许
resume_patrol：巡逻 paused 时允许
cancel_patrol：巡逻 active/paused/returning_home 时允许
send_text_motion：只允许 前进/后退/左转/右转/停止
```

### 8.3 必须拒绝

```text
未知工具
直接 cmd_vel
修改地图/路线
删除文件
跳过急停
关闭安全检查
超过白名单参数范围
```

---

## 9. 推荐 ROS 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/inspection_ai/agent_request` | `std_msgs/String` JSON | 输入 | UI/语音给 Agent 的请求 |
| `/inspection_ai/agent_status` | `std_msgs/String` JSON | 输出 | Agent 当前状态 |
| `/inspection_ai/agent_event` | `std_msgs/String` JSON | 输出 | 决策、工具调用、失败事件 |
| `/inspection_ai/system_command` | `std_msgs/String` JSON | 输出 | 给 supervisor 的命令 |
| `/inspection_ai/say_text` | `ylhb_interfaces/SayText` | 输出 | 给 TTS 的播报 |
| `/inspection_ai/voice_session_status` | `std_msgs/String` JSON | 输入 | 语音会话状态 |
| `/patrol/status` | `std_msgs/String` JSON | 输入 | 巡逻状态 |
| `/patrol/event` | `std_msgs/String` JSON | 输入 | 巡逻事件 |

---

## 10. 调试命令

### 10.1 查看 Agent 状态

```bash
ros2 topic echo /inspection_ai/agent_status
```

### 10.2 查看 Agent 事件

```bash
ros2 topic echo /inspection_ai/agent_event
```

### 10.3 手动发请求：状态查询

```bash
ros2 topic pub --once /inspection_ai/agent_request std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"text\":\"现在状态怎么样\",\"timestamp\":0}'}"
```

### 10.4 手动发请求：开始巡逻

```bash
ros2 topic pub --once /inspection_ai/agent_request std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"text\":\"开始巡逻\",\"timestamp\":0}'}"
```

### 10.5 手动发请求：急停

```bash
ros2 topic pub --once /inspection_ai/agent_request std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"text\":\"急停\",\"timestamp\":0}'}"
```

### 10.6 检查最终系统命令

```bash
ros2 topic echo /inspection_ai/system_command
```

---

## 11. 典型问题排查

### 11.1 Agent 没反应

检查：

```bash
ros2 node list | grep agent
ros2 topic info /inspection_ai/agent_request
ros2 topic echo /inspection_ai/agent_event
```

判断：

```text
没有 agent 节点：launch 未启动 inspection_agent_node
没有订阅：话题名或参数错误
有 request 无 event：JSON 解析失败或节点异常
```

### 11.2 LLM 输出不是 JSON

处理顺序：

```text
1. response_format / strict schema 优先
2. 本地 json.loads 解析
3. 解析失败，最多 retry 一次
4. 仍失败则 reject，不执行任何工具
```

不要用正则硬抠 JSON 后直接执行工具。

### 11.3 Agent 调用了错误工具

检查：

```text
voice_intents.yaml
agent_tools.yaml
agent_policy.py
agent_event 里的 intent/tool_call/tool_result
```

常见原因：

```text
短词“停止”优先级过高，吞掉“取消巡逻”
工具描述太模糊
LLM fallback 覆盖了本地规则
policy 未做白名单检查
```

### 11.4 急停没有中断播报

检查：

```text
SayText.interrupt 是否为 true
priority 是否最高
voice_output_node 是否清空队列
system_command 是否为 emergency_stop
```

### 11.5 状态查询答不出来

检查：

```bash
ros2 topic echo /inspection_ai/system_status
ros2 topic echo /patrol/status
ros2 topic echo /inspection_ai/voice_session_status
```

如果状态 topic 没数据，Agent 只能回答“当前状态暂不可用”。

---

## 12. 测试要求

建议测试文件：

```text
src/ylhb_llm/test/test_agent_schema.py
src/ylhb_llm/test/test_agent_policy.py
src/ylhb_llm/test/test_inspection_agent_node.py
src/ylhb_llm/test/test_voice_stability.py
```

只测高价值行为：

```text
AgentDecision schema 校验
未知工具拒绝
急停永远允许
停止不等于取消巡逻
开始巡逻映射 start_patrol_mode
状态查询不调用 LLM
复杂问答允许 LLM fallback
LLM 输出非法 JSON 不执行工具
ToolResult schema 固定
agent_event 可回放
```

不要锁死完整中文文案。只检查：

```text
intent
tool name
arguments
reply_key
priority
interrupt
ok/status/error_code
```

---

## 13. 推荐第一阶段验收

1. “开始巡逻”输出 `AgentDecision.response_type=tool_call`，工具为 `start_patrol_mode`。
2. “暂停巡逻”工具为 `pause_patrol`。
3. “继续巡逻”工具为 `resume_patrol`。
4. “取消巡逻”工具为 `cancel_patrol`。
5. “停止”只触发 `send_text_motion(command="停止")`。
6. “急停”触发 `emergency_stop`，`safety_level=emergency`，`interrupt=true`。
7. “现在状态怎么样”不调用 LLM，使用状态模板回答。
8. “你能解释安全帽检测吗”可以进入 LLM fallback。
9. LLM 输出非法 JSON 时不执行工具。
10. 所有工具调用都有 `/inspection_ai/agent_event`。
11. UI 能显示 Agent 状态和最近一次工具调用。
12. 不破坏现有导航、巡逻、UI 功能。

---

## 14. 第一阶段不要做

```text
不做多机器人调度
不做长期数据库记忆
不做自动改路线
不做 LLM 直接生成 Nav2 goal
不做 LLM 直接发布 cmd_vel
不做 VLA 端到端控制
不重写 TTS
不重写巡逻执行器
```

---

## 15. 推荐实现顺序

```text
1. agent_schema.py：AgentDecision / ToolResult 校验
2. agent_policy.py：安全门控
3. agent_tools.py：白名单工具封装
4. inspection_agent_node.py：接收请求、产出事件、执行工具
5. voice_command_router_node.py：把语音请求转给 agent_request
6. ui_ros_bridge.py / ui_backend.py：显示 agent_status / agent_event
7. 测试与文档
```

---

## 16. 快速回滚

如果 Agent 出问题，不影响导航和巡逻：

```bash
ros2 topic pub --once /inspection_ai/system_command std_msgs/msg/String \
"{data: '{\"schema_version\":\"1.0\",\"source\":\"debug\",\"command\":\"emergency_stop\"}'}"
```

或者直接停用 Agent：

```bash
ros2 launch ylhb_llm llm.launch.py enable_inspection_agent:=false
```

保留原 UI / supervisor / patrol_executor 链路继续运行。
