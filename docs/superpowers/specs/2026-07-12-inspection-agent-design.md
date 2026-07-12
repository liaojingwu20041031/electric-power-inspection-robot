# 巡检任务 Agent 设计

## 目标

在不触碰导航、地图或既有路线控制面的前提下，提供一个能以真实 ROS 反馈闭环的任务级 Agent。Agent 只能通过 Supervisor、PatrolExecutor 和 BaseSkill 发起受控任务，不能发布 `/cmd_vel`、Nav2 goal 或修改路线资产。

## 架构

```text
UI / voice -> agent_request -> queue worker -> runtime -> policy -> operation manager
                                                   |             |
                                                   |             +-> Supervisor / Patrol / BaseSkill
                                                   v
                                          status aggregator <- ROS status/event topics
```

`InspectionAgentRuntime` 只负责 OpenAI-compatible tool-call 对话循环和会话历史；`InspectionAgentNode` 保存运行状态、审批和队列；`AgentOperationManager` 保存每一个命令的关联 ID、状态和超时；`RobotStatusAggregator` 只聚合只读观测。动作工具先返回已接受/运行的 operation，长巡逻通过查询工具继续观察，绝不把发布成功描述为完成。

## 协议与状态

- 保留服务端 assistant message 原样，包括 `tool_calls`；每条 tool message 以原始 `tool_call_id` 关联。
- 操作状态严格为 `created`、`sent`、`accepted`、`running`、`succeeded`、`failed`、`canceled`、`timeout`。
- 命令附加 `request_id`、`run_id`、`operation_id`、`tool_call_id`；旧节点忽略未知字段。PatrolExecutor 和 BaseSkill 的反馈回显已知关联字段。
- Agent run 状态严格为 `idle`、`thinking`、`waiting_confirmation`、`executing_tool`、`waiting_feedback`、`completed`、`failed`、`canceled`。
- 审批 topic 使用 JSON `std_msgs/String`，审批回复必须同时匹配 `run_id` 和 `tool_call_id`，30 秒未回复即拒绝。

## 安全边界

- `emergency_stop` 和 `stop_motion` 不经 LLM 等待；急停可抢占当前 run。
- 只读工具无需确认；相对运动与指定点导航首次执行需确认；`start_route` 始终需确认。
- 工具 Schema 禁止额外参数；重复的副作用调用（名称加规范化参数）连续第 3 次被拒绝。
- 每轮最多一个副作用工具，`parallel_tool_calls=false`；最多 8 步、4 次副作用工具。
- 状态过期必须显式标注 `stale`/`fresh=false`，不能称为当前状态。

## 路线与检测

路线目录通过 PatrolExecutor 已使用的 `resolve_route_file_path(..., auto)` 解析；重载后替换 Agent 的目录快照。默认冻结路线不修改。当前默认路线没有 `inspection_items`，且现有 YOLO 仅发布通用检测 JSON、未配置类别名称；因此 dispatcher 只会对明确配置且有处理器映射的项目返回检测结论，其余返回 `unsupported`，不编造发现。

检查点调度以 ROS 2 Action 传递 route/target/items、反馈、取消和结构化结果。PatrolExecutor 在 `target_reached` 后等待 dispatcher 的终态再继续；Agent 只调度 Action/查询结果，不包含视觉模型逻辑。

## 验证与部署

仅添加最小离线协议测试：原始 tool call 保存、正确 ID 写回、失败后重规划、重复副作用调用阻断、审批暂停、急停抢占、operation 超时和多线程状态更新。不会自动启动实车、巡逻、Nav2 或相对运动。实机阶段由现场人员按只读、巡逻、小范围运动、复合任务、故障门控顺序验收。
