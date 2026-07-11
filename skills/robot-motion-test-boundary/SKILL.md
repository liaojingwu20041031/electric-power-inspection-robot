---
name: robot-motion-test-boundary
description: Use when handling real YLHB robot hardware, patrol routes, Nav2 goals, teleoperation, or runtime testing that could cause physical robot movement.
---

# 实机运动测试边界

实地路线运行和任何实际机器人运动由用户在现场执行。Agent 可以做静态验证和不改变机器人状态的在线诊断，但不得把“测试”当作启动或驱动车辆的授权。

## 风险划分

| 类别 | 执行者 | 例子 |
| --- | --- | --- |
| 高风险实机运动 | 用户现场执行 | `inspection`/实机导航启动、巡逻开始或恢复、Nav2 action goal、遥控、发布 `/cmd_vel`、会使底盘或导航进入可执行状态的 service/parameter/lifecycle 调用 |
| 中低风险在线诊断 | Agent 可执行 | `ros2 node/topic list`、`topic info/echo/hz`、`param get`、`lifecycle get`、TF 与 costmap 观察、日志采集 |
| 诊断 topic 通信 | Agent 可执行，但须先确认 | 仅向用户明确指定、且不会被底盘、导航、lifecycle、AMCL 或安全控制消费的诊断 topic 做一次性通信检查 |

不确定命令是否会改变实机状态时，按高风险处理，不执行。

## 工作流程

1. 先区分请求是静态/只读诊断还是会造成真实运动。
2. 对高风险操作：提供启动命令、观察项、通过标准和需要回传的日志；明确由用户执行并等待结果。不得代替用户启动 `inspection`、实机 bringup/navigation 或发送运动指令。
3. 对中低风险操作：只运行必要的只读命令；发布诊断消息前确认 topic 没有执行器副作用。
4. 报告时分别标注“静态验证”“在线只读观察”“用户实机验证”。不得用前两者宣称路线或避障已通过。

## 实机回传

用户完成路线测试后，优先收集：机器人 pose、局部 footprint、local/global costmap、`/plan`、`/local_plan`、`/cmd_vel`，以及 controller_server 的错误日志，尤其是 `No valid trajectories` 与 `Trajectory Hits Obstacle`。
