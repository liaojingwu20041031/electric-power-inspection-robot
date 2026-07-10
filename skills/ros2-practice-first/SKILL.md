---
name: ros2-practice-first
description: Debugs and fixes ROS2 robot runtime problems with logs, dependency order, and real-device verification. Use for Nav2, AMCL, TF, lifecycle, launch sequencing, patrol startup, sensors, actuators, CAN, serial, cameras, SLAM, and Jetson deployment. Prevents mock-heavy or test-count-driven overengineering.
---

# ROS2 Practice First

本 Skill 用于 ROS2 机器人真实运行问题。核心原则：**自动化测试只能保护确定性逻辑，不能证明机器人在真实环境中可用。日志、ROS 图、TF、时序和实机结果才是运行问题的最终证据。**

## 强制规则

1. 先分析真实日志、提交差异和 ROS 依赖关系，再修改代码。
2. 优先修复真实故障链，不为了让测试通过而改变业务逻辑。
3. 默认不新增测试文件，不追求覆盖率和测试数量。
4. 除非用户明确要求，否则不运行全仓库 `colcon test`、全量 pytest 或重复测试。
5. Mock、Fake 和静态字符串断言不能作为 ROS2 运行功能完成的证明。
6. Nav2、AMCL、TF、lifecycle、传感器、电机、CAN、串口、视觉、SLAM 和 Jetson 性能问题，必须给出实机、仿真或 rosbag 验收步骤。
7. 修改应尽量小：只改造成真实故障的代码，不顺手重构无关模块，不扩展抽象层，不批量整理测试。
8. 禁止在一次故障修复中同时新增大量防御逻辑、状态字段、测试矩阵或诊断框架。

## 什么时候可以写自动化测试

只有同时满足以下条件时，才考虑在**已有测试文件**中增加最多 1 个回归用例：

- 故障已经由真实日志或实机复现确认；
- 根因是纯函数、数据格式、状态转换、方向映射或安全边界；
- 测试不依赖模拟 ROS 时序来证明真实节点已经工作；
- 用例能够阻止同一个确定性回归，而不是证明一套假想流程正确。

以下情况默认 **0 个新测试**：

- launch 启动顺序、ROS discovery、lifecycle 服务时序；
- AMCL 初始定位、`map -> odom`、Nav2 是否真正 active；
- 雷达、IMU、相机、电机、CAN、串口和 USB 设备；
- SLAM/定位质量、规划效果、角落卡死、避障距离；
- UI 文案、布局、日志文本和普通参数调整；
- 已有日志已经能够直接证明的运行问题。

## 调试流程

### 1. 建立真实故障链

从日志中区分：

- 哪些节点成功启动；
- 第一个真正失败的条件；
- 上游条件由谁产生；
- 是否存在循环依赖、错误启动顺序或探测假阴性；
- 失败是进程退出、ROS 接口缺失、TF 缺失、数据不新鲜，还是仅诊断探测失败。

不要把普通 WARN 当根因。例如无运动命令时的 `/cmd_vel timeout` 通常只是安全置零。

### 2. 对比最近提交

只检查与故障链相关的改动，优先寻找：

- 新增硬门控；
- 等待顺序变化；
- 生产者尚未启动却先等待其输出；
- 将诊断信号误当作运行必要条件；
- 单线程 executor 内同步等待异步 service；
- 超时过短或生命周期节点名称错误。

### 3. 做最小修复

优先：

- 调整启动顺序；
- 删除错误硬门控；
- 将不可靠探测降级为诊断警告；
- 使用真实功能信号作为门控，例如新鲜 `/amcl_pose`、有效 TF、Action server 可用；
- 保留急停、速度限制、方向映射和安全区域等真实安全边界。

### 4. 最低成本静态检查

默认只做与改动直接相关的最低成本检查，例如：

- 查看 diff；
- Python 语法检查；
- 只构建受影响包且可使用 `-DBUILD_TESTING=OFF`。

除非用户要求，不新增测试，不运行全量测试。

### 5. 必须提供实机验收

每次 ROS2 运行修复都必须提供：

- Jetson 上的构建命令；
- 启动命令；
- 用于确认关键 topic、TF、service、action 的命令；
- 预期日志顺序；
- 明确的通过/失败标准；
- 若失败，下一步应收集的最小日志。

## 巡逻启动顺序基线

巡逻启动应遵循生产者先于消费者：

1. 启动底盘、雷达、IMU，确认新鲜 `/odom`、`/scan` 和基础 TF；
2. 启动 Navigation，确认 `/map` 和 `/initialpose` 订阅者存在；
3. 启动 patrol executor；
4. patrol executor 从正式路线文件发布 `/initialpose`；
5. 等待初始位姿事件和新的 `/amcl_pose`；
6. 等待 `map -> odom`；
7. 等待 Nav2、keepout 和 `navigate_to_pose` 可用；
8. 最后发送巡逻 `start` 命令。

禁止在 patrol executor 启动之前，把依赖初始位姿或 `map -> odom` 的条件设为硬门控。lifecycle 查询出现假阴性时，应以 `/amcl_pose`、TF 和 Action server 等真实功能信号为准。

## Codex 输出要求

执行修改时，最终只报告：

- 根因；
- 修改的文件和关键改动；
- 明确声明没有新增哪些多余测试、没有运行哪些全量测试；
- 实机验证命令与通过标准。

不得用“所有 Mock 测试通过”宣称 ROS2 实机问题已经解决。