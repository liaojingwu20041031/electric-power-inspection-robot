---
name: good-testing
description: Enforces lean, risk-based testing for this ROS2 student robot project. Use whenever planning, adding, reviewing, deleting, or running tests, and when changing robot control, Nav2/SLAM, patrol, mapping, UI, voice, scripts, schemas, or launch configuration.
---

# Lean ROS2 Testing

本项目的目标是尽快把电力巡检机器人在真实设备上跑通，而不是追求测试数量、覆盖率或企业级测试体系。

## 第一原则

默认不新增测试。只有测试能够低成本阻止真实事故或高概率回归时，才允许增加。

在写测试前，必须先回答：

1. 它具体阻止哪一种真实故障？
2. 这个故障能否在无硬件、无网络、无模型的环境中稳定复现？
3. 现有测试是否已经覆盖同一行为？
4. 自动化测试是否比一次实机、仿真或 rosbag 验收更可信、更便宜？
5. 能否只运行一个测试文件或一个测试用例完成验证？

任一问题没有明确答案，就不要新增自动化测试。

## 测试预算

- 文档、注释、格式、普通日志、UI 文案或布局调整：新增测试数为 0。
- Nav2、SLAM、感知阈值和等待时间等普通调参：新增测试数为 0，只做配置解析和实机验收。
- 修复纯函数、解析器、状态转换或协议字段：优先修改已有测试文件，通常只增加 1～3 个用例。
- 新增独立且确定性的核心模块：最多新增 1 个测试文件。
- 安全关键功能可突破上述限制，但必须说明保护的事故场景。
- 默认不运行全仓库测试。只有跨包重构、阶段验收或发布前才运行全量测试。

## 自动化测试应该保护什么

优先保留：

- 急停、控制锁、危险命令拦截。
- `cmd_vel` 到左右轮的方向、速度映射和限幅。
- 机器人尺寸、footprint、地图 unknown/occupied/free 语义。
- 巡逻启动、暂停、继续、取消和终态恢复的状态机。
- 路线、地图、移动端和 UI 使用的稳定 schema/API 契约。
- 纯解析器、坐标转换、地图文件读写和确定性几何算法。

## 必须依赖实机、仿真或 rosbag 的问题

不要用大量 mock 假装下列问题已经被测试：

- 电机正反转、轮胎打滑、底盘惯性和实际制动距离。
- CAN、串口、USB 掉线和设备权限。
- 雷达、IMU、相机噪声以及 TF 时间同步。
- Nav2 转弯、避障、恢复行为和局部规划器动态表现。
- SLAM 地图质量、定位漂移和真实场地重定位。
- 视觉模型精度、漏检、误检和 Jetson 推理性能。
- UI 操作到机器人真实动作的端到端时延。

这些场景应生成简短的实机验收清单，而不是复杂 fake/mock 测试。

## 禁止事项

- 不为提高测试数量、覆盖率或“看起来专业”而写测试。
- 不重复验证已有测试已经保护的同一契约。
- 不测试私有实现细节、完整中文文案、控件顺序、固定高度和普通样式。
- 不锁死普通调参的小数值；只有安全上限、地图格式或协议值才能精确断言。
- 不为 fake/mock 建复杂框架；fake 超过约 30 行时，优先改为实机/集成验收。
- 不测试第三方 ROS 驱动或 vendor 目录内部实现。
- 不为了让测试容易写而无必要地重构运行时代码。
- 不在每次修改后执行 `colcon test` 全仓库扫描。

## 本仓库现有测试的处理原则

- `ylhb_base`：保留底盘方向映射、NMEA 解析、机器人几何、地图语义和关键启动配置。
- `ylhb_llm`：Agent 新功能优先补到已有 `agent_tools`、`agent_policy`、`agent_schema`、`inspection_agent_runtime` 或 `inspection_agent_node`，不要再为每个小类创建新测试文件。
- QML 静态测试只保留关键 backend 绑定、危险旧命令不存在和控制命令路由；删除或放松中文文案、帮助文本、控件结构和视觉细节断言。
- 启动脚本保留一个核心入口烟雾测试即可，不为帮助信息的每一行建立契约。
- 硬件相关脚本默认标记为人工验收，不进入日常 pytest。

## 开发时的验证层级

### 1. 快速开发

只构建受影响包，并关闭 CMake 测试：

```bash
colcon build --packages-select <package> --cmake-args -DBUILD_TESTING=OFF
```

Python 修改只运行对应文件或用例：

```bash
python3 -m pytest -q path/to/test_file.py
python3 -m pytest -q path/to/test_file.py::test_name
```

### 2. 功能完成

运行受影响包的测试，不扩散到无关包：

```bash
colcon test --packages-select <package>
colcon test-result --verbose
```

### 3. 阶段验收

仅在跨包重构、里程碑或交付前运行全量测试，并执行真实机器人验收清单。

## Agent 输出要求

每次涉及测试时，先给出一段简短决策：

- 风险级别：安全关键 / 核心契约 / 普通逻辑 / UI与调参 / 硬件动态。
- 测试决策：不新增、修改已有、或新增一个测试文件。
- 最小验证命令。
- 仍需进行的实机、仿真或 rosbag 验收。

如果没有新增测试，直接说明“现有测试或实机验收已经足够”，不要为了形式补测试。
