# 项目答辩系统架构图

本目录基于 GitHub 仓库 `liaojingwu20041031/electric-power-inspection-robot` 最新 `main` 分支的真实代码、Launch、参数、协议和运行文档生成。图中没有把 README 中的旧图直接扩写，也没有把仓库外的 Spring、Robot Bridge、Web/小程序或检查点检测服务伪装成本仓库内部实现。

## 分析基线

```text
分支：main
工作区：clean
HEAD：c16622e feat(maps): upload default maps for review
HEAD 与 origin/main：c16622e5739c08ee7011ea4a6a78ccc9f3373e94（2026-07-19 刷新远端后相同）
```

重点分析范围：

- `README.md`、`src/电力巡检机器人使用与调试手册.md`；
- `src/ylhb_base/launch/*.launch.py`、`src/ylhb_llm/launch/llm.launch.py`、`src/ylhb_mobile_bridge/launch/*.launch.py`、`src/ylhb_perception/launch/perception.launch.py`、`src/ylhb_3d_mapping/launch/*.launch.py`；
- `src/ylhb_llm/config/llm.yaml`、`src/ylhb_mobile_bridge/config/mobile_bridge.yaml`、底盘、EKF、SLAM、感知与三维配置；
- `docs/protocol/robot-platform-v1.md`；
- Agent Runtime、AgentSpec、Schema、Policy、ToolPack、OperationManager、System Supervisor、Patrol Executor、Mobile Bridge、MapManager、MapUploadWorker、DeploymentStore、PlatformCloudClient 相关源码。

## 状态标记口径

| 标记 | 含义 |
|---|---|
| 已实现 | 当前仓库存在可执行节点、源码或明确的本地运行链路 |
| 可选启用 | 当前仓库已实现，但 Launch 默认关闭、需要额外硬件/模型/凭据或与其他模式互斥 |
| 外部系统 / 协议已定义 | 当前仓库提供协议或客户端，但服务端源码不在本仓库 |
| 后续规划 | README、手册或协议明确说明尚未完成，不能作为已交付功能陈述 |

关键边界：RTK 当前只发布 `/gps/*`，不参与 AMCL、Nav2 或 `map -> odom`；ZED 三维输出不作为 Nav2 二维地图；Agent 不直接调用 `/cmd_vel`、原始 Nav2 goal、删图或改路线；Spring、Robot Bridge、Web 路线编辑和正式检查点检测服务不属于本仓库内部节点。

## 答辩页面建议与讲解词

### 第 3 页：系统总架构

使用：[01-答辩系统总架构图.svg](./01-答辩系统总架构图.svg)，源文件为 [01-答辩系统总架构图.mmd](./01-答辩系统总架构图.mmd)。

30～60 秒讲解词：

> 系统采用 Jetson 上的 ROS 2 五层架构：最上层接入本体 UI、手机和语音；Agent 层负责意图理解，但所有动作都必须经过 Schema、Policy 和 ToolPack；Supervisor 负责启动顺序、模式管理与软件急停；巡逻执行器通过 Nav2 Action 完成路线闭环；底层由 EKF、SLAM/AMCL、感知和硬件驱动提供真实反馈。图中 A 是语音 Agent 闭环，B 是自主巡逻闭环，C 是地图上传、平台审核、路线修订和 Deployment 下发闭环。仓库外的平台模块均明确标为外部系统。

### 第 4 页：ROS 2 节点通信拓扑

使用：[02-ROS2节点通信拓扑.svg](./02-ROS2节点通信拓扑.svg)，源文件为 [02-ROS2节点通信拓扑.mmd](./02-ROS2节点通信拓扑.mmd)。

30～60 秒讲解词：

> 这张图强调真实通信类型。蓝线是 Topic，紫线是 Service，绿色粗线是 Nav2 Action，橙色虚线是 TF，青线是 HTTP、WebSocket 和 HTTPS。巡逻执行器只通过 `NavigateToPose` 调用 Nav2；`/cmd_vel` 最终进入底盘控制器；底盘 `/odom` 与 IMU 进入 EKF；SLAM 或 AMCL 提供 `map -> odom`，EKF 提供 `odom -> base_footprint`，URDF 再连接 `base_link` 和各传感器坐标系。文件和 SQLite 没有被误画成 ROS 接口。

### 第 5 页：AI Agent 任务闭环

使用：[03-AI-Agent任务闭环时序.svg](./03-AI-Agent任务闭环时序.svg)，源文件为 [03-AI-Agent任务闭环时序.mmd](./03-AI-Agent任务闭环时序.mmd)。

30～60 秒讲解词：

> 语音先在本地做唤醒和 VAD，再把命令音频送到 DashScope ASR。Planner 只能选择 AgentSpec 暴露的高层工具，返回后还要经过聊天结构校验和能力 Schema、Policy 双层校验。副作用工具发送后不会立即宣称成功，而是进入 `waiting_feedback`，等待 Supervisor、Patrol Executor、Nav2 和底盘的真实 ROS 反馈。成功、失败、取消和超时都有独立收尾；超时会触发取消或停止。急停还保留 UI 到 Supervisor 的本地快速通道。

### 第 6 页：地图与路线平台闭环

使用：[04-地图路线平台闭环.svg](./04-地图路线平台闭环.svg)，源文件为 [04-地图路线平台闭环.mmd](./04-地图路线平台闭环.mmd)。

30～60 秒讲解词：

> 手机确认默认地图后，MapManager 先归档旧 `my_map` 和旧路线，再让新的 `my_map.yaml/pgm` 在本地生效。随后才创建 SQLite 上传任务和不可变快照，由 MapUploadWorker 通过 HTTPS 上传。上传成功只表示平台产生 `PENDING_REVIEW` 地图资产，不等于审核通过。平台审核后才能编辑并发布 Route Revision，再形成 Deployment。Jetson 主动 heartbeat 拉取部署，逐项校验路线和地图哈希，通过 staging 与 `os.replace` 原子安装，最后由 Patrol Executor 执行。图中特别把五个状态拆开，避免答辩时混淆。

### 第 7 页：通信接口矩阵

使用：[05-通信接口矩阵.md](./05-通信接口矩阵.md)。答辩时建议只展示每组 1～2 条核心接口，其余作为备查页或答辩问答证据。

讲解词：

> 通信矩阵把每条关键链路落实到发送方、接收方、类型、触发条件、安全约束和源码位置。它能回答“这个 Topic 谁发、谁收”“为什么不是 Service”“云平台如何保证安全”“CAN、USB、串口和 ALSA 分别在哪里使用”等追问。

## SVG 生成

Mermaid 源文件使用 UTF-8，推荐在仓库根目录执行：

```bash
npx -y @mermaid-js/mermaid-cli -i docs/architecture/01-答辩系统总架构图.mmd -o docs/architecture/01-答辩系统总架构图.svg -w 1920 -H 1080 -b transparent
npx -y @mermaid-js/mermaid-cli -i docs/architecture/02-ROS2节点通信拓扑.mmd -o docs/architecture/02-ROS2节点通信拓扑.svg -w 1920 -H 1080 -b transparent
npx -y @mermaid-js/mermaid-cli -i docs/architecture/03-AI-Agent任务闭环时序.mmd -o docs/architecture/03-AI-Agent任务闭环时序.svg -w 1920 -H 1080 -b transparent
npx -y @mermaid-js/mermaid-cli -i docs/architecture/04-地图路线平台闭环.mmd -o docs/architecture/04-地图路线平台闭环.svg -w 1920 -H 1080 -b transparent
```

若目标机器没有 `mmdc`，可以使用上述 `npx` 命令临时运行 Mermaid CLI；若网络也不可用，保留 `.mmd`，不要创建伪造 SVG。

## 证据定位

- 节点与默认开关：`src/ylhb_base/launch/`、`src/ylhb_llm/launch/llm.launch.py`、`src/ylhb_mobile_bridge/launch/`、`src/ylhb_perception/launch/perception.launch.py`、`src/ylhb_3d_mapping/launch/zed_spatial_mapping.launch.py`。
- Agent 约束：`src/ylhb_llm/config/robot_capabilities.yaml`、`agent_schema.py`、`agent_policy.py`、`agent_tools.py`、`inspection_agent_runtime.py`、`agent_operation_manager.py`。
- 巡逻闭环：`system_supervisor_node.py`、`patrol_executor_node.py`、`navigation*.launch.py`、Nav2 参数与行为树。
- 地图上传与部署：`map_manager.py`、`map_upload.py`、`platform_store.py`、`platform_cloud_client.py`、`platform_api.py`、`docs/protocol/robot-platform-v1.md`。
- TF：`src/ylhb_base/config/ekf.yaml`、`slam_toolbox_params.yaml`、`urdf/ylhb.urdf.xacro`。

