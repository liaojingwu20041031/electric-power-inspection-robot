# DG-202611 最终融合技术报告

**版本**：2026-09-13 现场核验版<br>
**开发分支**：`codex/dg-robot-integration`  
**工作区**：`dg202611_robot_integration`<br>
**当前代码 HEAD**：`0a5ad6b`（本地开发分支，含后续 HVS 显示入口）
**历史证据基线**：`4f3ffb4`（毫米波资料核验、跨板证据和验收文档归档时的阶段性提交）

## 1. 目标与边界

本阶段把三个模型、已采集的UWB数据、RDK X5事件相机接口和毫米波规范化接口整理成可验证的ROS 2侧链。学长基础系统 `/home/nvidia/ros2_DL`、机器人备份和RDK备份均不在本阶段修改范围内。

辅助传感器默认采用观察门控模式：先证明数据到达、质量判断和模型调用，再决定是否让传感器改变LiDAR结果。新节点不发布 `/cmd_vel`，不直接控制底盘。

UWB材料统一写成“3个物理基站、6路逻辑观测接口”。已有UWB数据用于回放和MODEL-1接口验证，不据此宣称机器人地图坐标的绝对定位精度。

## 2. 系统结构

```text
RDK X5事件相机 ──> HVS桥 ──> 事件质量/特征/掩码 ─┐
毫米波资料/原始采集 ──> 型号确认与协议解析 ──> 规范化CSV/回放 ─> points/quality ─┼─> MODEL-2
已有UWB记录 ───> UWB回放 ──> pose/quality ─────────┘       │
                                                           ├─> MODEL-1融合
LiDAR、IMU、轮速、GNSS、AMCL、scan-match ────────────────────┘
                                                           ↓
                                             MODEL-3重定位与恢复
```

核心话题：

- MODEL-2：`/dg/lidar/filtered_scan`、`/dg/lidar/stable_features`、`/dg/lidar/quality`；
- MODEL-1：`/dg/fusion/pose`、`/dg/fusion/odom`、`/dg/fusion/status`；
- MODEL-3：`/dg/relocalization/match_quality`、`/cmd_vel_recovery`；
- 事件相机：`/dg/event_camera/features`、`/dg/event_camera/feature_quality`、`/dg/event_camera/quality`、`/dg/event_camera/assist_quality`、`/dg/event_camera/motion_mask`、`/dg/event_camera/camera_info`；
- 毫米波：`/dg/mmwave/points`、`/dg/mmwave/quality`；
- 传输诊断：`/dg/auxiliary/transport_quality`。

## 3. 已完成的代码工作

- 三模型侧链启动入口已经具备UWB回放、事件相机观察、毫米波回放和传输监视器开关；
- RDK脚本支持通过 `DG_RDK_USER`、`DG_RDK_HOST`、`DG_RDK_WORKSPACE`、`DG_RDK_RECORD_DIR` 和 `DG_RDK_TROS_SETUP`配置，不再依赖固定登录目录；
- RDK HVS桥默认记录目录由运行用户的 `$HOME`推导，空参数时使用 `$HOME/dg202611_rdk_records`；
- 传输监视器新增消息头时间戳、ROS接收时刻、本机接收时刻、消息年龄和尽力估算延迟；
- MODEL-3新增最终运动门控：只有显式使能恢复运动，并且LiDAR、里程计和IMU数据均新鲜时，才允许发布非零恢复速度；
- 导航健康聚合不再把尚未收到首帧的LiDAR、里程计或IMU误判为健康；
- 生产UI入口固定使用受保护基础工作区，Mobile Bridge固定由systemd单实例管理；
- 事件相机内参必须同时满足“参数声明有效”和实际`CameraInfo`有效，才能离开纯诊断判断；
- 三路 EVS/APS/融合画面采集脚本现在等待并记录 rosbag 与桥接进程的真实退出码，避免异常采集被写成成功；
- 事件相机原始事件流继续在RDK本地保存，跨板优先传输特征、质量和CameraInfo；
- 毫米波资料已确认 3 Mbps、8N1 及 Demo 帧头/帧尾和点字段；由于目录中没有原始串口测量文件，实时解码仍待目标型号原始帧验证，规范化 CSV 回放接口继续保留，动态激光掩码仍关闭；
- RDK X5目标架构已使用官方HBN后端完成`dg_hvs_bridge_node`编译和链接，目标文件SHA-256为`2c69c888021834ef4ffd79900216fa9a3c81115f714f30a91bcfe8aa9736d93b`；运行时必须显式设置`LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:${LD_LIBRARY_PATH:-}"`，本次跨板证据按该条件运行。

## 4. 证据状态

已有软件协议、UWB真实采集和RDK真实事件相机采集证据。2026-09-13核验确认两板`/dg/link_test`双向DDS基础发现成功；RDK官方厂商样例已持续看到EVS与APS，独立HBN桥已在RDK本地发布事件业务话题。审计记录中的机器人部署快照仍为旧版本`c75d48e`（该快照不是当前代码 HEAD），且底层传感器和底盘没有启动，因此本阶段尚未形成机器人运动闭环。以下事项仍需在Linux ROS 2或真实设备上完成：

- NVIDIA对RDK业务事件话题的持续接收已完成：2026-09-13T05:40:35Z发现全部事件话题，收到`/dg/event_camera/quality=GOOD`、`/dg/event_camera/feature_quality=COMPARED`，features接收频率约9.2–9.9 Hz；证据为`evidence/cross_board/20260913/nvidia_receive.txt`，SHA-256为`f830d919bc9f5208e2672a50fa1008a0af8e96c62a948e43ff107929fe68e408`。
- `/dg/auxiliary/transport_quality`仍待新版传输监视器部署，当前不填写该话题的新鲜度或延迟结论；
- RDK当前配置下的30至60秒事件相机长测归档（已有一次真实采集，仍需把当前配置和验收输出绑定到证据包）；
- UWB回放进入MODEL-1后的`UWB_AIDED`状态证据；
- 毫米波厂商协议资料已从用户提供目录核验，串口 Demo 协议要点已归档；目标实物准确型号、固件和原始串口帧仍待采集，当前不能宣称实时解析器完成；
- 三模型在真实ROS 2图中的完整状态闭环；
- 用户现场执行的静止和低速运动验收；
- RDK与NVIDIA系统时间同步。此前现场核验曾发现RDK系统时间回到2000年；在最新`date -Is`确认前，不能据此计算跨板真实延迟。

## 5. 安全与声明规则

- `enable_cmd_vel_arbiter`默认关闭；
- `enable_mmwave_dynamic_mask`默认关闭；
- 事件相机内参、LiDAR外参和运动补偿未完成前保持`DIAGNOSTIC_ONLY`；
- 毫米波资料存在型号和供电差异，首次上电使用独立稳定5V电源，具体参数以实物和厂商资料为准；
- 只有日志、rosbag、CSV和现场记录支持的结果才能写入正式指标；
- 软件预检、回放结果和机器人实机结果必须在报告中分开标注。

## 6. 当前结论

代码接口和观察链已经准备好。本地开发分支当前 HEAD 为`0a5ad6b`；毫米波资料核验已完成但原始测量数据尚未出现，审计记录中的机器人部署快照为旧版本`c75d48e`，因此本地安全门控、传输审计和启动修复尚未上板。RDK目标架构已经完成HBN后端编译并形成真实相机采集证据，NVIDIA跨板业务话题接收也已通过；`/dg/auxiliary/transport_quality`仍需新版监视器部署后再验收。下一步是补UWB回放证据和毫米波真实数据回放，再由现场人员完成三模型真实ROS 2闭环及实机静态、低速验收。当前不能把事件相机或毫米波写成已经主动改变LiDAR结果，也不能把软件预检写成实机指标。
