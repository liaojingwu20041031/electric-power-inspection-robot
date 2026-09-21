# DG-202611 最终证据索引

**更新时间**：2026-09-13<br>
**规则**：`READY`表示已有文件或已完成静态验证；`PENDING`表示必须在后续设备或ROS 2环境中补齐；`NOT_CLAIMED`表示不能作为正式指标。

| 编号 | 证据 | 来源 | 状态 | 用途 |
|---|---|---|---|---|
| E01 | 三模型核心测试 | `src/ylhb_base/test/` | READY | 保护确定性逻辑，不代表实机 |
| E02 | UWB真实原始日志和CSV | `src/ylhb_base/data/uwb/20260905/`、`evidence/uwb/20260905/` | READY | 回放、质量门控、3物理源/6逻辑接口 |
| E03 | RDK事件相机真实采集 | `/root/dg202611_hvs_live_20260913/20260913_130147_apx003cc`及`source_sha256.txt`；EVS/APS厂商样例输出 | READY（RDK本地采集） | 已形成真实EVS、APS、RAW8、EVT2、CSV和APS帧证据；跨板消费由E05单独记录 |
| E04 | RDK↔NVIDIA DDS基础发现 | 两侧`/dg/link_test`双向收发 | READY（基础发现） | 基础跨板连通；事件业务话题接收由E05单独记录 |
| E05 | 事件特征和质量到达Jetson | `evidence/cross_board/20260913/nvidia_receive.txt`（SHA-256=`f830d919bc9f5208e2672a50fa1008a0af8e96c62a948e43ff107929fe68e408`）；`/dg/event_camera/features`、`/dg/event_camera/feature_quality`、`/dg/event_camera/quality`、`/dg/event_camera/assist_quality`、`/dg/event_camera/camera_info` | READY（跨板业务接收） | 2026-09-13T05:40:35Z，quality=`GOOD`、feature_quality=`COMPARED`，features约9.2–9.9 Hz；证明Jetson端业务话题可接收，MODEL-2观察调用仍待新版部署 |
| E06 | 传输时间审计 | `/dg/auxiliary/transport_quality` | READY（代码）/PENDING（新版监视器部署） | 新鲜度和延迟记录；当前跨板业务接收已通过，但新版监视器尚未部署 |
| E07 | UWB进入MODEL-1 | `/dg/uwb/quality`、`/dg/fusion/status` | PENDING | `UWB_AIDED`消费证据 |
| E08 | 毫米波厂商协议资料与真实数据 | `evidence/mmwave/20260913/`；厂商协议 PDF、上位机/驱动清单和哈希 | PARTIAL（协议资料已确认，原始实测数据待采集） | 生成目标型号解析器和 MODEL-2 观察回放 |
| E09 | MODEL-3完整状态闭环 | 真实ROS 2日志和状态时间线 | PENDING | 重定位、候选拒绝、恢复 |
| E10 | 用户现场实机运动 | 现场日志、照片、rosbag | PENDING | 低速受控验证 |
| E11 | 双板现场只读核验 | `docs/dg202611/LIVE_INTEGRATION_AUDIT_20260913.md` | READY | 部署版本、服务、输入缺口和时钟状态 |
| E12 | 最终验收命令清单 | `docs/dg202611/FINAL_ACCEPTANCE_COMMANDS_20260913.md` | READY | 现场按顺序执行静态、回放和实机验收 |
| E13 | RDK HBN目标架构编译 | RDK `dg_hvs_bridge_node`目标文件及CMake输出；目标SHA-256=`2c69c888021834ef4ffd79900216fa9a3c81115f714f30a91bcfe8aa9736d93b` | READY（编译）/PENDING（运行） | 证明官方HBN后端在RDK目标架构上可构建，不替代实时业务接收证据 |
| E14 | 最终本地发布包 | 工作区根目录中按最终本地提交短 SHA 命名的 `dg202611_ws_<commit>.tar.gz`、`dg202611_release_<commit>.patch` 及对应 SHA-256 清单 | READY（最终包按本地提交生成并单独核验） | 发布包、补丁、源码清单和提交号一致，供部署前复核 |

## 现有文件清单

- `docs/dg202611/UWB_MEASURED_RESULTS.md`
- `docs/dg202611/RDK_HVS_BRIDGE_STATUS_20260912.md`
- `docs/dg202611/AUXILIARY_TRANSPORT_ONLY.md`
- `evidence/mmwave/20260913/README.md`
- `docs/dg202611/FINAL_ACCEPTANCE_COMMANDS_20260913.md`
- `src/ylhb_base/data/uwb/20260905/README.md`
- `src/dg_hvs_bridge/README.md`

## 归档要求

每次现场测试至少保存：

```text
测试时间
设备和账号（不含密码）
Git提交号
启动参数
ROS_DOMAIN_ID / RMW_IMPLEMENTATION
话题类型、QoS、频率
TF和frame_id
rosbag或原始CSV
节点日志
文件SHA-256
现场照片或截图
```

编译通过、RDK本地真实采集、基础DDS发现和厂商协议资料核验分别只证明各自边界；未经目标型号原始帧、Jetson业务接收、三模型闭环或现场实机数据补齐的`PENDING`/`PARTIAL`项目不能改为`READY`，也不能用于方案书正式指标。
