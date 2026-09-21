# RDK X5 事件相机启动与接入

## RDK 账号与路径约定（2026-09-12）

当前地瓜派使用 `root` 账号；IP 可能随热点变化。脚本不把账号、板端工作区或记录目录写死，统一支持以下环境变量（也兼容不带 `DG_` 前缀的别名）：

```bash
export DG_RDK_USER=root
export DG_RDK_HOST=192.168.43.13
export DG_RDK_WORKSPACE=/root/dg202611_rdk
export DG_RDK_RECORD_DIR=/root/dg202611_rdk_records
export DG_RDK_TROS_SETUP=/opt/tros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

`DG_RDK_WORKSPACE`用于记录板端工作区位置，`DG_RDK_RECORD_DIR`用于保存探测、采集和校验文件；目录不存在时由采集脚本创建。密码不写入脚本、日志或证据。需要从电脑连接时使用：

```bash
ssh "${DG_RDK_USER}@${DG_RDK_HOST}"
```

## 当前状态（2026-09-13）

- RDK目标架构已经使用官方HBN后端完成`dg_hvs_bridge_node`编译和链接，目标文件为`/root/dg202611_hvs_verify_20260913_1228/build2/dg_hvs_bridge_node`，SHA-256为`2c69c888021834ef4ffd79900216fa9a3c81115f714f30a91bcfe8aa9736d93b`。
- 厂商APX003CC实时样例已经看到EVS约208.5–208.75 FPS、APS约29.79–29.82 FPS，`VC0/VC1`均ready，说明当前RDK侧真实采集链已经得到运行证据。
- 一次真实采集已保存到`/root/dg202611_hvs_live_20260913/20260913_130147_apx003cc`，包含`events.csv`、`events.raw8`、`events.evt2`、218张APS图、2张融合图和`source_sha256.txt`。这证明RDK本地真实采集和写盘正常。
- 独立验证桥已在RDK本地发布`/dg/event_camera/features`、`/dg/event_camera/feature_quality`、`/dg/event_camera/quality`、`/dg/event_camera/camera_info`等话题；运行时需显式设置`LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:${LD_LIBRARY_PATH:-}"`。
- NVIDIA于2026-09-13T05:40:35Z已发现并持续接收事件业务话题，quality=`GOOD`、feature_quality=`COMPARED`、features约9.2–9.9 Hz；证据为`evidence/cross_board/20260913/nvidia_receive.txt`，SHA-256为`f830d919bc9f5208e2672a50fa1008a0af8e96c62a948e43ff107929fe68e408`。
- 因此当前分层为：`SOFTWARE_READY`、`RDK_LIVE_CAPTURE_PASS`、`RDK_LOCAL_TOPIC_PASS`、`CROSS_BOARD_RECEIVE_PASS`；`/dg/auxiliary/transport_quality`仍待新版监视器部署。编译通过不等于跨板运行通过，但本次业务话题跨板接收已有实证。

## 历史基线结论（2026-09-11）

事件相机的软件接入路线可以继续，当前状态是：

- **DG 侧车软件已准备好**：能够校验事件字段、回放真实 CSV、提取事件特征并输出质量统计；
- **RDK X5 上有 APX003CC 用户态资源**：发现传感器 `.so`、EVS 实时采集示例以及 RAW8、EVT2、CSV 转换工具；
- **官方适配条件仍需核对**：时佩的 RDK X5 适配文档指定适配镜像；官方固件说明同时指出，RDK X5 原厂系统不包含适配的 APX003CC 驱动，必须使用适配固件；
- **真实事件流尚未取得**：当前没有 `/dev/video*`、`/dev/media*` 或 `/dev/v4l-subdev*`；
- **当前阻塞点是镜像/驱动集成和启动配置**：APX003CC overlay 当前没有进入活动设备树；镜像是否为时佩适配镜像仍需确认。
- **CAM1 映射已经只读查实**：物理 CAM1 上的模组对应软件 `cam0 / vin_vcon0 / MIPI host0 / i2c-6 / i2c@341d0000`，因此后续应选择 `dtoverlay_cam0_apx003cc`；软件 `cam1 / i2c-4 / i2c@340f0000` 不对应这次实物连接。

所以不能把“有 `.so` 和样例”写成“驱动已就绪”，也不能把现在的状态写成事件相机实机通过。映射已解决，下一步只读确认当前镜像身份并完成启动分区备份；确认后才讨论启用 `cam0` overlay 的可回滚变更。

## 已确认的板端资源

当前板端探测到 RDK X5 V1.0、RDK OS 3.4.1、Ubuntu 22.04、Linux 6.1.83，TROS Humble 环境位于：

官方适配资料给出的目标镜像是 `shimetapi_rdk_x5_3.4.1_0609.img`，并给出了对应 MD5。当前板端尚未完成该镜像文件或可靠版本标识的校验，不能据此假定已经刷入适配镜像。

```bash
/opt/tros/humble/setup.bash
```

APX003CC 传感器库：

```text
/usr/hobot/lib/sensor/libapx003cc.so
/usr/hobot/lib/sensor/libapx003cc.so.1
/usr/hobot/lib/sensor/libapx003cc.so.1.0.0
```

厂商样例根目录：

```text
/app/multimedia_samples/sample_apx003cc
```

已经确认的关键程序和源码包括：

```text
sample_evs_release/evs_mode/01_evs_live_player/evs_live_player.RAW8
sample_evs_release/evs_mode/04_raw8_to_evt2/raw8_to_evt2.cpp
sample_evs_release/evs_mode/02_evt2_to_csv/evt2_to_csv
sample_hvs/
```

系统同时带有 CAM0 和 CAM1 的 APX003CC overlay。CAM1 overlay 描述的传感器地址是 `0x3c`，使用 4 条 CSI 数据通道，但该 overlay 当前未进入活动设备树。overlay 文件和用户态库的存在只能说明资源在系统中，不能证明内核驱动和 CSI 链路已正确注册。

## 已确认的事件数据契约

厂商工具最终使用 `Metavision::EventCD`：

```text
x          像素横坐标
y          像素纵坐标
p          极性，0 或 1
t          时间戳，单位为微秒
```

已确认的 EVS 有效尺寸为 `768 × 608`。厂商 RAW8 解析代码按每个 MIPI 包 `1,048,576` 字节处理，每包包含 32 个子帧；解析后可以转换成 EVT2。EVT2 事件使用 32 位字，随后可用厂商工具导出：

```text
x,y,polarity,timestamp
```

DG 侧车的 ROS 2 输入是 `sensor_msgs/msg/PointCloud2`，字段对应为：

```text
pixel_x,pixel_y,polarity,time_offset_us
```

其中 `time_offset_us` 是相对于该 ROS 消息头时间戳的微秒偏移。CSV 转 ROS 时必须保持时间顺序，并记录原始文件 SHA-256、采集时间和转换命令。

## DG 双机接入结构

```text
APX003CC + 厂商采集程序（RDK X5）
  -> RAW8 / EVT2 / x,y,polarity,timestamp CSV
  -> DG wrapper
  -> /dg/event_camera/events
  -> event_feature_node（RDK X5）
  -> /dg/event_camera/features
  -> /dg/event_camera/feature_quality
  -> 局域网 DDS
  -> Jetson 记录、显示并提供给后续模型验证
```

事件原始流和特征提取优先留在 RDK X5，Jetson 只接收低带宽特征和质量信息。两块板联调时使用相同的 `ROS_DOMAIN_ID`。该侧链不发布 `/cmd_vel`，不接管机器人底盘。

## 第一步：只读探测

进入侧车包解压目录后运行：

```bash
bash probe_rdk_x5_event_camera.sh
# 或者显式指定输出文件：
# bash probe_rdk_x5_event_camera.sh "$DG_RDK_RECORD_DIR/evs_probe_$(date +%Y%m%d_%H%M%S).txt"
```

脚本会自动尝试加载 `/opt/tros/humble/setup.bash`，并记录：

- 视频、媒体、I2C 设备与活动设备树；
- APX003CC 传感器库和厂商 EVS 工具；
- CAM0/CAM1 overlay 的关键目标控制器、地址和 CSI 通道；
- 当前启动配置中已经启用的 overlay；
- TROS/ROS 2 环境、话题和相关进程。

该脚本不会运行 `i2cdetect`，不会加载模块，不会修改 `/boot`，不会重启，也不会发布 ROS 消息。

## 第二步：先确认镜像和登录环境

时佩资料中的示例账号可能是 `sunrise`；本次板端实际使用 `root`，SSH 目标由 `DG_RDK_USER` 与 `DG_RDK_HOST` 组成。板端当前应先记录系统版本、适配镜像标识和启动分区文件，核对是否属于时佩适配镜像。官方固件说明明确要求 RDK X5 使用适配固件后才验证 APX003CC 识别和双流采集，因此不能跳过这一核对直接改 overlay。

## 第三步：确认 CAM1 与 overlay 映射

只读证据已经完成对应：

- `i2c-6 / i2c@341d0000` 在 `0x3c` 的芯片 ID 寄存器 `0x3428` 返回 `0x08 0x08`；
- `i2c-4 / i2c@340f0000` 在同一地址读取返回 Remote I/O error；
- 基础设备树把物理连接对应到 `vin_vcon0 / bus6`，而不是 `vin_vcon2 / bus4`；
- 因此物理 CAM1 与软件 `cam0` 的命名不同，后续使用 `dtoverlay_cam0_apx003cc`。

这项映射来自板端 I2C、活动设备树和 overlay 文件的交叉核对，不是根据接口丝印猜测。

## 第四步：在确认适配条件后可回滚地启用相机

只有镜像条件和已确认的物理 CAM1 → 软件 cam0 映射都具备备份后才执行：

1. 保存当前 `/boot/config.txt`、`/boot/config/config.txt`、活动设备树和文件 SHA-256；
2. 记录 SD 卡镜像版本，并准备可从 Windows 恢复启动分区的办法；
3. 只启用确认正确的 `dtoverlay_cam0_apx003cc`；
4. 重启后检查设备节点、内核日志和传感器识别结果；
5. 若设备没有出现或启动异常，立即按备份恢复。

这一步只能在地瓜派专用任务中按已确认的命令执行，不能轮流试错 CAM0/CAM1，也不能跳过镜像核对。

## 历史桥接状态（2026-09-12）

以下内容是历史记录，提交`a39b086`早于当前证据基线`4f3ffb4`，用于保留演进轨迹。已在独立集成分支`codex/dg-robot-integration`完成`dg_hvs_bridge`的第一轮加固，本轮包含：

- EVS、APS、融合图、运动掩码、CameraInfo、质量诊断和可选 features 话题；
- APS 实际格式、尺寸、字节数、取帧超时、解码失败、APS 帧率和事件包率统计；
- NV12、Gray8、RAW8、BayerRG8、RGB888 以及明确的 unpacked RAW10 解析；
- 运行库 RPATH 加固；
- 事件相机辅助 MODEL-2 的安全观察模式，不发布 `/cmd_vel`。

当前证据边界仍是：底层厂商双流示例已能看到 EVS 与 APS；官方 Toolkit ROS 桥接此前只稳定拿到 EVS，APS ISP 路径仍报告失败。因此尚未标记 `HVS_DUAL_STREAM_PASS`，也没有把事件相机写成已经完成机器人实测。下一次板端联调先运行官方 `live_record_display`，再运行 `dg_hvs_bridge`，比较两者的 APS 计数和错误日志。

## 历史只读证据（2026-09-11）

地瓜派专用任务已保存以下只读证据：

- `rdk_x5_apx003cc_i2c_mapping_and_id_read_20260911.txt`：I2C、芯片 ID、设备树和 overlay 映射；
- `rdk_x5_apx003cc_mapping_conclusion_20260911.txt`：物理 CAM1 → 软件 cam0 的结论；
- `sidecar_v3_probe_rdk_x5_event_camera_20260911_214655.txt`：新版侧车包、TROS、用户态库和厂商工具探测结果。

这些历史证据证明了资源存在和映射关系；其‘尚未证明实时事件流’的结论已由2026-09-13的厂商实时样例和RDK真实采集证据更新，但不替代跨板业务接收验收。

## 三路画面采集命令

在已确认桥接二进制和 HBN 配置可用的 RDK 上，可以运行：

```bash
source /opt/tros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

run_three_view_capture.sh 30
```

默认保存 EVS 事件活动图 `/dg/event_camera/motion_mask`、APS 压缩图 `/dg/event_camera/aps/image_raw/compressed` 和融合压缩图 `/dg/event_camera/fused/image/compressed`。需要同时保存 EVS 原始事件和 APS/fused 文件时，使用：

```bash
DG_HVS_RECORD_RAW=true run_three_view_capture.sh 30
```

脚本只启动事件相机桥和 rosbag，不启动底盘、不启动 Nav2，也不发布 `/cmd_vel`。结束后检查 `SESSION.txt`、`bridge.log`、`rosbag.log` 和 `manifest.sha256`；质量话题应显示 `state=GOOD`、`pair_valid=true` 且 `fused_frames` 增长。
## 第五步：采集一段真实事件数据

设备正常识别且适配固件验证通过后，先在厂商示例目录编译实时采集程序。教程中的 `evs_live_player.RAW8` 是 Makefile 的目标文件，源码包不一定预先带有这个二进制：

```bash
cd /app/multimedia_samples/sample_apx003cc/sample_evs_release/evs_mode/01_evs_live_player
make
ls -lh evs_live_player.RAW8
file evs_live_player.RAW8
```

确认生成 `ELF 64-bit` 可执行文件后，再按厂商示例使用 `sudo ./evs_live_player.RAW8 -s 49`（实际索引仍以板端样例为准）采集一段短 RAW8。随后按板端现有工具链完成：

```text
真实 RAW8 -> raw8_to_evt2 -> EVT2 -> evt2_to_csv -> 真实 CSV
```

至少保存：

- 原始 RAW8、EVT2、CSV；
- 三者 SHA-256；
- 相机型号、端口、镜像版本、overlay 配置；
- 采集开始/结束时间、场景和转换命令；
- 事件数量、频率、时间戳单调性、坐标范围和极性统计。

## 第六步：接入 DG 侧车

取得真实 CSV 后可先验证回放链路：

```bash
bash run_rdk_event_csv_replay.sh /data/events.csv 50.0 false
# 运行前可通过 DG_RDK_WORKSPACE / DG_RDK_RECORD_DIR 指定板端路径
```

回放会标记为 `source_kind=MEASURED_REPLAY`。实时 wrapper 准备好后再运行：

```bash
bash run_rdk_event_feature_sidecar.sh
python3 inspect_event_topic.py --timeout-sec 10 --output-json event_topic_probe.json
```

验收时必须区分：

- `SOFTWARE_READY`：脚本、接口和回放链路可用；
- `MEASURED_REPLAY_PASS`：真实采集数据已成功回放；
- `LIVE_HARDWARE_PASS`：真实相机持续输出并经 ROS 2 实时处理。

当前状态已达到`SOFTWARE_READY`、RDK本地`LIVE_HARDWARE_PASS`和`CROSS_BOARD_RECEIVE_PASS`；真实数据回放仍可单独标记`MEASURED_REPLAY_PASS`。`/dg/auxiliary/transport_quality`仍需新版监视器部署，机器人实机消费尚未验收。

## 参考资料

- 时佩《RDK X5 载板适配》：硬件连接、CAM1 默认端口、登录方式和 EVS 示例；
- 时佩《设备固件烧录》：RDK X5 原厂系统与 APX003CC 适配固件的边界；
- 时佩 EVS 环境配置页：镜像名称、校验值和登录环境。

这三份资料只作为操作依据。任何刷写固件、修改启动配置或重启动作，都要先在地瓜派专用任务里完成当前状态备份，并确认恢复路径。
