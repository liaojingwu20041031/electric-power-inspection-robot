# DG-202611 双板现场只读核验（2026-09-13）

本记录只包含只读检查和已形成的采集证据。未修改机器人基础工作区、机器人备份、RDK基础备份或设备配置，未启动Nav2、底盘bringup或运动指令。

## 版本与边界

- 本地开发分支：`codex/dg-robot-integration`；当前代码 HEAD：`0a5ad6b`；阶段性证据基线提交：`4f3ffb4`。本审计记录对应阶段性提交时的现场快照。
- 最终发布包、补丁和源码SHA-256清单按最终本地提交的短 SHA 绑定；`4f3ffb4`仅作为已归档的阶段性证据提交。
- 本次核验区分三层事实：RDK目标架构编译、两板DDS基础发现、RDK本地真实采集/业务话题发布。编译通过和基础DDS通过均不能替代NVIDIA业务接收或机器人实机闭环。

## NVIDIA机器人

- 机器人网络和SSH正常，当前无线地址为`192.168.43.12`。
- 本次审计时的部署软链指向`/home/nvidia/dg202611_ws_c75d48e`，部署记录中的提交为`c75d48e`；这是现场历史快照，不是当前 HEAD`0a5ad6b`。本地集成分支尚未部署，基础工作区未被本次本地开发修改。
- `dg202611-models.service`和`ylhb-mobile-bridge.service`处于运行状态并已设为开机启动；三模型当前保持观察模式，Nav2、速度仲裁器和UWB实时回放关闭，`recovery_motion_armed=false`。
- `/scan`、`/odom`、`/imu/data`当时没有发布者，ZLAC、RPLidar、IMU和EKF底层进程没有启动；`/cmd_vel`存在UI/命令节点发布端，但没有底盘订阅端。本次没有发送运动指令。
- 三模型状态为`STANDBY/INITIALIZING`，原因是巡检任务未激活且底层传感器输入不存在。
- Mobile Bridge本地监听正常；云平台心跳和地图上传出现`URLError/HTTP 503`，属于外部云平台不可用，不代表本地桥或SSH离线。
- 与RDK的`/dg/link_test`双向收发已成功；事件业务话题跨板接收也已通过，详见下方证据。`/dg/auxiliary/transport_quality`仍待新版传输监视器部署。

## RDK X5

- RDK网络和SSH正常，当前无线地址为`192.168.43.13`，当前使用`root`账号。
- ROS 2 Humble、TROS和官方APX003CC样例存在；厂商样例实时识别已看到EVS约208.5–208.75 FPS、APS约29.79–29.82 FPS，`VC0/VC1`均ready。
- RDK目标架构独立验证目录中的`dg_hvs_bridge_node`已使用官方HBN后端完成编译和链接：`/root/dg202611_hvs_verify_20260913_1228/build2/dg_hvs_bridge_node`，SHA-256为`2c69c888021834ef4ffd79900216fa9a3c81115f714f30a91bcfe8aa9736d93b`。这只证明构建成功。
- RDK真实事件相机采集证据位于`/root/dg202611_hvs_live_20260913/20260913_130147_apx003cc`，包含约2,033,009行`events.csv`、约511MB`events.raw8`、约1.15GB`events.evt2`、218张APS图和2张融合图；原始文件SHA-256已写入`source_sha256.txt`。
- 当前独立桥接实例运行于验证目录，已在RDK本地发布：`/dg/event_camera/features`、`/dg/event_camera/feature_quality`、`/dg/event_camera/quality`、`/dg/event_camera/camera_info`、`/dg/event_camera/events`、`/dg/event_camera/motion_mask`、`/dg/event_camera/aps/image_raw`、`/dg/event_camera/fused/image`。运行时显式设置`LD_LIBRARY_PATH="$HV_TOOLKIT_ROOT/lib/x5:/opt/tros/humble/lib:${LD_LIBRARY_PATH:-}"`后，桥接正常工作。
- NVIDIA于2026-09-13T05:40:35Z发现全部事件业务话题，并收到`/dg/event_camera/quality=GOOD`、`/dg/event_camera/feature_quality=COMPARED`，features约9.2–9.9 Hz。证据为`evidence/cross_board/20260913/nvidia_receive.txt`，SHA-256为`f830d919bc9f5208e2672a50fa1008a0af8e96c62a948e43ff107929fe68e408`；当前可记录`CROSS_BOARD_RECEIVE_PASS`。
- `/dg/auxiliary/transport_quality`新版监视器尚未部署，因此只记录业务接收通过，不填写该话题的新鲜度或延迟结论。此前现场记录曾发现RDK系统时间回到2000年；在最新`date -Is`和校时结果确认前，不填写跨板真实延迟。

## 当前结论

代码侧三模型、UWB回放和辅助传感器观察接口已经具备；RDK HBN目标架构已编译，RDK真实相机采集、本地话题发布和NVIDIA跨板业务接收均已形成证据，两板基础DDS已通过。尚未完成的是新版`transport_quality`监视器部署、UWB回放进入MODEL-1的状态证据、毫米波真实数据、三模型真实ROS 2闭环，以及用户现场静止和低速运动验收。

下一次现场顺序固定为：

1. 核实两板系统时间并记录`date -Is`；
2. 在RDK启动桥前设置`HV_TOOLKIT_ROOT`、`AMENT_PREFIX_PATH`和`LD_LIBRARY_PATH`，然后在NVIDIA先启动业务话题接收端，再启动RDK HBN桥；
3. 已验证`features + feature_quality + quality + camera_info`跨板接收；待新版监视器部署后再补`assist_quality`和`transport_quality`的完整审计；
4. 保持观察模式，完成UWB和毫米波回放验收；
5. 如需上板，先部署按最终本地提交生成的新包并复核版本；
6. 底盘电源开启后由现场人员检查真实`/scan`、`/odom`、`/imu/data`、TF和底盘订阅者，再执行低速短距离动作。
