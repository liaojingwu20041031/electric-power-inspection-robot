# DG-202611 机器人融合准备说明

这份分支用于把 MODEL-1、MODEL-2、MODEL-3 和已经采集的真实 UWB 数据接到
现有机器人系统。它是独立开发副本，不替换保存好的机器人备份。仓库提供独立的
`dg202611-models.service` 安装脚本；只有现场部署并执行安装脚本后才会加入开机启动。

## 已接好的链路

- MODEL-2：`/scan`、`/odom` -> `/dg/lidar/quality`。
- MODEL-1：里程计、GNSS、AMCL、scan-match、UWB pose/quality -> `/dg/fusion/*`。
- MODEL-3：定位健康、主动扫描、候选位姿验证、恢复状态 -> `/dg/relocalization/*`。
- UWB：保存的真实 CSV -> 六路逻辑观测 -> 三个物理源去重 -> 质量门控 ->
  `/dg/uwb/pose`、`/dg/uwb/quality` -> MODEL-1。

三个真实基站的坐标为 A2=`(0,0)`、A3=`(2.4,0)`、A4=`(1.2,1.8)`。六路逻辑接口
不会被当作六个独立物理基站。

## 安全部署步骤

在机器人独立工作区构建，不覆盖 `/home/nvidia/ros2_DL`：

```bash
cd /home/nvidia/dg202611_ws
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_DL/install/setup.bash
colcon build --symlink-install \
  --packages-select ylhb_base ylhb_mobile_bridge \
  --allow-overriding ylhb_base ylhb_mobile_bridge
source install/setup.bash
```

若要在不改 `/home/nvidia/ros2_DL` 源码的前提下预览本版本 UI，先结束旧 inspection
进程，再从独立发布目录运行：

```bash
cd /home/nvidia/dg202611_ws
export DISPLAY=:1
./scripts/run_dg_inspection.sh fullscreen:=true
```

这个入口仍从 `/home/nvidia/ros2_DL` 读取机器人地图、路线、凭据和硬件配置，只把已
构建的 DG overlay 放到 ROS 包搜索顺序末端加载。验证失败时退出它，再使用原来的
`/home/nvidia/ros2_DL/scripts/run_on_jetson.sh inspection` 即可回到原 UI。本轮不自动
替换机器人已有的 UI 开机启动项。

第一次只做旁路观察：

```bash
./scripts/run_dg_integration.sh
```

默认行为有三道隔离：不启动 Nav2、不启动 UWB 回放、不启动速度仲裁器。即使以后
显式启动仲裁器，默认输出也只是 `/dg/test_cmd_vel`，不会直接控制底盘。

## 巡检任务门控与开机运行

生产集成入口订阅 `/inspection_ai/system_status`。只有
`patrol_mode_state=running/returning_home`，且 Supervisor 报告 `bringup`、
`navigation`、`patrol_executor` 均为 `running` 并持续稳定 2 秒后，MODEL-3 与综合导航
健康才进入 `ACTIVE`。启动过程中显示 `WARMUP`；无任务、暂停、终止、组件缺失或状态
超过 3 秒未更新时回到 `STANDBY`，MODEL-3 只发布零恢复速度并清理上一次任务状态。

安装长期运行服务前，先确认没有手工启动的 DG 集成进程。安装脚本检测到重复实例会
直接拒绝，不会同时启动两套模型：

```bash
cd /home/nvidia/dg202611_ws
./scripts/install_dg_models_service.sh
```

该服务固定使用旁路观察配置：不启动 Nav2、不回放 UWB、不启用速度仲裁器，恢复速度
仍隔离在 `/cmd_vel_recovery`，不会接管原系统 `/cmd_vel`。查看与移除命令：

```bash
systemctl --no-pager --full status dg202611-models.service
./scripts/uninstall_dg_models_service.sh
```

现场完成 MODEL-3 受控运动验收前，不改变上述旁路配置。

回放真实 UWB 稳态数据，但先不让它参与地图融合：

```bash
./scripts/run_dg_integration.sh \
  enable_uwb_replay:=true
```

检查：

```bash
ros2 topic echo /dg/uwb/quality --once
ros2 topic echo /dg/uwb/pose --once
ros2 topic echo /dg/fusion/status --once
```

## 独立证明 MODEL-1 消费 UWB

因为现有 UWB 数据在另一场地采集，不能把单位变换直接冒充成机器人地图对齐。仓库
提供了一个独立的、无运动输出的证据入口：它只启动 UWB 回放和 MODEL-1 融合节点，
屏蔽实时里程计、GNSS、AMCL和scan-match输入，并把输出坐标系明确写成
`uwb_replay_test`。

```bash
./scripts/run_uwb_model1_evidence.sh static
```

另外两个场景为 `move` 和 `anchor_fault`。该入口不会启动速度仲裁器、不会发布 TF，
也不发布任何运动话题。验证命令：

```bash
ros2 topic echo /dg/fusion/status | grep -m 1 -A 20 UWB_AIDED
ros2 topic echo /dg/fusion/pose --once | grep -A 3 frame_id
```

通过标准是状态中同时出现 `fusion_mode=UWB_AIDED`、`accepted_source=UWB`、
`output_frame_id=uwb_replay_test`。这证明真实 UWB 数据被 MODEL-1 消费，但不代表完成
UWB 到机器人地图的现场标定，也不作为定位精度证据。

## UWB 与地图对齐

UWB 坐标系和机器人 `map` 坐标系还缺一个二维刚体变换。部署时在同一参考点记录
UWB `(x,y)` 和机器人地图 `(x,y)`，确认方向后设置：

```text
map_x = cos(yaw) * uwb_x - sin(yaw) * uwb_y + offset_x
map_y = sin(yaw) * uwb_x + cos(yaw) * uwb_y + offset_y
```

完成现场对齐后，才在正式地图坐标下启用 MODEL-1 的 UWB 更新：

```bash
./scripts/run_dg_integration.sh \
  enable_uwb_replay:=true \
  uwb_alignment_enabled:=true \
  uwb_to_map_yaw:=0.0 \
  uwb_to_map_offset_x:=0.0 \
  uwb_to_map_offset_y:=0.0
```

具体的三个变换值要用机器人现场地图参考点填写。缺少这一步时，程序会发布真实
UWB 质量和位置，但 MODEL-1 不会把它当作地图坐标更新。

## 实机运动边界

MODEL-1、MODEL-2 和 UWB 都可以在底盘不动时验证。MODEL-3 完整闭环需要机器人
转动，速度仲裁器也只有在现场有人看护、底盘驱动器供电正常、急停可用时才启用：

```bash
enable_cmd_vel_arbiter:=true cmd_vel_output_topic:=/cmd_vel
```

这两个参数不会写入默认配置或开机自启动。实机验证完成前，三模型输出保持在
`/dg/*` 旁路话题，不替换学长原系统的 `/odom`、TF 或手机控制链路。

## 保留的底盘方向修复

独立分支保留了已经在机器人上验证有效的手机控制方向修复：
`ylhb_mobile_bridge/ros_bridge.py` 发布时只反转 `linear.x`，不改变 `angular.z`，也
不修改 `base_kinematics.yaml`。
