# DG-202611 机器人实机验收记录

**用途**：由现场人员执行，代理只做只读检查和记录整理。  
**底盘电源**：在完成静态检查前保持关闭。  
**当前RDK目标**：`root@192.168.43.13`（IP随热点变化时以现场实际地址为准）。

## 1. 启动前记录

```bash
whoami
pwd
hostname
ip -br addr
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
ros2 doctor --report
date -Is
timedatectl status
```

记录：

```text
日期时间：
NVIDIA地址：
RDK地址：
NVIDIA系统时间：
RDK系统时间：
两板时钟是否已同步：
Git提交号：
ROS_DOMAIN_ID：
RMW_IMPLEMENTATION：
底盘电源：OFF
急停状态：
```

## 2. RDK事件相机检查

RDK端先确认：

```bash
whoami
pwd
echo "$HOME"
date -Is
timedatectl status
ls -ld /app/multimedia_samples/sample_apx003cc 2>/dev/null || true
ls -l /dev/video* /dev/media* /dev/v4l-subdev* 2>&1 || true
```

使用当前用户目录进行30至60秒长测，确认：

只有设备节点存在且两板时间正确后，才开始本次长测和跨板延迟记录。

```text
EVS持续有数据
APS持续有数据
pair_valid=true
fused_frames持续增加
evs_get_frame_errors=0
aps_get_frame_errors=0
```

实时HBN桥运行时不要再启动会重复发布`/dg/event_camera/features`的sidecar。

## 3. 跨板发现和观察模式

两侧设置：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

先做无运动测试。RDK发布：

```bash
ros2 topic pub -r 1 /dg/link_test std_msgs/msg/String \
  "{data: 'rdk_to_jetson'}"
```

NVIDIA只读确认：

```bash
ros2 topic echo /dg/link_test --once
ros2 topic list | grep '/dg/'
ros2 topic info /dg/event_camera/features -v
ros2 topic echo /dg/event_camera/quality --once
ros2 topic echo /dg/event_camera/features --once
ros2 topic echo /dg/event_camera/camera_info --once
```

NVIDIA观察侧链使用：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_hybrid_vision_assist:=true \
  enable_auxiliary_transport_monitor:=true \
  enable_mmwave_observation:=true \
  enable_mmwave_dynamic_mask:=false
```

通过标准：

- 事件质量不是`UNSEEN`；
- `features`持续到达；
- `model2_event_assist_observable=true`；
- `/dg/auxiliary/transport_quality`出现消息年龄和延迟字段；
- 新增节点不发布`/cmd_vel`；
- 不改变`/scan`或`/dg/lidar/filtered_scan`。

## 4. UWB回放和三模型检查

使用已采集的UWB文件回放，不重新安装基站：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_multisource_fusion:=true \
  enable_uwb_replay:=true \
  enable_scan_map:=true
```

记录：

```bash
ros2 topic echo /dg/uwb/quality --once
ros2 topic echo /dg/fusion/status --once
ros2 topic echo /dg/fusion/pose --once
ros2 topic echo /dg/lidar/quality --once
ros2 topic echo /dg/relocalization/match_quality --once
```

通过标准：

- 出现UWB质量状态；
- MODEL-1状态出现`UWB_AIDED`或明确拒绝原因；
- 物理源去重不重复计数；
- UWB断链、拒绝、恢复状态可在日志中区分；
- UWB异常不会单独触发MODEL-3运动恢复。

## 5. 毫米波回放

只有拿到真实毫米波数据并转换为规范化CSV后执行：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_mmwave_replay:=true \
  mmwave_input_file:=/path/to/normalized_mmwave.csv \
  enable_mmwave_observation:=true \
  enable_mmwave_dynamic_mask:=false
```

通过标准：

- `frame_id=mmwave_link`；
- 点云字段完整；
- 质量话题新鲜；
- MODEL-2能观察到毫米波数据；
- 动态掩码仍为关闭状态。

## 6. 低速运动验收

只有完成前面所有静态检查、确认底盘电池和急停状态后，由现场人员：

1. 打开底盘电源；
2. 确认周围无人和无障碍风险；
3. 低速、短距离执行前后左右动作；
4. 随时准备断电或急停；
5. 保存`/cmd_vel`、`/odom`、TF、模型状态、控制器日志和现场照片。

代理不代替现场人员执行运动命令。

## 7. 结果填写

```text
静态ROS检查：PASS / FAIL
RDK本地采集：PASS / FAIL
跨板DDS：PASS / FAIL
事件相机Jetson接收：PASS / FAIL
UWB MODEL-1回放：PASS / FAIL
毫米波规范化回放：PASS / FAIL / 未执行
MODEL-3闭环：PASS / FAIL / 未执行
低速实机：PASS / FAIL / 未执行
主要故障：
日志目录：
rosbag/CSV目录：
照片目录：
最终结论：
```
