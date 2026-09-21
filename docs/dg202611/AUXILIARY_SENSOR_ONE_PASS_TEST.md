# 事件相机与毫米波雷达一次性实测计划

## 目标

本计划用于在硬件到齐后一次收齐后续算法、方案书和答辩所需原始数据。现场人员只负责摆放硬件、测量尺寸、按场景移动目标和确认安全；代码负责建目录、保存原始流、格式校验、ROS 2 回放、质量诊断和结果留证。

完成一次采集不等于指标已经通过。只有原始文件、协议、时间、外参、失败 trial、ROS 2 记录和评测结果全部可追溯，才可把该传感器阶段关闭。

## 开始前必须拿到的信息

### HU293 毫米波雷达

- 铭牌上的完整型号和硬件版本；
- 供电电压、电流、接口电平和引脚；
- UART 或 CAN 的速率；
- 帧头、帧长、字节序、校验和、坐标轴、单位、目标数及输出频率；
- 开始/停止输出命令；
- 厂商上位机、示例程序和协议文件。

没有上述协议时可以只读保存 `serial_raw.bin`，但不能生成可信的 x/y/速度数据，也不能完成 MODEL-2 融合验证。

### 事件相机

- EVS 模组完整型号；
- RDK X5 系统镜像与驱动版本；
- `evs_live_player`、`raw8_to_evt2`、`evt2_to_csv` 的实际版本和命令；
- 分辨率、镜头和曝光/阈值设置；
- RDK X5 与 Jetson 的网络地址和时间同步状态。

## 一次建好采集目录

在保存数据的计算机上执行：

```bash
python3 scripts/create_auxiliary_test_session.py \
  --output-root /data/dg202611_auxiliary \
  --operator peng \
  --radar-model '<铭牌完整型号>' \
  --event-camera-model '<模组完整型号>' \
  --carrier-model RDK-X5
```

脚本固定创建 26 个 trial：空场 2 次，其余 8 类场景各 3 次。每个 trial 都有 `event_raw`、`mmwave_raw`、`normalized`、`rosbag`、`logs` 和 `photos` 目录，并生成 `operator_log.csv`。失败 trial 也填写 `status` 和 `failure_reason`，不删除、不覆盖。

## 安装尺寸与时间必须记录

在 `00_calibration/sensor_mount_measurements.json` 记录：

```json
{
  "reference_frame": "laser_link",
  "mmwave_link": {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_rad": 0.0},
  "event_camera_optical_frame": {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0},
  "measurement_method": "tape_and_angle_marker",
  "measured_by": "peng"
}
```

所有数值必须现场测量，不能保留示例中的零值。同步前在 RDK X5 和 Jetson 分别保存：

```bash
date --iso-8601=ns
timedatectl status
chronyc tracking 2>&1 || true
```

输出写入 `00_calibration/time_sync.txt`。两机时钟未对齐时，仍保存原始数据，但不能解释跨传感器延迟或做严格逐帧关联。

## 固定场景

| 场景 | 动作 | 主要证据 |
|---|---|---|
| AUX00_EMPTY | 空场，所有设备静止 | 噪声、虚警、热像素 |
| AUX01_STATIC_RANGE | 目标分别位于卷尺 1/2/3 m | 距离偏差、静态检测边界 |
| AUX02_LATERAL_SLOW | 单人慢速横穿 | 动态扇区与激光剔除 |
| AUX03_LATERAL_FAST | 单人快速横穿 | 快速目标和事件率 |
| AUX04_APPROACH_AWAY | 沿卷尺线靠近、远离 | 径向速度符号和幅值 |
| AUX05_OCCLUSION | 从固定遮挡物后进出 | 丢失、重现和轨迹连续性 |
| AUX06_MULTI_TARGET | 两人反向交叉 | 多目标 ID 和误关联 |
| AUX07_LIGHT_CHANGE | 明暗切换，先静止后移动 | 事件相机光照敏感性 |
| AUX08_CONTROLLED_YAW | 有看护时车体转动；无底盘电池时整体手持刚性支架转动 | 自运动、外参和跨帧稳定性 |

每次开始/结束填写本地时间、实测距离、移动方法、光照、遮挡、文件路径和异常。移动速度没有测速设备时写明“按地面距离/视频时间计算”，不要凭感觉填速度。

## 数据保存

毫米波首次接入先保留原始字节。当前资料给出的 Demo 默认串口为 3000000 baud、8N1；若目标实物上位机显示不同值，以实物/固件设置为准：

```bash
python3 scripts/capture_serial_sensor_raw.py \
  --device /dev/ttyUSB0 \
  --baud 3000000 \
  --duration-sec 60 \
  --sensor '<完整型号>' \
  --output-dir '<当前trial>/mmwave_raw'
```

事件相机每个 trial 同时保留 RAW8、EVT2 和 CSV。转换命令及标准错误写入 `logs/`，不能只留最后的 CSV。

转换后的数据执行：

```bash
python3 scripts/validate_auxiliary_sensor_data.py event_camera events.csv \
  --output events.validation.json

python3 scripts/validate_auxiliary_sensor_data.py mmwave mmwave.csv \
  --output mmwave.validation.json
```

校验报告会保存源文件 SHA-256，并统计事件包缺口、最大包间隔、平均事件率、毫米波缺失帧号、有效帧率、最大帧间隔和每帧目标数。

## ROS 2 接入复核

事件数据：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false enable_cmd_vel_arbiter:=false \
  enable_event_camera_replay:=true \
  enable_event_feature_processing:=true \
  event_camera_input_file:=events.csv
```

毫米波与 MODEL-2：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false enable_cmd_vel_arbiter:=false \
  enable_mmwave_replay:=true enable_mmwave_dynamic_mask:=true \
  mmwave_input_file:=mmwave.csv \
  mmwave_to_lidar_x:='<实测米>' \
  mmwave_to_lidar_y:='<实测米>' \
  mmwave_to_lidar_yaw:='<实测弧度>'
```

验收时保留以下话题：

```text
/scan
/dg/lidar/filtered_scan
/dg/lidar/stable_features
/dg/lidar/quality
/dg/mmwave/points
/dg/mmwave/quality
/dg/event_camera/events
/dg/event_camera/quality
/dg/event_camera/features
/dg/event_camera/feature_quality
```

MODEL-2 的 `/dg/lidar/quality` 必须出现 `auxiliary_dynamic_data_fresh=True`、非零 `auxiliary_dynamic_sector_count`，且场景中存在对应激光返回时出现非零 `auxiliary_dynamic_point_count`。事件特征诊断必须出现 `repeatability_status=COMPARED`。仅看到话题名称不算融合成功。

## 可以结束硬件阶段的条件

1. 26 个 trial 都有记录，失败项未删除；
2. 毫米波厂商原始协议和事件相机版本信息已保存；
3. 安装位置、坐标轴和时间同步有文字、数值和照片；
4. 每个有效 trial 都保留原始文件、转换文件、校验 JSON 和 SHA-256；
5. 事件与毫米波 ROS 2 话题能持续发布，frame 和字段完全匹配；
6. MODEL-2 的诊断证明毫米波数据被实际消费；
7. 事件特征及相邻包重复率被实际记录；
8. 与 `/scan` 同步的 rosbag 完整，新增节点不是 `/cmd_vel` 发布者；
9. 原始数据复制到 Windows 后再次校验哈希。

满足这些条件后，算法调参、回放和方案书统计都可离线继续；若缺厂商协议、外参、时间同步或关键场景，后续仍可能需要重测。
