# 事件相机与毫米波雷达到货前接入说明

## 当前完成边界

当前代码已经定义两种传感器进入机器人 ROS 2 系统的稳定接口，并支持对真实测量文件进行严格校验、回放和留证。两个入口默认关闭，只发布观测数据和诊断，不发布 `/cmd_vel`，也不改变 Nav2、MODEL-1、MODEL-2 或 MODEL-3 的决策。

```text
事件相机真实驱动:       等待 RDK X5 与相机
事件相机官方 CSV 解析:  已完成
事件相机 ROS 2 回放:    已完成
事件特征/重复率侧链:    已完成，等待真实数据调参
毫米波真实驱动:         已有厂商 Demo 协议资料，等待目标实物型号/原始帧
毫米波规范化 CSV 解析:  已完成
毫米波 ROS 2 回放:      已完成
毫米波辅助动态点剔除:   已完成，默认关闭，等待协议/外参/真实数据验证
真实传感器指标验收:     等待数据、标定和验证
```

“回放节点能够运行”只能证明接口和数据链路可用，不能写成硬件实测完成或算法已经提高指标。

## 事件相机链路

厂商 RDK X5 适配资料给出的流程是：

```text
EVS MIPI 模块
  -> RDK X5 CAM1
  -> evs_live_player 录制 RAW8
  -> raw8_to_evt2 转 EVT2
  -> evt2_to_csv 转 CSV
  -> auxiliary_sensor_replay_node
  -> /dg/event_camera/events (sensor_msgs/PointCloud2)
  -> /dg/event_camera/quality (diagnostic_msgs/DiagnosticArray)
  -> event_feature_node
  -> /dg/event_camera/features (sensor_msgs/PointCloud2)
  -> /dg/event_camera/feature_quality (diagnostic_msgs/DiagnosticArray)
```

官方 CSV 契约：

```csv
x,y,polarity,timestamp
352,240,1,1234567890
```

- `x`: 0–767；
- `y`: 0–607；
- `polarity`: 0 或 1；
- `timestamp`: 微秒，必须单调不减。

仓库解析器采用流式读取，不把几百 MB 的事件 CSV 一次性装入内存。回放时默认按 20 ms 分包。PointCloud2 字段为 `pixel_x`、`pixel_y`、`polarity`、`time_offset_us`。

厂商资料：

- <https://forum.shimetapi.cn/wiki/zh/evs-camera/mipi-modules/carrier-boards/rdk-x5.html>
- <https://forum.shimetapi.cn/wiki/zh/evs-camera/mipi-modules/getting-started.html>
- <https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5/>

RDK X5 和 Jetson 是两台计算机。相机先接 RDK X5；真实运行时可以在 RDK X5 上转换/发布，Jetson 通过同一局域网接收 ROS 2 话题。不能把 MIPI 排线直接插到 Jetson 并假设厂商驱动兼容。

`event_feature_node` 按固定像素网格聚合事件，过滤单个热像素和稀疏噪声，并用一对一空间匹配计算相邻事件包的重复率。该结果处于相机像素坐标系，只用于事件相机质量和特征证据；没有相机到激光外参及运动补偿前，不会被当成机器人绝对位姿或直接控制依据。

## 毫米波雷达链路

用户提供目录已包含 4D 成像雷达使用说明和 Demo 输出协议，但产品资料列出多个型号，尚未确认 HU293 对应实物、固件和完整字节序。代码不猜串口字节；先保留每个原始字节，再用目标型号协议转换成下面的规范化 CSV：

```csv
timestamp_us,frame_id,track_id,x_m,y_m,z_m,radial_velocity_mps,snr_db,confidence
1000000,1,0,2.30,-0.20,0.15,-0.45,18.0,0.92
```

字段必须使用米、米每秒、dB 和微秒；`confidence` 范围为 0–1。同一个 `frame_id` 的检测必须连续。回放输出：

- `/dg/mmwave/points`：`sensor_msgs/PointCloud2`，含 x/y/z、径向速度、SNR、置信度和 track ID；
- `/dg/mmwave/quality`：检测数量、动态检测数量、轨迹数量和质量统计。

启用 `enable_mmwave_dynamic_mask` 后，MODEL-2 只接受 frame、时间新鲜度、径向速度、SNR 和置信度均合格的检测，再根据 `mmwave_to_lidar_x/y/yaw` 转到激光坐标系，剔除同方位、同距离的激光点。诊断同时保留纯激光的 `raw_temporal_match_ratio` 和融合后的动态扇区/点数量，便于证明毫米波是否真正被消费。该开关默认关闭；安装外参未测量前不得在正式结果中启用。

当前厂商说明书给出的 Demo 串口默认值是 3000000 baud、8N1，并给出了帧头、帧尾和点字段；实物到手后仍必须核对完整型号、固件、UART/CAN 选择、电压和电平、引脚、实际波特率/CAN 速率、字节序、校验、坐标轴、单位、最大目标数、输出频率和 ROS/上位机资料。只要这些信息确定，厂商解码器可以接在规范化 CSV 或上述 ROS 2 接口前，不需要改三模型的外部接口。

## 现有工具

在 Jetson 上可用隔离话题执行一次接口冒烟检查：

```bash
./scripts/run_auxiliary_ros_smoke.sh
```

该脚本只使用明确标记为 `SYNTHETIC_CONTRACT_FIXTURE` 的小型数据，检查事件
回放、特征重复率、毫米波回放、MODEL-2 动态扇区消费以及进程组清理。它不发布
运动指令，输出不能作为真实硬件或老师指标证据。

校验事件 CSV：

```bash
python3 scripts/validate_auxiliary_sensor_data.py event_camera /data/events.csv \
  --output /data/events.validation.json
```

校验转换后的毫米波 CSV：

```bash
python3 scripts/validate_auxiliary_sensor_data.py mmwave /data/mmwave.csv \
  --output /data/mmwave.validation.json
```

第一次连接串口雷达时只读保存原始流：

```bash
python3 scripts/capture_serial_sensor_raw.py \
  --device /dev/ttyUSB0 \
  --baud 3000000 \
  --duration-sec 60 \
  --sensor HU293 \
  --output-dir /data/mmwave_capture_001
```

该脚本不发送协议命令，输出 `serial_raw.bin`、SHA-256 和 `capture_manifest.json`。若商家规定雷达必须先接收启动命令，则应在拿到协议后编写专用驱动，不能用这个只读脚本替代。

回放事件数据：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_event_camera_replay:=true \
  enable_event_feature_processing:=true \
  event_camera_input_file:=/data/events.csv
```

回放毫米波数据：

```bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_mmwave_replay:=true \
  enable_mmwave_dynamic_mask:=true \
  mmwave_to_lidar_x:=<实测米> \
  mmwave_to_lidar_y:=<实测米> \
  mmwave_to_lidar_yaw:=<实测弧度> \
  mmwave_input_file:=/data/mmwave.csv
```

验收：

```bash
ros2 topic info /dg/event_camera/events -v
ros2 topic echo /dg/event_camera/quality --once
ros2 topic info /dg/event_camera/features -v
ros2 topic echo /dg/event_camera/feature_quality --once
ros2 topic info /dg/mmwave/points -v
ros2 topic echo /dg/mmwave/quality --once
ros2 topic echo /dg/lidar/quality --once
ros2 topic info /cmd_vel -v
```

通过条件是数据话题持续更新、诊断显示 `source_kind=MEASURED_REPLAY`、时间戳和 frame 正确、验证 JSON 的哈希对应原始文件，事件特征节点报告有效相邻包，MODEL-2 报告实际掩蔽的毫米波动态扇区/点，并且新增节点不是 `/cmd_vel` 发布者。

## 到货后的最小数据集

完整的一次性目录、26 个 trial、现场记录字段和关闭硬件阶段的条件见 `AUXILIARY_SENSOR_ONE_PASS_TEST.md`。开始采集前先运行 `scripts/create_auxiliary_test_session.py`，避免数据散落或遗漏失败场景。

### 事件相机

至少保留以下场景，每段包含 RAW8、EVT2、CSV、开始/结束时间、布置照片和说明：

1. 机器人和目标都静止；
2. 人横穿激光雷达视野；
3. 标签/纹理板慢速移动；
4. 标签/纹理板快速移动；
5. 明暗突变；
6. 机器人原地旋转或手持相机受控转动。

### 毫米波雷达

至少采集：空场、单人静止、单人横穿、单人靠近/远离、多人、遮挡、不同距离的已知位置。每段保留原始串口/CAN 数据、上位机导出、规范化 CSV、布置尺寸和时间线。

## 后续与三模型的连接

- MODEL-2：毫米波动态目标位置标记激光扫描中的动态扇区的软件链路已实现；事件相机已有相邻时间窗特征和重复率证据链路。两者仍需真实协议、时间同步和外参验证后启用。
- MODEL-1：只有在外参、时钟和误差模型验证后，毫米波/事件视觉里程计才可作为测量源；原始点和事件不能直接当作绝对位姿。
- MODEL-3：可把经过验证的事件运动估计或毫米波稳定结构作为候选验证证据；当前不允许单一辅助传感器直接触发恢复运动。

当前已经把能在无硬件条件下确定的算法和接口写好。真实数据到手后进行协议适配、外参测量和阈值冻结；未标定数据不会写进定位状态。
