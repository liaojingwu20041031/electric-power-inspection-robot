# MODEL-1 高程融合到货前准备

## 目标与边界

老师要求同时给出平面和高程定位精度。原有 MODEL-1 只输出二维位姿，因此当前增加一条默认关闭的高程侧链：GNSS 提供绝对高程锚点，机器人已有 ZED 2i 的重力对齐视觉里程计提供短时相对高度变化。

当前完成的是软件接口、异常门控、记录和评测能力。未运行 ZED 实机、未测外参、未取得独立 Z 真值，因此不能宣称已经达到 20 cm。

## 数据流

```text
/gps/fix.altitude
  -> WGS84 到本地 ENU 的 up
  -> MODEL-1 绝对高程锚定与缓慢校正

/zed/zed_node/odom.pose.pose.position.z
  -> 时间戳、协方差和跳变门控
  -> GNSS 中断期间传播相对高度变化

MODEL-1
  -> /dg/fusion/odom.pose.pose.position.z
  -> /dg/fusion/pose.pose.pose.position.z
  -> /dg/fusion/status 中的 altitude、altitude_uncertainty、altitude_anchored、altitude_source
```

ZED 重启或局部坐标原点重置产生的大幅 Z 跳变会被拒绝，并把当前样本作为新的相对基线；跳变不会直接写入绝对高程。

## 安全启动方式

ZED 默认会发布 TF。它与现有 Nav2 同时运行时必须关闭 ZED 的 TF 所有权，避免出现第二套 `odom -> camera_link` 或 `map -> odom`：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_on_jetson.sh zed publish_tf:=false publish_map_tf:=false
```

另一个终端启用高程侧链；命令仲裁仍关闭：

```bash
cd ~/dg202611_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_base dg_navigation_integration.launch.py \
  enable_nav2:=false \
  enable_cmd_vel_arbiter:=false \
  enable_relative_height:=true \
  relative_height_odom_topic:=/zed/zed_node/odom
```

在真机执行前先确认：

```bash
ros2 topic info /zed/zed_node/odom -v
ros2 topic echo /zed/zed_node/odom --once
ros2 topic echo /dg/fusion/status --once
ros2 topic info /cmd_vel -v
```

## 一次采全的现场记录

`real_robot_capture.py` 现在默认保存原始 GNSS、ZED odom、UWB、事件相机、毫米波、三模型输出、TF 和底盘状态。已知测试点需同时填写统一 `map` 坐标系中的独立真值：

```bash
python3 scripts/real_robot_capture.py \
  --scenario vertical_static_point_01 \
  --duration 60 \
  --ground-truth-x <m> \
  --ground-truth-y <m> \
  --ground-truth-z <m> \
  --ground-truth-frame map
```

评测器分别输出水平误差和绝对高程误差。没有填写 `--ground-truth-z` 的历史数据仍能评测 XY，但不会把缺失的 Z 当作 0 或伪造高程指标。

## 实机验收

1. 静止 60 秒，确认 ZED 的 Z 漂移、时间戳、帧名和协方差；
2. 在两个已知高度点之间移动，核对高度变化方向和尺度；
3. GNSS 正常时建立绝对锚点；
4. 中断 GNSS 输入，确认 ZED 继续传播 Z 且不改变 XY；
5. 重启 ZED，确认出现 `RELATIVE_HEIGHT_JUMP` 且融合高程不突跳；
6. 使用独立测量的 XY/Z 真值计算误差，不用 AMCL、ZED 或 GNSS 自己充当其自身真值。
