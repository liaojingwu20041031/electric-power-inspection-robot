# RDK X5 HVS 双流实测状态（2026-09-12）

## 设备边界

本次实测只在 RDK X5（当时记录为 `root@192.168.43.13`）进行，未连接、未修改机器人 Jetson（`nvidia@192.168.43.12`）和 `/home/nvidia/ros2_DL`。历史证据中的旧记录路径只用于校验已采集文件，不作为当前账号或记录目录要求。

## 已通过：厂商底层双流

在 RDK 上运行：

```bash
timeout -s INT -k 10 60 stdbuf -oL -eL \
  /app/multimedia_samples/sample_apx003cc/sample_hvs/dual_vc_vin_src/dual_vc_vin \
  -c 1 -d 0
```

实测日志确认：

- `VC0/VC1 are ready`；
- APS 已创建 ISP 节点并持续取帧；
- EVS 约 `208.5–209.0 FPS`；
- APS 约 `29.79–29.85 FPS`；
- 程序持续运行 60 秒后由 `timeout` 结束，退出码 `124` 只表示到时停止。

证据目录：

```text
evidence/rdk_hvs_20260912_141259/
```

因此可以记录为：

```text
RDK_VENDOR_DUAL_STREAM_PASS
```

## 已通过：DG RDK ROS 双流桥

桥接已经改为直接复用厂家 HBN 双 VC 管线：EVS 与 APS 分别由独立线程取帧，APS 使用 RDK `MEM_PIX_FMT_NV12`，并在 APS 到来时从最近的 EVS 硬件时间戳中选择配对帧。

板端启动和话题验证结果：

- ROS 节点：`/dg_hvs_bridge_node`；
- EVS 质量：`evs_fresh=true`；
- APS 质量：`aps_fresh=true`；
- 配对：`pair_valid=true`，实测 `pair_skew_ms≈0.020 ms`；
- APS 源频率：约 `30.0 Hz`；
- APS 格式：`NV12`，`1632×1224`，两 plane，`size0=1997568`、`size1=998784`；
- 融合图：`fused_frames>0`，约 `10 Hz`；
- 事件话题：约 `14.9 Hz`，由桥接层限速和消息体积共同决定；
- EVS/APS 取帧错误：均为 `0`；
- 节点没有 `/cmd_vel` 发布者或订阅者。

桥接录制模式也已在 RDK 上写出 RAW8、EVT2、CSV、APS PGM 和融合 JPEG。配对阈值设置为 `40.0 ms`：APS 周期约 `33.33 ms`，增加少量调度余量；更大的时间间隔仍会进入 `DEGRADED` 并跳过融合图。

说明：RDK 掉线前板端已验证 v7 的双线程 HBN 采集和 GOOD 状态；40.0 ms 是本地已提交的鲁棒性配置，尚未在 RDK 恢复连接后重新跑一段长测，因此下一次上板需补做确认。

当前可以记录为：

```text
RDK_ROS_HVS_BRIDGE_PASS
```

## 仍未完成：机器人跨板联调

以上 PASS 只表示 RDK 相机侧和 RDK 本地 ROS 桥已经通过，不表示机器人 Jetson 三模型已经消费事件相机数据。下一阶段仍需：

1. RDK 与机器人建立 ROS 2 网络通信；
2. 在机器人侧只接收 APS 压缩图、事件特征、运动掩码和 quality，默认不传完整 EVS；
3. 先以诊断观察模式接入 MODEL-2，不修改激光结果；
4. 完成相机内参、APS/EVS 映射、相机到激光雷达外参和 IMU/轮速运动补偿后，再开启事件辅助判断；
5. 记录机器人端实际消费证据，再单独判断三模型融合状态。

## 证据和边界

- 当前 HBN 模式的 `sensor_index` 与 I2C 总线由厂家 `hvs_config` 决定；YAML 中的 `sensor_index`、`i2c_bus` 用于请求和诊断，不覆盖厂家配置。
- 完整事件数据保存在 RDK，本节点不发布 `/cmd_vel`。
- 在 APS ROS 输出通过之前的旧 Toolkit `Backend::MipiHvs` 失败记录仍保留，但不再代表当前 HBN 桥的状态。
- 当前相机内参仍为占位值，未完成标定前不能把事件相机写成已经改变 MODEL-2 激光判断。