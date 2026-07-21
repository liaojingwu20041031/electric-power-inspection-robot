# 机器人端 GNSS Heartbeat 对接设计

## 目标

严格按照 `/home/nvidia/PC_WJ/platform_realtime_gps_and_track_modification_plan_v3.md` 完成机器人端 GPS 对接，使现有 WTRTK980 数据通过云端 heartbeat 的顶层 `gnssFix` 字段上报，同时不改变巡逻、导航和命令链路。

## 修改范围

- `src/ylhb_mobile_bridge/ylhb_mobile_bridge/ros_bridge.py`
- `src/ylhb_mobile_bridge/ylhb_mobile_bridge/platform_cloud_client.py`
- `src/ylhb_mobile_bridge/config/mobile_bridge.yaml`
- `src/ylhb_mobile_bridge/package.xml`
- 现有的 `src/ylhb_mobile_bridge/test/` 回归用例

现有 `wtrtk980_nmea_node`、`bringup.launch.py` 和 `/gps/nmea_sentence` 行为保持不变。

## 数据流

```text
WTRTK980
  -> /gps/fix (sensor_msgs/NavSatFix)
  -> /gps/rtk_status (diagnostic_msgs/DiagnosticArray)
MobileRosBridge
  -> gnssFix 快照
PlatformCloudClient
  -> heartbeat 顶层 gnssFix
```

`MobileRosBridge` 新增以下参数，默认值与 v3 文档一致：

```yaml
gps_fix_topic: "/gps/fix"
gps_status_topic: "/gps/rtk_status"
gps_stale_timeout_sec: 3.0
```

## GNSS 快照合同

`gnssFix` 是 heartbeat 顶层可选对象，不嵌入 `patrol` 或 `health`：

```json
{
  "valid": true,
  "stale": false,
  "frame": "gps_link",
  "latitude": 31.1234567,
  "longitude": 121.1234567,
  "altitude": 12.4,
  "quality": 4,
  "fixType": "RTK_FIXED",
  "satellites": 18,
  "hdop": 0.7,
  "differentialAge": 0.4,
  "baseStationId": "1001",
  "ageSec": 0.2,
  "observedAt": "2026-07-21T10:30:00.123Z"
}
```

`fixType` 严格按 GGA quality 映射：0=`NO_FIX`、1=`SINGLE_POINT`、2=`DGPS`、4=`RTK_FIXED`、5=`RTK_FLOAT`，其他值为 `UNKNOWN`。

## 有效性与时效性

- `/gps/fix` 回调保存坐标、海拔、frame 和观测时间。
- 接收时间同时保存为本机单调时钟；`ageSec` 和 `stale` 只用单调时钟计算，避免系统时间回拨。
- 纬度、经度必须是有限数值并位于合法范围；海拔非有限值时上报 `null`。
- `valid=true` 必须同时满足坐标合法、`quality > 0` 且 `stale=false`。
- 无 GPS 数据时 `gnssFix=null`；数据无效或过期时保留诊断快照，但 `valid=false`。
- NaN、Infinity 和原始 NMEA 不进入 JSON。

## Heartbeat 与诊断

`PlatformCloudClient._heartbeat_payload()` 在现有字段基础上增加：

```python
"gnssFix": snapshot.get("gnssFix")
```

`health` 增加 `gpsAgeSec`、`gpsFixType`、`gpsSatellites` 和 `gpsHdop`。GPS 故障不得阻断 heartbeat、事件上传或命令拉取。

`robot_status()` 返回 `gnssFix` 和 `lastGpsAgeSec`；`debug_status()` 的 topics 增加 `/gps/fix` 与 `/gps/rtk_status` 可用性。

日志只记录定位状态变化、过期、恢复、非法坐标或状态解析失败；不按 GPS 发布频率输出 INFO，也不记录原始 NMEA。

## 验证

- 在已有测试文件中增加最小确定性回归：quality 映射、无效数值过滤、stale 和 heartbeat 字段透传。
- 只运行相关测试并构建 `ylhb_mobile_bridge`，不运行全仓库测试。
- 实机验收检查 `/gps/fix`、`/gps/rtk_status`、本地状态接口和实际 heartbeat；GPS 断开后确认 heartbeat 与命令链路仍工作。
- 实机是否可用以真实 ROS 话题和平台收到的 heartbeat 为准，Mock/Fake 不作为完成证明。
