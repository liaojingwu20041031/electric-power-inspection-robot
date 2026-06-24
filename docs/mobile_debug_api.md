# Mobile Bridge 手机调试接口

Base URL：`http://<robot_ip>:8000`

本文档对应当前 ROS bridge 后端，供手机 APP 的底盘调试页和建图页直接对接。
本阶段不要求 APP 接入导航、任务、感知和巡逻功能。

Mobile Bridge 提供局域网内的底盘低速调试、建图控制、状态查询和地图预览。
手机 APP 使用接口前，必须先在机器人上启动 `mobile_bridge`：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

Bridge 启动后，APP 可以通过 HTTP 启动 `bringup` 和 `mapping`，不再要求用户
另外通过 SSH 启动这两个模式。若要做到机器人开机后手机直接可用，后续应把
`mobile_bridge` 配置为 systemd 自启动服务。

端口 `8000` 只用于可信局域网，不应映射到公网。

## 统一响应

除 WebSocket 鉴权失败会直接关闭连接外，HTTP 响应和 WebSocket 数据帧统一
使用以下信封：

```json
{
  "ok": true,
  "message": "system status",
  "data": {},
  "error": null,
  "timestamp": 1710000000.0
}
```

失败时 `ok=false`，常见 `error` 为 `validation_error`、
`invalid_request`、`process_error`、`ros_unavailable`、`no_map`、
`unauthorized`、`not_found`。

APP 不能只根据 HTTP 状态码判断业务成功，必须同时检查 JSON 中的 `ok`：

- 正常业务响应：HTTP `200`，`ok=true`。
- `no_map`、进程启动失败等业务错误：当前通常仍是 HTTP `200`，
  `ok=false`。
- 请求体校验失败：HTTP `422`，`error=validation_error`。
- token 缺失或错误：HTTP `401`，`error=unauthorized`。
- 系统 mode 不存在：HTTP `404`，`error=not_found`。

建议 APP 使用以下基础类型：

```ts
type ApiResponse<T> = {
  ok: boolean;
  message: string | null;
  data: T | null;
  error: string | null;
  timestamp: number;
};
```

默认配置为 `require_token=false`。启用 token 后：

- HTTP：`Authorization: Bearer <token>` 或 `X-API-Token: <token>`
- WebSocket：地址后附加 `?token=<token>`
- WebSocket token 错误时服务端使用关闭码 `1008`，不会先发送 JSON 错误帧。

HTTP JSON 请求应发送 `Content-Type: application/json`。无请求体的 POST
接口不需要发送 `{}`。

## 系统进程状态

### 查询底盘和建图进程

`GET /api/debug/system/status`

`data` 包含 `bringup` 和 `mapping`：

```json
{
  "bringup": {
    "mode": "bringup",
    "command": ["/home/nvidia/ros2_DL/scripts/run_on_jetson.sh", "bringup"],
    "pid": 12345,
    "started_at": 1710000000.0,
    "running": true,
    "returncode": null,
    "log_path": "/home/nvidia/ros2_DL/logs/mobile_bridge/bringup.log",
    "log_tail": "...",
    "managed_by_bridge": true
  },
  "mapping": {
    "mode": "mapping",
    "command": null,
    "pid": null,
    "started_at": null,
    "running": false,
    "returncode": null,
    "log_path": null,
    "log_tail": "",
    "managed_by_bridge": false
  }
}
```

- `managed_by_bridge=true`：该进程由本次 bridge 实例启动，可通过停止接口管理。
- `running=false` 且 `returncode` 非空：进程已退出，APP 应显示 `log_tail`。
- `managed_by_bridge=false`：bridge 没有启动记录，停止接口不会查找或强杀 SSH
  手动启动的 ROS 进程。
- `started_at` 和统一信封中的 `timestamp` 均为 Unix 时间戳，单位为秒。
- `log_tail` 最多读取日志文件末尾约 8 KiB，不保证包含完整启动日志。

建议 APP 类型：

```ts
type ProcessStatus = {
  mode: "bringup" | "mapping";
  command: string[] | null;
  pid: number | null;
  started_at: number | null;
  running: boolean;
  returncode: number | null;
  log_path: string | null;
  log_tail: string;
  managed_by_bridge: boolean;
};
```

### 启动和停止

`POST /api/debug/system/start/bringup`

`POST /api/debug/system/start/mapping`

`POST /api/debug/system/stop/mapping`

`POST /api/debug/system/stop/bringup`

以上接口均无请求体。成功时 `data=null`，结果文字位于 `message`，例如：

```json
{
  "ok": true,
  "message": "bringup started pid=12345",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

重复启动运行中的 mode 仍返回 `ok=true`，`message` 为
`bringup already running` 或 `mapping already running`。停止未由 bridge
启动的 mode 也返回 `ok=true`，但 `message` 为
`<mode> was not started by bridge`，APP 应结合 `managed_by_bridge` 展示状态，
不能只看 `ok`。

后端使用 argv list、`shell=False` 调用
`scripts/run_on_jetson.sh <mode>`。长运行输出写入每个模式的日志文件，不接到
未消费的 stdout pipe。

停止 `bringup` 前，APP 应先提示并停止 `mapping`。服务端不会自动连带停止，
也不会停止不是 bridge 启动的进程。

## APP 底盘页流程

推荐页面内容：连接状态、底盘启动状态、`/odom`、`/scan`、`/imu/data`、TF
状态、低速方向按钮、急停按钮和最近进程日志。

推荐流程：

1. 连接 `http://<robot_ip>:8000`。
2. 调用 `GET /api/debug/system/status`。
3. 用户点击“启动底盘”，调用
   `POST /api/debug/system/start/bringup`。
4. 轮询 `GET /api/debug/status`，等待 `/odom`、`/scan`、
   `/imu/data` 和 TF 就绪。
5. 就绪后解锁低速方向按钮。

`GET /api/debug/status` 的主要 `data` 字段：

```ts
type DebugStatus = {
  online: boolean;
  topics: {
    "/cmd_vel": boolean;
    "/odom": boolean;
    "/scan": boolean;
    "/map": boolean;
    "/imu/data": boolean;
  };
  nodes: {
    zlac8015d_canopen_controller: boolean;
    slam_toolbox: boolean;
    bt_navigator: boolean;
    controller_server: boolean;
    planner_server: boolean;
    amcl: boolean;
    map_server: boolean;
    bringup: boolean;
    rplidar_node: boolean;
    imu: boolean;
    tf: boolean;
  };
  last_odom_age_sec: number | null;
  last_scan_age_sec: number | null;
  last_map_age_sec: number | null;
  last_imu_age_sec: number | null;
  scan_range_min: number | null;
  scan_range_max: number | null;
  zlac_status: string;
  mapping_status: "running" | "not_running";
  nav2_status: "running" | "not_running";
  task_status: string;
  system_mode: string;
  pose: { frame: string; x: number; y: number; yaw: number } | null;
  velocity: { linear_x: number; angular_z: number } | null;
  map_meta: MapMeta | null;
};
```

建议解锁底盘按钮的最低条件为：

- `topics["/odom"]`、`topics["/scan"]`、`topics["/imu/data"]` 为 `true`
- `nodes.tf=true`
- `last_odom_age_sec`、`last_scan_age_sec`、`last_imu_age_sec` 非空且不大于
  APP 设定的新鲜度阈值，例如 3 秒

低速控制：

`POST /api/cmd_vel`

```json
{ "linear_x": 0.03, "angular_z": 0.0, "duration_ms": 300 }
```

也可使用 `POST /api/debug/chassis/test`，请求体额外包含用于 UI 标记的
`mode`，例如：

```json
{
  "mode": "forward",
  "linear_x": 0.03,
  "angular_z": 0.0,
  "duration_ms": 300
}
```

`mode` 是必填字符串，只用于响应消息，不参与底盘运动计算。

服务端处理规则：

- `linear_x` 超过有效范围时会被限幅；默认最大绝对值为 `0.15 m/s`，
  配置只能把该值调低。
- `angular_z` 超过有效范围时会被限幅；默认最大绝对值为 `0.5 rad/s`，
  配置只能把该值调低。
- `duration_ms` 必须为 `50..3000`；越界不是限幅，而是返回 HTTP `422`。
- 三个速度请求字段都有默认值：`linear_x=0`、`angular_z=0`、
  `duration_ms=300`。

急停使用 `POST /api/stop` 或 `POST /api/debug/chassis/stop`。
两者均无请求体。`/api/stop` 还会向任务话题发布“停止当前任务”，底盘调试页
通常应优先使用它作为总急停。

## APP 建图页流程

推荐页面内容：底盘前置状态、建图进程状态、地图预览、保存地图、停止建图和
日志/错误区。

`GET /api/debug/mapping/status` 返回：

- `mapping_status`：根据 SLAM Toolbox 节点是否存在返回 `running` 或
  `not_running`。
- `bringup_ready`：只检查 ROS graph 中 `/odom`、`/scan`、`/imu/data`
  话题存在且 `/tf` 有发布者，不检查消息新鲜度。
- `map_available`：bridge 是否收到过 `/map`。
- `recommended_next_action`：
  `start_bringup`、`start_mapping`、`wait_for_map` 或
  `continue_mapping_or_save`。
- `process`：建图进程的 PID、退出码、日志路径和日志尾部。
- `last_map_age_sec`：最近一次收到 `/map` 距当前的秒数，尚未收到时为
  `null`。
- `map_meta`：地图原始尺寸、分辨率、frame 和原点，尚未收到时为 `null`。

`mapping_status` 在启动后的短暂窗口可能仍为 `not_running`，此时如果
`process.running=true` 且 `recommended_next_action=wait_for_map`，APP 应显示
“建图进程启动中”，不要重复调用启动接口。

推荐流程：

1. 确认 `bringup_ready=true`；否则引导用户先启动底盘。
2. 用户点击“开始建图”，调用
   `POST /api/debug/system/start/mapping`，兼容入口为
   `POST /api/debug/mapping/start`。
3. 使用 `WS /ws/map?downsample=1` 或
   `GET /api/debug/mapping/map_snapshot?downsample=1` 显示地图。
4. 调用 `POST /api/debug/mapping/save` 保存地图。
5. 调用 `POST /api/debug/system/stop/mapping` 或兼容停止入口。

保存地图请求：

```json
{ "map_name": "my_map" }
```

`map_name` 必须匹配 `^[A-Za-z0-9_-]+$`。无 `/map` 数据时，地图快照接口
返回 `ok=false,error=no_map`，APP 应显示“等待 /map 数据”。

保存成功时 `data` 为：

```ts
type SavedMap = {
  yaml_path: string;
  pgm_path: string;
  output: string;
};
```

保存地图最多等待约 60 秒。APP 应给保存按钮增加 loading 状态并避免重复提交。

地图快照成功时 `data` 为：

```ts
type MapMeta = {
  frame_id: string;
  timestamp: number;
  resolution: number;
  width: number;
  height: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
};

type MapSnapshot = {
  map_meta: MapMeta;
  png_base64: string;
};
```

React Native 可将图片源设置为
`data:image/png;base64,${png_base64}`。`downsample` 仅允许 `1..16`：
HTTP 超出范围返回 `422`；WebSocket 中超出范围会被截断到该范围，无法解析时
回退为 `1`。

## 状态与实时接口

- `GET /api/status`：机器人基础状态。
- `GET /api/debug/status`：话题、节点、数据新鲜度、位姿、速度和地图元信息。
- `WS /ws/status`：按 `status_rate_hz` 推送状态。
- `WS /ws/map?downsample=1`：按 `map_stream_rate_hz` 推送 PNG 地图快照。

`GET /api/status` 的 `data` 类型：

```ts
type RobotStatus = {
  online: boolean;
  can_status: "online" | "unknown";
  zlac_status: string;
  task_status: string;
  system_mode: string;
  mapping_status: "running" | "not_running";
  nav2_status: "running" | "not_running";
  last_odom_age_sec: number | null;
  last_scan_age_sec: number | null;
  pose: { frame: string; x: number; y: number; yaw: number } | null;
  velocity: { linear_x: number; angular_z: number } | null;
  timestamp: number;
};
```

注意该 `data.timestamp` 是 ROS bridge 生成状态的时间；外层
`ApiResponse.timestamp` 是构造响应信封的时间，两者可能略有差异。

WebSocket URL 应由 Base URL 转换：

- `http://192.168.1.50:8000` -> `ws://192.168.1.50:8000/ws/status`
- `https://example` -> `wss://example/ws/status`

当前 WebSocket 是服务端单向推送，APP 不需要发送消息。`/ws/map` 在没有地图
时会持续推送 `ok=false,error=no_map`，不是关闭连接。默认状态频率约 2 Hz，
地图频率约 1 Hz。

`/ws/status` 或 `/ws/map` 断开及服务端处理异常时，bridge 会发布一次零速度。
APP 仍应在进入后台、离开控制页和网络断开时主动调用急停；不要依赖
WebSocket 断开作为唯一安全机制。

建议自动重连使用有限退避，例如 1、2、5 秒，并在重连期间锁定方向按钮。

## APP 最小接口清单

| 页面 | 方法 | 路径 | 请求体 |
|---|---|---|---|
| 连接 | GET | `/api/status` | 无 |
| 状态 | GET | `/api/debug/status` | 无 |
| 系统 | GET | `/api/debug/system/status` | 无 |
| 底盘 | POST | `/api/debug/system/start/bringup` | 无 |
| 底盘 | POST | `/api/cmd_vel` | `VelocityCommand` |
| 底盘 | POST | `/api/stop` | 无 |
| 建图 | GET | `/api/debug/mapping/status` | 无 |
| 建图 | POST | `/api/debug/system/start/mapping` | 无 |
| 建图 | GET | `/api/debug/mapping/map_snapshot?downsample=1` | 无 |
| 建图 | POST | `/api/debug/mapping/save` | `{ "map_name": "my_map" }` |
| 建图 | POST | `/api/debug/system/stop/mapping` | 无 |
| 实时 | WS | `/ws/status` | 无 |
| 实时 | WS | `/ws/map?downsample=1` | 无 |

后端还保留 `/api/debug/mapping/start`、`/api/debug/mapping/stop` 兼容入口。
APP 新代码建议统一使用 `/api/debug/system/start|stop/{mode}`。

## 常见错误

- Bridge 未启动：手机无法连接 `8000`；先在 Jetson 本机执行
  `curl http://localhost:8000/api/status`。
- 底盘未就绪：`bringup_ready=false`；检查 `/odom`、`/scan`、
  `/imu/data` 和 `/tf`。即使为 `true`，仍需检查数据 age。
- 启动失败：查看系统状态中的 `returncode`、`log_path` 和 `log_tail`。
- `no_map`：建图刚启动或 `/map` 未发布，APP 继续等待并重试。
- 无法停止：`managed_by_bridge=false` 表示进程不是 bridge 启动，应由原
  SSH/终端会话管理。
- HTTP `422`：请求体字段缺失、类型错误、`duration_ms` 越界、
  `map_name` 不合法或 HTTP `downsample` 超出范围。
- WebSocket 关闭码 `1008`：token 缺失或错误。
