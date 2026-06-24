# Mobile Bridge APP 调试接口

Base URL：`http://<robot_ip>:8000`

本文档只覆盖当前 APP 需要的功能：状态查看、底盘低速控制、建图调试。APP
本阶段不要接入导航、任务、感知、巡逻等接口。

机器人端需先启动 mobile bridge：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

Bridge 监听 `8000` 端口，只建议在可信局域网使用，不要映射到公网。

## 重要约定

- APP 只允许通过系统进程端口启动/停止 `bringup` 和 `mapping`。
- `POST /api/debug/system/start/navigation` 和
  `POST /api/debug/system/stop/navigation` 返回 HTTP `404`，APP 不应调用。
- 建图预览和保存只接受 SLAM Toolbox 当前发布的 `/map`。旧的导航静态地图
  `map_server` 不会被当作建图结果返回给 APP。
- 检测到 `map_server` 与 SLAM Toolbox 同时运行时，bridge 拒绝缓存、预览和
  保存 `/map`，避免无法区分静态地图与实时地图；应先停止外部导航进程。
- 每次开始或停止建图时，bridge 会清空上一次缓存的地图，避免 APP 看到旧图。
- 无请求体的 `POST` 不需要发送 `{}`。
- JSON 请求建议发送 `Content-Type: application/json`。

## 统一响应

HTTP 响应和 WebSocket 数据帧使用统一信封：

```ts
type ApiResponse<T> = {
  ok: boolean;
  message: string | null;
  data: T | null;
  error: string | null;
  timestamp: number;
};
```

成功示例：

```json
{
  "ok": true,
  "message": "bringup started pid=12345",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

失败示例：

```json
{
  "ok": false,
  "message": "no current SLAM map has been received",
  "data": null,
  "error": "no_map",
  "timestamp": 1710000000.0
}
```

APP 判断业务结果必须看 `ok`，不能只看 HTTP 状态码：

| 场景 | HTTP | `ok` | `error` |
|---|---:|---|---|
| 正常成功 | 200 | `true` | `null` |
| 无当前建图地图 | 200 | `false` | `no_map` |
| 进程启动/保存失败 | 200 | `false` | `process_error` |
| 请求体字段错误 | 422 | `false` | `validation_error` |
| token 缺失或错误 | 401 | `false` | `unauthorized` |
| 不支持的系统 mode | 404 | `false` | `not_found` |

默认配置 `require_token=false`。启用 token 后：

- HTTP：`Authorization: Bearer <token>` 或 `X-API-Token: <token>`
- WebSocket：`ws://<robot_ip>:8000/ws/status?token=<token>`
- WebSocket token 错误时直接关闭，关闭码为 `1008`

## 页面一：连接和状态

### GET `/api/status`

用途：轻量连接检查和基础状态展示。

请求体：无。

返回 `data`：

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

说明：

- `online=true` 只表示 bridge 进程可响应。
- `nav2_status` 可能存在于返回中，但本阶段 APP 不使用。
- `data.timestamp` 是状态生成时间；外层 `ApiResponse.timestamp` 是响应生成时间。

### GET `/api/debug/status`

用途：底盘页和建图页的详细状态源。

请求体：无。

返回 `data`：

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
    bringup: boolean;
    rplidar_node: boolean;
    imu: boolean;
    tf: boolean;

    // 后端可能返回以下导航相关字段，APP 本阶段忽略。
    bt_navigator?: boolean;
    controller_server?: boolean;
    planner_server?: boolean;
    amcl?: boolean;
    map_server?: boolean;
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

APP 判断底盘是否可调试，建议使用：

- `topics["/odom"] === true`
- `topics["/scan"] === true`
- `topics["/imu/data"] === true`
- `nodes.tf === true`
- `last_odom_age_sec`、`last_scan_age_sec`、`last_imu_age_sec` 都不为 `null`
  且小于 APP 新鲜度阈值，例如 3 秒

### GET `/api/debug/system/status`

用途：查看 bridge 管理的 `bringup` 和 `mapping` 进程状态、PID 和日志尾部。

请求体：无。

返回 `data`：

```ts
type SystemStatus = {
  bringup: ProcessStatus;
  mapping: ProcessStatus;
};

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

字段说明：

- `managed_by_bridge=true`：该进程由当前 bridge 实例启动，可通过停止接口停止。
- `managed_by_bridge=false`：bridge 没有该进程记录，停止接口不会查找或强杀
  SSH 手动启动的 ROS 进程。
- `running=false` 且 `returncode !== null`：进程已经退出，APP 应展示
  `log_tail`。
- `log_tail` 最多读取日志文件末尾约 8 KiB。

示例：

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

## 页面二：底盘低速控制

### POST `/api/debug/system/start/bringup`

用途：启动底盘基础 bringup，包括底盘、雷达、IMU、TF 等底层链路。

请求体：无。

成功响应：

```json
{
  "ok": true,
  "message": "bringup started pid=12345",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

重复启动时仍返回 `ok=true`：

```json
{
  "ok": true,
  "message": "bringup already running",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

### POST `/api/debug/system/stop/bringup`

用途：停止由 bridge 启动的 bringup。

请求体：无。

注意：

- 停止 `bringup` 前，APP 应先停止 `mapping`。
- 服务端不会自动连带停止 `mapping`。
- 若 `message` 为 `bringup was not started by bridge`，说明 bridge 没有该进程
  记录，APP 应提示用户检查是否由 SSH/终端手动启动。

### POST `/api/cmd_vel`

用途：底盘低速点动控制。

请求体：

```ts
type VelocityCommand = {
  linear_x?: number;    // 默认 0.0，单位 m/s
  angular_z?: number;   // 默认 0.0，单位 rad/s
  duration_ms?: number; // 默认 300，范围 50..3000
};
```

示例：

```json
{ "linear_x": 0.03, "angular_z": 0.0, "duration_ms": 300 }
```

服务端规则：

- `linear_x` 会按配置限幅，默认最大绝对值 `0.30 m/s`；后端安全上限为
  `0.35 m/s`，底盘控制器还会按自身配置做最终限幅。
- `angular_z` 会按配置限幅，默认最大绝对值 `0.55 rad/s`；后端安全上限为
  `0.55 rad/s`，底盘控制器还会按自身配置做最终限幅。
- `duration_ms` 越界返回 HTTP `422`，不会自动限幅。
- 每次请求到期后，bridge 会自动发布一次零速度。

推荐 APP 按住按钮时每 200 到 300 ms 重发一次，松手、离开页面、网络异常时调用
急停接口。

### POST `/api/debug/chassis/test`

用途：底盘调试页可选接口，行为等同 `/api/cmd_vel`，多一个 UI 用的 `mode` 字段。

请求体：

```ts
type ChassisTestRequest = VelocityCommand & {
  mode: string;
};
```

示例：

```json
{
  "mode": "forward",
  "linear_x": 0.03,
  "angular_z": 0.0,
  "duration_ms": 300
}
```

`mode` 只用于响应文案，不参与运动计算。

### POST `/api/stop`

用途：总急停。底盘页建议优先使用此接口。

请求体：无。

行为：

- 立即发布零速度。
- 同时向任务话题发布停止文本，方便上层任务停止。

### POST `/api/debug/chassis/stop`

用途：仅底盘急停。

请求体：无。

行为：立即发布零速度，不发布任务停止文本。

## 页面三：建图调试

建图页必须先确认底盘就绪，再启动 mapping。`mapping.launch.py` 只启动
SLAM Toolbox，不启动底盘、雷达或 IMU。

### GET `/api/debug/mapping/status`

用途：建图页主状态接口。

请求体：无。

返回 `data`：

```ts
type MappingStatus = {
  mapping_status: "running" | "not_running";
  bringup_ready: boolean;
  map_available: boolean;
  recommended_next_action:
    | "start_bringup"
    | "start_mapping"
    | "wait_for_map"
    | "continue_mapping_or_save";
  last_map_age_sec: number | null;
  map_meta: MapMeta | null;
  process: ProcessStatus | null;
};
```

字段说明：

- `mapping_status`：根据 SLAM Toolbox 节点是否存在判断。
- `bringup_ready`：只检查 `/odom`、`/scan`、`/imu/data` 和 `/tf` 是否存在；
  它不代表数据一定新鲜，APP 仍应参考 `/api/debug/status` 中的 age 字段。
- `map_available=true`：bridge 已收到当前 SLAM Toolbox 发布的 `/map`。
- 若 `nodes.map_server=true`，bridge 不接受 `/map`，APP 应提示用户先停止
  外部启动的导航或静态地图服务。
- `last_map_age_sec=null`：还没有收到当前建图地图，或 SLAM Toolbox 已停止。
- `map_meta=null`：同上。
- `process`：bridge 管理的 mapping 进程状态。

推荐 UI 逻辑：

| 条件 | APP 显示/动作 |
|---|---|
| `recommended_next_action="start_bringup"` | 提示先启动底盘 |
| `recommended_next_action="start_mapping"` | 允许点击开始建图 |
| `recommended_next_action="wait_for_map"` | 显示“建图启动中，等待地图数据” |
| `recommended_next_action="continue_mapping_or_save"` | 显示实时地图，允许保存 |

### POST `/api/debug/system/start/mapping`

用途：启动建图。

请求体：无。

行为：

- 启动前清空 bridge 中缓存的旧地图。
- 调用 `scripts/run_on_jetson.sh mapping`。
- 长运行日志写入 `/home/nvidia/ros2_DL/logs/mobile_bridge/mapping.log`。

成功响应示例：

```json
{
  "ok": true,
  "message": "mapping started pid=12345",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

重复启动时仍返回 `ok=true`：

```json
{
  "ok": true,
  "message": "mapping already running",
  "data": null,
  "error": null,
  "timestamp": 1710000000.0
}
```

兼容入口：`POST /api/debug/mapping/start`，新 APP 建议使用
`/api/debug/system/start/mapping`。

### GET `/api/debug/mapping/map_snapshot?downsample=1`

用途：HTTP 拉取当前建图 PNG 快照。

请求体：无。

Query：

- `downsample`：整数 `1..16`，默认 `1`。HTTP 越界返回 `422`。

成功返回 `data`：

```ts
type MapSnapshot = {
  map_meta: MapMeta;
  png_base64: string;
};

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
```

APP 显示图片：

```ts
const uri = `data:image/png;base64,${snapshot.png_base64}`;
```

无当前 SLAM 地图时返回：

```json
{
  "ok": false,
  "message": "no map has been received",
  "data": null,
  "error": "no_map",
  "timestamp": 1710000000.0
}
```

`no_map` 的常见原因：

- 刚启动建图，SLAM Toolbox 还没有发布 `/map`。
- 底盘、雷达、TF 未就绪。
- 建图进程已停止。
- 只有旧的导航静态地图存在，但没有当前 SLAM Toolbox 地图。
- SLAM Toolbox 与 `map_server` 同时运行，bridge 为避免地图来源混淆而拒绝
  返回地图。

### WS `/ws/map?downsample=1`

用途：实时推送当前建图 PNG 快照，默认约 1 Hz。

Query：

- `downsample`：无法解析时回退为 `1`，超出范围会截断到 `1..16`。

数据帧同 `ApiResponse<MapSnapshot>`。没有当前地图时，连接不会关闭，会持续推送
`ok=false,error=no_map`。

### POST `/api/debug/mapping/save`

用途：保存当前 SLAM Toolbox 地图。

请求体：

```ts
type MappingSaveRequest = {
  map_name?: string; // 默认 my_map，只允许 A-Z a-z 0-9 _ -
};
```

示例：

```json
{ "map_name": "my_map" }
```

调用条件：

- `GET /api/debug/mapping/status` 中 `map_available=true`。
- 建议保存按钮进入 loading，避免重复点击。

无当前 SLAM 地图时不会调用 `map_saver_cli`，直接返回：

```json
{
  "ok": false,
  "message": "no current SLAM map has been received",
  "data": null,
  "error": "no_map",
  "timestamp": 1710000000.0
}
```

成功返回 `data`：

```ts
type SavedMap = {
  yaml_path: string;
  pgm_path: string;
  output: string;
};
```

示例：

```json
{
  "yaml_path": "/home/nvidia/ros2_DL/maps/my_map.yaml",
  "pgm_path": "/home/nvidia/ros2_DL/maps/my_map.pgm",
  "output": "..."
}
```

保存过程最多等待约 60 秒。

### POST `/api/debug/system/stop/mapping`

用途：停止由 bridge 启动的建图进程。

请求体：无。

行为：

- 停止 mapping 进程。
- 清空 bridge 中缓存的建图地图，APP 后续预览会得到 `no_map`。

兼容入口：`POST /api/debug/mapping/stop`，新 APP 建议使用
`/api/debug/system/stop/mapping`。

## 实时状态

### WS `/ws/status`

用途：实时推送 `RobotStatus`，默认约 2 Hz。

数据帧类型：`ApiResponse<RobotStatus>`。

WebSocket 断开或服务端处理异常时，bridge 会发布一次零速度。APP 仍应在以下场景
主动调用 `/api/stop`：

- 用户松开方向按钮
- APP 进入后台
- 离开底盘控制页
- 网络断开或重连
- WebSocket 收到异常或长时间无数据

建议自动重连使用有限退避，例如 1、2、5 秒，并在重连期间禁用方向按钮。

## 推荐调用流程

### 底盘页

1. `GET /api/status`，确认 bridge 可连接。
2. `GET /api/debug/system/status`，显示 bringup 进程状态。
3. 用户点击启动底盘：`POST /api/debug/system/start/bringup`。
4. 轮询 `GET /api/debug/status` 或订阅 `WS /ws/status`。
5. 满足底盘就绪条件后解锁方向按钮。
6. 方向按钮调用 `POST /api/cmd_vel`。
7. 松手、离开页面、异常时调用 `POST /api/stop`。

### 建图页

1. `GET /api/debug/mapping/status`。
2. 若 `recommended_next_action="start_bringup"`，引导用户先启动底盘。
3. 用户点击开始建图：`POST /api/debug/system/start/mapping`。
4. 使用 `WS /ws/map?downsample=1` 或
   `GET /api/debug/mapping/map_snapshot?downsample=1` 显示地图。
5. `map_available=true` 后允许保存：
   `POST /api/debug/mapping/save`。
6. 保存完成后停止建图：`POST /api/debug/system/stop/mapping`。

## APP 最小接口清单

| 页面 | 方法 | 路径 | 请求体 |
|---|---|---|---|
| 连接 | GET | `/api/status` | 无 |
| 状态 | GET | `/api/debug/status` | 无 |
| 状态 | GET | `/api/debug/system/status` | 无 |
| 底盘 | POST | `/api/debug/system/start/bringup` | 无 |
| 底盘 | POST | `/api/debug/system/stop/bringup` | 无 |
| 底盘 | POST | `/api/cmd_vel` | `VelocityCommand` |
| 底盘 | POST | `/api/stop` | 无 |
| 底盘 | POST | `/api/debug/chassis/stop` | 无 |
| 建图 | GET | `/api/debug/mapping/status` | 无 |
| 建图 | POST | `/api/debug/system/start/mapping` | 无 |
| 建图 | GET | `/api/debug/mapping/map_snapshot?downsample=1` | 无 |
| 建图 | POST | `/api/debug/mapping/save` | `MappingSaveRequest` |
| 建图 | POST | `/api/debug/system/stop/mapping` | 无 |
| 实时 | WS | `/ws/status` | 无 |
| 实时 | WS | `/ws/map?downsample=1` | 无 |

兼容但不推荐新 APP 使用：

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/debug/mapping/start` | 等同启动 mapping，会清空旧地图缓存 |
| POST | `/api/debug/mapping/stop` | 等同停止 mapping，会清空地图缓存 |
| POST | `/api/debug/chassis/test` | 等同 `/api/cmd_vel`，多 `mode` 字段 |

明确不支持给 APP 使用：

| 方法 | 路径 | 结果 |
|---|---|---|
| POST | `/api/debug/system/start/navigation` | HTTP 404, `error=not_found` |
| POST | `/api/debug/system/stop/navigation` | HTTP 404, `error=not_found` |

## 常见问题

- 手机连不上：先在 Jetson 本机执行
  `curl http://localhost:8000/api/status`，确认 bridge 是否启动。
- 底盘未就绪：检查 `/api/debug/status` 中 `/odom`、`/scan`、`/imu/data`、
  `nodes.tf` 和对应 age 字段。
- 启动失败：查看 `/api/debug/system/status` 中对应进程的 `returncode`、
  `log_path` 和 `log_tail`。
- APP 看到旧地图：新接口已在建图启动/停止时清空缓存，并且只接受
  SLAM Toolbox 的当前地图；若仍出现旧图，检查 APP 是否使用了本地图片缓存。
- 建图一直 `no_map` 且 `nodes.map_server=true`：先停止外部导航或静态地图
  服务，不能与 APP 建图调试同时运行。
- `no_map`：建图刚启动、SLAM 尚未发布地图、底盘数据缺失，或建图已停止。
- 保存失败：只有 `map_available=true` 后才能保存；保存时按钮需要 loading。
- 停止无效：`managed_by_bridge=false` 表示该进程不是本次 bridge 启动，应由
  原 SSH/终端会话管理。
- HTTP `422`：检查 `duration_ms`、`downsample`、`map_name` 和请求体类型。
