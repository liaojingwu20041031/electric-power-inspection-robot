# Robot Platform Protocol v1 — Heartbeat Pull

Jetson 是唯一发起公网连接的一方。公网 Bridge 不访问 Jetson；不需要公网 IP、端口映射、Tailscale、MQTT、Kafka、Redis 或 WebSocket。`/api/platform/v1/*` 保留为本机或可信局域网调试接口，不是公网主链路。

机器人从受保护环境文件读取 `YLHB_CLOUD_ENABLED=false`、`YLHB_CLOUD_BASE_URL`（启用时必须 HTTPS）、`YLHB_CLOUD_ROBOT_TOKEN`、`YLHB_CLOUD_CA_FILE`、`YLHB_CLOUD_REQUEST_TIMEOUT_SEC=10`、`YLHB_CLOUD_IDLE_HEARTBEAT_SEC=3`、`YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC=1`、`YLHB_CLOUD_MAX_BACKOFF_SEC=30`、`YLHB_SOFTWARE_VERSION`。Token 不进入 YAML、日志或仓库，TLS 始终使用系统证书（可额外指定 CA），没有关闭校验的配置。

机器人轮询 `POST /robot-api/v1/heartbeat`，上传协议版本、机器人/启动身份、软件版本、巡逻状态、活动 execution/deployment、接收命令号、事件序号、map/odom 位姿及 odom/scan/imu/Nav2/模式/错误健康信息。响应一次最多一条命令，包含 `serverTime`、`nextHeartbeatSec`、`acceptedEventSequence` 和 `command`。

命令白名单为 `START`、`PAUSE`、`RESUME`、`TAKEOVER`、`CANCEL`。收到命令先事务持久化为 `RECEIVED`，再 ACK，成功后 `ACKED`；ROS timer 真正 publish 成功后才写 `DISPATCHED`。ACK 只表示已持久接收，真实结果由 route_started/route_paused/route_resumed/manual_takeover/route_canceled、`command_rejected` 或 `command_failed` 写入 `APPLIED`、`REJECTED`、`FAILED`。新 leaseToken 等顶层投递字段不参与业务幂等比较。

`START` 缺少本地部署时顺序下载 manifest、route、YAML、PGM 到 staging；校验 robot/deployment/revision、传输哈希、地图哈希、YAML image、文件大小和路线地图绑定，随后原子安装。部署保留 manifest 的原始 YAML/PGM 文件名，`routePath`、`mapYamlPath`、`mapPgmPath` 为精确最终路径；不会覆盖 `maps/my_map.*` 或 `maps/route_patrol_*.json`，本地 `auto` 路线选择保持不变。

事件仅由 `DeploymentStore.append_event` 分配单调 `sequence`，字段包含 `command_id`、`execution_id`、`deployment_id`、`request_id`、`robot_id`、`boot_id`、`occurred_at`。PAUSE/RESUME/CANCEL/TAKEOVER 使用各自的一次性 command/request ID，路线进度和终态继续绑定 START。每批最多 100 条；服务器只确认连续序号。开启或断线恢复时先 heartbeat 取得服务器游标，再从该位置补传。网络错误按 1、2、4、8、15、30 秒加抖动；巡检继续、本地急停仍有效。

Cloud Client 线程始终存在。配置缺失为 `UNCONFIGURED`，关闭为 `DISABLED`，失败退避为 `BACKOFF`；QML 通过 `/mobile_bridge/cloud_status` 和 `/mobile_bridge/set_cloud_enabled` 控制持久 override，不写 `.env`、不执行 Shell，也不停止当前巡检。配置 Cloud Link 后，旧 `/api/platform/v1` 写接口默认返回 `409 INBOUND_CONTROL_DISABLED`；仅显式设置 `YLHB_ALLOW_INBOUND_PLATFORM_CONTROL=true` 才允许可信局域网调试。
