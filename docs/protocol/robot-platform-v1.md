# Robot Platform Protocol v1 — Heartbeat Pull

Jetson 是唯一发起公网连接的一方。公网 Bridge 不访问 Jetson；不需要公网 IP、端口映射、Tailscale、MQTT、Kafka、Redis 或 WebSocket。`/api/platform/v1/*` 保留为本机或可信局域网调试接口，不是公网主链路。

机器人从受保护环境文件读取 `YLHB_CLOUD_ENABLED=false`、`YLHB_CLOUD_BASE_URL`（启用时必须 HTTPS）、`YLHB_CLOUD_ROBOT_TOKEN`、`YLHB_CLOUD_CA_FILE`、`YLHB_CLOUD_REQUEST_TIMEOUT_SEC=10`、`YLHB_CLOUD_IDLE_HEARTBEAT_SEC=3`、`YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC=1`、`YLHB_CLOUD_MAX_BACKOFF_SEC=30`、`YLHB_SOFTWARE_VERSION`。Token 不进入 YAML、日志或仓库，TLS 始终使用系统证书（可额外指定 CA），没有关闭校验的配置。

机器人轮询 `POST /robot-api/v1/heartbeat`，上传协议版本、机器人/启动身份、软件版本、巡逻状态、活动 execution/deployment、接收命令号、事件序号、map/odom 位姿及 odom/scan/imu/Nav2/模式/错误健康信息。响应一次最多一条命令，包含 `serverTime`、`nextHeartbeatSec`、`acceptedEventSequence` 和 `command`。

命令白名单为 `START`、`PAUSE`、`RESUME`、`TAKEOVER`、`CANCEL`。收到命令先事务持久化为 `RECEIVED`，再 ACK，成功后 `ACKED`，由 ROS timer 从队列下发；真实巡逻事件才将其标为 `APPLIED` 或 `REJECTED`。`commandId` 和 `requestId` 都持久化去重，重启不重复执行已应用命令。禁止 `cmd_vel`、Shell、ROS 节点名、路径和系统命令。

`START` 缺少本地部署时顺序下载 manifest、route、YAML、PGM 到 staging；校验 robot/deployment/revision、传输哈希、地图哈希、YAML image、文件大小和路线地图绑定，随后原子安装。部署保留 manifest 的原始 YAML/PGM 文件名，`routePath`、`mapYamlPath`、`mapPgmPath` 为精确最终路径；不会覆盖 `maps/my_map.*` 或 `maps/route_patrol_*.json`，本地 `auto` 路线选择保持不变。

事件仅由 `DeploymentStore.append_event` 分配单调 `sequence`，字段包含 `command_id`、`execution_id`、`deployment_id`、`request_id`、`robot_id`、`boot_id`、`occurred_at`。每批最多 100 条；服务器只确认连续序号。确认游标回退时机器人从服务器确认位置补传，事件不删除。网络错误按 1、2、4、8、15、30 秒加 0–20% 抖动（并遵守 `Retry-After`）；巡检继续、本地急停仍有效、网络中断不接收新远程命令。
