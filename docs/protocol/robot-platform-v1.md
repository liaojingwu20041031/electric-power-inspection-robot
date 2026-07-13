# Robot Platform Protocol v1

平台与机器人只通过 HTTP 交换本协议。所有正式机器人接口前缀为 `/api/platform/v1`，均要求 `Authorization: Bearer <token>`；适配器接口前缀为 `/bridge/v1`，同样要求 Bearer Token。Token 只来自环境变量，不写入 YAML、日志或仓库。

标识符：`robotId` 是机器人配置键；`routeRevisionId` 是不可变路线版本；`deploymentId` 将路线和地图精确绑定；`taskId` 属于平台业务任务；`executionId` 是一次执行；`executorRouteId` 是路线 JSON 中的路线 ID；`requestId` 是控制请求幂等键；`bootId` 是机器人桥接进程实例；`sequence` 是 SQLite 持久事件序号。

## API 与规则

- `GET /health` 返回 `robotId`、`bootId`、`state`、`mapPose`、`odomPose`；没有可靠 AMCL/TF map 位姿时 `mapPose=null`，绝不把 odom 标为 map。
- `PUT /deployments/{deploymentId}` 使用 multipart：`manifest`、`route`、`yaml`、`pgm`。`manifest` 至少有 `schemaVersion`、`robotId`、`routeRevisionId`、`routeContentSha256`、`mapAssetId`、`mapImageSha256`、`yamlName`、`pgmName`。拒绝绝对文件名和路径穿越；相同 deploymentId 同哈希返回幂等结果，不同哈希返回 `409 DEPLOYMENT_CONFLICT`。
- `POST /executions/{executionId}/start` JSON 为 `deploymentId`、`executorRouteId`、`profile`、`requestId`，返回 `202 {"accepted":true,"state":"STARTING","executionId":"..."}`，不等待 Nav2。平台任务只允许 deploymentId，禁止 `auto` 和机器人路径；本地人工巡逻仍允许 `route_file_path=auto`。
- `POST /executions/{executionId}/{pause|resume|takeover|cancel}` JSON 必须有 `requestId`，应用层幂等。`GET /executions/{executionId}` 查状态；`GET /events?afterSequence=0&limit=100` 的下界排他、按 sequence 升序、事件永不因重启归零或删除。

路线哈希是递归排序对象键、保持数组顺序、UTF-8 紧凑 JSON 的 SHA-256；PGM 哈希是原始字节 SHA-256。部署先校验文件名、哈希、YAML `image`、路线和地图绑定，再原子安装与写 SQLite。连接超时默认 3 秒、读取超时默认 10 秒；GET/PUT 可有限重试，POST 仅携带 requestId 时可重试。

事件保留原字段，并增加 `schema_version`、`robot_id`、`boot_id`、`execution_id`、`deployment_id`、`request_id`、`sequence`、`route_id`、`route_path`、`state`、`occurred_at`。检查点可带 `target_id`、`target_name`、`target_index`、`target_count`、`completed_target_count`、`progress`。事件包括 `command_accepted`、`route_started`、`target_navigation_started`、`target_reached`、`target_task_started`、`target_task_finished`、`route_paused`、`route_resumed`、`return_home_started`、`route_finished`、`route_failed`、`route_canceled`。

状态映射：`idle→CREATED`、`starting→DISPATCHING`、`running→RUNNING`、`paused→PAUSED`、`manual_takeover→MANUAL_TAKEOVER`、`returning_home/waiting_loop→RUNNING`、`succeeded→COMPLETED`、`failed→FAILED`、`canceled→CANCELLED`。统一错误为 `{code,message,requestId,details?}`；代码包括 `AUTH_FAILED`、`ROBOT_UNREACHABLE`、`PLATFORM_UNREACHABLE`、`ROBOT_BUSY`、`DEPLOYMENT_NOT_FOUND`、`DEPLOYMENT_CONFLICT`、`EXECUTION_NOT_FOUND`、`EXECUTION_CONFLICT`、`ROUTE_HASH_MISMATCH`、`MAP_HASH_MISMATCH`、`INVALID_ROUTE`、`INVALID_MAP`、`INVALID_REQUEST`、`TIMEOUT`、`INTERNAL_ERROR`。

示例：

```bash
curl -H 'Authorization: Bearer token-placeholder' http://127.0.0.1:8000/api/platform/v1/health
curl -X POST -H 'Authorization: Bearer token-placeholder' -H 'Content-Type: application/json' -d '{"robotId":"robot-001","deploymentId":"deploy-1","executorRouteId":"route-1","requestId":"req-1"}' http://127.0.0.1:8001/bridge/v1/executions/exe-1/start
```
