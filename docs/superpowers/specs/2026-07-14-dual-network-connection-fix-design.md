# 双网络连接修复设计

## 目标

移动端使用“主地址 + 备用地址”的单一连接生命周期；连接、刷新、页面切换和 WebSocket 重连均不得发送急停。机器人端提供稳定机器人身份、单次 Bridge 实例身份和只表示候选值的网卡地址。

## 机器人端

- `/api/status` 保持现有信封和状态字段，新增 `apiVersion=1.1`、稳定 `robotId`、每次进程启动变化的 `bridgeInstanceId`。
- 优先使用已配置的 `robot_id`；为空时用 `/etc/machine-id` 生成稳定 UUID，不写系统配置。
- `network.candidateEndpoints` 返回 `url/interface/type/linkUp`；旧 `appEndpoints` 保留兼容，不再向新客户端声明首项是首选地址。
- `/ws/status` 和 `/ws/map` 断开或异常只清理连接记录，不调用 `stop_motion()`。

## 移动端

- `RobotConnectionController` 独占连接配置、持久化、探测顺序、连接 operation generation、Status/Map WebSocket generation 和自动切换。
- `robotStore` 只接收控制器状态并保留与连接无关的业务 API 动作；完整初始化仅由根布局调用一次。
- HTTP 使用 `AbortController`，区分 timeout、aborted、network_error、HTTP 4xx/5xx 和业务信封错误。只有 timeout/network_error 可触发读路径备用地址探测；写请求不重发。
- 配置固定为主地址、可选备用地址、自动切换和刷新周期；使用 AsyncStorage 保存并一次性迁移旧 `baseUrl`。
- 急停仅由明确按钮触发；仅在两个地址已确认 robotId 相同时并行发送，单地址 1500ms，任一成功即成功。
- 地图流只有 `onopen` 或有效首帧后进入 connected；错误不得显示 connected。

## 验证边界

移动端最多六项连接回归；机器人端三项 API/WebSocket 回归。Jetson 只运行相关测试、受影响包构建、只读 HTTP/端口检查；不改网络、不驱动机器人、不构建 APK。最终 APK 在 Windows PC 按现有原生 Android 工程构建。
