# Jetson 云平台连接运维

> Robot Platform Protocol v1 见 [协议文档](protocol/robot-platform-v1.md)。本文只覆盖 Jetson 部署、连接开关、观察、备份和回退，不授权任何实机运动。

## 1. 架构边界

网桥核心进程、本地 APP 服务和云平台连接是三个概念。核心进程同时承载 ROS、FastAPI
和 Cloud Client；本地 APP 开关只门控局域网手机接口；云平台开关只控制 Jetson 主动
发起的 HTTPS 连接。任一业务开关都不会停止核心进程。

Jetson 是公网连接的唯一发起方：

```text
Jetson mobile bridge -> HTTPS https://example.com/robot-api/v1
Robot Bridge          -X-> Jetson
```

Jetson 不需要公网 IP、端口映射或开放 8000。`/api/platform/v1` 只用于本机/可信局域网调试；配置 Cloud Link 后入站写控制默认关闭。

## 2. `platform.env` 变量表

文件：`/home/nvidia/.config/ylhb/platform.env`，权限 `0600`。Token 不写 YAML、launch、仓库或日志。

| 变量 | 必填 | 示例 | 用途 | 敏感 | 修改后重启 |
| --- | --- | --- | --- | --- | --- |
| `YLHB_CLOUD_ENABLED` | 是 | `true` | systemd 启动后默认是否连接 | 否 | 是；UI override 可即时改变 |
| `YLHB_LOCAL_APP_ENABLED` | 否 | `true` | 本地 APP 服务默认是否开放 | 否 | 是；UI override 可即时改变 |
| `YLHB_CLOUD_BASE_URL` | 是 | `https://example.com` | 公网 Robot Bridge 根 URL；必须 HTTPS | 否 | 是 |
| `YLHB_CLOUD_ROBOT_TOKEN` | 是 | `token-placeholder` | robot-001 设备凭据 | 是 | 是 |
| `YLHB_CLOUD_CA_FILE` | 否 | `/etc/ssl/certs/private-ca.pem` | 私有 CA；公有 CA 留空 | 否 | 是 |
| `YLHB_CLOUD_REQUEST_TIMEOUT_SEC` | 否 | `10` | 单次 HTTP 超时 | 否 | 是 |
| `YLHB_CLOUD_IDLE_HEARTBEAT_SEC` | 否 | `3` | 空闲 fallback 心跳间隔 | 否 | 是 |
| `YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC` | 否 | `1` | 活动 fallback 心跳间隔 | 否 | 是 |
| `YLHB_CLOUD_MAX_BACKOFF_SEC` | 否 | `30` | 最大退避 | 否 | 是 |
| `YLHB_SOFTWARE_VERSION` | 否 | `79df249` | heartbeat 软件版本 | 否 | 是 |
| `YLHB_ROBOT_ID` | 是 | `robot-001` | 机器人身份 | 否 | 是 |
| `YLHB_PLATFORM_STORAGE_DIR` | 否 | `/home/nvidia/.local/share/ylhb/platform` | deployment、command、event SQLite 根目录 | 否 | 是 |
| `YLHB_PLATFORM_API_TOKEN` | 本地 API 启用时 | `token-placeholder` | `/api/platform/v1` 本地调试鉴权 | 是 | 是 |
| `YLHB_ALLOW_INBOUND_PLATFORM_CONTROL` | 否 | `false` | 是否允许本地入站写控制；生产保持 false | 否 | 是 |

受保护 env 示例仅使用占位值：

```dotenv
YLHB_CLOUD_ENABLED=true
YLHB_LOCAL_APP_ENABLED=true
YLHB_CLOUD_BASE_URL=https://example.com
YLHB_CLOUD_ROBOT_TOKEN=token-placeholder
YLHB_CLOUD_REQUEST_TIMEOUT_SEC=10
YLHB_CLOUD_IDLE_HEARTBEAT_SEC=3
YLHB_CLOUD_ACTIVE_HEARTBEAT_SEC=1
YLHB_CLOUD_MAX_BACKOFF_SEC=30
YLHB_SOFTWARE_VERSION=79df249
YLHB_ROBOT_ID=robot-001
YLHB_PLATFORM_STORAGE_DIR=/home/nvidia/.local/share/ylhb/platform
YLHB_ALLOW_INBOUND_PLATFORM_CONTROL=false
```

不要提交该文件。公有 CA 场景不配置 `YLHB_CLOUD_CA_FILE`；禁止关闭 TLS 校验。

## 3. 安装与启动 systemd

```bash
cd /home/nvidia/ros2_DL
./scripts/install_platform_bridge_service.sh
sudo systemctl status ylhb-mobile-bridge --no-pager
sudo systemctl enable ylhb-mobile-bridge
sudo systemctl restart ylhb-mobile-bridge
```

生产 UI/Supervisor 必须传：

```text
mobile_bridge_managed_externally:=true
```

其配置语义是 `mobile_bridge_managed_externally=true`；ROS 2 launch 命令行使用 `:=true` 赋值语法。

仓库的 UI 自启动 wrapper 已传该参数。此时 Supervisor 不再启动、停止或重启 mobile bridge，只观察端口状态，避免双实例。

## 4. 确认只有一个 Mobile Bridge

```bash
systemctl is-active ylhb-mobile-bridge
pgrep -af 'ylhb_mobile_bridge mobile_bridge_server'
ros2 node list --no-daemon | grep mobile_bridge
ss -lntp | grep ':8000'
```

通过标准：

- systemd active；
- 只有一个 `mobile_bridge_server` 进程；
- 只有一个对应 ROS node；
- 8000 只有一个监听者。

若有第二个开发 launch，停止开发终端中的实例；不要 kill systemd 后让 Supervisor 接管生产进程。

## 5. Cloud status Topic

```bash
ros2 topic info /mobile_bridge/cloud_status -v --no-daemon
ros2 topic echo /mobile_bridge/cloud_status --once
```

主要字段：configured、desiredEnabled、connected、state、heartbeatInFlight、consecutiveFailures、serverBaseUrl、pendingEventCount、pendingCommandCount、lastReceivedCommandId、lastUploadedSequence、latestLocalEventSequence、activeExecutionId、activeDeploymentId、lastSuccessAt、lastError、nextHeartbeatSec、nextRetrySec。

`CONNECTED` 时下一轮 heartbeat 会保持连接状态，仅 `heartbeatInFlight=true`；`nextHeartbeatSec` 是正常心跳间隔。只有网络/HTTP 通信失败才进入 `BACKOFF`，此时 `nextRetrySec` 是退避重试时间。UI 主卡以 `connected=true` 为优先级，不把短暂 raw `CONNECTING` 显示为断线；raw state 保留在诊断区。

Topic 使用可靠、Transient Local QoS，UI 晚加入也能获得最近状态。

## 6. SetBool 连接开关

只控制 Cloud Link，不控制巡检：

```bash
ros2 service type /mobile_bridge/set_cloud_enabled
ros2 service call /mobile_bridge/set_cloud_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /mobile_bridge/set_cloud_enabled std_srvs/srv/SetBool '{data: true}'
```

- false：持久化 override，进入 DISABLED，停止 heartbeat/领命令/上传事件。
- 关闭连接不会停止当前巡检，不会发布 pause/cancel/急停。
- true：进入 CONNECTING，成功后 CONNECTED，并从服务器连续游标补传积压事件。
- 配置缺失时开启返回失败，state 保持 UNCONFIGURED。

## 7. UI 开关与状态

本体 UI“连接与服务”页有两个独立 Switch：

- 本地 APP Switch 调用 `/mobile_bridge/set_local_app_enabled`；关闭不会改变
  `cloudStatus.desiredEnabled`，Cloud Client 继续 heartbeat 和事件补传。
- 云平台 Switch 调用 `/mobile_bridge/set_cloud_enabled`；关闭不会改变
  `localAppStatus.enabled`，手机仍可访问本地 API/WS。
- 两个 Switch 都不会调用 systemd、Shell 或核心进程启停命令。
- Service 成功只表示请求被接受；Switch 保持用户目标，直到状态 Topic 确认，5 秒无确认会提示而不伪造成功。

云平台卡显示：

| 状态 | 含义 | 操作 |
| --- | --- | --- |
| `UNCONFIGURED` | URL/Token/CA 配置无效 | 修复 env，重启服务 |
| `DISABLED` | 用户或 env 关闭连接 | 可手动开启 |
| `CONNECTING` | 正在建立/恢复连接 | 等待或看日志 |
| `CONNECTED` | 最近心跳成功 | 正常 |
| `BACKOFF` | 网络/HTTP 失败，指数退避 | 看 lastError、TLS、DNS、服务状态 |

活动 execution 中关闭云开关必须确认，明确说明本地 APP 和当前巡检不会停止，积压事件
在重连后补传。关闭本地 APP 使用另一份确认文案，明确说明云平台连接和当前巡检不受影响。

本地 APP override 保存在 `bridge_settings.local_app_enabled_override`；云平台 override
仍保存在 `cloud_state.cloud_enabled_override`，两者不共享字段。

## 8. 入站控制默认关闭

生产保持：

```text
YLHB_ALLOW_INBOUND_PLATFORM_CONTROL=false
```

配置 Cloud Link 后，Jetson `/api/platform/v1` 的 deploy/start/pause/resume/takeover/cancel 写接口返回 `409 INBOUND_CONTROL_DISABLED`。暂停 Cloud Link 不会自动开放入站控制。

## 9. 日志与脱敏检查

```bash
sudo journalctl -u ylhb-mobile-bridge -n 200 --no-pager
sudo journalctl -u ylhb-mobile-bridge -f
```

Cloud Client 只应记录 HTTP 状态、异常类型和脱敏 URL。验证 Token 未打印时，不输出 Token 本身：

```bash
sudo journalctl -u ylhb-mobile-bridge --since today --no-pager \
  | grep -E 'Authorization:|Bearer |YLHB_CLOUD_ROBOT_TOKEN|token=' && echo '发现疑似敏感日志' || echo '未发现常见敏感模式'
```

还应人工确认 serverBaseUrl 不含 userinfo、query 或 fragment。

## 10. 备份 Robot 平台 SQLite

默认目录：

```text
~/.local/share/ylhb/platform/platform.db
~/.local/share/ylhb/platform/deployments/
```

在线备份：

```bash
mkdir -p /home/nvidia/backups/ylhb-platform
sqlite3 /home/nvidia/.local/share/ylhb/platform/platform.db \
  ".backup '/home/nvidia/backups/ylhb-platform/platform.db.backup'"
tar -C /home/nvidia/.local/share/ylhb/platform -czf \
  /home/nvidia/backups/ylhb-platform/deployments.backup.tgz deployments
```

备份文件必须限制权限，不上传公共位置。恢复前停止 systemd，保留故障副本，再同时恢复数据库和 deployment 文件。

## 11. 无运动连接验收

```bash
ros2 topic echo /mobile_bridge/cloud_status --once
sudo systemctl status ylhb-mobile-bridge --no-pager
```

通过标准：CONNECTED、pendingCommandCount=0、服务器 robot online=true、日志无 Token、只有一个 bridge。此步骤不创建 START，也不启动巡逻。

## 12. 回退到本地 auto 巡逻

回退只改变云连接来源，不代表授权机器人运动：

1. 通过 SetBool 关闭 Cloud Link，确认 DISABLED。
2. 保留 platform SQLite 与 pending event，不删除，便于恢复后补传。
3. 如需停止 systemd，仅在确认本地调试模式会单独启动 mobile bridge 时执行。
4. 本地路线使用仓库既有 `auto` 解析和正式路线文件，不使用云 deployment 路径。
5. 由现场用户确认急停、清场、传感器、TF、Nav2、地图和路线后，执行既有本地巡逻入口。

现场命令参考（仅用户授权后执行）：

```bash
./scripts/run_on_jetson.sh inspection
```

恢复云连接时重新启用 systemd/SetBool，等待 CONNECTED 和事件补传完成；不要在本地巡逻仍活动时领取新的 START。

## 13. 常见故障

| 症状 | 检查 |
| --- | --- |
| UNCONFIGURED | HTTPS URL、robot token、CA 文件、robotId |
| BACKOFF HTTP 401 | token 与服务器 robotId 映射；按轮换流程更新 |
| BACKOFF TLS | 域名、证书链、时间、CA 文件；禁止关闭校验 |
| CONNECTED 但 server offline | 对比服务器 lastSeen、代理路径、时间同步 |
| pendingEventCount 不下降 | accepted sequence、事件缺口、events batch 409 |
| command 长期 ACKED | ROS publish/Patrol event、system supervisor、commandId 绑定 |
| 双实例 | systemd + 开发 launch；确保 external managed=true |
| 入站接口 409 | 生产预期；不要为方便而打开入站控制 |
