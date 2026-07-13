# ylhb_llm 本体 QML 操控台

`inspection_display_ui_node` 保持原 ROS 2 console script 名称，入口改为 PyQt5
`QGuiApplication + QQmlApplicationEngine`。

## 架构边界

| 模块 | 职责 |
|---|---|
| `ui_ros_bridge.py` | ROS 2 话题订阅/发布、`/cmd_vel`、语音服务客户端 |
| `ui_backend.py` | QObject 属性、QML slot、安全锁、日志和命令封装 |
| `ui_models.py` | 轻量 UI 状态和最多 200 条事件缓存 |
| `qml/` | 亮色暖色 QML 页面和组件 |

本体 UI 不直接调用 mobile bridge HTTP API，也不使用其内部 `ProcessManager`。
启动、停止和重启统一经过：

```text
/inspection_ai/system_command
  -> system_supervisor_node
  -> ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

连接控制分为三层，不能混用：

1. 网桥核心进程同时承载 ROS、FastAPI 和 Cloud Client；生产由 systemd 常驻管理。
2. 本地 APP 服务使用 `/mobile_bridge/local_app_status` 和
   `/mobile_bridge/set_local_app_enabled`，只控制手机 API/WS 是否开放。
3. 云平台连接使用原有 `/mobile_bridge/cloud_status` 和
   `/mobile_bridge/set_cloud_enabled`，只控制主动 HTTPS Cloud Link。

生产模式启动 UI 时传 `mobile_bridge_managed_externally:=true`；Supervisor 只检查本机
端口，不创建或终止第二个进程。页面显示两个独立 Switch：关闭本地 APP 不影响云连接
和当前巡检；关闭云连接不影响本地 APP 和当前巡检。只有停止网桥核心进程才会同时中断
两条连接，生产模式不向普通用户显示核心进程启停按钮。

## 启动

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_llm llm.launch.py \
  enable_display_ui:=true \
  enable_system_supervisor:=true \
  mobile_bridge_managed_externally:=true \
  fullscreen:=true
```

保留 `enable_display_ui`、`enable_system_supervisor`、`fullscreen`、`display`
和 `force_local_display` launch 参数。

## 依赖

```bash
sudo apt install python3-pyqt5.qtquick qml-module-qtquick2 \
  qml-module-qtquick-window2 qml-module-qtquick-controls2 \
  qml-module-qtquick-layouts qml-module-qtqml-models2
```

缺少 QtQml 时，节点会打印 QML 路径、错误信息和安装建议后退出。

## 页面和安全策略

页面包括总览、APP 桥接、系统状态、运动控制、建图和日志。运动控制默认锁定；
解锁后单次按钮只发布短时低速 `/cmd_vel`，10 秒无操作自动重新锁定。急停始终可用，
会发布零速度和 `emergency_stop` 系统命令。

QML 不包含视频、3D、地图实时预览、粒子、Blur、ShaderEffect 或高频动画。
ROS spin 使用低频定时器，mobile bridge HTTP 健康检查由 supervisor 的 1 Hz
status timer 驱动。

## 巡逻页现场调试

路线预览图缓存：

```bash
file /tmp/ylhb_route_preview_*.png
ls -lh /tmp/ylhb_route_preview_*.png
python3 - <<'PY'
from pathlib import Path
from PIL import Image

for path in Path('/tmp').glob('ylhb_route_preview_*.png'):
    with Image.open(path) as image:
        image.verify()
    print(f'{path}: ok')
PY
rm -f /tmp/ylhb_route_preview_*.png
```

巡逻状态和命令链路：

```bash
ros2 topic echo /patrol/status
ros2 topic echo /patrol/event
ros2 topic info /patrol/command -v
ros2 topic echo /inspection_ai/system_status
```
# UI 自启动

用户态自启动使用 wrapper，不直接裸跑主 launch：

```bash
./scripts/install_ui_autostart.sh
./scripts/start_inspection_ui_autostart.sh
./scripts/uninstall_ui_autostart.sh
```

wrapper 会设置 `WS_DIR` 并调用：

```bash
scripts/run_on_jetson.sh inspection fullscreen:=true
```

日志写入 `runs/ui_autostart/inspection_ui_YYYYmmdd_HHMMSS.log`。DISPLAY 异常时先看日志里的 `DISPLAY`、`XAUTHORITY`，再手动执行 wrapper 复现。

## 连接与服务页

生产模式必须让 systemd 独占 mobile bridge，并传：

```text
mobile_bridge_managed_externally=true
```

页面顶部对称显示“本地 APP 服务”和“云平台连接”两张卡片，并分别维护 pending、
结果提示和确认对话框。中部连接图按两套状态单独着色，网桥核心节点保持独立；摘要区
显示本地 APP、云平台、待上传事件和当前 execution；详细原始字段默认折叠。

本地 APP Switch 调用 `/mobile_bridge/set_local_app_enabled`。关闭后手机 `/api/status`、
`/api/debug/**`、`/ws/status`、`/ws/map` 等接口停用，HTTP 返回 503
`local_app_disabled`；`/api/platform/v1/**` 和 Cloud Client 不受影响。

云平台 Switch 继续调用 `/mobile_bridge/set_cloud_enabled`，显示 `UNCONFIGURED`、
`DISABLED`、`CONNECTING`、`CONNECTED`、`BACKOFF`。两个开关都不执行 shell、不调用
systemd，也不启动第二个 bridge。

关闭连接不会停止当前巡检，只暂停 heartbeat、云端命令领取与事件上传；重新开启后会从服务器连续游标补传事件。活动 execution 中关闭会弹出二次确认。

无运动检查：

```bash
ros2 topic echo /mobile_bridge/cloud_status --once
ros2 topic echo /mobile_bridge/local_app_status --once
ros2 service type /mobile_bridge/set_cloud_enabled
ros2 service type /mobile_bridge/set_local_app_enabled
pgrep -af 'ylhb_mobile_bridge mobile_bridge_server'
```

完整环境变量、systemd、日志脱敏、SQLite 备份和本地 auto 回退见 [Jetson 云平台连接运维](cloud_platform_connection.md)。
