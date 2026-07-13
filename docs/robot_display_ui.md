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

开发模式由 `system_supervisor_node` 管理 mobile bridge。生产模式由 systemd 常驻，
启动 UI 时传 `mobile_bridge_managed_externally:=true`；Supervisor 只检查本机端口，
不会创建或终止第二个进程。UI 另订阅 `/mobile_bridge/cloud_status`，开关只调用
`/mobile_bridge/set_cloud_enabled`，关闭云连接不会停止当前巡检。

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
