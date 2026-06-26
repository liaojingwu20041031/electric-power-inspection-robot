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

`system_supervisor_node` 是 mobile bridge 的唯一本机进程控制来源。UI 只展示
`/inspection_ai/system_status` 中的 `mobile_bridge`、`mobile_bridge_http`、
`mobile_bridge_url` 和 `jetson_ip`。

## 启动

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ylhb_llm llm.launch.py \
  enable_display_ui:=true \
  enable_system_supervisor:=true \
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
