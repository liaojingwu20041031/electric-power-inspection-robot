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

屏幕外壳遮挡不对称时可独立调整四边安全区域（范围 0～120 像素）：

```bash
./scripts/run_on_jetson.sh inspection \
  ui_safe_margin_left:=36 \
  ui_safe_margin_right:=36 \
  ui_safe_margin_top:=28 \
  ui_safe_margin_bottom:=36
```

背景仍铺满屏幕，侧栏、顶部状态栏、页面、滚动条、关闭和急停按钮都限制在安全区域内。

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

巡逻页按“任务头部 → 路线地图与任务控制 → 四项关键指标 → 运行详情”组织。宽屏时地图占工作区约三分之二，960×640 时地图与控制自动上下排列；页面保持纵向滚动且关闭横向滚动。路线聚焦、完整地图和重绘入口均位于地图卡片工具栏，普通巡逻状态更新不会重新请求图片，也不会重置用户当前缩放和拖动位置。

启动巡逻和结束巡逻都必须经过页面确认框。启动确认明确提示底盘、雷达、导航和巡逻执行器可能使机器人移动；取消、Esc 或关闭确认框不会发送命令。运行详情默认折叠，启动阶段、就绪项、任务和事件模型仅在对应区域展开时实例化，以减少无效 QML delegate 工作。

路线预览继续使用缓存 PNG、`Image`、`PinchArea` 和 `MouseArea`，没有引入 Canvas、ShaderEffect、模糊或持续动画。宽高和图片 Ready 事件通过 100 ms 单次 Timer 合并为一次适应计算；手动缩放或拖动后关闭自动适应，只有更换图片或点击“适应”才恢复。预览图使用蓝色路线加浅色描边、半透明红色禁行区、带白色外圈的起点/检查点和紧凑图例。

本轮未设置 `QT_QUICK_BACKEND`、`QSG_RENDER_LOOP`，也未修改 launch 图形配置；Jetson 仍使用当前 Qt/GPU 渲染方式。Offscreen 测试只能验证 QML 加载和交互契约，不能证明 NVIDIA GPU MMU fault 已消失。

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

用户确认“关闭操控台”后，UI 先发布零速度和一次 `emergency_stop`，再把本轮 session id
写入 `${XDG_RUNTIME_DIR}/ylhb/inspection-stop-<wrapper-pid>`。完整 inspection 栈退出后，
wrapper 仅在 marker 与本轮 session 匹配时认定为主动关闭并停止重启。Qt/X11 崩溃或节点异常
没有匹配 marker，仍按原来的 60 秒最多 3 次策略恢复完整栈。systemd 管理的 Mobile Bridge
不随 UI 主动关闭而停止。

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

完整环境变量、systemd、日志脱敏、SQLite 备份和本地 auto 回退见 [Jetson 云平台连接运维](云平台连接运维.md)。

## 显示会话与自动恢复

UI 是完整 inspection 栈的生命周期锚点：UI 退出会有意关闭 Agent、语音和 Supervisor，避免后台残留；禁止给 UI 单独 respawn。手工 `./scripts/run_on_jetson.sh inspection` 不自动重启。桌面自启动 wrapper 则在旧节点全部退出、图形会话恢复并等待 4 秒后，重新启动**完整** inspection 栈；60 秒内最多三次，超过即停止。

## 导航动态安全只读诊断

Supervisor 的 `navigationSafety` 字段汇总 `/scan` 新鲜度、`map→laser_link`、local/global
obstacle layer、Collision Monitor lifecycle 和 `/cmd_vel_safe` 底盘订阅数。巡逻启动会拒绝
过期 `/scan`、未 active 的 Collision Monitor、缺失 global/local `/scan` 订阅或没有安全速度
订阅者；手动建图不要求 global costmap 运行。

```bash
ros2 topic hz /scan
ros2 topic info /scan -v
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_safe -v
ros2 run tf2_ros tf2_echo map laser_link
ros2 topic hz /plan
ros2 lifecycle get /collision_monitor
ros2 topic echo /inspection_ai/system_status --once
```

本机 ROS 2 Humble 1.1.20 不发布 `/collision_monitor_state`，因此以
`ros2 lifecycle get /collision_monitor` 和 Supervisor `navigationSafety.collisionMonitorReady`
为准。检查 `/scan` 订阅者时应同时看到 Collision Monitor、controller/local costmap 和
planner/global costmap；`/cmd_vel` 由 Collision Monitor 订阅，`/cmd_vel_safe` 只由实际底盘后端订阅。

UI 只接受本机 `:N` X11 socket，不把 `localhost:10.0` SSH 转发当控制台。X11/Xwayland 运行中断开时 Qt 进程退出，整栈关闭；解锁/会话恢复后由自启动 wrapper 恢复。`xset s off; xset s noblank; xset -dpms` 只能禁用 X11 屏保/DPMS，不能替代 GNOME 锁屏或系统挂起。

可选 kiosk 设置（仅当前 GNOME 支持的键，`enable` 前备份、`disable` 恢复；默认不改电池模式）：

```bash
./scripts/configure_robot_console_kiosk.sh status
./scripts/configure_robot_console_kiosk.sh enable
./scripts/configure_robot_console_kiosk.sh disable
```

1080p 页面最大内容宽度为 1540px，双卡布局；960×640 自动单列并纵向滚动。现场锁屏/解锁验证由操作员执行：确认 UI/整栈退出、解锁后 wrapper 仅启动一套完整栈，并用 `pgrep -af 'inspection_agent_node|voice_session_node|voice_output_node|system_supervisor_node|inspection_display_ui_node'` 确认没有残留或重复。

## Mobile Bridge 所有权与连接控制

手工 inspection 与桌面 autostart 都通过 `run_on_jetson.sh` 的同一套解析逻辑选择 Mobile Bridge owner。`YLHB_MOBILE_BRIDGE_OWNER=auto` 为默认值：systemd unit active 时选择 `systemd`；unit 不存在时选择 `supervisor`；unit enabled 但 inactive/failed 时仍保留 systemd 所有权并报告异常，避免 Supervisor 趁 systemd 重启期间创建第二份实例。可显式使用 `systemd` 或 `supervisor`，launch 参数 `mobile_bridge_managed_externally:=true|false` 优先级最高。

Supervisor owner 下，`auto_start_mobile_bridge:=true` 会在初始化后启动一次现有 `mobile_bridge` ManagedProcess；启动前同时检查 8000 和 ROS graph，发现外部实例时报告 `ownership_conflict`。systemd owner 下 Supervisor 永不启动、停止或重启该服务。UI 退出仍通过 `OnProcessExit → Shutdown` 关闭完整 inspection 栈；只有 Supervisor 自己启动的内部 Mobile Bridge 会随它清理。

页面将三层状态分开：核心服务来自 Topic、SetBool Service、Supervisor 核心状态和兼容 TCP 摘要；本地 APP 卡只看本地状态与本地 Service；云平台卡只看 Cloud 状态与 Cloud Service。启动前 8 秒显示“正在等待”，之后才显示“未启动”。状态 3～8 秒为更新延迟，超过 8 秒为过期。Switch 仅在对应状态已收到、对应 Service ready、无 pending（云端还要求 configured）时启用，并始终显示禁用原因。内部 owner 可从主页面启动核心；systemd owner 只显示 `sudo systemctl restart ylhb-mobile-bridge.service` 排障提示，UI 不执行 sudo。

只读检查：

```bash
systemctl is-active ylhb-mobile-bridge.service
ss -lntp | grep ':8000'
ros2 node list | grep mobile_bridge
ros2 topic info /mobile_bridge/cloud_status -v --no-daemon
ros2 topic info /mobile_bridge/local_app_status -v --no-daemon
ros2 service list | grep '/mobile_bridge/set_.*_enabled'
pgrep -af mobile_bridge_server
```

通过标准是一个 `/mobile_bridge` 节点、8000 一个监听者、一个实际 `mobile_bridge_server` 子进程。systemd 状态中的 `ros2 run` 父进程与 server 子进程属于同一实例，不应误判为两套 Bridge。
