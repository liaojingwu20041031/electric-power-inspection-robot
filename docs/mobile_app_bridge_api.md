# 电力巡检机器人 Mobile Bridge API

## 后端启动方式

```bash
cd ~/ros2_ws
colcon build --packages-select ylhb_mobile_bridge
source install/setup.bash
ros2 launch ylhb_mobile_bridge mobile_bridge.launch.py
```

默认监听 `0.0.0.0:8000`，仅建议在局域网内使用，不要暴露到公网。

## API 列表

- `GET /api/status`
- `POST /api/cmd_vel`
- `POST /api/text_command`
- `POST /api/task`
- `POST /api/stop`
- `GET /api/debug/status`
- `POST /api/debug/chassis/test`
- `POST /api/debug/chassis/stop`
- `GET /api/debug/mapping/status`
- `POST /api/debug/mapping/start`
- `POST /api/debug/mapping/save`
- `POST /api/debug/mapping/stop`
- `GET /api/debug/navigation/status`
- `POST /api/debug/navigation/start`
- `POST /api/debug/navigation/set_initial_pose`
- `POST /api/debug/navigation/goal`
- `POST /api/debug/navigation/cancel`
- `WebSocket /ws/status`

## curl 测试命令

```bash
curl http://<jetson_ip>:8000/api/status
curl http://<jetson_ip>:8000/api/debug/status

curl -X POST http://<jetson_ip>:8000/api/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"linear_x":0.03,"angular_z":0.0,"duration_ms":300}'

curl -X POST http://<jetson_ip>:8000/api/stop

curl -X POST http://<jetson_ip>:8000/api/debug/chassis/test \
  -H "Content-Type: application/json" \
  -d '{"mode":"forward","linear_x":0.03,"angular_z":0.0,"duration_ms":300}'

curl -X POST http://<jetson_ip>:8000/api/debug/mapping/start
curl -X POST http://<jetson_ip>:8000/api/debug/mapping/save \
  -H "Content-Type: application/json" \
  -d '{"map_name":"my_map"}'
curl -X POST http://<jetson_ip>:8000/api/debug/mapping/stop

curl -X POST http://<jetson_ip>:8000/api/debug/navigation/set_initial_pose \
  -H "Content-Type: application/json" \
  -d '{"x":0.0,"y":0.0,"yaw":0.0}'

curl -X POST http://<jetson_ip>:8000/api/debug/navigation/goal \
  -H "Content-Type: application/json" \
  -d '{"x":1.0,"y":0.5,"yaw":0.0,"label":"checkpoint_001"}'
```

## ROS2 topic 对应关系

- `/api/cmd_vel` -> `/cmd_vel` (`geometry_msgs/Twist`)
- `/api/text_command` -> `/inspection_ai/text_command` (`std_msgs/String`)
- `/api/task` -> 转中文文本后发布到 `/inspection_ai/text_command`
- `/api/debug/navigation/set_initial_pose` -> `/initialpose`
- 状态读取：`/odom`、`/scan`、`/map`、`/zlac8015d/status`、`/zlac8015d/fault`
- 导航目标：`navigate_to_pose` action

## 调试 API 说明

底盘测试 API 会限速并自动停车。mapping/navigation 启动只允许调用 `./scripts/run_on_jetson.sh mapping` 和 `./scripts/run_on_jetson.sh navigation`。停止操作只停止 bridge 自己启动的进程，不会 kill 系统里其他 ROS2 节点。

地图保存 API 调用 Nav2 `map_saver_cli`，保存命令等效于：

```bash
ros2 run nav2_map_server map_saver_cli -f <target> --ros-args -p save_map_timeout:=10.0
```

`save_map_timeout:=10.0` 用于等待 SLAM Toolbox 发布 `/map`，避免 Nav2 默认 2 秒等待在现场建图时偶发超时。返回值包含生成的 `yaml_path`、`pgm_path` 和 `map_saver_cli` 输出。

## 安全注意事项

- 第一次底盘测试请架空轮子。
- 建图和导航测试前确认机器人周围安全。
- 后端限制最大线速度不超过 `0.15 m/s`，最大角速度不超过 `0.5 rad/s`。
- 所有 `/cmd_vel` 动作必须带 duration，超时自动发布 0 速度。
- 手机端和 Jetson bridge 仅在局域网使用。
