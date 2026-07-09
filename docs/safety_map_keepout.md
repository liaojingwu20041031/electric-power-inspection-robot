# 安全地图与 Keepout 导航

Keepout 不修改 `maps/my_map.pgm`。禁行区写在路线文件 `maps/route_patrol_001.json` 的 `keepout_zones`，需要启动 keepout 导航时生成临时 mask：

```bash
python3 scripts/generate_keepout_mask.py \
  --map maps/my_map.yaml \
  --route maps/route_patrol_001.json \
  --output-dir /tmp \
  --name keepout_mask_power_room_a
```

mask 语义固定：禁行区黑色 `0`，非禁行区白色 `254`。mask 的 `resolution`、尺寸、`origin x/y` 必须和原地图一致，`origin yaw` 固定为 `0`。

检查：

```bash
python3 scripts/check_keepout_setup.py \
  --map maps/my_map.yaml \
  --route maps/route_patrol_001.json \
  --mask /tmp/keepout_mask_power_room_a.yaml \
  --nav2-params src/ylhb_base/config/nav2_params_keepout.yaml
```

启动：

```bash
./scripts/run_on_jetson.sh navigation_keepout enable_local_keepout:=false
```

先只验证 global path 绕行；确认后再低速短距离测试 `enable_local_keepout:=true`。

Keepout lifecycle 节点：

- `keepout_filter_mask_server` 发布 `keepout_filter_mask`
- `costmap_filter_info_server` 发布 `keepout_costmap_filter_info`
- `lifecycle_manager_keepout` 管理上面两个节点

路线安全检查：

```bash
python3 scripts/validate_route_safety.py \
  --route maps/route_patrol_001.json \
  --nav2-params src/ylhb_base/config/nav2_params.yaml
```

v3 路线里 `targets[].pose` 是执行主字段，`targets[].location` 只做扩展/调试。`pose` 缺失且 `location.type=="map_pose"` 时可回填；两者同时存在但 x/y/yaw 不一致时校验失败。
