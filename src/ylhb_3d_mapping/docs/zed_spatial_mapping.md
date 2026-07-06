# ZED SDK Spatial Mapping

`ylhb_3d_mapping` 只负责调用 ZED SDK Spatial Mapping 导出三维点云或网格，不接入 Nav2，也不生成 `map.yaml`/`pgm`。

默认导出融合点云：

```bash
./scripts/run_on_jetson.sh zed_3d_mapping
```

不要和 `./scripts/run_on_jetson.sh zed` 或 `zed_wrapper` 同时运行；ZED 2i 相机通常只能被一个 SDK 客户端占用。

命令话题：

```bash
ros2 topic pub --once /inspection_ai/mapping3d_command std_msgs/msg/String \
  '{"data":"{\"command\":\"start\"}"}'
ros2 topic pub --once /inspection_ai/mapping3d_command std_msgs/msg/String \
  '{"data":"{\"command\":\"stop_and_export\"}"}'
```

输出目录为 `runs/3d_mapping/map_<timestamp>/`，包含 `metadata.json`、`status.json` 和 `pointcloud.ply` 或 `mesh.obj`。

常见错误：

- `pyzed.sl import failed`：ZED SDK Python API 未安装或环境未 source。
- `Camera.open failed`：相机未连接、权限不足，或已被 `zed_wrapper` 占用。
