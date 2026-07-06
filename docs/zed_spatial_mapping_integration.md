# ZED Spatial Mapping 集成说明

本仓库第一版三维建图使用 ZED SDK Spatial Mapping，而不是 Nav2 栅格地图流程。原因很简单：三维点云/网格是巡检记录和现场复核资产，Nav2 仍使用二维 SLAM/AMCL 地图。

## 输出类型

- `fused_point_cloud`：默认值，导出 `pointcloud.ply`，适合快速查看三维点云。
- `mesh`：导出 `mesh.obj`，可选 `save_texture`，耗时和显存压力更高。

## 启动

先确认没有其他 ZED 进程占用相机：

```bash
ps -ef | grep -E 'zed_wrapper|zed_camera|zed_spatial_mapping' | grep -v grep
```

终端 1 启动节点：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_on_jetson.sh zed_3d_mapping
```

不要与 `./scripts/run_on_jetson.sh zed` 或其他 `zed_wrapper` 进程同时运行，ZED 2i 会被抢占。

## 命令

终端 2 观察状态：

```bash
ros2 topic echo /inspection_ai/mapping3d_status
```

终端 3 开始采集：

```bash
ros2 topic pub --once /inspection_ai/mapping3d_command std_msgs/msg/String \
  '{"data":"{\"command\":\"start\"}"}'
```

状态通常依次为 `opening_camera`、`tracking_enabled`、`mapping_enabled`、`running`。
进入 `running` 后，缓慢移动相机或机器人扫过目标区域。节点不发布 `/cmd_vel`，
不会自动控制底盘。

采集完成后导出：

```bash
ros2 topic pub --once /inspection_ai/mapping3d_command std_msgs/msg/String \
  '{"data":"{\"command\":\"stop_and_export\"}"}'
```

导出期间状态会经过 `extracting`、`saving`，成功后变为 `succeeded`。也支持：

- `stop`：停止采集但不导出。
- `export`：对当前已打开的相机地图执行导出。
- `reset`：重置节点状态，适合失败后重新开始。

状态发布到 `/inspection_ai/mapping3d_status`，导出结果发布到 `/inspection_ai/mapping3d_result`。

## 查看结果

输出位于 `runs/3d_mapping/map_<timestamp>/`：

- `metadata.json`
- `status.json`
- `pointcloud.ply` 或 `mesh.obj`

查看最近一次输出：

```bash
ls -lt ~/ros2_DL/runs/3d_mapping | head
ls -lh ~/ros2_DL/runs/3d_mapping/map_*/pointcloud.ply
```

PLY/OBJ 可用 CloudCompare、MeshLab 或支持点云/网格的三维查看器打开。

mesh 模式启动：

```bash
./scripts/run_on_jetson.sh zed_3d_mapping map_type:=mesh
```

如需纹理可加 `save_texture:=true`，但耗时和显存压力更高。

## 非导航用途

这些文件不是 Nav2 地图，不会写入 `maps/my_map.yaml`，也不会修改 `src/ylhb_base/config/nav2_params.yaml`。巡逻和导航仍按现有二维地图流程执行。

## 常见错误

- `pyzed.sl import failed`：ZED SDK Python API 未安装，或当前终端没有 source 正确环境。
- `Camera.open failed`：相机未连接、权限不足，或已被 `zed_wrapper` / 其他 ZED 程序占用。
- 一直没有进入 `running`：检查 USB 连接、电源和 `dmesg`，再确认没有多个 ZED 进程。
