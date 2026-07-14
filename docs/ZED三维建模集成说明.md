# ZED 3D 建模集成说明

本仓库的三维建模改为“双阶段”：现场录制 ZED SVO，离线从 SVO 重建点云。`zed_3d_mapping` 实时建图模式已从 `run_on_jetson.sh` 删除；它在 Orin Nano 8GB 上容易 OOM，最终质量也不稳定。

## 采集

先确认没有其他 ZED 进程占用相机：

```bash
ps -ef | grep -E 'zed_wrapper|zed_camera|zed_spatial_mapping' | grep -v grep
```

启动 SVO 采集：

```bash
cd ~/ros2_DL
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_on_jetson.sh zed_3d_capture duration_sec:=30
```

输出位于：

```text
runs/3d_capture/capture_<timestamp>/
```

目录包含：

- `capture.svo2`
- `metadata.json`
- `status.json`

采集时慢速移动，同一区域多角度扫几遍；目标保持在 0.25m 到 4m 内，避开暗光、玻璃、纯白反光面和强背光。

## 离线重建

默认高质量安全档：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2
```

快速检查档：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2 profile:=fast_check
```

更慢的高质量档：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2 profile:=quality_plus
```

如果 `quality_plus` 出现 `exit code -9` 或内核 OOM 记录，改回默认档。默认档参数为 `NEURAL`、`high` 分辨率、`near` 范围、1024MB 空间映射上限。

## 查看结果

离线输出位于：

```text
runs/3d_reconstruct/reconstruct_<timestamp>/
```

包含：

- `pointcloud.ply`
- `metadata.json`
- `status.json`

PLY 可用 CloudCompare、MeshLab 或支持点云的三维查看器打开。`export_point_count` 只表示点数，不代表几何质量；如果点很多但形状仍差，先用 ZED Depth Viewer / Explorer 检查 SVO 的原始图像、曝光、深度和 confidence。

## 非导航用途

这些文件不是 Nav2 地图，不会写入 `maps/my_map.yaml`，也不会修改 `src/ylhb_base/config/nav2_params.yaml`。巡逻和导航仍按现有二维地图流程执行。
