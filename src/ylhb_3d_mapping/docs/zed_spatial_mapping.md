# ZED 3D Capture and Offline Reconstruction

`ylhb_3d_mapping` 用 ZED 2i 采集三维巡检素材并导出点云。当前 Orin Nano 8GB 不再使用实时 `zed_3d_mapping` 模式；实时 Spatial Mapping 容易 OOM，质量也受现场算力限制。

## 采集 SVO

先确认没有其他 ZED 进程占用相机：

```bash
ps -ef | grep -E 'zed_wrapper|zed_camera|zed_spatial_mapping' | grep -v grep
```

现场只录制 SVO：

```bash
./scripts/run_on_jetson.sh zed_3d_capture duration_sec:=30
```

输出位于 `runs/3d_capture/capture_<timestamp>/`：

- `capture.svo2`
- `metadata.json`
- `status.json`

默认采集 `HD720@30`，不现场计算深度，减少卡顿和内存压力。

## 离线重建

用录好的 SVO 离线导出点云：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2
```

默认 profile 是 `quality_safe`：

- `depth_mode=NEURAL`
- `resolution_preset=high`
- `range_preset=near`
- `spatial_mapping_max_memory_mb=1024`

快速检查 SVO 是否可用：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2 profile:=fast_check
```

机器负载低时可尝试更慢的高质量档：

```bash
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=runs/3d_capture/capture_<timestamp>/capture.svo2 profile:=quality_plus
```

如果 `quality_plus` 被 OOM killer 杀掉，改回默认 `quality_safe`。

## 质量要点

采集时慢速移动，同一区域从多个角度多扫几遍；目标尽量保持在 0.25m 到 4m 内。避免暗光、纯白反光面、玻璃、强背光和过近贴脸拍摄。

如果离线点云仍然差，先用 ZED Depth Viewer / Explorer 看原始 SVO 的图像、曝光和深度质量。原始深度不好时，继续堆点云参数不会明显改善最终模型。

输出 PLY 用 CloudCompare、MeshLab 或支持点云的三维查看器打开。`metadata.json` 里的 `export_point_count` 只表示点数，不等于几何质量。

## 非导航用途

这些文件不是 Nav2 地图，不会写入 `maps/my_map.yaml`，也不会修改 `src/ylhb_base/config/nav2_params.yaml`。巡逻和导航仍按现有二维地图流程执行。
