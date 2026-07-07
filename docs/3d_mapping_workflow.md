# ZED 3D 双阶段建模流程

## 路线

正式流程分两步：

1. 现场只录 ZED SVO：保存 `capture.svo2`，尽量减少 Orin Nano 现场负载。
2. 离线重建点云：从最新 SVO 或指定 session 生成 `pointcloud.ply`。

Orin Nano 不实时输出最终精品模型。ZED 深度、位姿跟踪和空间融合同时跑会占用明显 GPU/内存，现场优先保证采集完整，精品点云放到离线阶段处理。

## latest/index

采集完成后会写：

- `runs/3d_capture/latest.json`
- `runs/3d_capture/index.json`
- `runs/3d_capture/capture_xxx/metadata.json`
- `runs/3d_capture/capture_xxx/status.json`

重建完成后会写：

- `runs/3d_reconstruct/latest.json`
- `runs/3d_reconstruct/index.json`
- `runs/3d_reconstruct/reconstruct_xxx/metadata.json`
- `runs/3d_reconstruct/reconstruct_xxx/status.json`

`latest.json` 解决每次时间戳目录变化的问题，UI 和 CLI 默认都用最新采集。

## CLI

```bash
./scripts/run_on_jetson.sh zed_3d_capture duration_sec:=0
./scripts/run_on_jetson.sh zed_3d_reconstruct latest
./scripts/run_on_jetson.sh zed_3d_reconstruct input:=latest profile:=quality_safe
./scripts/run_on_jetson.sh zed_3d_reconstruct session:=capture_xxx
```

`duration_sec:=0` 表示持续采集，直到停止命令或 SIGINT/SIGTERM。`profile` 可用 `fast_check`、`quality_safe`、`quality_plus`。

## UI

进入“三维建模”页面：

1. 点击“开始采集”。
2. 现场走完需要的路线后，点击“停止并保存 SVO”。
3. 有最新 SVO 后，点击“快速重建”或“高质量重建”。
4. 最新模型路径显示为 `pointcloud.ply`。

Foxglove 查看参数：

- Fixed Frame: `zed_3d_map`
- Topic: `/inspection_ai/mapping3d_pointcloud`
- Color field: `z`

离线 `PLY` 可用 CloudCompare、MeshLab 或 Open3D 查看。

## 边界

本流程不写 `maps/my_map.yaml`，不修改 `nav2_params.yaml`，不接入 Nav2 导航地图。
