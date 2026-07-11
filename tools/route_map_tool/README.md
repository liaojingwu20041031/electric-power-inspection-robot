# 智能电网机器人巡检路线标注工具

打开 `route_map_tool.html` 即可使用，无需安装依赖。

## 使用方法

1. 选择或拖入建图后的 `.yaml` 文件，例如 `my_map.yaml`。
2. 选择或拖入对应的 `.pgm` 文件，例如 `my_map.pgm`。
3. 可选：选择或拖入已有路线 `.json`，继续编辑之前标过的点；支持 v2/v3。工具只加载单路线且 `schedules` 为空的文件。
4. 在右上方选择模式：
   - `起点`：点击地图设置 `start_pose`。
   - `巡检点`：点击地图追加多个 `targets`。
   - `方向`：选择起点或某个巡检点后，点击/拖拽地图确定 `yaw`。
   - `禁行区`：点击地图为当前 `keepout_zones` polygon 追加顶点。
   - `拖动`：平移地图视图。
5. 拖动已有点位可以微调坐标。
6. 巡检点列表里的 `↑` / `↓` 会改变导航顺序，也就是 `routes[].target_ids` 的顺序。
7. 巡检点列表里的 `↗` 会把该点设为当前方向点，并切到方向模式。
8. 禁行区列表可新增、选择、重命名、启用/禁用、删除禁区，并编辑、拖动或删除当前 polygon 顶点。
9. 点击 `下载 route.json` 生成巡检执行器可加载的路线文件；默认导出 v3，可切换 v2。

下载前工具会检查起点和全部巡检点：

- 必须在当前 PGM 地图范围内。
- 必须落在 ROS map_server 判定的 free 区域。
- 车体 footprint 不得碰到障碍区、未知区、越界区或 hard_keepout 禁行区。
- 如果点位不安全，工具会阻止下载并提示具体点位。

禁行区只保存在 route JSON 顶层 `keepout_zones`。Nav2 keepout mask 需要运行时生成，不在 `maps/` 里维护第二份禁区 JSON。

## `mask_padding_m`

每个 `hard_keepout` 都有“虚拟墙边界补偿（m）”，对应 `keepout_zones[].mask_padding_m`。

- 默认值是当前地图 resolution；当前 `my_map` 为 `0.025m`。
- 可填范围是 `0` 到当前地图 resolution，步进也是当前地图 resolution。
- 它只补偿 polygon 栅格化的二值虚拟墙边界，不是机器人半径、InflationLayer 半径或最终避障距离。
- 最终避让由 Nav2 InflationLayer 的 `cost_scaling_factor: 6.0` 与 `inflation_radius: 0.35` 产生。

导入旧路线缺少该字段时，工具在内存中按当前地图 resolution 补齐；导出会显式写入。

## 导入与导出边界

导入 v3 后，工具会保留原始 JSON 模板，并只覆盖实际可编辑的地图、起点、目标 pose/location、目标顺序、禁行区、活动路线和重新计算的 `safety`。因此会保留 `site`、`areas`、目标 aliases/area_id/inspection_items、路线 aliases/description 与其他合法扩展字段。

为避免静默丢数据，遇到下列文件会拒绝加载：

- `routes.length !== 1`：当前工具只支持单路线文件。
- `schedules.length > 0`：当前工具不编辑 schedules。

编辑点位、方向或禁行区后，旧 `safety` 结果不会沿用，JSON 预览会按工具现有安全检查重新生成。

## 浏览器人工验收

加载 `maps/my_map.yaml`、`maps/my_map.pgm` 与 `maps/route_patrol_001.json` 后，确认起点、四个巡检点、yaw 箭头、目标顺序、禁行 polygon 和 `mask_padding_m=0.025` 正常。拖动一个巡检点或禁行顶点后恢复正式坐标，下载 JSON 并重新导入，确认点位、polygon、顺序和扩展字段仍在。不要提交人工验收期间的临时坐标。

## 多地图尺寸处理

工具每次读取 PGM 头部的真实宽高，不写死地图尺寸；换成其他 `.pgm/.yaml` 后会自动重新适配画布。

已标点位始终以 ROS `map` 坐标保存，不会因为换地图而被偷偷改坐标。如果起点或巡检点超出当前地图范围，或落在 unknown/occupied 像素，右侧会出现提示，地图上的越界点也会变红。

## 坐标说明

导出的 JSON 固定使用：

- `frame_id: "map"`
- `start_pose.pose` 和 `targets[].pose` 均为 ROS `map` 坐标系下的米制坐标。

默认导出 `version: 3`。v3 里：

- `targets[].pose` 是巡逻执行主字段。
- `targets[].location` 是扩展/调试字段，类型为 `map_pose`。
- v2 导出仍保留，供旧执行器兼容。

工具按 ROS map_server 地图规则换算：

```text
x = origin_x + pixel_x * resolution
y = origin_y + (image_height - pixel_y) * resolution
```

方向 `yaw` 使用弧度，工具里的箭头方向会直接写入对应的 `pose.yaw`。

工具支持 ROS 地图 YAML 里两种常见 `origin` 写法：

```text
resolution: 0.05
origin: [-2.89, -6.37, 0]
```

```text
resolution: 0.05
origin:
- -5.89
- -13.3
- 0
```

界面底部会显示当前加载地图的 `origin`、`resolution`、像素尺寸和坐标范围。导出前请确认这些信息与实际使用的 `.yaml/.pgm` 一致，且右侧没有越界或非空闲区告警。

旧示例地图参数如下，仅作为格式示例，不能代表当前地图：

```text
resolution: 0.05
origin: [-2.89, -6.37, 0]
```
