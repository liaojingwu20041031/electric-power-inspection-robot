# 二值 Keepout 安全地图操作

## 设计与适用条件

路线 JSON 的启用 `hard_keepout` 会选择 keepout Profile；没有启用禁区时选择 normal Profile。keepout 不修改 `maps/my_map.pgm`，而是从路线 polygon 生成 global/local 两张二值 mask：禁行单元为 `0`，其余为 `254`。

`mask_padding_m` 仅是 polygon 栅格化的边界补偿。当前地图 resolution 为 `0.025m`，正式默认值和推荐值也是 `0.025m`，允许范围为 `0..map.resolution`。它不是 footprint、InflationLayer 半径或最终避让距离；最终避让保持由 InflationLayer `6.0 / 0.35` 完成。

KeepoutFilter 必须在 InflationLayer 前：前者把二值禁区写入 costmap，后者才可对虚拟墙和真实墙壁以相同方式膨胀。global 和 local 使用相同语义的 binary mask，规划仍由既有 SmacPlanner2D 与 DWB 完成。

## 路线与资产流程

1. 在 [`tools/route_map_tool/route_map_tool.html`](../tools/route_map_tool/route_map_tool.html) 加载地图与路线，编辑 polygon 和 `mask_padding_m`。
2. 保持 v3 路线的 `map` 身份与实际 YAML/PGM 一致；修改路线后先进行安全检查。
3. 生成与核验 keepout 资产：

```bash
cd ~/ros2_DL
python3 scripts/generate_keepout_mask.py \
  --map maps/my_map.yaml \
  --route maps/route_patrol_001.json \
  --nav2-params src/ylhb_base/config/nav2_params_keepout.yaml \
  --output-dir maps/keepout

python3 scripts/check_keepout_setup.py \
  --map maps/my_map.yaml \
  --route maps/route_patrol_001.json \
  --nav2-params src/ylhb_base/config/nav2_params_keepout.yaml \
  --output-dir maps/keepout

python3 scripts/validate_route_safety.py \
  --map maps/my_map.yaml \
  --route maps/route_patrol_001.json \
  --nav2-params src/ylhb_base/config/nav2_params_keepout.yaml \
  --report
```

第二条命令必须输出 `keepout setup OK`。安全检查的 `status` 不能是 `unsafe`；不要用 `--suggest-target` 自动改点位。

## Profile 与 lifecycle

supervisor 先完成路线/地图绑定及路线安全检查；有启用 `hard_keepout` 时再生成、检查 mask，随后启动 `navigation_keepout.launch.py`。该 launch 的 `lifecycle_manager_keepout` 管理四个节点：

- `keepout_global_mask_server`
- `keepout_global_filter_info_server`
- `keepout_local_mask_server`
- `keepout_local_filter_info_server`

之后正常确认 Nav2 的 planner/controller lifecycle。RViz 中应同时观察原始 `/keepout_global_mask`、`/keepout_local_mask` 与膨胀后的 global/local costmap；全局路径必须绕开禁区，虚拟墙的膨胀外观应与真实墙一致。

## 现场通过标准与错误分类

现场人员完成短距离验收：确认 `keepout masks ready`，四个 mask/filter 节点及 planner/controller active；路径绕开禁区，空旷区不无故卡死，暂停/恢复/取消与返航可用。

- 地图绑定失败：路线 v3 的 YAML、PGM SHA-256、resolution、origin 或尺寸不匹配。
- mask 生成失败：polygon、路线契约或输出目录异常。
- checker 失败：global/local mask 或 filter 参数与路线不一致。
- lifecycle 失败：先检查 keepout 四节点，再检查 planner/controller。
- 路线 unsafe：不改 Nav2 参数，先修正路线/禁区或现场地图。

实际运动和恢复策略由现场人员执行；不要从文档命令直接替代现场安全检查。
