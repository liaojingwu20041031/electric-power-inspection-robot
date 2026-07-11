# 路线 JSON 字段参考

正式路线由 `ylhb_mobile_bridge.patrol_route_store.validate_route_file()` 校验，v3 还由 `validate_route_map_binding()` 绑定到实际地图。所有坐标都在 `map` frame、单位为米；yaw 单位为弧度。

状态标记：**执行** 表示运行逻辑直接使用，**透传** 表示校验后保留，**工具只读** 表示路线工具会保留但无编辑 UI，**拒绝** 表示当前明确不接受。

| 字段路径 | 类型 / 必填 / 单位与范围 | 绘图工具 | 实际读取与影响 | 状态 |
|---|---|---|---|---|
| `version` | 整数，必填，`2` 或 `3` | 可选 v2/v3 | route store 决定 schema；正式路线为 v3 | 执行 |
| `frame_id` | 字符串，必填，必须 `map` | 固定写入 | route store | 执行 |
| `map.yaml` / `map.image` | 非空字符串，v3 必填 | 由加载地图写入 | 地图绑定文件名 | 执行 |
| `map.resolution` | 正数，v3 必填，m/cell | 由 YAML 写入 | 地图绑定；也是 v3 `mask_padding_m` 默认和上限 | 执行 |
| `map.origin` | 三个有限数，v3 必填，m/rad | 由 YAML 写入 | 地图绑定 | 执行 |
| `map.width` / `map.height` | 正整数，v3 必填，pixel | 由 PGM 写入 | 地图绑定 | 执行 |
| `map.image_sha256` | 64 位小写 SHA-256，v3 必填 | 由 PGM 写入 | 防止路线用于错误地图 | 执行 |
| `active_route_id` | 可选非空字符串，若有必须引用现有 route | 可编辑 | 默认巡逻路线 | 执行 |
| `start_pose.name` | 非空字符串，默认 `start` | 可编辑 | UI/状态名称 | 执行 |
| `start_pose.frame_id` | v3 必填，必须 `map` | 固定写入 | 初始位姿约束 | 执行 |
| `start_pose.pose.x/y/yaw` | 有限数，必填，m/rad | 可编辑、可拖动与设 yaw | 初始定位与返航位姿 | 执行 |
| `start_pose.location` | v3 必填 `map_pose`；与 pose 一致 | 工具重写 | v3 扩展位置契约 | 执行 |
| `start_pose.publish_initial_pose` | 布尔，默认 `false` | 可编辑 | 是否发布 `/initialpose` | 执行 |
| `start_pose.covariance.x/y/yaw` | 非负数；存在或需发布初始位姿时必填 | 可编辑 | 初始位姿协方差 | 执行 |
| `targets[].id` / `name` | 非空字符串；id 唯一 | name 可编辑，id 由工具管理 | 路线引用与 UI 名称 | 执行 |
| `targets[].pose.x/y/yaw` | 有限数，v2 必填；v3 可由 location 回填 | 可编辑、可拖动与设 yaw | NavigateToPose 目标 | 执行 |
| `targets[].location` | v3 必填 `map_pose`，与 pose 一致 | 工具重写 | v3 位置契约 | 执行 |
| `targets[].aliases` | 字符串数组，默认 `[]` | 工具只读、导出保留 | Agent/业务别名 | 透传 |
| `targets[].area_id` | 字符串或 `null` | 工具只读、导出保留 | 区域关联 | 透传 |
| `targets[].inspection_items` | 字符串数组，默认 `[]` | 工具只读、导出保留 | 检查项元数据 | 透传 |
| `targets[].task_duration_sec` | 非负数，默认 `0`，s | 可编辑 | 到点任务等待 | 执行 |
| `targets[].safety` | 对象 | 工具重新计算 | 当前点位安全结果 | 执行（结果） |
| `routes[].id` / `name` | 非空字符串；id 唯一 | 单路线可编辑 | 路线选择/显示 | 执行 |
| `routes[].aliases` / `description` | 字符串数组 / 字符串或 `null` | 工具只读、导出保留 | 业务描述 | 透传 |
| `routes[].target_ids` | id 数组，必须引用 targets | 由目标顺序重写 | 巡检顺序 | 执行 |
| `routes[].return_to_start` | 布尔，默认 `false` | 可编辑 | 结束是否返航 | 执行 |
| `routes[].loop.enabled` | 布尔，默认 `false` | 可编辑 | 是否循环 | 执行 |
| `routes[].loop.wait_sec` | 非负数，默认 `600`，s | 可编辑 | 下一轮等待 | 执行 |
| `routes[].loop.max_cycles` | 非负整数，`0` 为不限轮数 | 可编辑 | 最大轮数 | 执行 |
| `routes[].goal_timeout_sec` | 正数，默认 `120`，s | 可编辑 | 单目标超时 | 执行 |
| `routes[].max_retries_per_checkpoint` | 非负整数，默认 `0` | 可编辑 | 单点最大重试 | 执行 |
| `routes[].failure_policy` | `abort` 或 `abort_and_return_home`，默认 `abort` | 可编辑 | 失败终止或尝试返航 | 执行 |
| `keepout_zones[].id` / `name` | id 非空且唯一；name 扩展字段 | 可编辑 | 禁行区标识/显示 | 执行 |
| `keepout_zones[].type` | 必须 `hard_keepout` | 固定 hard_keepout | Profile 选择和 mask 输入 | 执行 |
| `keepout_zones[].enabled` | 布尔，必填 | 可编辑 | 仅启用 zone 触发 keepout Profile | 执行 |
| `keepout_zones[].polygon[]` | 至少 3 个 `{x,y}` 有限点；不可自交、非零面积 | 可编辑、追加、拖动、删除 | 二值 mask 栅格化与安全检查 | 执行 |
| `keepout_zones[].mask_padding_m` | v3 默认 `map.resolution`；非负且不大于 resolution，m | 可编辑 | polygon 二值墙边界补偿；正式为 `0.025` | 执行 |
| `schedules[]` | 数组，默认 `[]` | 仅空数组可加载 | schedule 配置 | 执行（interval） |
| `schedules[].mode` | 仅 `interval` | 不可编辑 | `daily` **拒绝**：`daily schedule is not implemented; use interval or remove this schedule` | 拒绝 daily |
| `schedules[].id/route_id/enabled/period_sec` | id/route 必填；enabled 布尔；interval 的 period 为正秒数 | 工具不可编辑 | interval 定时巡逻 | 执行 |
| `safety` | 对象 | 工具重新计算 | 全路线安全结果 | 执行（结果） |
| `site` | 对象 | 工具只读、导出保留 | 站点业务元数据 | 透传 |
| `areas[]` | 对象数组；id 唯一，name 非空 | 工具只读、导出保留 | 区域业务元数据 | 透传 |

## 兼容性与工具边界

v2 没有完整地图身份；缺少 `mask_padding_m` 被允许，字段存在时只校验其为有限非负数。运行前实际地图的边界由后续 checker 判断。v3 缺少 `mask_padding_m` 会规范化为 `map.resolution` 并显式写出。

路线工具保存原始 v3 模板，在导出时保留未编辑的合法扩展字段；但它明确拒绝多 route 文件和非空 `schedules`，避免只取第一条路线或静默清空调度。不要把 `mask_padding_m` 当作最终避障距离：二值墙后的 InflationLayer 才定义实际避让外观。
