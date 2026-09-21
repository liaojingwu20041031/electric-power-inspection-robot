# DG-202611 一次性实机采集流程

本流程用于尽量一次收齐方案书需要的机器人数据。采集器只订阅 ROS 2 话题、读取
系统状态并记录 rosbag，不会发布运动命令。所有移动、遮挡和故障注入均由现场人员
明确执行。

## 开始前

1. 机器人上层和底盘电池均有电，急停按钮可立即使用。
2. 手机控制、UI、建图/导航和底盘单独验证正常。
3. `/home/nvidia/ros2_DL` 保持原样，三模型位于 `/home/nvidia/dg202611_ws`。
4. 在地面用卷尺标记起点、两个中间点和终点，记录坐标并拍照。若要计算位置误差，
   这些坐标必须与待评价位姿使用同一坐标系（默认 `map`）；只有相对距离而没有坐标系
   对齐时，只保存轨迹，不填写绝对位置误差。
5. 每次实验前记下场景编号；不能在看完结果后修改阈值或删除失败trial。

## 从 Windows 安全部署

部署脚本先校验发布包 SHA256，再上传到机器人，并解压到带提交号的新目录；若目录已
存在会拒绝覆盖。它只读取 `/home/nvidia/ros2_DL/install` 作为底层环境，不修改学长
工作区，也不发送运动命令。电脑和机器人进入同一网络后，在 Windows PowerShell 执行：

```powershell
cd C:\Users\peng\Documents\工作区1\dg202611_robot_integration
.\scripts\deploy_release_from_windows.ps1 -RobotHost <机器人当前IP>
```

成功标志为 `DEPLOY_READY=/home/nvidia/dg202611_ws_<提交号>`。部署目录内保存
`deploy_preflight.log`、`DEPLOYMENT_RECORD.json` 和
`DEPLOYED_SOURCE_MANIFEST.sha256`，现场测试前先保留这三份记录。

确认预检记录无误后，再显式切换活动版本；脚本只允许指向通过部署预检的版本目录，
并拒绝覆盖同名普通目录：

```bash
/home/nvidia/dg202611_ws_<提交号>/scripts/activate_dg_release.sh \
  /home/nvidia/dg202611_ws_<提交号>
```

屏幕会同时打印新旧目标。需要回滚时，把参数换成上一个版本目录再次运行即可；
`/home/nvidia/ros2_DL` 不参与切换。

## 采集命令

在一个独立终端运行：

```bash
cd /home/nvidia/dg202611_ws
./scripts/run_real_robot_capture.sh \
  --scenario EXP02_IDLE_REAL \
  --duration 180 \
  --operator peng \
  --hardware-notes "上下电池已供电；急停可用；底盘静止"
```

等待终端显示 `CAPTURE_READY:` 后再开始本场景动作。

采完整条巡检路线时，可让采集器先等待 `/patrol/status` 进入 `running`，并在巡检终态
后继续保留 5 秒数据再自动停止：

```bash
./scripts/run_real_robot_capture.sh \
  --scenario EXP01_ROUTE_FUSION_R1 \
  --duration 900 \
  --wait-for-patrol-start \
  --stop-on-patrol-terminal \
  --post-roll-sec 5 \
  --startup-timeout-sec 120 \
  --operator peng \
  --hardware-notes "正式路线R1；现场人员看护；急停可用"
```

命令启动后先去 UI 点击“启动本地巡检”。旧的终态消息不会让采集器提前结束；只有本次
实际观察到 `running` 后出现的 `succeeded/failed/canceled` 才会触发停止。若 120 秒
内没有进入运行态，本次记录标为失败并保留已有日志，避免把空等数据误当成路线结果。

结果默认保存在：

```text
/home/nvidia/dg202611_ws/results/real_robot/YYYYmmdd_HHMMSS_<scenario>/
```

每个目录自动包含 `metadata.json`、`result.json`、`rosbag/`、资源采样、节点/话题/
参数快照、CAN和手机桥接状态、日志及SHA256清单。还会生成 `structured/`：

- `lidar_features.csv`：MODEL-2 每帧稳定特征二维坐标；
- `lidar_quality.csv`：有效点、异常点、动态候选、几何质量和时间一致性；
- `fusion_status_samples.csv`：MODEL-1 模式、接受源、置信度和坐标系；
- `relocalization_status_samples.csv`：MODEL-3 完整状态时间线；
- `relocalization_events.csv`：每次尝试唯一终态，可直接计算成功率和恢复时间；
- `position_samples.csv`：仅在命令提供同坐标系卷尺真值时写入有效样本；
- `diagnostic_samples.csv`：上述诊断消息的无损 JSON 字段备份。

采集器与结构化记录节点都没有运动发布器。`capture_status=COMPLETE` 要求 rosbag 和
结构化记录同时完整，但仍只表示采集完整，不代表性能合格。

## 有卷尺真值的定点采集

机器人中心停在已标定的 `map` 坐标点 `(x, y)` 后，保持静止并执行：

```bash
./scripts/run_real_robot_capture.sh \
  --scenario EXP01_POSITION_P0 \
  --duration 60 \
  --ground-truth-x 0.00 \
  --ground-truth-y 0.00 \
  --ground-truth-frame map \
  --estimated-pose-topic /dg/fusion/pose \
  --hardware-notes "卷尺点P0；机器人中心对准标记；朝向按现场记录"
```

对 P1、P2、P3 分别重复一次并写入各自坐标。若不能确认卷尺坐标和 `map` 的原点、
方向一致，就不要填 `--ground-truth-*`；原始位姿仍会进入 rosbag，后续不会被误算成
定位精度。

## 推荐顺序

| 顺序 | 场景编号 | 时长 | 现场动作 | 主要用途 |
|---:|---|---:|---|---|
| 1 | EXP02_IDLE_REAL | 180 s | 机器人静止 | 节点、频率、TF、CPU/GPU/内存基线 |
| 2 | EXP03_LIDAR_CLEAR | 120 s | 环境保持静态 | MODEL-2正常质量基线 |
| 3 | EXP03_LIDAR_PARTIAL | 120 s | 纸板遮挡约1/3雷达视场 | 局部遮挡检测与降级 |
| 4 | EXP03_LIDAR_BLOCKED | 60 s | 短时遮挡大部分视场后恢复 | 拒绝与恢复滞回 |
| 5 | EXP03_LIDAR_DYNAMIC | 120 s | 一人在机器人前方横向往返 | 动态目标与稳定特征 |
| 6 | EXP01_POSITION_P0-P3 | 每点60 s | 机器人中心停在同坐标系卷尺标记处 | 定点平面误差；无同坐标系真值则跳过指标 |
| 7 | EXP01_ROUTE_BASELINE_R1-R3 | 每次按路线结束 | 三次相同路线，三模型关闭 | 原系统轨迹基线 |
| 8 | EXP01_ROUTE_FUSION_R1-R3 | 每次按路线结束 | 三次相同路线，三模型旁路开启 | 融合输出对比 |
| 9 | EXP04_INVALID_CANDIDATE | 60 s | 机器人静止，运行预先冻结的错误候选注入 | 验证错误候选拒绝 |
| 10 | EXP04_ACTIVE_RECOVERY_R01-R20 | 每次最长60 s | 空旷区域，逐次执行冻结的定位异常场景 | 恢复成功率与时间 |
| 11 | EXP02_ENDURANCE | 600 s | 三模型同时运行并正常使用手机/UI | 稳定性和资源裕量 |

第9、10项涉及恢复状态和可能的旋转动作。开始前必须确认速度仲裁器所有权、周围空旷
和急停；采集器本身不会启动这些动作。

MODEL-2 重复率仅对第2～5项中机器人保持静止的区间解释。评价器按同一雷达坐标系中
相邻帧特征的 0.12 m 一对一空间匹配计算 `R_min/R_avg/R_pool`；零特征帧单独计数，
不会被当成 100% 重复率。路线运动中的局部雷达坐标会随车体变化，不作为该指标。

## MODEL-3 错误候选负例

该负例只验证“错误候选被拒绝”，不让错误候选与真实 scan-to-map 结果竞争，也不接管
底盘。由现场人员在两个终端执行：

```bash
# 终端A：观察模式；关闭真实候选生产器和速度仲裁器
./scripts/run_dg_integration.sh \
  enable_nav2:=false \
  enable_scan_map:=false \
  enable_cmd_vel_arbiter:=false

# 终端B：仅在 MODEL-3 明确请求候选后发布一次受控错误诊断
ros2 run ylhb_base invalid_candidate_injector_node \
  --max-injections 1 \
  --timeout-sec 45
```

同时以 `EXP04_INVALID_CANDIDATE` 运行采集器。现场人员需先用冻结的定位异常步骤让
MODEL-3 进入候选请求阶段；注入器若未观察到请求会以失败退出，不会伪造成功。
通过证据为 `/dg/relocalization/match_quality` 出现
`CONTROLLED_INVALID_CANDIDATE`，随后 `/dg/relocalization/status` 明确记录拒绝原因；
`/cmd_vel` 不应由该工具发布。真实恢复正例必须恢复 `enable_scan_map:=true`，并单独测试。

## UWB数据的使用

UWB硬件数据已经采完，不需要在上述流程中重新连接。为了明确证明 MODEL-1 消费
真实 UWB，同时避免把未标定坐标写成地图坐标，使用独立证据入口分别回放三份数据：

```bash
./scripts/run_uwb_model1_evidence.sh static
```

另外两个参数为 `move` 和 `anchor_fault`。每份回放时单独启动一次采集，场景编号分别
使用 `UWB_STATIC_REPLAY`、`UWB_MOVE_REPLAY` 和 `UWB_ANCHOR_FAULT_REPLAY`。

证据入口使用明确命名的 `uwb_replay_test` 输出坐标系，禁用实时里程计、GNSS、AMCL
和scan-match输入，不发布 TF，也没有运动输出。`/dg/fusion/status` 必须出现
`fusion_mode=UWB_AIDED`、`accepted_source=UWB` 和
`output_frame_id=uwb_replay_test`。这证明软件消费链，不证明 UWB 到机器人地图的现场
对齐或定位精度。

## 每次结束后的现场检查

```bash
session=$(ls -dt results/real_robot/* | head -n 1)
cat "$session/result.json"
test -f "$session/rosbag/metadata.yaml"
test -f "$session/structured/capture_summary.json"
(cd "$session" && sha256sum -c manifest.sha256)
```

同时记录肉眼观察：机器人是否按预期动作、是否人工接管、是否碰撞/安全中止、UI和
手机是否掉线。任何失败trial都保留。

## 传回Windows

完成全部场景后，在 Windows PowerShell 执行：

```powershell
scp -r nvidia@<机器人当前IP>:/home/nvidia/dg202611_ws/results/real_robot "$env:USERPROFILE\Downloads\DG202611_real_robot_20260907"
```

传回后运行统一评价入口：

```powershell
python scripts/evaluate_real_robot_sessions.py `
  "$env:USERPROFILE\Downloads\DG202611_real_robot_20260907"
```

程序会逐个校验原始清单，在原始采集目录之外新建评价目录，汇总位置样本、MODEL-2
重复率和 MODEL-3 终态。原始 rosbag、CSV 和日志不会被覆盖。没有同坐标系真值、没有
有效特征或没有终态时，对应指标保持为空，不会自动填成通过。
