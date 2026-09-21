# 老师四项要求：采集与严格验收入口

## 目的

`scripts/teacher_requirements_evaluator.py` 只负责判断现有真实证据能否支持老师的四项客观指标。它不会把节点启动、仿真、空 CSV、缺少独立真值或人工补写的结果判为通过。

结果有四种：

- `PASS`：证据完整且指标达到；
- `FAIL`：证据完整，但数值没有达到；
- `NOT_PROVEN`：没有对应真实采集；
- `INCOMPLETE_EVIDENCE`：做过采集，但缺条件、真值、日志或样本量。

## 采集前冻结协议

每次正式采集从 `docs/dg202611/teacher_protocols/` 复制对应 JSON，填写实际方法后，在采集命令中加入：

```bash
./scripts/run_real_robot_capture.sh   --scenario <场景编号>   --duration <秒数>   --teacher-protocol /绝对路径/本次协议.json   <其他参数>
```

采集器会把协议内容和源文件 SHA256 写入 `metadata.json`，同时复制为会话内的 `teacher_protocol.json`。关键协议、原始 rosbag、结构化 CSV 和终端日志还必须被会话 `manifest.json` 的 SHA-256 清单覆盖。看完结果后不能改协议；任何修改都会破坏会话清单校验。

## 四项要求各自需要什么

### 1. 并发退化定位精度

必须同时具备：

- 会话必须保留可校验的原始 `rosbag/metadata.yaml`，不能只提交手工整理的 CSV；
- `GNSS/BDS` 与激光状态在同一个导航诊断样本中均为退化、拒绝、过期或失败；
- 协议明确记录两种退化并发；
- 采集器使用 `--ground-truth-x/y/z` 记录独立 XY/Z 真值；
- 真值方法和坐标系在协议中写清；
- 默认至少 30 个并发时间窗内样本；
- XY P95 和 Z P95 均小于 0.20 m。

### 2. 动态目标剔除后的特征重复率

必须同时具备：

- 机器人保持静止，现场存在真实动态目标；
- 动态过滤开启；
- `lidar_quality.csv` 确实出现动态候选或辅助动态剔除点；
- 使用同一激光坐标系中的二维特征匹配；
- 默认至少 30 个有效相邻帧对；
- 不允许存在被忽略的零特征帧对；
- 汇总 `R_pool >= 95%`。

### 3. 重定位成功率

必须同时具备：

- `VIOLENT_MOTION`、`CROSS_DOMAIN`、`PERCEPTION_DEGRADATION` 三类真实触发均覆盖；
- 每次为现场看护的受控实机试验；
- 每次都有唯一终态 `RECOVERED/FAILED/MANUAL_REQUIRED`；
- 默认至少 20 次有效尝试；
- 成功率严格大于 95%。

建议三类各做 20 次，共 60 次。这样 58/60 为 96.67%，能够超过 95%。失败试验不能删除。

### 4. 北斗短报文双向闭环

必须有真实短报文终端、卫星传输、已开通服务、厂商协议和远端接收设备。会话中还需保存机器人端与远端原始日志，以及：

```text
structured/beidou_short_message_events.csv
```

CSV 字段为：

```text
timestamp,test_id,event,message_id,peer,transport_reference,payload_sha256
```

同一个 `test_id` 必须按时间顺序出现：

```text
ROBOT_SEND_ACCEPTED
REMOTE_RECEIVED
REMOTE_REPLY_SENT
ROBOT_REPLY_RECEIVED
```

RTK、NTRIP 或 4G 不能替代这套证据。

## 统一验收

从机器人下载全部真实采集目录后，在 Windows 或 Linux 运行：

```bash
python scripts/teacher_requirements_evaluator.py <真实采集根目录>
```

程序会在原始目录外创建 `*_teacher_evaluation_*`，生成 `summary.json` 和中文 `summary.md`。返回码为 0 表示四项全部通过，返回码 3 表示仍有缺口或指标失败。

默认样本量和阈值可以通过命令行显式调整，但最终采用什么统计口径应在正式测试前与老师确认并冻结。
