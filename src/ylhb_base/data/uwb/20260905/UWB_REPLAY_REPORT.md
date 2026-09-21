# UWB真实数据回放报告

基站坐标采用 A2=(0,0), A3=(2.4,0), A4=(1.2,1.8)。原始数据未修改。

| 数据 | 帧数 | 接受 | 状态统计 | 估计输出 |
|---|---:|---:|---|---:|
| static_raw_20260905_165208 | 3000 | 2949 | GOOD 2949, REJECTED 21, RECOVERING 30 | 2979 |
| move_raw_20260905_170425 | 3000 | 2870 | GOOD 2869, DEGRADED 1, REJECTED 57, RECOVERING 73 | 2943 |

回放链路：CSV -> UwbRangeObservation -> UwbQualityGate -> 2D estimator。

限制：本批采集未记录独立真值坐标和每帧主机时间戳；设备文本包含 LO=[no solution]，因此本报告只证明真实测距数据可被算法解析、门控和估计，不证明绝对位置精度或机器人实时闭环。

文件：static/raw_20260905_165208.log、move/raw_20260905_170425.log、processed/*.csv。
