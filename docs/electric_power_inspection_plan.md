# 电力行业巡检机器人迁移计划

本文档记录从 `ylhb-smart-retail-robot` 迁移到电力行业巡检机器人的第一版方案。当前阶段优先保证机器人端闭环可跑，再逐步重构后台和业务状态机。

## 一句话方案

机器人沿规划路线自主巡检，路线中持续做安全检测，到达检查点后语音提示、调整相机/云台、采集图像或视频，调用检测服务识别设备状态和异常，后台实时显示告警并生成巡检记录。

## 范围拆分

| 层级 | 内容 | 第一阶段做法 |
|---|---|---|
| 底层运动 | 底盘、雷达、IMU、定位、导航、避障 | 复用 `ylhb_base` |
| 感知输入 | ZED 2i 图像、深度、相机信息 | 复用 `ylhb_perception` |
| 路线级检测 | 人员、安全帽、火源、烟雾、障碍物 | 先用 YOLO/TensorRT，后续接 LocateAnything 辅助复核 |
| 检查点级检测 | 开关/刀闸、表计/指示灯、漏油、异物、烟火 | 到点采集关键帧，调用 LocateAnything 或专项检测服务 |
| 任务层 | 巡检任务、检查点状态、暂停/恢复/取消/人工接管 | 新增巡检状态机，逐步替换旧零售任务状态机 |
| 后台/UI | 路线规划、任务下发、告警、巡检记录 | 先改现有 PyQt UI，后续可拆成 Web 前端 |
| 三维空间 | LingBot-Map 三维重建和点位对齐 | 作为演示增强项，不进入 P0 实时闭环 |

## 巡检任务数据模型草案

```yaml
site:
  id: substation_demo
  name: 实训变电站
areas:
  - id: switch_area
    name: 开关柜区
routes:
  - id: route_main
    name: 主巡检路线
    checkpoints:
      - id: cp_001
        name: 1号开关柜
        pose: {x: 1.2, y: 0.8, yaw: 1.57}
        camera_pose: {pan: 0.0, tilt: -15.0, zoom: 1.0}
        inspection_items:
          - switch_state
          - indicator_light
          - smoke_fire
      - id: cp_002
        name: 变压器油位观察点
        pose: {x: 3.6, y: 1.4, yaw: 0.0}
        camera_pose: {pan: 12.0, tilt: -10.0, zoom: 1.0}
        inspection_items:
          - oil_leak
          - foreign_object
```

## ROS 任务接口草案

第一阶段可以沿用现有自定义消息，先把 `TaskEvent.raw_json` 承载巡检 JSON；稳定后再新增专用消息。

```json
{
  "task_type": "inspection_route",
  "task_id": "inspection_20260619_001",
  "route_id": "route_main",
  "command": "start",
  "checkpoints": [
    {
      "checkpoint_id": "cp_001",
      "name": "1号开关柜",
      "pose": {"x": 1.2, "y": 0.8, "yaw": 1.57},
      "items": ["switch_state", "indicator_light", "smoke_fire"]
    }
  ]
}
```

推荐后续新增话题：

```text
/inspection/task_command       # 后台下发任务、暂停、恢复、取消、人工接管
/inspection/task_status        # 机器人回传任务状态
/inspection/checkpoint_result  # 检查点检测结果
/inspection/alarm              # 路线级或检查点级告警
/inspection/record             # 巡检记录摘要
```

## 机器人状态机草案

```text
IDLE
  -> TASK_RECEIVED
  -> NAVIGATING
  -> ROUTE_MONITORING
  -> ARRIVED_CHECKPOINT
  -> SPEAKING_ARRIVAL
  -> CAMERA_ADJUSTING
  -> CAPTURING
  -> DETECTING
  -> RESULT_REPORTING
  -> NAVIGATING_NEXT
  -> FINISHED

任意状态可进入：
  PAUSED
  MANUAL_TAKEOVER
  CANCELLED
  FAULT
```

## LocateAnything 接入边界

LocateAnything-3B 更适合检查点级“复杂目标定位”，不建议替代所有实时检测：

| 场景 | 是否适合 |
|---|---|
| 到点后识别刀闸开合状态 | 适合 |
| 到点后定位漏油区域 | 适合 |
| 到点后找异物/烟火 | 适合 |
| 路线中实时检测行人和障碍物 | 可辅助，不建议主路径依赖 |
| 实时闭环避障 | 不适合，仍应交给 Nav2/雷达/深度避障 |

## LingBot-Map 接入边界

LingBot-Map 适合作为三维空间感知和展示模块：

```text
机器人巡检视频或图片序列
  -> LingBot-Map 流式/离线三维重建
  -> 点云、相机轨迹、关键帧
  -> 巡检点位对齐
  -> 后台三维可视化
```

第一阶段不要把 LingBot-Map 放进机器人运动控制闭环，避免算力和稳定性风险。

## 第一阶段 TODO

- [x] 克隆旧仓库到新 GitHub 仓库
- [x] 切换 README 和项目身份
- [x] 新增电力巡检迁移计划
- [ ] 改 UI 首页任务文案为巡检任务
- [ ] 新增巡检检查点配置文件
- [ ] 新增到点语音提示和检查点采集动作
- [ ] 新增巡检结果/告警 JSON 输出
- [ ] 接入第一版安全帽/人员/火源检测模型
- [ ] 接入 LocateAnything 检查点级检测服务
