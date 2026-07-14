# Dual Network Connection Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复机器人端身份/候选地址契约与移动端单一主备连接状态机，消除隐式急停、并发连接和错误切换。

**Architecture:** 机器人端在现有 Mobile Bridge 状态边界补充兼容字段并移除状态流断线停车。移动端新增一个可注入依赖的连接控制器，Store 仅订阅其状态并调用公开动作；HTTP 超时和错误分类位于统一请求层。

**Tech Stack:** Python/FastAPI/ROS2、React Native/Expo/TypeScript、AsyncStorage、pytest、TypeScript typecheck。

---

### Task 1: 机器人端状态契约

**Files:**
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/mobile_bridge_server.py`
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/network_status.py`
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/ros_bridge.py`
- Test: `src/ylhb_mobile_bridge/test/test_mobile_bridge_server.py`

- [ ] 在现有 status 测试中断言 `apiVersion`、稳定 `robotId`、相同应用内固定 `bridgeInstanceId` 和 `candidateEndpoints`。
- [ ] 运行指定测试，确认新断言先失败。
- [ ] 在 Bridge 应用创建时生成 `bridgeInstanceId = str(uuid.uuid4())`，稳定 ID 优先使用已有 `robot_id`，否则从 machine-id 生成 UUID5。
- [ ] 保留旧字段并新增候选地址字段，运行指定测试通过。

### Task 2: WebSocket 断线不得停车

**Files:**
- Modify: `src/ylhb_mobile_bridge/ylhb_mobile_bridge/mobile_bridge_server.py`
- Test: `src/ylhb_mobile_bridge/test/test_mobile_bridge_server.py`

- [ ] 增加 status/map WebSocket 断线后 `stop_motion` 调用数保持为零的两个用例。
- [ ] 删除两个状态流异常处理中的 `bridge.stop_motion()`，保留进程最终退出停车。
- [ ] 运行三个机器人端关键回归和相关现有测试。

### Task 3: 移动端统一 HTTP 超时

**Files:**
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/api/http.ts`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/api/robotApi.ts`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/api/types.ts`

- [ ] 用内部 AbortController 合并调用方 signal，并按探测 2000ms、普通请求 3000ms、急停 1500ms传参。
- [ ] 将 HTTP 错误归类为 `http_4xx`/`http_5xx` 并保存具体 status；业务信封错误保持独立，timeout/aborted/network_error 明确区分。
- [ ] 确保 finally 清理定时器，任何请求都不会遗留 pending。

### Task 4: 主备连接控制器与六项回归

**Files:**
- Create: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/network/RobotConnectionController.ts`
- Replace: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/network/endpointManager.ts`
- Create: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/network/RobotConnectionController.test.ts`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/package.json`

- [ ] 建立 `RobotConnectionConfig`、`ConnectionPhase`、独立 status/map generation 和 connection operation AbortController。
- [ ] 实现最近成功地址/主地址优先、仅网络失败探测备用、robotId 不同拒绝切换、写请求不重发。
- [ ] 实现显式急停并行主备、同 robotId 约束和 1500ms 单地址超时。
- [ ] 用六项测试覆盖：连接无 stop、主超时切备、旧连接结果丢弃、刷新周期不改地址/连接、不同 robotId 拒绝、地图错误非 connected。

### Task 5: Store、生命周期、设置页和持久化

**Files:**
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/store/robotStore.ts`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/app/_layout.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/app/(tabs)/index.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/app/status.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/app/settings.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/app/(tabs)/mapping.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/components/ControlPad.tsx`
- Modify: `/home/nvidia/PC_WJ/ylhb-robot-mobile/src/components/GlobalSafetyBar.tsx`

- [ ] 根布局初始化控制器一次，首页和状态页删除完整连接初始化。
- [ ] Store 删除任意 Endpoint CRUD，暴露保存配置、单独刷新周期、测试主/备、交换和恢复默认。
- [ ] AsyncStorage 保存配置与最近成功地址，旧 `baseUrl` 迁移后写入迁移标记；失败时使用默认值继续启动。
- [ ] 地图页面只启停地图流；设置页仅保留需求规定的主备操作。

### Task 6: 验证、提交和主分支发布

**Files:**
- Verify both repositories only.

- [ ] 移动端执行 `npm ci`、六项关键测试和 `npm run typecheck`，不运行 Gradle/Expo Android 构建。
- [ ] 机器人端执行相关 pytest、`colcon build --packages-select ylhb_mobile_bridge`，进行只读 localhost/真实网卡 status 与监听端口检查。
- [ ] 检查 `usesCleartextTraffic=true`、版本号、versionCode，整理 Windows release APK 构建命令和产物核验命令。
- [ ] 复核两个仓库 diff/status，分别提交修复；按用户明确要求合入并推送 `main`，禁止强推。
