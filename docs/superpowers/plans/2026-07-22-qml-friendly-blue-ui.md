# QML Friendly Blue UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将现有 QML 界面改成浅蓝、现代、亲和的机器人助手风格，同时保持全部数据绑定和操作行为不变。

**Architecture:** 先用 `Theme.qml` 统一颜色与尺寸，再调整公共组件和主窗口，最后只修改各页面的视觉属性。所有 `backend.*` 调用、页面映射、业务条件和命令字符串保持原样。

**Tech Stack:** Qt Quick 2.12、Qt Quick Controls 2.12、Qt Quick Layouts 1.12、pytest 静态契约测试、colcon。

---

### Task 1: 全局主题、背景与导航

**Files:**
- Modify: `src/ylhb_llm/qml/Theme.qml`
- Modify: `src/ylhb_llm/qml/Main.qml`
- Modify: `src/ylhb_llm/qml/components/TopStatusBar.qml`
- Modify: `src/ylhb_llm/qml/components/WarmButton.qml`
- Modify: `src/ylhb_llm/qml/components/SafetyStopButton.qml`

- [ ] **Step 1: 记录功能基线**

Run: `git diff -- src/ylhb_llm/qml && grep -Rho 'backend\.[A-Za-z0-9_]*' src/ylhb_llm/qml | sort -u > /tmp/ylhb-qml-backend-before.txt`

Expected: QML 当前无未提交差异，并生成原有绑定清单。

- [ ] **Step 2: 替换全局主题值**

在 `Theme.qml` 中将背景、卡片、浅蓝、品牌蓝、辅助天蓝、主文字、辅助文字和边框设置为设计稿指定值；保留成功、警告、危险状态色与 `stateColor()` 行为。

- [ ] **Step 3: 调整主窗口视觉**

删除旧背景图片和白色蒙层，改为 `Theme.background` 加两个低透明度边角浅蓝圆形装饰；侧栏使用白色卡片背景和轻边框。保留 `pageSources`、`currentPage: 1`、Loader、关机对话框和所有调用不变。

- [ ] **Step 4: 强化导航与公共按钮层级**

导航选中态使用 `Theme.primarySoft` 与 `Theme.primary`；显示文案改为“巡检任务”和“AI 助手”。主按钮、紧急停止按钮和顶部状态改成统一 8px 圆角与清晰状态胶囊。

- [ ] **Step 5: 运行导航契约测试**

Run: `source /opt/ros/humble/setup.bash && PYTHONPATH=src/ylhb_llm:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ylhb_llm/test/test_qml_navigation.py`

Expected: 原有测试通过；如导航显示文案断言受影响，仅把断言更新为新显示名称，不改命令或绑定断言。

### Task 2: 公共卡片与核心页面视觉

**Files:**
- Modify: `src/ylhb_llm/qml/components/ConnectionCard.qml`
- Modify: `src/ylhb_llm/qml/components/ConnectionPath.qml`
- Modify: `src/ylhb_llm/qml/components/MetricTile.qml`
- Modify: `src/ylhb_llm/qml/components/RoutePreviewViewer.qml`
- Modify: `src/ylhb_llm/qml/components/StatusCard.qml`
- Modify: `src/ylhb_llm/qml/pages/PatrolPage.qml`
- Modify: `src/ylhb_llm/qml/pages/VoiceAiPage.qml`
- Modify: `src/ylhb_llm/qml/pages/BridgePage.qml`

- [ ] **Step 1: 统一公共卡片**

将卡片圆角统一为 8px，卡片背景使用 `Theme.surface`，次级块使用 `Theme.surfaceAlt`，边框使用 `Theme.border`；仅添加低透明度、低偏移的轻微阴影，不增加新组件或依赖。

- [ ] **Step 2: 调整巡检任务页**

页面外边距改为 24px、控件间距收敛至 8–12px；开始按钮保持品牌蓝，暂停按钮使用浅蓝次层级，继续按钮使用品牌蓝，结束按钮保持危险色。不得修改任何 `enabled`、`onClicked`、确认对话框或路线预览行为。

- [ ] **Step 3: 调整 AI 助手页**

删除页面内独立的灰青色硬编码主题，改为引用 `Theme`；统一卡片、状态块和执行记录颜色。保留 `safeMarkdown()`、语音状态计算、Agent 数据和步骤详情逻辑。

- [ ] **Step 4: 调整连接页**

只统一页面背景、卡片圆角、边框、间距和按钮颜色；保留两个 Switch、Binding 恢复模式、服务开关、诊断折叠和高级操作逻辑。

### Task 3: 次级页面统一与回归验证

**Files:**
- Modify: `src/ylhb_llm/qml/pages/Mapping3DPage.qml`
- Modify: `src/ylhb_llm/qml/pages/StatusPage.qml`
- Modify: `src/ylhb_llm/qml/pages/LogsPage.qml`
- Modify: `src/ylhb_llm/qml/pages/StartupLoadingPage.qml`
- Modify: `src/ylhb_llm/qml/pages/DashboardPage.qml`
- Modify: `src/ylhb_llm/qml/pages/ControlPage.qml`
- Modify: `src/ylhb_llm/qml/pages/MappingPage.qml`
- Test: `src/ylhb_llm/test/test_qml_navigation.py`

- [ ] **Step 1: 统一次级页面视觉**

把页面边距统一为 24px、圆角统一为 8px，并移除与新主题冲突的硬编码颜色。保留三维建模、状态、日志、建图和控制页面的所有现有调用与启用条件。

- [ ] **Step 2: 对比绑定清单**

Run: `grep -Rho 'backend\.[A-Za-z0-9_]*' src/ylhb_llm/qml | sort -u > /tmp/ylhb-qml-backend-after.txt && diff -u /tmp/ylhb-qml-backend-before.txt /tmp/ylhb-qml-backend-after.txt`

Expected: 无差异。

- [ ] **Step 3: 运行目标测试**

Run: `source /opt/ros/humble/setup.bash && PYTHONPATH=src/ylhb_llm:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ylhb_llm/test/test_qml_navigation.py src/ylhb_llm/test/test_ui_backend.py`

Expected: 全部通过。

- [ ] **Step 4: 构建目标包**

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select ylhb_llm`

Expected: `Summary: 1 package finished`。

- [ ] **Step 5: 检查最终差异**

Run: `git diff --check && git diff --stat && git status --short`

Expected: 无空白错误；只包含本次 QML 视觉修改、必要的显示文案测试调整，以及此前用户已有的 GPS 改动。
