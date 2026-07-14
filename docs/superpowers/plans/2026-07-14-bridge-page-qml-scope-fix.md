# Bridge Page QML Scope Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除 BridgePage 云网络信息区域的未定义属性运行时错误。

**Architecture:** 保留现有数据与布局，仅为承载属性的 `ColumnLayout` 增加对象 ID，并通过该 ID 显式访问属性。

**Tech Stack:** QML、ROS 2 Humble、colcon

---

### Task 1: 修正 QML 属性作用域

**Files:**
- Modify: `src/ylhb_llm/qml/pages/BridgePage.qml:248`

- [x] **Step 1: 修正属性访问**

给云网络详情布局增加 `id: cloudRouteDetails`，并将子组件中的 `cloudEgress`、`alternateRoute` 引用分别改为 `cloudRouteDetails.cloudEgress`、`cloudRouteDetails.alternateRoute`。

- [x] **Step 2: 执行最低成本验证**

Run: `python3 -m pytest -q src/ylhb_llm/test/test_qml_navigation.py`

Expected: 相关 QML 测试通过，运行输出不再包含这两个未定义属性错误。

- [x] **Step 3: 构建相关包**

Run: `colcon build --packages-select ylhb_llm`

Expected: `ylhb_llm` 构建成功。
