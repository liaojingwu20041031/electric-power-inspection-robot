# GitHub README Showcase Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将根 README 更新为蓝白工程科技风的项目展示页，并准确同步当前已实现的 QML、3D 资产上传和 GNSS 云状态能力。

**Architecture:** 仅修改根目录 `README.md`，复用仓库现有 Logo、实机照片、Markdown/HTML 与 Mermaid。项目能力按“展示概览 → 技术闭环 → 部署调试 → 安全边界”排列，不改变业务代码。

**Tech Stack:** GitHub Flavored Markdown、HTML、Mermaid、Shields.io

---

### Task 1: 重构 README 展示结构

**Files:**
- Modify: `README.md`

- [x] **Step 1: 更新品牌首屏**

  使用 `记录照片/机器人卡通形象LOGO.png` 作为居中 Logo，保留项目标题、定位、技术徽章和章节导航；采用蓝白配色，不引入脚本或外部主题。

- [x] **Step 2: 更新项目能力与架构**

  准确加入新版友好蓝 QML、GNSS heartbeat 和以下三维上传数据流：

  ```text
  SVO 采集 -> PLY 重建 -> scene asset ready -> SQLite/快照 -> HTTPS multipart -> 平台资产 ID
  ```

  明确 PLY 不写入 `maps/my_map.yaml`，平台审核与 Web 页面不属于本仓库已完成功能。

- [x] **Step 3: 整理部署、开发与安全信息**

  保留现有启动命令和专项文档链接，合并后半部分重复的安全地图、UI 自启动和平台运维说明。

### Task 2: 静态验证

**Files:**
- Verify: `README.md`

- [x] **Step 1: 检查本地链接和图片**

  解析 README 中相对路径，确认 Logo、实机照片、源码路径和文档入口存在。

- [x] **Step 2: 检查逻辑边界**

  对照 `scene_upload.py`、`ros_bridge.py`、`system_supervisor_node.py`、`Mapping3DPage.qml` 和 `Theme.qml`，确认状态、触发方式、坐标系边界与平台职责描述一致。

- [x] **Step 3: 检查 Markdown 差异**

  运行 `git diff --check -- README.md`，预期无空白错误；人工检查标题层级、代码块闭合和 Mermaid 节点。
