# 项目展示与中文手册更新 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 README 变成展示优先、开发可达的项目入口，并将中文项目手册同步到当前 Robot Bridge、UI 与 Mobile Bridge 事实边界。

**Architecture:** README 仅承担项目封面、能力快照、阅读路径和开发导航；长篇现场步骤仍由 `src/电力巡检机器人使用与调试手册.md` 与 `docs/` 专项文档承载。两份文档复用已有实拍图片与链接，不新增运行代码或图片资产。

**Tech Stack:** GitHub Flavored Markdown、HTML 图片表格、Mermaid、仓库现有照片与文档链接。

---

### Task 1: 重组 README 展示封面与开发导航

**Files:**
- Modify: `README.md:1-300`
- Verify: `README.md` 内部链接、图片路径和重复段落

- [x] **Step 1: 保留 Hero 与徽章，替换摘要和导航为展示入口**

  在标题下保留现有实拍 Hero 和技术徽章；将摘要替换为“实机硬件、导航巡逻、感知、
  Agent、QML 操控台与移动端桥接”的一句话定位，并增加以下链接组：

  ```markdown
  [查看实机与能力](#能力快照) · [部署到 Jetson](#快速开始) ·
  [扩展与调试](#开发者导航) · [完整中文手册](src/电力巡检机器人使用与调试手册.md)
  ```

- [x] **Step 2: 增加能力快照和三条阅读路径**

  在实机照片前写入三栏事实表：

  ```markdown
  | 已具备 | 已验证/可复现 | 后续业务接线 |
  |---|---|---|
  | ROS 2 硬件接入、Nav2 路线巡逻、ZED/TensorRT、QML UI、Mobile Bridge | 构建、地图/路线校验、按现场安全流程的实机验收入口 | Spring Bridge、Web/小程序正式业务接线、检测服务、告警与报告 |
  ```

  紧随其后增加“项目展示”“Jetson 部署”“开发扩展”三项阅读路径，分别链接到实机
  展示、快速开始、开发者导航和专项文档。

- [x] **Step 3: 压缩重复文字并补全开发者导航**

  保留现有“实机展示”“项目亮点”“系统架构”“快速开始”和安全地图章节；删除
  “本仓库用于机器人研发、联调与实验验证”重复句。新增“开发者导航”表，列出：
  `ylhb_base`、`ylhb_mobile_bridge`、`ylhb_llm`、`ylhb_perception`、
  `ylhb_3d_mapping`、路线文件、`docs/`，每项只写职责和链接。

- [x] **Step 4: 按主题重排文档门户**

  将现有“项目文档”分为“现场操作”“平台连接”“路线与安全”“三维与 AI”“硬件资料”
  五组。Robot Platform Protocol 和云平台运维文档必须保留；明确 heartbeat 基础设施已
  存在，但 Spring Bridge 与 Web/小程序正式业务接线尚未完成。

### Task 2: 同步中文项目手册的当前能力与运维入口

**Files:**
- Modify: `src/电力巡检机器人使用与调试手册.md:1-90`, `src/电力巡检机器人使用与调试手册.md:1400-1765`
- Verify: 手册中的启动、安全与 bridge 描述与 README、`docs/本体QML操控台.md`、`docs/云平台连接运维.md` 一致

- [x] **Step 1: 在手册开头增加能力快照和按角色阅读**

  在“项目定位与交付”前增加：

  ```markdown
  > 安全提示：bringup、navigation、巡逻和底盘命令可能使机器人运动；现场操作员确认场地、急停和设备状态后执行。
  ```

  并加入三项索引：现场验收（硬件/导航/巡逻章节）、机器人开发（包职责与数据流）、
  平台联调（Mobile Bridge、云平台运维和协议文档）。

- [x] **Step 2: 修正项目边界的事实表述**

  在“项目定位与交付”和“初始化框架边界”中明确：Robot/Mobile Bridge 的公网 heartbeat、
  部署配置和协议文档已经具备；Spring Bridge 与 Web/小程序正式业务接线仍属于后续工作。
  不将静态测试、Offscreen UI 或协议文档表述为现场机器人功能验收。

- [x] **Step 3: 补充 UI 生命周期和 Mobile Bridge 所有权索引**

  在 AI/UI 与 Mobile Bridge 章节增加简明说明：

  ```text
  inspection_display_ui_node 退出
    -> OnProcessExit
    -> Shutdown 完整 inspection 栈
  ```

  同时说明 `YLHB_MOBILE_BRIDGE_OWNER=auto` 统一决定 systemd 或 Supervisor 所有权；
  仅 Supervisor 自己启动的内部 bridge 随完整栈关闭，外部 systemd bridge 不由 UI 停止。
  将详情链接到 `docs/本体QML操控台.md` 和 `docs/云平台连接运维.md`。

### Task 3: 文档质量校验

**Files:**
- Verify: `README.md`, `src/电力巡检机器人使用与调试手册.md`

- [x] **Step 1: 验证 README 图片与相对 Markdown 链接**

  运行下列只读检查；预期不存在 `MISSING`：

  ```bash
  python3 - <<'PY'
  import re
  from pathlib import Path
  for name in ("README.md", "src/电力巡检机器人使用与调试手册.md"):
      text = Path(name).read_text(encoding="utf-8")
      for target in re.findall(r'!?(?:\[[^]]*\])\(([^)#]+)', text):
          if target.startswith(("http://", "https://", "mailto:")):
              continue
          path = (Path(name).parent / target).resolve()
          if not path.exists():
              print("MISSING", name, target)
  PY
  ```

- [x] **Step 2: 检查展示文案和格式**

  运行：

  ```bash
  git diff --check
  grep -n "本仓库用于机器人研发、联调与实验验证。" README.md
  ```

  预期 `git diff --check` 成功，第二条命令只出现一次。

- [x] **Step 3: 审阅最终 diff，不提交或推送**

  运行：

  ```bash
  git diff -- README.md src/电力巡检机器人使用与调试手册.md
  git status --short
  ```

  保留工作区改动供用户审阅；只有用户明确要求时才创建后续提交和推送。
