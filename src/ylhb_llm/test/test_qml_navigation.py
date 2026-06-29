from pathlib import Path


def test_main_navigation_uses_patrol_and_voice_pages_not_control_or_mapping():
    qml = Path("src/ylhb_llm/qml/Main.qml").read_text(encoding="utf-8")

    assert "property int currentPage: 1" in qml
    assert "pages/PatrolPage.qml" in qml
    assert "pages/VoiceAiPage.qml" in qml
    assert "ControlPage.qml" not in qml
    assert "MappingPage.qml" not in qml


def test_patrol_page_binds_preview_image_without_showing_url_as_main_text():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert "id: routePreviewPane" in qml
    assert "RoutePreviewViewer" in qml
    assert "source: backend.routePreviewImageSource" in qml
    assert "?v=" not in qml
    assert "routePreviewImage.source =" not in qml
    assert "Image {" not in qml.split("id: routePreviewPane", 1)[1].split("ColumnLayout {", 1)[0]
    assert "text: backend.routePreviewImageUrl" not in qml
    assert "阶段流程" in qml
    assert "backend.patrolProgressLabel" in qml
    assert "诊断信息" in qml
    assert "routePreviewImageSource" in qml
    assert "Image.status" in qml
    assert "image_valid" in qml
    assert "image_error" in qml
    assert "Nav2 Action" not in qml
    assert "等待 Nav2 导航服务启动完成。" in qml
    assert "导航目标被拒绝，正在重试。" in qml
    assert "等待底盘稳定" in qml
    assert "等待导航稳定" in qml
    assert "等待执行器发布初始位姿" in qml
    assert "发送巡逻 start" in qml
    assert "当前按手动启动流程执行，导航启动后会等待约 20 秒，请不要重复点击。" in qml
    assert "function patrolStateLabel()" in qml
    assert "就绪: 可启动巡逻" in qml
    assert "待命: 等待巡逻依赖" in qml
    assert "GridLayout" in qml
    assert "完成" in qml
    assert "当前" in qml


def test_route_preview_viewer_has_zoom_pan_and_error_controls():
    qml = Path("src/ylhb_llm/qml/components/RoutePreviewViewer.qml").read_text(encoding="utf-8")

    assert "function zoomIn()" in qml
    assert "function zoomOut()" in qml
    assert "function reset()" in qml
    assert "function fit()" in qml
    assert "MouseArea" in qml
    assert "onWheel" in qml
    assert "onPositionChanged" in qml
    assert "PinchArea" in qml
    assert "scale: root.zoom" in qml
    assert "smooth: !root.dragging" in qml
    assert "cache: true" in qml
    assert "路线预览图解码失败" in qml


def test_patrol_page_sends_controls_to_supervisor():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert 'backend.startPatrolMode()' in qml
    assert 'backend.sendSystemCommand("pause_patrol")' in qml
    assert 'backend.sendSystemCommand("resume_patrol")' in qml
    assert 'backend.sendSystemCommand("cancel_patrol")' in qml
    assert 'backend.sendSystemCommand("reload_patrol_route")' in qml
    assert 'backend.sendPatrolCommand("pause")' not in qml
    assert 'backend.sendPatrolCommand("resume")' not in qml
    assert 'backend.sendPatrolCommand("cancel")' not in qml
    assert 'backend.sendPatrolCommand("reload")' not in qml
    assert "一键启动巡逻模式" in qml
    assert "!root.patrolCommandSent && !root.patrolRunning" in qml
    assert "!root.patrolStarting && !root.patrolRunning && backend.routePreviewOk" not in qml
    assert 'backend.patrolModeState === "running"' not in qml
    assert 'backend.patrolModeState === "command_sent"' in qml
    assert 'root.patrolCommandSent' not in qml.split('text: "取消巡逻"', 1)[1].split('onClicked:', 1)[0]
    assert 'root.navigationActive' in qml.split('text: "取消巡逻"', 1)[1].split('onClicked:', 1)[0]
