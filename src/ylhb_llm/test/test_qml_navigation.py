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
    assert "backend.patrolProgressLabel" in qml
    assert "backend.patrolOverviewProgressLabel" in qml
    assert "routePreviewImageSource" in qml
    assert "Image.status" in qml
    assert "image_valid" in qml
    assert "image_error" in qml
    assert "Nav2 Action" not in qml
    assert "backend.patrolStateLabel" in qml


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
    assert 'backend.sendSystemCommand("stop_robot_stack")' in qml
    assert 'backend.sendSystemCommand("reload_patrol_route")' in qml
    assert 'backend.sendSystemCommand("stop_navigation")' in qml
    assert 'backend.sendSystemCommand("stop_bringup")' in qml
    assert 'backend.sendPatrolCommand("pause")' not in qml
    assert 'backend.sendPatrolCommand("resume")' not in qml
    assert 'backend.sendPatrolCommand("cancel")' not in qml
    assert 'backend.sendPatrolCommand("reload")' not in qml
    assert "一键启动巡逻模式" in qml
    assert "enabled: backend.patrolCanStart" in qml
    assert "!root.patrolStarting && !root.patrolRunning && backend.routePreviewOk" not in qml
    assert 'backend.patrolModeState === "running"' not in qml
    assert 'backend.patrolModeState === "command_sent"' not in qml
    assert 'property bool patrolRunning' not in qml
    assert 'property bool patrolCommandSent' not in qml
    assert 'backend.patrolMainStatusLabel' in qml
    assert 'backend.patrolCanPause' in qml
    assert 'backend.patrolCanResume' in qml
    assert 'backend.patrolCanCancel' in qml
    controls = qml.split('Label { text: "主控制"', 1)[1].split('Label { text: "路线预览"', 1)[0]
    assert 'backend.sendSystemCommand("stop_navigation")' not in controls
    assert 'backend.sendSystemCommand("stop_bringup")' not in controls
    assert 'backend.sendSystemCommand("reload_patrol_route")' not in controls
    stop_button = qml.split('backend.sendSystemCommand("stop_robot_stack")', 1)[0].rsplit('WarmButton', 1)[1]
    assert 'root.patrolCommandSent' not in stop_button
    assert 'root.navigationActive' not in stop_button


def test_patrol_page_keeps_advanced_controls_and_lists_collapsed():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    advanced = qml.split('Label { text: "高级/诊断"', 1)[1]
    assert 'backend.sendSystemCommand("stop_navigation")' in advanced
    assert 'backend.sendSystemCommand("stop_bringup")' in advanced
    assert 'backend.sendSystemCommand("reload_patrol_route")' in advanced
    assert 'backend.refreshRoutePreview()' in advanced
    assert 'property bool advancedVisible: false' in qml
    assert 'property bool tasksVisible: false' in qml
    assert 'property bool eventsVisible: false' in qml
    assert 'visible: root.advancedVisible' in qml
    assert 'visible: root.tasksVisible' in qml
    assert 'visible: root.eventsVisible' in qml


def test_patrol_page_shows_known_and_unknown_startup_steps_and_collapses_diagnostics():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert '"waiting_map_to_odom"' in qml
    assert '"waiting_nav2_active"' in qml
    assert '"waiting_executor_response"' in qml
    assert '"patrol_failed"' in qml
    assert 'backend.systemStatus.startup_step_label' in qml
    assert 'property bool diagnosticsVisible: false' in qml
    assert 'checked: root.diagnosticsVisible' in qml
    assert 'visible: root.diagnosticsVisible' in qml


def test_voice_ai_page_sends_text_to_language_agent():
    qml = Path("src/ylhb_llm/qml/pages/VoiceAiPage.qml").read_text(encoding="utf-8")

    assert "发送到语言 Agent" in qml
    assert "backend.sendAgentText(commandText.text)" in qml
    assert "backend.sendTextCommand(commandText.text)" not in qml
    assert "backend.voiceActivityText" in qml
    assert "backend.voiceActivityTone" in qml
