from pathlib import Path


def test_main_navigation_uses_patrol_and_voice_pages_not_control_or_mapping():
    qml = Path("src/ylhb_llm/qml/Main.qml").read_text(encoding="utf-8")

    assert "property int currentPage: 1" in qml
    assert "StartupLoadingPage.qml" in qml
    assert "backend.uiReady" in qml
    assert "pages/PatrolPage.qml" in qml
    assert "连接与服务" in qml
    assert "pages/Mapping3DPage.qml" in qml
    assert "pages/VoiceAiPage.qml" in qml
    assert "三维建模" in qml
    assert "ControlPage.qml" not in qml
    assert "MappingPage.qml" not in qml


def test_status_page_shows_3d_mapping_process_card():
    qml = Path("src/ylhb_llm/qml/pages/StatusPage.qml").read_text(encoding="utf-8")

    assert '"3d_mapping"' in qml
    assert "三维建模" in qml
    assert "backend.mapping3dStatus.state" in qml
    assert "backend.mapping3dStateText" in qml
    assert "Theme.stateColor" in qml


def test_mapping3d_page_uses_backend_slots_not_json_commands():
    qml = Path("src/ylhb_llm/qml/pages/Mapping3DPage.qml").read_text(encoding="utf-8")

    assert "backend.start3dCapture()" in qml
    assert "backend.stop3dCapture()" in qml
    assert 'backend.reconstructLatest3dMap("fast_check")' in qml
    assert 'backend.reconstructLatest3dMap("quality_plus")' in qml
    assert "backend.mapping3dCanReconstruct" in qml
    assert "backend.latestSvoFile" in qml
    assert "backend.latestModelFile" in qml
    assert "zed_3d_map" in qml
    assert "/inspection_ai/mapping3d_pointcloud" in qml
    assert "资源管理" in qml
    assert "mapping3d_assets" in qml
    assert "backend.rename3dAsset" in qml
    assert "backend.delete3dAsset" in qml
    assert "backend.setLatest3dCapture" in qml
    assert "backend.reconstruct3dCapture" in qml
    assert "Theme.stateColor" in qml
    assert 'backend.sendSystemCommand("reconstruct' not in qml


def test_ui_bridge_connects_mapping3d_status_and_result():
    bridge = Path("src/ylhb_llm/ylhb_llm/ui_ros_bridge.py").read_text(encoding="utf-8")
    node = Path("src/ylhb_llm/ylhb_llm/inspection_display_ui_node.py").read_text(encoding="utf-8")

    assert "mapping3d_status_topic" in bridge
    assert "mapping3d_result_topic" in bridge
    assert "mapping3dStatus = pyqtSignal(dict)" in bridge
    assert "signals.mapping3dStatus.connect(backend.update_mapping3d_status)" in node
    assert "signals.mapping3dResult.connect(backend.update_mapping3d_result)" in node


def test_bridge_page_separates_local_and_cloud_controls():
    qml = Path("src/ylhb_llm/qml/pages/BridgePage.qml").read_text(encoding="utf-8")
    bridge = Path("src/ylhb_llm/ylhb_llm/ui_ros_bridge.py").read_text(encoding="utf-8")
    node = Path("src/ylhb_llm/ylhb_llm/inspection_display_ui_node.py").read_text(encoding="utf-8")

    assert "连接与服务" in qml
    assert "本地 APP 服务" in qml
    assert "云平台连接" in qml
    assert qml.count("Switch {") == 2
    assert "backend.setLocalAppEnabled" in qml
    assert "backend.setCloudEnabled" in qml
    assert 'target: localAppSwitch' in qml
    assert 'target: cloudSwitch' in qml
    assert 'localAppSwitch.checked =' not in qml
    assert 'cloudSwitch.checked =' not in qml
    assert "setCloudEnabled(local" not in qml
    assert "setLocalAppEnabled(cloud" not in qml
    assert "云平台连接和当前巡检不会受到影响" in qml
    assert "本地 APP 和当前巡检不会停止" in qml
    assert "ScrollBar.horizontal.policy: ScrollBar.AlwaysOff" in qml
    assert "mobile_bridge_managed_externally" in qml
    assert "高级服务操作" in qml
    assert 'visible: !backend.systemStatus.mobile_bridge_managed_externally' in qml
    assert 'visible: root.advancedExpanded && !backend.systemStatus.mobile_bridge_managed_externally' in qml
    assert 'backend.sendSystemCommand("restart_mobile_bridge")' in qml
    assert "cloud_status_topic" in bridge
    assert "set_cloud_enabled_service_name" in bridge
    assert "local_app_status_topic" in bridge
    assert "set_local_app_enabled_service_name" in bridge
    assert "localAppStatus = pyqtSignal(dict)" in bridge
    assert "localAppControlResult = pyqtSignal(bool, bool, str)" in bridge
    assert "cloudControlResult = pyqtSignal(bool, bool, str)" in bridge
    assert "signals.cloudStatus.connect(backend.update_cloud_status)" in node
    assert "signals.localAppStatus.connect(backend.update_local_app_status)" in node
    assert "signals.localAppControlResult.connect(backend.update_local_app_control_result)" in node
    assert "signals.cloudControlResult.connect(backend.update_cloud_control_result)" in node
    assert "cloudDisplayState" in qml
    assert "cloudRequestedEnabled" in qml
    assert "Math.min(parent.width - 40, 1540)" in qml
    assert "property real uiScale" in qml
    assert "visible: false" in qml.split("id: diagnosticBody", 1)[1]


def test_inspection_launch_keeps_ui_as_full_stack_lifecycle_anchor():
    launch = Path("src/ylhb_llm/launch/llm.launch.py").read_text(encoding="utf-8")
    autostart = Path("scripts/start_inspection_ui_autostart.sh").read_text(encoding="utf-8")
    assert "OnProcessExit(" in launch
    assert "EmitEvent(event=Shutdown(" in launch
    assert "respawn=True" not in launch
    assert 'run_on_jetson.sh" inspection' in autostart
    assert "crash-loop limit" in autostart


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
    assert "backend.routePreview.map_identity.image" not in qml
    assert "backend.routePreview.map_identity || ({})" in qml


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
    assert "property bool autoFit: true" in qml
    assert "property real minZoom: 0.05" in qml
    assert "property real maxZoom: 6.0" in qml
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
    assert 'backend.setRoutePreviewMode("route_focus")' in qml
    assert 'backend.setRoutePreviewMode("full_map")' in qml
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

    assert "发送给语言智能体" in qml
    assert "model: backend.agentMessages" in qml
    assert "backend.sendAgentText(commandText.text)" in qml
    assert "backend.clearAgentMessages()" in qml
    assert "checked: backend.agentDebugVisible" in qml
    assert "visible: backend.agentDebugVisible" in qml
    assert "backend.sendTextCommand(commandText.text)" not in qml
    assert "backend.voiceActivityText" in qml
    assert "backend.voiceActivityTone" in qml
    assert "backend.voiceTtsStatus" in qml
