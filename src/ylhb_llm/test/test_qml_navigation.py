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
    assert "source: backend.routePreviewImageSource" in qml
    assert "?v=" not in qml
    assert "routePreviewImage.source =" not in qml
    assert "asynchronous: true" in qml
    assert "sourceSize.width: 1600" in qml
    assert "routePreviewImage.status === Image.Ready" in qml
    assert "text: backend.routePreviewImageUrl" not in qml
    assert "阶段流程" in qml
    assert "backend.patrolProgressLabel" in qml
    assert "诊断信息" in qml
    assert "routePreviewImageSource" in qml
    assert "Image.status" in qml
    assert "image_valid" in qml
    assert "image_error" in qml
    assert "Nav2 Action" not in qml
    assert "Nav2 动作服务" not in qml


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
