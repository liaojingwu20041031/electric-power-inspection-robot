from pathlib import Path


def test_main_navigation_uses_patrol_and_voice_pages_not_control_or_mapping():
    qml = Path("src/ylhb_llm/qml/Main.qml").read_text(encoding="utf-8")

    assert "pages/PatrolPage.qml" in qml
    assert "pages/VoiceAiPage.qml" in qml
    assert "ControlPage.qml" not in qml
    assert "MappingPage.qml" not in qml


def test_patrol_page_binds_preview_image_without_showing_url_as_main_text():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert "property string previewImageSource" in qml
    assert "source: root.previewImageSource" in qml
    assert "?v=" not in qml
    assert "routePreviewImage.source =" not in qml
    assert "text: backend.routePreviewImageUrl" not in qml
    assert "阶段流程" in qml
    assert "backend.patrolProgressLabel" in qml
    assert "诊断信息" in qml
    assert "routePreviewImageSource" in qml
    assert "Image.status" in qml


def test_patrol_page_sends_controls_to_patrol_command_topic():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert 'backend.sendPatrolCommand("pause")' in qml
    assert 'backend.sendPatrolCommand("resume")' in qml
    assert 'backend.sendPatrolCommand("cancel")' in qml
    assert 'backend.sendPatrolCommand("reload")' in qml
    assert 'backend.sendSystemCommand("pause_patrol")' not in qml
    assert 'backend.sendSystemCommand("resume_patrol")' not in qml
    assert 'backend.sendSystemCommand("cancel_patrol")' not in qml
    assert 'backend.sendSystemCommand("reload_patrol_route")' not in qml
