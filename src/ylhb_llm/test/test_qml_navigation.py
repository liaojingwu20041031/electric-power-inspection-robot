import os
import shutil
import subprocess
import textwrap
import time
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
    assert "import QtQml 2.15" in qml
    assert qml.count("restoreMode: Binding.RestoreBindingOrValue") == 2
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
    assert "signals.bridgeAvailability.connect(backend.update_bridge_availability)" in node
    assert "cloudDisplayState" in qml
    assert "cloudRequestedEnabled" in qml
    assert "Math.min(parent.width - 40, 1540)" in qml
    assert "property real uiScale" in qml
    assert "visible: false" in qml.split("id: diagnosticBody", 1)[1]
    assert "coreUnavailable" not in qml
    assert 'objectName: "localAppSwitch"' in qml
    assert 'objectName: "cloudSwitch"' in qml
    assert "localAppControlAvailable" in qml
    assert "cloudControlAvailable" in qml
    assert "启动网桥核心服务" in qml
    assert "sudo systemctl restart ylhb-mobile-bridge.service" in qml
    assert "backend.localAppStatus.appEndpoints" in qml
    assert "Math.min(2, endpoints.length)" in qml
    assert "backend.localAppStatus.appUrl || backend.appUrl" in qml
    assert "readOnly: true" in qml
    assert "selectByMouse: true" in qml
    assert "backend.cloudStatus.cloudEgress" in qml
    assert "backend.cloudStatus.alternateCloudRoutes" in qml
    assert "networkMode" in qml
    assert "切换网卡" not in qml
    assert "停用网卡" not in qml
    assert "route metric" not in qml


def test_inspection_launch_keeps_ui_as_full_stack_lifecycle_anchor():
    launch = Path("src/ylhb_llm/launch/llm.launch.py").read_text(encoding="utf-8")
    autostart = Path("scripts/start_inspection_ui_autostart.sh").read_text(encoding="utf-8")
    qml = Path("src/ylhb_llm/qml/Main.qml").read_text(encoding="utf-8")
    assert "OnProcessExit(" in launch
    assert "EmitEvent(event=Shutdown(" in launch
    assert "respawn=True" not in launch
    assert 'run_on_jetson.sh" inspection' in autostart
    assert "crash-loop limit" in autostart
    assert "onClosing: function(close)" in qml
    assert "close.accepted = false" in qml
    assert "backend.requestInspectionShutdown()" in qml
    for name in (
        "ui_safe_margin_left",
        "ui_safe_margin_right",
        "ui_safe_margin_top",
        "ui_safe_margin_bottom",
    ):
        assert f"DeclareLaunchArgument('{name}'" in launch


def test_intentional_shutdown_marker_stops_restart_but_crashes_still_recover(tmp_path):
    workspace = tmp_path / "workspace"
    scripts = workspace / "scripts"
    fake_bin = tmp_path / "bin"
    runtime = tmp_path / "runtime"
    scripts.mkdir(parents=True)
    fake_bin.mkdir()
    runtime.mkdir()
    shutil.copy2("scripts/start_inspection_ui_autostart.sh", scripts)
    runner = scripts / "run_on_jetson.sh"
    runner.write_text(textwrap.dedent("""\
        #!/usr/bin/env bash
        count_file="${WS_DIR}/launch_count"
        count=0
        [ ! -f "${count_file}" ] || count="$(cat "${count_file}")"
        printf '%s\n' "$((count + 1))" >"${count_file}"
        if [ "${YLHB_TEST_INTENTIONAL:-false}" = true ]; then
          printf '%s\n' "${YLHB_INSPECTION_SESSION_ID}" >"${YLHB_INSPECTION_STOP_MARKER}"
        fi
        exit 1
    """), encoding="utf-8")
    runner.chmod(0o755)
    for name, body in {
        "pgrep": "#!/usr/bin/env bash\nexit 1\n",
        "sleep": "#!/usr/bin/env bash\nexit 0\n",
    }.items():
        path = fake_bin / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)

    env = os.environ.copy()
    env.update({
        "PATH": os.pathsep.join((str(fake_bin), env["PATH"])),
        "XDG_RUNTIME_DIR": str(runtime),
        "YLHB_UI_INHIBIT_IDLE": "false",
        "YLHB_INSPECTION_AUTO_RESTART": "true",
        "YLHB_TEST_INTENTIONAL": "true",
    })
    script = scripts / "start_inspection_ui_autostart.sh"
    intentional = subprocess.run([str(script)], env=env, text=True, timeout=10, check=False)
    assert intentional.returncode == 0
    assert (workspace / "launch_count").read_text(encoding="utf-8").strip() == "1"
    assert not list(runtime.glob("ylhb/inspection-stop-*"))

    (workspace / "launch_count").unlink()
    env["YLHB_TEST_INTENTIONAL"] = "false"
    crashed = subprocess.run([str(script)], env=env, text=True, timeout=10, check=False)
    assert crashed.returncode == 0
    assert (workspace / "launch_count").read_text(encoding="utf-8").strip() == "4"
    assert "systemctl" not in script.read_text(encoding="utf-8")


def test_ui_shutdown_request_is_idempotent_safe_and_marks_its_session(tmp_path):
    repo = Path.cwd()
    marker = tmp_path / "inspection-stop"
    script = textwrap.dedent(f"""
        import os, time
        from PyQt5.QtCore import QCoreApplication
        from ylhb_llm.ui_backend import UiBackend
        from ylhb_llm.ui_models import UiState

        class Bridge:
            def __init__(self): self.twists = []; self.commands = []
            def publish_twist(self, linear=0.0, angular=0.0): self.twists.append((linear, angular))
            def publish_system_command(self, command, **extra): self.commands.append(command)

        os.environ['YLHB_INSPECTION_SESSION_ID'] = 'session-123'
        os.environ['YLHB_INSPECTION_STOP_MARKER'] = {str(marker)!r}
        app = QCoreApplication([])
        bridge = Bridge()
        backend = UiBackend(bridge, UiState())
        backend.startup_timer.stop()
        emitted = []
        backend.shutdownRequested.connect(lambda: emitted.append(True))
        backend.requestInspectionShutdown()
        backend.requestInspectionShutdown()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not emitted:
            app.processEvents(); time.sleep(0.01)
        assert backend.shutdownPending is True
        assert bridge.twists == [(0.0, 0.0)]
        assert bridge.commands == ['emergency_stop']
        assert {str(marker)!r} and open({str(marker)!r}, encoding='utf-8').read().strip() == 'session-123'
        assert emitted == [True]
        assert any('正在关闭操控台' in item.get('message', '') for item in backend.logs)
    """)
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join((str(repo / 'src/ylhb_llm'), env.get('PYTHONPATH', '')))
    result = subprocess.run(
        ['/usr/bin/python3', '-c', script], cwd=repo, env=env,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    assert result.returncode == 0, result.stderr


def test_main_window_uses_configurable_bezel_safe_area_offscreen():
    repo = Path.cwd()
    qml = (repo / 'src/ylhb_llm/qml/Main.qml').read_text(encoding='utf-8')
    shutdown_dialog = qml.split('id: shutdownDialog', 1)[1]
    assert 'height: Math.min(safeArea.height - 40, 340)' in shutdown_dialog
    assert 'width: shutdownDialog.availableWidth' in shutdown_dialog
    script = textwrap.dedent(f"""
        from types import SimpleNamespace
        from PyQt5.QtCore import QPoint, QPointF, Qt, QUrl
        from PyQt5.QtGui import QGuiApplication
        from PyQt5.QtQml import QQmlComponent, QQmlEngine
        from PyQt5.QtQuick import QQuickItem
        from PyQt5.QtTest import QTest
        from ylhb_llm.ui_backend import UiBackend
        from ylhb_llm.ui_models import UiState

        class Bridge:
            values = {{
                'ui_safe_margin_left': 36,
                'ui_safe_margin_right': 34,
                'ui_safe_margin_top': 28,
                'ui_safe_margin_bottom': 32,
            }}
            def __init__(self): self.twists = []; self.commands = []
            def get_parameter(self, name): return SimpleNamespace(value=self.values[name])
            def publish_twist(self, *args, **kwargs): self.twists.append(args)
            def publish_system_command(self, command, **kwargs): self.commands.append(command)

        app = QGuiApplication([])
        bridge = Bridge()
        backend = UiBackend(bridge, UiState())
        backend.startup_timer.stop()
        invalid = Bridge()
        invalid.values = {{
            'ui_safe_margin_left': -5,
            'ui_safe_margin_right': 34,
            'ui_safe_margin_top': 200,
            'ui_safe_margin_bottom': 32,
        }}
        invalid_backend = UiBackend(invalid, UiState())
        invalid_backend.startup_timer.stop()
        assert invalid_backend.uiSafeMarginLeft == 0
        assert invalid_backend.uiSafeMarginTop == 120
        assert len(invalid_backend.logs) == 2
        engine = QQmlEngine()
        engine.rootContext().setContextProperty('backend', backend)
        component = QQmlComponent(engine, QUrl.fromLocalFile(str({str((repo / 'src/ylhb_llm/qml/Main.qml').resolve())!r})))
        while component.isLoading(): app.processEvents()
        window = component.create()
        assert window is not None, [str(error.toString()) for error in component.errors()]
        window.show()
        for width, height in ((1920, 1080), (960, 640)):
            window.setWidth(width); window.setHeight(height); app.processEvents()
            safe = window.findChild(QQuickItem, 'safeArea')
            sidebar = window.findChild(QQuickItem, 'sidebar')
            close_button = window.findChild(QQuickItem, 'closeConsoleButton')
            stop_button = window.findChild(QQuickItem, 'emergencyStopButton')
            assert safe and sidebar and close_button and stop_button
            assert round(safe.x()) == 36 and round(safe.y()) == 28
            assert round(safe.width()) == width - 36 - 34
            assert round(safe.height()) == height - 28 - 32
            for item in (sidebar, close_button, stop_button):
                top_left = item.mapToScene(QPointF(0, 0))
                bottom_right = item.mapToScene(QPointF(item.width(), item.height()))
                assert top_left.x() >= 36 and top_left.y() >= 28
                assert bottom_right.x() <= width - 34 and bottom_right.y() <= height - 32
            assert close_button.height() >= 48
            assert stop_button.height() >= 52
            point = close_button.mapToScene(QPointF(close_button.width() / 2, close_button.height() / 2))
            QTest.mouseClick(window, Qt.LeftButton, pos=QPoint(round(point.x()), round(point.y())))
            app.processEvents()
            cancel = window.findChild(QQuickItem, 'cancelShutdownButton')
            confirm = window.findChild(QQuickItem, 'confirmShutdownButton')
            assert cancel and confirm and cancel.property('visible') and confirm.property('visible')
            for item in (cancel, confirm):
                top_left = item.mapToScene(QPointF(0, 0))
                bottom_right = item.mapToScene(QPointF(item.width(), item.height()))
                assert top_left.x() >= 36 and top_left.y() >= 28
                assert bottom_right.x() <= width - 34 and bottom_right.y() <= height - 32
            point = cancel.mapToScene(QPointF(cancel.width() / 2, cancel.height() / 2))
            QTest.mouseClick(window, Qt.LeftButton, pos=QPoint(round(point.x()), round(point.y())))
            app.processEvents()
        point = stop_button.mapToScene(QPointF(stop_button.width() / 2, stop_button.height() / 2))
        QTest.mouseClick(window, Qt.LeftButton, pos=QPoint(round(point.x()), round(point.y())))
        app.processEvents()
        assert bridge.commands == ['emergency_stop']
        window.close()
    """)
    env = os.environ.copy()
    env['QT_QPA_PLATFORM'] = 'offscreen'
    env['QT_QUICK_BACKEND'] = 'software'
    env['PYTHONPATH'] = os.pathsep.join((str(repo / 'src/ylhb_llm'), env.get('PYTHONPATH', '')))
    result = subprocess.run(
        ['/usr/bin/python3', '-c', script], cwd=repo, env=env,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    assert result.returncode == 0, result.stderr
    assert 'Binding loop' not in result.stderr


def test_bridge_switches_receive_mouse_clicks_and_call_independent_services():
    repo = Path.cwd()
    script = textwrap.dedent(f"""
        import os
        from PyQt5.QtCore import QObject, QPoint, QPointF, Qt, QUrl
        from PyQt5.QtGui import QGuiApplication
        from PyQt5.QtQml import QQmlComponent, QQmlEngine
        from PyQt5.QtQuick import QQuickItem
        from PyQt5.QtTest import QTest
        from ylhb_llm.ui_backend import UiBackend
        from ylhb_llm.ui_models import UiState

        class Bridge:
            def __init__(self): self.cloud = []; self.local = []; self.system = []
            def call_cloud_enabled(self, enabled): self.cloud.append(enabled)
            def call_local_app_enabled(self, enabled): self.local.append(enabled)
            def publish_system_command(self, command, **extra): self.system.append(command)

        app = QGuiApplication([])
        bridge = Bridge()
        backend = UiBackend(bridge, UiState())
        backend.startup_timer.stop()
        backend.update_system_status({{'mobile_bridge_owner': 'supervisor', 'mobile_bridge_core_state': 'running'}})
        backend.update_local_app_status({{'enabled': False, 'state': 'DISABLED', 'httpAvailable': True}})
        backend.update_cloud_status({{'configured': True, 'desiredEnabled': False, 'connected': False, 'state': 'DISABLED'}})
        backend.update_bridge_availability({{'localAppServiceReady': True, 'cloudServiceReady': True, 'localAppStatusPublishers': 1, 'cloudStatusPublishers': 1}})
        engine = QQmlEngine()
        engine.rootContext().setContextProperty('backend', backend)
        component = QQmlComponent(engine)
        component.setData(b'''import QtQuick 2.12\n        import QtQuick.Window 2.12\n        import "file://{repo / 'src/ylhb_llm/qml/pages'}" as Pages\n        Window {{ width: 1280; height: 800; visible: true; Pages.BridgePage {{ anchors.fill: parent }} }}''', QUrl())
        while component.isLoading(): app.processEvents()
        window = component.create()
        assert window is not None, [str(error.toString()) for error in component.errors()]
        app.processEvents()
        for name in ('localAppSwitch', 'cloudSwitch'):
            switch = window.findChild(QQuickItem, name)
            assert switch is not None and switch.property('enabled')
            point = switch.mapToScene(QPointF(switch.width() / 2, switch.height() / 2))
            QTest.mouseClick(window, Qt.LeftButton, pos=QPoint(round(point.x()), round(point.y())))
            app.processEvents()
        assert bridge.local == [True]
        assert bridge.cloud == [True]
    """)
    env = os.environ.copy()
    env['QT_QPA_PLATFORM'] = 'offscreen'
    env['QT_QUICK_BACKEND'] = 'software'
    env['PYTHONPATH'] = os.pathsep.join((
        str(repo / 'src/ylhb_llm'),
        str(repo / 'src/ylhb_mobile_bridge'),
        env.get('PYTHONPATH', ''),
    ))
    result = subprocess.run(
        ['/usr/bin/python3', '-c', script], cwd=repo, env=env,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "Not restoring previous value" not in result.stderr


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
    assert "function scheduleFit()" in qml
    assert "interval: 100" in qml
    assert "onWidthChanged: scheduleFit()" in qml
    assert "onHeightChanged: scheduleFit()" in qml
    assert "onWidthChanged: { if (autoFit) fit() }" not in qml
    assert "onHeightChanged: { if (autoFit) fit() }" not in qml
    assert "if (routePreviewImage.status !== Image.Ready)" in qml
    assert "reset()\n            return" not in qml
    assert "sourceSize.width: 1600" in qml
    assert "拖动查看 · 双指缩放 · 点击适应恢复全图" in qml


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
    assert "启动巡逻任务" in qml
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
    assert 'id: startPatrolDialog' in qml
    assert 'id: stopPatrolDialog' in qml
    assert 'onClicked: startPatrolDialog.open()' in qml
    assert 'onClicked: stopPatrolDialog.open()' in qml
    assert qml.count('backend.startPatrolMode()') == 1
    assert qml.count('backend.sendSystemCommand("stop_robot_stack")') == 1
    assert 'onAccepted: backend.startPatrolMode()' in qml
    assert 'onAccepted: backend.sendSystemCommand("stop_robot_stack")' in qml


def test_patrol_page_keeps_advanced_controls_and_lists_collapsed():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert 'backend.sendSystemCommand("stop_navigation")' in qml
    assert 'backend.sendSystemCommand("stop_bringup")' in qml
    assert 'backend.sendSystemCommand("reload_patrol_route")' in qml
    assert 'backend.refreshRoutePreview()' in qml
    assert 'property bool detailsVisible: false' in qml
    assert 'property bool advancedVisible: false' in qml
    assert 'property bool tasksVisible: false' in qml
    assert 'property bool eventsVisible: false' in qml
    assert 'model: root.detailsVisible && root.diagnosticsVisible' in qml
    assert 'model: root.detailsVisible && root.tasksVisible' in qml
    assert 'model: root.detailsVisible && root.eventsVisible' in qml


def test_patrol_page_startup_stages_match_supervisor_flow_and_collapse_diagnostics():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    for step in (
        'starting_bringup',
        'starting_navigation',
        'navigation_process_spawned',
        'navigation_ready',
        'starting_executor',
        'executor_process_spawned',
        'executor_ready',
        'patrol_command_sent',
        'patrol_started',
        'patrol_failed',
    ):
        assert f'"{step}"' in qml
    for obsolete_step in (
        'waiting_after_bringup',
        'waiting_after_navigation',
        'waiting_after_executor',
        'waiting_nav2_active',
        'patrol_start_sent',
    ):
        assert f'"{obsolete_step}"' not in qml
    assert 'backend.systemStatus.startup_step_label' in qml
    assert 'property bool diagnosticsVisible: false' in qml
    assert 'checked: root.diagnosticsVisible' in qml
    assert 'visible: root.detailsVisible && root.diagnosticsVisible' in qml


def test_patrol_page_prioritizes_route_map_and_responsive_workspace():
    qml = Path("src/ylhb_llm/qml/pages/PatrolPage.qml").read_text(encoding="utf-8")

    assert 'objectName: "patrolPage"' in qml
    assert 'property bool wideLayout: root.availableWidth >= 1200' in qml
    assert 'property real contentMaxWidth: 1540' in qml
    assert 'width: Math.min(root.availableWidth - 40, root.contentMaxWidth)' in qml
    assert 'columns: 12' in qml
    assert 'Layout.columnSpan: root.wideLayout ? 8 : 12' in qml
    assert 'Layout.columnSpan: root.wideLayout ? 4 : 12' in qml
    assert 'property real mapPreferredHeight:' in qml
    assert 'ScrollBar.horizontal.policy: ScrollBar.AlwaysOff' in qml
    assert '路线地图' in qml
    assert '路线聚焦' in qml
    assert '完整地图' in qml
    assert '重绘预览' in qml
    assert '当前目标' in qml
    assert '总体进度' in qml
    assert '当前轮次' in qml
    assert '下一轮' in qml


def test_patrol_page_loads_responsively_and_confirms_start_and_stop():
    repo = Path.cwd()
    script = textwrap.dedent(f"""
        from PyQt5.QtCore import QPoint, QPointF, Qt, QUrl
        from PyQt5.QtGui import QGuiApplication
        from PyQt5.QtQml import QQmlComponent, QQmlEngine
        from PyQt5.QtQuick import QQuickItem
        from PyQt5.QtTest import QTest
        from ylhb_llm.ui_backend import UiBackend
        from ylhb_llm.ui_models import UiState

        class Bridge:
            def __init__(self): self.system = []
            def publish_system_command(self, command, **extra): self.system.append((command, extra))

        def click(window, item):
            point = item.mapToScene(QPointF(item.width() / 2, item.height() / 2))
            QTest.mouseClick(window, Qt.LeftButton, pos=QPoint(round(point.x()), round(point.y())))
            app.processEvents()

        app = QGuiApplication([])
        bridge = Bridge()
        state = UiState(route_preview={{'targets': [], 'safety_warnings': [], 'map_identity': {{}}}})
        backend = UiBackend(bridge, state, route_preview_loader=lambda **kwargs: {{'ok': False, 'targets': []}})
        backend.startup_timer.stop()
        engine = QQmlEngine()
        engine.rootContext().setContextProperty('backend', backend)
        component = QQmlComponent(engine)
        component.setData(b'''import QtQuick 2.12\\nimport QtQuick.Window 2.12\\nimport "file://{repo / 'src/ylhb_llm/qml/pages'}" as Pages\\nWindow {{ width: 1920; height: 1080; visible: true; Pages.PatrolPage {{ anchors.fill: parent }} }}''', QUrl())
        while component.isLoading(): app.processEvents()
        window = component.create()
        assert window is not None, [str(error.toString()) for error in component.errors()]
        app.processEvents()

        page = window.findChild(QQuickItem, 'patrolPage')
        assert page is not None
        heights = []
        for width, height in ((1920, 1080), (1280, 800), (960, 640)):
            window.setWidth(width); window.setHeight(height); app.processEvents()
            assert page.property('contentWidth') <= width
            heights.append(page.property('mapPreferredHeight'))
        assert heights[0] > heights[1] > heights[2]
        window.setWidth(1920); window.setHeight(1080); app.processEvents()

        start = window.findChild(QQuickItem, 'startPatrolButton')
        click(window, start)
        assert bridge.system == []
        cancel = window.findChild(QQuickItem, 'cancelStartPatrolButton')
        click(window, cancel)
        assert bridge.system == []
        click(window, start)
        confirm = window.findChild(QQuickItem, 'confirmStartPatrolButton')
        click(window, confirm)
        assert [item[0] for item in bridge.system] == ['start_patrol_mode']

        backend.update_system_status({{'patrol_executor': 'running', 'patrol_mode_state': 'running'}})
        backend.update_patrol_status({{'state': 'running'}})
        app.processEvents()
        stop = window.findChild(QQuickItem, 'stopPatrolButton')
        click(window, stop)
        assert [item[0] for item in bridge.system] == ['start_patrol_mode']
        cancel_stop = window.findChild(QQuickItem, 'cancelStopPatrolButton')
        click(window, cancel_stop)
        click(window, stop)
        confirm_stop = window.findChild(QQuickItem, 'confirmStopPatrolButton')
        click(window, confirm_stop)
        assert [item[0] for item in bridge.system] == ['start_patrol_mode', 'stop_robot_stack']
    """)
    env = os.environ.copy()
    env['QT_QPA_PLATFORM'] = 'offscreen'
    env['QT_QUICK_BACKEND'] = 'software'
    env['PYTHONPATH'] = os.pathsep.join((
        str(repo / 'src/ylhb_llm'),
        str(repo / 'src/ylhb_mobile_bridge'),
        env.get('PYTHONPATH', ''),
    ))
    result = subprocess.run(
        ['/usr/bin/python3', '-c', script], cwd=repo, env=env,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    assert result.returncode == 0, result.stderr


def test_voice_ai_page_is_lightweight_task_view():
    qml = Path("src/ylhb_llm/qml/pages/VoiceAiPage.qml").read_text(encoding="utf-8")

    assert "电力巡检机器人 AI Agent" in qml
    assert "当前目标" in qml
    assert "当前步骤" in qml
    assert "最近执行过程" in qml
    assert "最终结果" in qml
    assert "slice(-8)" in qml
    assert "Text.MarkdownText" in qml
    assert "查看详情" in qml
    assert "backend.callVoiceService" in qml
    assert "sendAgentText" not in qml
    assert "clearAgentMessages" not in qml
    assert "agentDebugVisible" not in qml
    assert "单次采集" not in qml
    assert "model: backend.agentMessages" not in qml
    assert "backend.voiceActivityText" in qml
    assert "backend.voiceActivityTone" in qml
