import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    id: root
    objectName: "patrolPage"
    clip: true
    contentWidth: availableWidth
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

    property bool wideLayout: root.availableWidth >= 1200
    property real contentMaxWidth: 1540
    property real mapPreferredHeight: root.availableWidth >= 1500 ? 560 : (root.availableWidth >= 1100 ? 470 : 360)
    property bool detailsVisible: false
    property bool advancedVisible: false
    property bool diagnosticsVisible: false
    property bool tasksVisible: false
    property bool eventsVisible: false
    property bool inspectionProfile: backend.patrolStartProfile === "inspection"
    property var readiness: backend.systemStatus.patrol_readiness || ({})
    property var previewMap: backend.routePreview.map_identity || ({})
    property int startupStageIndex: stageIndex(backend.systemStatus.startup_step || "")
    property var readinessItems: [
        { "label": "底盘", "key": "bringup" },
        { "label": "导航", "key": "navigation" },
        { "label": "执行器", "key": "executor" },
        { "label": "路线文件", "key": "route_file" }
    ]
    property var startupStages: [
        { "label": "启动底盘", "step": "starting_bringup" },
        { "label": "启动导航", "step": "starting_navigation" },
        { "label": "导航进程已创建", "step": "navigation_process_spawned" },
        { "label": "导航已就绪", "step": "navigation_ready" },
        { "label": "启动巡逻执行器", "step": "starting_executor" },
        { "label": "执行器进程已创建", "step": "executor_process_spawned" },
        { "label": "执行器已就绪", "step": "executor_ready" },
        { "label": "巡逻命令已发送", "step": "patrol_command_sent" },
        { "label": "巡逻运行", "step": "patrol_started" },
        { "label": "巡逻启动失败", "step": "patrol_failed" }
    ]

    function stageIndex(step) {
        for (var i = 0; i < startupStages.length; i++) {
            if (startupStages[i].step === step) return i
        }
        return -1
    }

    function stageMark(index) {
        if (backend.patrolActive && root.startupStages[index].step === "patrol_started") return "当前"
        if (root.startupStageIndex < 0) return "等待"
        if (index < root.startupStageIndex) return "完成"
        return index === root.startupStageIndex ? "当前" : "等待"
    }

    function patrolStateColor() {
        if (backend.patrolError.length > 0 || backend.patrolStatus.state === "failed") return Theme.danger
        if (backend.patrolActive || backend.patrolStatus.state === "succeeded") return Theme.success
        return backend.patrolStarting ? Theme.info : Theme.warning
    }

    ColumnLayout {
        x: Math.max(20, (root.availableWidth - width) / 2)
        width: Math.min(root.availableWidth - 40, root.contentMaxWidth)
        spacing: 18

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 112
            radius: 14
            color: Theme.surface
            border.color: Theme.border

            RowLayout {
                anchors.fill: parent
                anchors.margins: 22
                spacing: 18

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 7
                    Label { text: "巡逻任务"; color: Theme.text; font.pixelSize: 30; font.bold: true }
                    Label {
                        Layout.fillWidth: true
                        text: (backend.routePreview.route_name || backend.routePreview.active_route_id || "路线未加载")
                            + "  ·  " + (backend.patrolMainStatusLabel || backend.patrolStateLabel)
                            + "  ·  " + (root.inspectionProfile ? "感知巡检" : "导航巡逻")
                        color: Theme.muted
                        font.pixelSize: 15
                        elide: Text.ElideRight
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 168
                    Layout.preferredHeight: 48
                    radius: 24
                    color: Qt.rgba(root.patrolStateColor().r, root.patrolStateColor().g, root.patrolStateColor().b, 0.12)
                    border.color: root.patrolStateColor()
                    Row {
                        anchors.centerIn: parent
                        spacing: 9
                        Rectangle { width: 10; height: 10; radius: 5; color: root.patrolStateColor(); anchors.verticalCenter: parent.verticalCenter }
                        Label {
                            text: backend.patrolMainStatusLabel || backend.patrolStateLabel
                            color: root.patrolStateColor()
                            font.pixelSize: 16
                            font.bold: true
                        }
                    }
                }
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 12
            columnSpacing: 16
            rowSpacing: 16

            Rectangle {
                Layout.columnSpan: root.wideLayout ? 8 : 12
                Layout.fillWidth: true
                Layout.preferredHeight: root.mapPreferredHeight + 128
                radius: 14
                color: Theme.surface
                border.color: Theme.border

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 12

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 10
                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            Label { text: "路线地图"; color: Theme.text; font.pixelSize: 20; font.bold: true }
                            Label {
                                Layout.fillWidth: true
                                text: backend.routePreview.route_name || backend.routePreview.active_route_id || "等待路线"
                                color: Theme.muted
                                font.pixelSize: 13
                                elide: Text.ElideRight
                            }
                        }
                        Button {
                            text: "路线聚焦"
                            checkable: true
                            checked: backend.routePreviewMode === "route_focus"
                            implicitHeight: 42
                            onClicked: backend.setRoutePreviewMode("route_focus")
                        }
                        Button {
                            text: "完整地图"
                            checkable: true
                            checked: backend.routePreviewMode === "full_map"
                            implicitHeight: 42
                            onClicked: backend.setRoutePreviewMode("full_map")
                        }
                        Button {
                            text: "重绘预览"
                            implicitHeight: 42
                            onClicked: backend.refreshRoutePreview()
                        }
                    }

                    RoutePreviewViewer {
                        id: routePreviewPane
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.preferredHeight: root.mapPreferredHeight
                        source: backend.routePreviewImageSource
                        previewOk: backend.routePreviewOk
                        loading: backend.routePreviewLoading
                        message: !backend.routePreviewOk
                            ? backend.routePreviewMessage
                            : (backend.routePreview.image_exists !== true ? "路线预览图文件不存在" : "路线预览图未生成")
                        onRetryRequested: backend.refreshRoutePreview()
                    }

                    Flow {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Math.max(42, childrenRect.height)
                        spacing: 8
                        Rectangle {
                            width: mapLabel.implicitWidth + 20; height: 28; radius: 14; color: Theme.surfaceAlt; border.color: Theme.border
                            Label { id: mapLabel; anchors.centerIn: parent; text: "地图 " + (root.previewMap.image || "-"); color: Theme.muted; font.pixelSize: 12 }
                        }
                        Rectangle {
                            width: resolutionLabel.implicitWidth + 20; height: 28; radius: 14; color: Theme.surfaceAlt; border.color: Theme.border
                            Label { id: resolutionLabel; anchors.centerIn: parent; text: "分辨率 " + String(backend.routePreview.map_resolution || "-") + " m"; color: Theme.muted; font.pixelSize: 12 }
                        }
                        Rectangle {
                            width: targetLabel.implicitWidth + 20; height: 28; radius: 14; color: Theme.surfaceAlt; border.color: Theme.border
                            Label { id: targetLabel; anchors.centerIn: parent; text: "检查点 " + String(backend.routePreview.target_count || 0); color: Theme.muted; font.pixelSize: 12 }
                        }
                        Rectangle {
                            width: keepoutLabel.implicitWidth + 20; height: 28; radius: 14; color: Theme.surfaceAlt; border.color: Theme.border
                            Label { id: keepoutLabel; anchors.centerIn: parent; text: "禁行区 " + String(backend.routePreview.keepout_count || 0); color: Theme.muted; font.pixelSize: 12 }
                        }
                        Button {
                            height: 42
                            text: (backend.routePreview.safety_warnings || []).length > 0
                                ? String((backend.routePreview.safety_warnings || []).length) + " 条安全提示"
                                : "路线校验通过"
                            palette.buttonText: (backend.routePreview.safety_warnings || []).length > 0 ? Theme.warning : Theme.success
                            onClicked: {
                                root.detailsVisible = true
                                root.diagnosticsVisible = true
                            }
                        }
                    }
                }
            }

            Rectangle {
                Layout.columnSpan: root.wideLayout ? 4 : 12
                Layout.fillWidth: true
                Layout.preferredHeight: root.wideLayout ? root.mapPreferredHeight + 128 : 430
                radius: 14
                color: Theme.surface
                border.color: Theme.border

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 20
                    spacing: 13

                    Label { text: "任务控制"; color: Theme.text; font.pixelSize: 20; font.bold: true }
                    Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        columnSpacing: 10
                        rowSpacing: 8
                        Label { text: "当前状态"; color: Theme.muted }
                        Label { Layout.fillWidth: true; text: backend.patrolMainStatusLabel || backend.patrolStateLabel; color: root.patrolStateColor(); font.bold: true; horizontalAlignment: Text.AlignRight; elide: Text.ElideRight }
                        Label { text: "当前路线"; color: Theme.muted }
                        Label { Layout.fillWidth: true; text: backend.routePreview.route_name || backend.routePreview.active_route_id || "未加载"; color: Theme.text; horizontalAlignment: Text.AlignRight; elide: Text.ElideRight }
                        Label { text: "当前目标"; color: Theme.muted }
                        Label { Layout.fillWidth: true; text: backend.currentTargetLabel || "等待任务"; color: Theme.text; horizontalAlignment: Text.AlignRight; elide: Text.ElideRight }
                        Label { text: "启动就绪"; color: Theme.muted }
                        Label { Layout.fillWidth: true; text: backend.patrolReady ? "依赖已就绪" : "启动时自动检查"; color: backend.patrolReady ? Theme.success : Theme.warning; horizontalAlignment: Text.AlignRight }
                    }

                    Label {
                        Layout.fillWidth: true
                        visible: backend.patrolStarting || backend.patrolError.length > 0
                        text: backend.patrolError.length > 0
                            ? backend.patrolError
                            : (backend.systemStatus.startup_step_label || "正在准备巡逻依赖，请勿重复操作")
                        color: backend.patrolError.length > 0 ? Theme.danger : Theme.warning
                        wrapMode: Text.Wrap
                    }

                    Item { Layout.fillHeight: true }

                    WarmButton {
                        objectName: "startPatrolButton"
                        text: backend.patrolStarting ? "启动准备中" : "启动巡逻任务"
                        enabled: backend.patrolCanStart
                        Layout.fillWidth: true
                        Layout.preferredHeight: 54
                        font.pixelSize: 16
                        onClicked: startPatrolDialog.open()
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 10
                        WarmButton {
                            text: "暂停"
                            enabled: backend.patrolCanPause
                            buttonColor: Theme.warning
                            Layout.fillWidth: true
                            Layout.preferredHeight: 50
                            onClicked: backend.sendSystemCommand("pause_patrol")
                        }
                        WarmButton {
                            text: "继续"
                            enabled: backend.patrolCanResume
                            Layout.fillWidth: true
                            Layout.preferredHeight: 50
                            onClicked: backend.sendSystemCommand("resume_patrol")
                        }
                    }
                    WarmButton {
                        objectName: "stopPatrolButton"
                        text: "结束巡逻"
                        enabled: backend.patrolCanCancel
                        buttonColor: Theme.danger
                        Layout.fillWidth: true
                        Layout.preferredHeight: 50
                        onClicked: stopPatrolDialog.open()
                    }
                }
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: root.availableWidth >= 1000 ? 4 : 2
            columnSpacing: 12
            rowSpacing: 12
            StatusCard { Layout.fillWidth: true; title: "当前目标"; value: backend.currentTargetLabel || "未开始"; statusColor: backend.patrolActive ? Theme.info : Theme.muted }
            StatusCard { Layout.fillWidth: true; title: "总体进度"; value: backend.patrolOverviewProgressLabel || backend.patrolProgressLabel || "未开始"; statusColor: backend.patrolActive ? Theme.success : Theme.muted }
            StatusCard { Layout.fillWidth: true; title: "当前轮次"; value: backend.patrolCycleLabel || "未开始"; statusColor: Theme.primary }
            StatusCard { Layout.fillWidth: true; title: "下一轮"; value: backend.patrolNextCycleLabel || "无等待"; statusColor: backend.patrolStatus.state === "waiting_loop" ? Theme.warning : Theme.muted }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: detailsColumn.implicitHeight + 32
            radius: 14
            color: Theme.surface
            border.color: Theme.border

            ColumnLayout {
                id: detailsColumn
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.margins: 16
                spacing: 10

                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "运行详情"; color: Theme.text; font.pixelSize: 19; font.bold: true; Layout.fillWidth: true }
                    Label { text: root.detailsVisible ? "收起诊断与任务信息" : "按需展开，减少页面渲染负担"; color: Theme.muted; font.pixelSize: 13 }
                    Switch { checked: root.detailsVisible; onToggled: root.detailsVisible = checked }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    visible: root.detailsVisible
                    spacing: 12

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "启动阶段与就绪项"; color: Theme.text; font.bold: true; Layout.fillWidth: true }
                        Switch { checked: root.diagnosticsVisible; onToggled: root.diagnosticsVisible = checked }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        visible: root.detailsVisible && root.diagnosticsVisible
                        GridLayout {
                            Layout.fillWidth: true
                            columns: root.availableWidth >= 1000 ? 4 : 2
                            columnSpacing: 8
                            rowSpacing: 8
                            Repeater {
                                model: root.detailsVisible && root.diagnosticsVisible ? root.startupStages : []
                                delegate: Rectangle {
                                    required property var modelData
                                    required property int index
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: 46
                                    radius: 8
                                    color: backend.systemStatus.startup_step === modelData.step ? Theme.infoSoft : Theme.surfaceAlt
                                    border.color: backend.systemStatus.startup_step === modelData.step ? Theme.info : Theme.border
                                    RowLayout {
                                        anchors.fill: parent
                                        anchors.margins: 9
                                        Label { Layout.fillWidth: true; text: modelData.label; color: Theme.text; font.pixelSize: 12; elide: Text.ElideRight }
                                        Label { text: root.stageMark(index); color: root.stageMark(index) === "当前" ? Theme.info : Theme.muted; font.bold: root.stageMark(index) === "当前" }
                                    }
                                }
                            }
                        }
                        Repeater {
                            model: root.detailsVisible && root.diagnosticsVisible ? root.readinessItems : []
                            delegate: RowLayout {
                                required property var modelData
                                Layout.fillWidth: true
                                Label { text: modelData.label; color: Theme.text; Layout.fillWidth: true }
                                Label { text: root.readiness[modelData.key] === true ? "就绪" : "等待"; color: root.readiness[modelData.key] === true ? Theme.success : Theme.warning }
                            }
                        }
                        Label { Layout.fillWidth: true; text: "路线文件: " + (backend.routePreview.route_file || "未找到"); color: Theme.muted; wrapMode: Text.Wrap }
                        Label { Layout.fillWidth: true; text: "预览图片: " + (backend.routePreview.image_path || backend.routePreview.image_url || "未生成"); color: Theme.muted; wrapMode: Text.Wrap }
                        Label { Layout.fillWidth: true; text: "图片状态: " + (backend.routePreview.image_exists === true ? "存在" : "不存在") + " / valid=" + String(backend.routePreview.image_valid === true) + " / " + String(backend.routePreview.image_bytes || 0) + " bytes"; color: Theme.muted; wrapMode: Text.Wrap }
                        Label { Layout.fillWidth: true; text: "image_error: " + (backend.routePreview.image_error || "-"); color: Theme.muted; wrapMode: Text.Wrap }
                        Label { Layout.fillWidth: true; text: "source: " + (backend.routePreviewImageSource || "-"); color: Theme.muted; wrapMode: Text.Wrap }
                        Label { Layout.fillWidth: true; text: "Image.status: " + String(routePreviewPane.imageStatus) + " / " + (backend.routePreview.source || backend.routePreview.message || "-"); color: Theme.muted; wrapMode: Text.Wrap }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "高级操作"; color: Theme.text; font.bold: true; Layout.fillWidth: true }
                        Switch { checked: root.advancedVisible; onToggled: root.advancedVisible = checked }
                    }
                    GridLayout {
                        Layout.fillWidth: true
                        visible: root.detailsVisible && root.advancedVisible
                        columns: root.availableWidth >= 900 ? 3 : 1
                        columnSpacing: 10
                        rowSpacing: 10
                        WarmButton { text: "关闭导航"; enabled: backend.systemStatus.navigation === "running"; buttonColor: Theme.danger; Layout.fillWidth: true; Layout.preferredHeight: 48; onClicked: backend.sendSystemCommand("stop_navigation") }
                        WarmButton { text: "关闭底盘"; enabled: backend.systemStatus.bringup === "running"; buttonColor: Theme.danger; Layout.fillWidth: true; Layout.preferredHeight: 48; onClicked: backend.sendSystemCommand("stop_bringup") }
                        WarmButton { text: "重新加载路线"; enabled: backend.patrolControlsEnabled; Layout.fillWidth: true; Layout.preferredHeight: 48; onClicked: backend.sendSystemCommand("reload_patrol_route") }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "巡逻点任务"; color: Theme.text; font.bold: true; Layout.fillWidth: true }
                        Switch { checked: root.tasksVisible; onToggled: root.tasksVisible = checked }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        visible: root.detailsVisible && root.tasksVisible
                        Repeater {
                            model: root.detailsVisible && root.tasksVisible ? (backend.routePreview.targets || []) : []
                            delegate: Rectangle {
                                required property var modelData
                                property var task: backend.patrolTasks[modelData.id] || ({})
                                Layout.fillWidth: true
                                Layout.preferredHeight: 76
                                radius: 8
                                color: backend.patrolStatus.target_id === modelData.id ? Theme.infoSoft : Theme.surfaceAlt
                                border.color: backend.patrolStatus.target_id === modelData.id ? Theme.info : Theme.border
                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 12
                                    ColumnLayout {
                                        Layout.fillWidth: true
                                        Label { text: modelData.name || modelData.id; color: Theme.text; font.bold: true }
                                        Label { text: "停留 " + String(task.task_duration_sec || 0) + " 秒 · " + (task.task_type || "未配置任务"); color: Theme.muted }
                                    }
                                    Label { text: task.task_status || "待配置"; color: Theme.info }
                                }
                            }
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "最近巡逻事件"; color: Theme.text; font.bold: true; Layout.fillWidth: true }
                        Switch { checked: root.eventsVisible; onToggled: root.eventsVisible = checked }
                    }
                    ListView {
                        Layout.fillWidth: true
                        Layout.preferredHeight: root.detailsVisible && root.eventsVisible ? 160 : 0
                        visible: root.detailsVisible && root.eventsVisible
                        clip: true
                        spacing: 6
                        model: root.detailsVisible && root.eventsVisible ? backend.patrolEvents : []
                        delegate: Rectangle {
                            required property var modelData
                            width: ListView.view.width
                            height: 42
                            radius: 7
                            color: Theme.surfaceAlt
                            border.color: Theme.border
                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 10
                                Label { text: modelData.timestamp || ""; color: Theme.muted; Layout.preferredWidth: 80 }
                                Label { text: JSON.stringify(modelData); color: Theme.text; Layout.fillWidth: true; elide: Text.ElideRight }
                            }
                        }
                    }
                }
            }
        }

        Item { Layout.preferredHeight: 8 }
    }

    Dialog {
        id: startPatrolDialog
        objectName: "startPatrolDialog"
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape
        title: "启动巡逻任务？"
        width: Math.min(root.availableWidth - 64, 560)
        height: 280
        anchors.centerIn: parent
        onAccepted: backend.startPatrolMode()
        contentItem: Label {
            text: "即将启动底盘、雷达、导航和巡逻执行器。\n机器人可能开始移动，请确认周围环境安全、急停可用，并且当前路线正确。"
            color: Theme.text
            font.pixelSize: 16
            wrapMode: Text.Wrap
            padding: 18
        }
        footer: Item {
            implicitHeight: 64
            RowLayout {
                anchors.fill: parent
                spacing: 10
                Item { Layout.fillWidth: true }
                WarmButton { objectName: "cancelStartPatrolButton"; text: "取消"; Layout.preferredWidth: 120; Layout.preferredHeight: 48; buttonColor: Theme.muted; onClicked: startPatrolDialog.close() }
                WarmButton { objectName: "confirmStartPatrolButton"; text: "确认启动"; Layout.preferredWidth: 140; Layout.preferredHeight: 48; onClicked: startPatrolDialog.accept() }
            }
        }
    }

    Dialog {
        id: stopPatrolDialog
        objectName: "stopPatrolDialog"
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape
        title: "结束当前巡逻？"
        width: Math.min(root.availableWidth - 64, 520)
        height: 240
        anchors.centerIn: parent
        onAccepted: backend.sendSystemCommand("stop_robot_stack")
        contentItem: Label {
            text: "将停止当前巡逻、导航和相关机器人运行栈。"
            color: Theme.text
            font.pixelSize: 16
            wrapMode: Text.Wrap
            padding: 18
        }
        footer: Item {
            implicitHeight: 64
            RowLayout {
                anchors.fill: parent
                spacing: 10
                Item { Layout.fillWidth: true }
                WarmButton { objectName: "cancelStopPatrolButton"; text: "取消"; Layout.preferredWidth: 120; Layout.preferredHeight: 48; buttonColor: Theme.muted; onClicked: stopPatrolDialog.close() }
                WarmButton { objectName: "confirmStopPatrolButton"; text: "确认结束"; Layout.preferredWidth: 140; Layout.preferredHeight: 48; buttonColor: Theme.danger; onClicked: stopPatrolDialog.accept() }
            }
        }
    }
}
