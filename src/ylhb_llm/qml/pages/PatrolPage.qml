import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    id: root
    clip: true
    contentWidth: availableWidth
    property var readiness: backend.systemStatus.patrol_readiness || ({})
    property bool diagnosticsVisible: false
    property bool inspectionProfile: backend.patrolStartProfile === "inspection"
    property int startupStageIndex: stageIndex(backend.systemStatus.startup_step || "")
    property var readinessItems: [
        { "label": "底盘", "key": "bringup" },
        { "label": "导航", "key": "navigation" },
        { "label": "执行器", "key": "executor" },
        { "label": "路线文件", "key": "route_file" }
    ]
    property var startupStages: [
        { "label": "启动底盘", "step": "starting_bringup" },
        { "label": "等待底盘稳定", "step": "waiting_after_bringup" },
        { "label": "启动导航", "step": "starting_navigation" },
        { "label": "等待导航稳定", "step": "waiting_after_navigation" },
        { "label": "启动巡逻执行器", "step": "starting_executor" },
        { "label": "等待执行器发布初始位姿", "step": "waiting_after_executor" },
        { "label": "等待 map->odom", "step": "waiting_map_to_odom" },
        { "label": "等待 Nav2 active", "step": "waiting_nav2_active" },
        { "label": "发送巡逻 start", "step": "patrol_start_sent" },
        { "label": "等待执行器响应", "step": "waiting_executor_response" },
        { "label": "巡逻启动失败", "step": "patrol_failed" },
        { "label": "巡逻运行", "step": "patrol_started" }
    ]

    function stageIndex(step) {
        for (var i = 0; i < startupStages.length; i++) {
            if (startupStages[i].step === step) {
                return i
            }
        }
        return -1
    }

    function stageMark(index) {
        if (backend.patrolActive && root.startupStages[index].step === "patrol_started") {
            return "当前"
        }
        if (root.startupStageIndex < 0) {
            return "等待"
        }
        if (index < root.startupStageIndex) {
            return "完成"
        }
        return index === root.startupStageIndex ? "当前" : "等待"
    }

    function patrolStateColor() {
        if (backend.patrolError.length > 0 || backend.patrolStatus.state === "failed") {
            return Theme.danger
        }
        if (backend.patrolActive || backend.patrolStatus.state === "succeeded") {
            return Theme.success
        }
        return backend.patrolStarting ? Theme.primary : Theme.warning
    }

    ColumnLayout {
        width: parent.width
        anchors.margins: 22
        spacing: 16

        Label { text: "巡逻模式"; color: Theme.text; font.pixelSize: 26; font.bold: true }

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 16

            RoutePreviewViewer {
                id: routePreviewPane
                Layout.fillWidth: true
                Layout.preferredHeight: root.availableWidth >= 1200 ? 420 : 380
                source: backend.routePreviewImageSource
                previewOk: backend.routePreviewOk
                loading: backend.routePreviewLoading
                message: !backend.routePreviewOk
                    ? backend.routePreviewMessage
                    : (backend.routePreview.image_exists !== true
                        ? "路线预览图文件不存在"
                        : "路线预览图未生成")
            }

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 12

                StatusCard {
                    Layout.fillWidth: true
                    title: "巡逻状态"
                    value: backend.patrolStateLabel
                    statusColor: root.patrolStateColor()
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: root.availableWidth >= 900 ? 188 : 284
                    radius: 8
                    color: Theme.surface
                    border.color: Theme.border
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 8
                        Label { text: "阶段流程"; color: Theme.muted }
                        GridLayout {
                            Layout.fillWidth: true
                            columns: root.availableWidth >= 900 ? 4 : 2
                            columnSpacing: 8
                            rowSpacing: 8
                            Repeater {
                                model: root.startupStages
                                delegate: Rectangle {
                                    required property var modelData
                                    required property int index
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: 44
                                    radius: 6
                                    color: backend.systemStatus.startup_step === modelData.step
                                        ? Theme.primary
                                        : (root.stageMark(index) === "完成" ? Theme.surface : Theme.background)
                                    border.color: Theme.border
                                    Column {
                                        anchors.centerIn: parent
                                        spacing: 1
                                        width: parent.width - 10
                                        Label {
                                            width: parent.width
                                            text: modelData.label
                                            color: Theme.text
                                            font.pixelSize: 11
                                            horizontalAlignment: Text.AlignHCenter
                                            elide: Text.ElideRight
                                        }
                                        Label {
                                            width: parent.width
                                            text: root.stageMark(index)
                                            color: root.stageMark(index) === "当前" ? Theme.text : Theme.muted
                                            font.pixelSize: 10
                                            horizontalAlignment: Text.AlignHCenter
                                        }
                                    }
                                }
                            }
                        }
                        Label {
                            text: backend.patrolStarting
                                ? "当前按手动启动流程执行，导航启动后会等待约 20 秒，请不要重复点击。"
                                : (backend.patrolStartupStep === "waiting_nav2"
                                ? "等待 Nav2 导航服务启动完成。"
                                : (backend.patrolStartupStep === "retrying_goal"
                                    ? "导航目标被拒绝，正在重试。"
                                    : (root.startupStageIndex < 0 && backend.patrolStartupStep.length > 0
                                        ? (backend.systemStatus.startup_step_label || backend.patrolStartupStep)
                                        : "")))
                            color: Theme.warning
                            visible: text.length > 0
                            Layout.fillWidth: true
                            wrapMode: Text.Wrap
                        }
                    }
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 128
                    radius: 8
                    color: Theme.surface
                    border.color: Theme.border
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 6
                        Label { text: "启动就绪项"; color: Theme.muted }
                        Repeater {
                            model: root.readinessItems
                            delegate: RowLayout {
                                required property var modelData
                                Layout.fillWidth: true
                                Label {
                                    text: modelData.label
                                    color: Theme.text
                                    Layout.fillWidth: true
                                }
                                Label {
                                    text: root.readiness[modelData.key] === true ? "就绪" : "等待"
                                    color: root.readiness[modelData.key] === true ? Theme.success : Theme.warning
                                }
                            }
                        }
                        Label {
                            text: backend.patrolError
                            color: Theme.warning
                            visible: backend.patrolError.length > 0
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                    }
                }
                StatusCard {
                    Layout.fillWidth: true
                    title: "当前路线"
                    value: backend.routePreview.route_name || backend.routePreview.active_route_id || "未加载"
                    statusColor: backend.routePreview.ok === true ? Theme.primary : Theme.warning
                }
                StatusCard {
                    Layout.fillWidth: true
                    title: "目标总数"
                    value: String(backend.routePreview.target_count || 0)
                    statusColor: Theme.primary
                }
                StatusCard {
                    Layout.fillWidth: true
                    title: "当前检查点"
                    value: backend.patrolProgressLabel || "未开始"
                    statusColor: backend.patrolActive ? Theme.success : Theme.muted
                }
                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: "诊断信息"
                        color: Theme.muted
                        Layout.fillWidth: true
                    }
                    Switch {
                        checked: root.diagnosticsVisible
                        onToggled: root.diagnosticsVisible = checked
                    }
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: root.diagnosticsVisible ? 184 : 0
                    visible: root.diagnosticsVisible
                    radius: 8
                    color: Theme.surface
                    border.color: Theme.border
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 6
                        Label { text: "诊断信息"; color: Theme.muted }
                        Label {
                            text: "路线文件: " + (backend.routePreview.route_file || "未找到正式巡逻路线文件")
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                        Label {
                            text: "预览图片: " + (backend.routePreview.image_path || backend.routePreview.image_url || "未生成")
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                        Label {
                            text: "图片状态: " + (backend.routePreview.image_exists === true ? "存在" : "不存在")
                                + " / valid=" + String(backend.routePreview.image_valid === true)
                                + " / " + String(backend.routePreview.image_bytes || 0) + " bytes"
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                        Label {
                            text: "image_url: " + (backend.routePreview.image_url || "-")
                                + " / image_error: " + (backend.routePreview.image_error || "-")
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                        Label {
                            text: "source: " + (backend.routePreviewImageSource || "-")
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                        Label {
                            text: "Image.status: " + String(routePreviewPane.imageStatus)
                                + " / " + (backend.routePreview.source || backend.routePreview.message || "-")
                            color: Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            Label {
                text: "启用感知巡检"
                color: Theme.text
                Layout.alignment: Qt.AlignVCenter
            }
            Switch {
                checked: root.inspectionProfile
                enabled: false
                onToggled: backend.setPatrolStartProfile(checked ? "inspection" : "navigation")
            }
            Item { Layout.fillWidth: true }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: root.availableWidth >= 1100 ? 4 : (root.availableWidth >= 700 ? 3 : 2)
            columnSpacing: 12
            rowSpacing: 10
            WarmButton {
                text: backend.patrolStarting
                    ? ("启动中: " + (backend.systemStatus.startup_step_label || "准备中"))
                    : "一键启动巡逻模式"
                enabled: backend.patrolCanStart
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.startPatrolMode()
            }
            WarmButton {
                text: "暂停巡逻"
                enabled: backend.patrolCanPause
                buttonColor: Theme.warning
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("pause_patrol")
            }
            WarmButton {
                text: "继续巡逻"
                enabled: backend.patrolCanResume
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("resume_patrol")
            }
            WarmButton {
                text: "取消巡逻"
                enabled: backend.patrolCanCancel
                buttonColor: Theme.danger
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("cancel_patrol")
            }
            WarmButton {
                text: "关闭导航"
                enabled: backend.systemStatus.navigation === "running"
                buttonColor: Theme.danger
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("stop_navigation")
            }
            WarmButton {
                text: "关闭底盘"
                enabled: backend.systemStatus.bringup === "running"
                buttonColor: Theme.danger
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("stop_bringup")
            }
            WarmButton {
                text: "重新加载路线"
                enabled: backend.patrolControlsEnabled
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.sendSystemCommand("reload_patrol_route")
            }
            WarmButton {
                text: "重绘预览"
                buttonColor: Theme.accent
                textColor: Theme.text
                Layout.fillWidth: true
                Layout.preferredHeight: 44
                onClicked: backend.refreshRoutePreview()
            }
        }

        Label { text: "巡逻点任务接口"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        Repeater {
            model: backend.routePreview.targets || []
            delegate: Rectangle {
                required property var modelData
                Layout.fillWidth: true
                height: 118
                radius: 8
                color: backend.patrolStatus.target_id === modelData.id ? Theme.background : Theme.surface
                border.color: backend.patrolStatus.target_id === modelData.id ? Theme.primary : Theme.border
                property var task: backend.patrolTasks[modelData.id] || {}
                GridLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    columns: 4
                    columnSpacing: 16
                    rowSpacing: 8
                    Label { text: modelData.name || modelData.id; color: Theme.text; font.bold: true; Layout.columnSpan: 4 }
                    Label { text: "停留 " + String(task.task_duration_sec || 0) + " 秒"; color: Theme.muted }
                    Label { text: "任务类型: " + (task.task_type || "未配置"); color: Theme.muted }
                    Label { text: "参数: {}"; color: Theme.muted }
                    Label { text: "状态: " + (task.task_status || "预留接口"); color: Theme.primary }
                }
            }
        }

        Label { text: "最近巡逻事件"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        ListView {
            Layout.fillWidth: true
            Layout.preferredHeight: 160
            clip: true
            spacing: 6
            model: backend.patrolEvents
            delegate: Rectangle {
                required property var modelData
                width: ListView.view.width
                height: 42
                radius: 6
                color: Theme.surface
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
