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
    property bool patrolStarting: backend.patrolModeState === "starting"
    property bool patrolRunning: backend.patrolModeState === "running"
        || backend.patrolStatus.state === "waiting_initial_pose"
        || backend.patrolStatus.state === "waiting_nav2"
        || backend.patrolStatus.state === "waiting_localization"
        || backend.patrolStatus.state === "running"
        || backend.patrolStatus.state === "paused"
        || backend.patrolStatus.state === "returning_home"
    property bool inspectionProfile: backend.patrolStartProfile === "inspection"
    property string previewImageSource: ""
    property var readinessItems: [
        { "label": "底盘", "key": "bringup" },
        { "label": "导航", "key": "navigation" },
        { "label": "执行器", "key": "executor" },
        { "label": "路线文件", "key": "route_file" },
        { "label": "Nav2 Action", "key": "nav2_action" }
    ]
    property var startupStages: [
        { "label": "底盘传感器", "step": "starting_bringup" },
        { "label": "地图/AMCL", "step": "starting_navigation" },
        { "label": "初始位姿", "step": "waiting_initial_pose_published" },
        { "label": "Nav2 动作服务", "step": "waiting_nav2_action" },
        { "label": "巡逻运行", "step": "patrol_started" }
    ]
    Component.onCompleted: previewImageRefreshTimer.restart()

    ColumnLayout {
        width: parent.width
        anchors.margins: 22
        spacing: 16

        Label { text: "巡逻模式"; color: Theme.text; font.pixelSize: 26; font.bold: true }

        RowLayout {
            Layout.fillWidth: true
            spacing: 16

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 430
                radius: 8
                color: Theme.surface
                border.color: Theme.border
                clip: true
                property string imageLoadError: ""

                Image {
                    id: routePreviewImage
                    anchors.fill: parent
                    anchors.margins: 12
                    source: root.previewImageSource
                    fillMode: Image.PreserveAspectFit
                    cache: false
                    visible: backend.routePreviewOk && status !== Image.Error
                    onStatusChanged: {
                        parent.imageLoadError = status === Image.Error
                            ? "路线预览加载失败，请检查图片文件和权限"
                            : ""
                    }
                }
                Connections {
                    target: backend
                    function onRoutePreviewChanged() {
                        root.previewImageSource = ""
                        previewImageRefreshTimer.restart()
                    }
                }
                Timer {
                    id: previewImageRefreshTimer
                    interval: 0
                    repeat: false
                    onTriggered: root.previewImageSource = backend.routePreviewImageSource
                }
                Label {
                    anchors.centerIn: parent
                    text: parent.imageLoadError || backend.routePreviewMessage || "路线图未生成"
                    color: Theme.muted
                    font.pixelSize: 18
                    visible: !backend.routePreviewOk || parent.imageLoadError.length > 0
                }
            }

            ColumnLayout {
                Layout.preferredWidth: 380
                spacing: 12

                StatusCard {
                    Layout.fillWidth: true
                    title: "巡逻状态"
                    value: root.patrolStarting
                        ? ("启动中: " + (backend.systemStatus.startup_step_label || "准备依赖"))
                        : (backend.patrolProgressLabel || backend.patrolStatusText)
                    statusColor: backend.patrolStatus.state === "running" || backend.patrolReady ? Theme.success : Theme.warning
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 102
                    radius: 8
                    color: Theme.surface
                    border.color: Theme.border
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 8
                        Label { text: "阶段流程"; color: Theme.muted }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8
                            Repeater {
                                model: root.startupStages
                                delegate: Rectangle {
                                    required property var modelData
                                    Layout.fillWidth: true
                                    height: 34
                                    radius: 6
                                    color: backend.systemStatus.startup_step === modelData.step
                                        ? Theme.primary
                                        : (root.patrolRunning && modelData.step === "patrol_started" ? Theme.success : Theme.background)
                                    border.color: Theme.border
                                    Label {
                                        anchors.centerIn: parent
                                        text: modelData.label
                                        color: Theme.text
                                        font.pixelSize: 11
                                        elide: Text.ElideRight
                                    }
                                }
                            }
                        }
                    }
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 150
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
                    statusColor: root.patrolRunning ? Theme.success : Theme.muted
                }
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 158
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
                                + " / " + String(backend.routePreview.image_bytes || 0) + " bytes"
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
                            text: "Image.status: " + String(routePreviewImage.status)
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

        RowLayout {
            Layout.fillWidth: true
            WarmButton {
                text: root.patrolStarting
                    ? ("启动中: " + (backend.systemStatus.startup_step_label || "准备中"))
                    : (backend.patrolModeState === "failed" ? "重新开始巡逻" : "开始巡逻")
                enabled: !root.patrolStarting && !root.patrolRunning && backend.routePreviewOk
                Layout.fillWidth: true
                onClicked: backend.startPatrolMode()
            }
            WarmButton {
                text: "暂停巡逻"
                enabled: backend.patrolControlsEnabled
                buttonColor: Theme.warning
                Layout.fillWidth: true
                onClicked: backend.sendPatrolCommand("pause")
            }
            WarmButton {
                text: "继续巡逻"
                enabled: backend.patrolControlsEnabled
                Layout.fillWidth: true
                onClicked: backend.sendPatrolCommand("resume")
            }
            WarmButton {
                text: "取消巡逻"
                enabled: backend.patrolControlsEnabled || root.patrolStarting
                buttonColor: Theme.danger
                Layout.fillWidth: true
                onClicked: backend.sendPatrolCommand("cancel")
            }
            WarmButton {
                text: "重新加载路线"
                enabled: backend.patrolControlsEnabled
                Layout.fillWidth: true
                onClicked: backend.sendPatrolCommand("reload")
            }
            WarmButton { text: "重绘预览"; buttonColor: Theme.accent; textColor: Theme.text; Layout.fillWidth: true; onClicked: backend.refreshRoutePreview() }
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
