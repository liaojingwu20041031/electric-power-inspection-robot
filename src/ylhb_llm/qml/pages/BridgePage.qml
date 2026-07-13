import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    id: root
    property bool advancedExpanded: false
    anchors.fill: parent
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
    ScrollBar.vertical.policy: ScrollBar.AsNeeded

    function coreUnavailable() {
        return backend.systemStatus.mobile_bridge === "stopped"
               && backend.systemStatus.mobile_bridge_http !== "http_ok"
    }

    function localColor() {
        if (coreUnavailable()) return Theme.danger
        var state = String(backend.localAppStatus.state || "UNAVAILABLE")
        if (state === "ENABLED") return Theme.success
        if (state === "DEGRADED") return Theme.warning
        if (state === "DISABLED") return Theme.muted
        return Theme.danger
    }

    function localSoftColor() {
        if (coreUnavailable()) return Theme.dangerSoft
        var state = String(backend.localAppStatus.state || "UNAVAILABLE")
        if (state === "ENABLED") return Theme.successSoft
        if (state === "DEGRADED") return Theme.warningSoft
        if (state === "DISABLED") return Theme.surfaceAlt
        return Theme.dangerSoft
    }

    function cloudColor() {
        if (coreUnavailable()) return Theme.danger
        var state = String(backend.cloudStatus.state || "UNCONFIGURED")
        if (state === "CONNECTED") return Theme.success
        if (state === "CONNECTING") return Theme.info
        if (state === "BACKOFF") return Theme.warning
        if (state === "DISABLED") return Theme.muted
        return Theme.danger
    }

    function cloudSoftColor() {
        if (coreUnavailable()) return Theme.dangerSoft
        var state = String(backend.cloudStatus.state || "UNCONFIGURED")
        if (state === "CONNECTED") return Theme.successSoft
        if (state === "CONNECTING") return Theme.infoSoft
        if (state === "BACKOFF") return Theme.warningSoft
        if (state === "DISABLED") return Theme.surfaceAlt
        return Theme.dangerSoft
    }

    Item {
        width: root.width
        height: content.height + 44

        ColumnLayout {
            id: content
            width: Math.min(parent.width - 32, 1040)
            x: Math.max(16, (parent.width - width) / 2)
            y: 22
            spacing: 16

            Label {
                text: "连接与服务"
                color: Theme.text
                font.pixelSize: 27
                font.bold: true
            }
            Label {
                Layout.fillWidth: true
                text: "管理手机 APP、云平台与网桥核心服务"
                color: Theme.muted
                font.pixelSize: 15
                wrapMode: Text.Wrap
            }

            GridLayout {
                Layout.fillWidth: true
                columns: content.width >= 900 ? 2 : 1
                columnSpacing: 16
                rowSpacing: 16

                ConnectionCard {
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignTop
                    title: "本地 APP 服务"
                    stateTitle: root.coreUnavailable() ? "网桥核心服务无响应" : backend.localAppStateText
                    description: root.coreUnavailable() ? "请先恢复网桥核心服务" : backend.localAppDescription
                    statusColor: root.localColor()
                    softColor: root.localSoftColor()

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "APP 地址"; color: Theme.muted }
                        Label {
                            Layout.fillWidth: true
                            text: backend.localAppStatus.appUrl || backend.appUrl || "-"
                            color: Theme.primary
                            horizontalAlignment: Text.AlignRight
                            elide: Text.ElideMiddle
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "HTTP 可用"; color: Theme.muted }
                        Label {
                            Layout.fillWidth: true
                            text: !root.coreUnavailable() && backend.localAppStatus.httpAvailable ? "可用" : "不可用"
                            color: !root.coreUnavailable() && backend.localAppStatus.httpAvailable ? Theme.success : Theme.warning
                            horizontalAlignment: Text.AlignRight
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Label { Layout.fillWidth: true; text: "允许手机通过局域网连接"; color: Theme.text; wrapMode: Text.Wrap }
                        BusyIndicator { running: backend.localAppControlPending; visible: running; implicitWidth: 28; implicitHeight: 28 }
                        Switch {
                            id: localAppSwitch
                            implicitWidth: 52
                            implicitHeight: 44
                            enabled: !root.coreUnavailable() && !backend.localAppControlPending && String(backend.localAppStatus.state || "UNAVAILABLE") !== "UNAVAILABLE"
                            onClicked: {
                                if (!checked) localDisableDialog.open()
                                else backend.setLocalAppEnabled(true)
                            }
                        }
                        Binding {
                            target: localAppSwitch
                            property: "checked"
                            value: !!backend.localAppStatus.enabled
                            when: !localDisableDialog.visible && !backend.localAppControlPending
                        }
                    }
                    Label {
                        Layout.fillWidth: true
                        visible: backend.localAppControlMessage.length > 0
                        text: backend.localAppControlMessage
                        color: backend.localAppControlMessage.indexOf("失败") >= 0 ? Theme.danger : Theme.muted
                        wrapMode: Text.Wrap
                    }
                }

                ConnectionCard {
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignTop
                    title: "云平台连接"
                    stateTitle: root.coreUnavailable() ? "网桥核心服务无响应" : backend.cloudStateText
                    description: root.coreUnavailable() ? "请先恢复网桥核心服务" : backend.cloudDescription
                    statusColor: root.cloudColor()
                    softColor: root.cloudSoftColor()

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "服务器"; color: Theme.muted }
                        Label {
                            Layout.fillWidth: true
                            text: backend.cloudStatus.serverBaseUrl || "未配置"
                            color: Theme.primary
                            horizontalAlignment: Text.AlignRight
                            elide: Text.ElideMiddle
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "最近心跳"; color: Theme.muted }
                        Label {
                            Layout.fillWidth: true
                            text: backend.cloudStatus.lastSuccessAt || "-"
                            color: Theme.text
                            horizontalAlignment: Text.AlignRight
                            elide: Text.ElideRight
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Label { Layout.fillWidth: true; text: "允许机器人连接云平台"; color: Theme.text; wrapMode: Text.Wrap }
                        BusyIndicator { running: backend.cloudControlPending; visible: running; implicitWidth: 28; implicitHeight: 28 }
                        Switch {
                            id: cloudSwitch
                            implicitWidth: 52
                            implicitHeight: 44
                            enabled: !root.coreUnavailable() && !backend.cloudControlPending && !!backend.cloudStatus.configured
                            onClicked: {
                                if (!checked) cloudDisableDialog.open()
                                else backend.setCloudEnabled(true)
                            }
                        }
                        Binding {
                            target: cloudSwitch
                            property: "checked"
                            value: !!backend.cloudStatus.desiredEnabled
                            when: !cloudDisableDialog.visible && !backend.cloudControlPending
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        WarmButton {
                            visible: String(backend.cloudStatus.state || "") === "BACKOFF"
                            enabled: !root.coreUnavailable() && !backend.cloudControlPending
                            text: "立即重试"
                            buttonColor: Theme.info
                            onClicked: backend.setCloudEnabled(true)
                        }
                        Label {
                            Layout.fillWidth: true
                            text: Number(backend.cloudStatus.pendingEventCount || 0) > 0
                                  ? "待上传事件 " + backend.cloudStatus.pendingEventCount + " 条"
                                  : "事件已同步"
                            color: Number(backend.cloudStatus.pendingEventCount || 0) > 0 ? Theme.warning : Theme.success
                            horizontalAlignment: Text.AlignRight
                        }
                    }
                    Label {
                        Layout.fillWidth: true
                        visible: backend.cloudControlMessage.length > 0
                        text: backend.cloudControlMessage
                        color: backend.cloudControlMessage.indexOf("失败") >= 0 ? Theme.danger : Theme.muted
                        wrapMode: Text.Wrap
                    }
                }
            }

            ConnectionPath {
                Layout.fillWidth: true
                localColor: root.localColor()
                cloudColor: root.cloudColor()
                coreRunning: backend.systemStatus.mobile_bridge === "running" || backend.systemStatus.mobile_bridge_http === "http_ok"
            }

            Rectangle {
                Layout.fillWidth: true
                implicitWidth: 320
                implicitHeight: 1
                Layout.preferredHeight: 40 + coreTitle.implicitHeight + coreGrid.implicitHeight
                                        + (managedLabel.visible ? managedLabel.implicitHeight + 10 : 0)
                                        + (advancedToggle.visible ? advancedToggle.implicitHeight + 10 : 0)
                                        + (advancedOperations.visible ? advancedOperations.implicitHeight + 10 : 0)
                radius: 16
                color: Theme.surface
                border.color: Theme.border
                ColumnLayout {
                    id: coreColumn
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.margins: 20
                    spacing: 10
                    Label { id: coreTitle; text: "网桥核心服务"; color: Theme.text; font.pixelSize: 18; font.bold: true }
                    GridLayout {
                        id: coreGrid
                        Layout.fillWidth: true
                        columns: content.width >= 720 ? 4 : 2
                        columnSpacing: 12
                        rowSpacing: 8
                        Label { text: "运行状态"; color: Theme.muted }
                        Label { text: backend.localizedStatus(backend.systemStatus.mobile_bridge || "stopped"); color: Theme.text }
                        Label { text: "管理方式"; color: Theme.muted }
                        Label { text: backend.systemStatus.mobile_bridge_managed_externally ? "systemd 自动维护" : "开发模式"; color: Theme.text }
                        Label { text: "ROS 通信"; color: Theme.muted }
                        Label { text: backend.systemStatus.mobile_bridge === "running" ? "正常" : "未连接"; color: Theme.text }
                        Label { text: "HTTP 进程状态"; color: Theme.muted }
                        Label { text: backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped"); color: Theme.text }
                    }
                    Label {
                        id: managedLabel
                        Layout.fillWidth: true
                        visible: !!backend.systemStatus.mobile_bridge_managed_externally
                        text: "由 systemd 自动维护，异常退出后将自动重启"
                        color: Theme.muted
                        wrapMode: Text.Wrap
                    }
                    Button {
                        id: advancedToggle
                        visible: !backend.systemStatus.mobile_bridge_managed_externally
                        text: "高级服务操作" + (root.advancedExpanded ? "  收起" : "  展开")
                        flat: true
                        implicitHeight: 44
                        onClicked: root.advancedExpanded = !root.advancedExpanded
                    }
                    RowLayout {
                        id: advancedOperations
                        Layout.fillWidth: true
                        visible: root.advancedExpanded && !backend.systemStatus.mobile_bridge_managed_externally
                        WarmButton { text: "启动核心服务"; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("start_mobile_bridge") }
                        WarmButton { text: "重启核心服务"; Layout.fillWidth: true; buttonColor: Theme.warning; onClicked: backend.sendSystemCommand("restart_mobile_bridge") }
                        WarmButton { text: "停止核心服务"; Layout.fillWidth: true; buttonColor: Theme.danger; onClicked: coreStopDialog.open() }
                    }
                }
            }

            GridLayout {
                Layout.fillWidth: true
                columns: content.width >= 720 ? 4 : 2
                columnSpacing: 12
                rowSpacing: 12
                MetricTile { Layout.fillWidth: true; label: "本地 APP"; value: root.coreUnavailable() ? "不可用" : (backend.localAppStatus.enabled ? "开启" : "关闭"); valueColor: root.localColor() }
                MetricTile { Layout.fillWidth: true; label: "云平台"; value: root.coreUnavailable() ? "无响应" : backend.cloudStateText.replace("云平台", ""); valueColor: root.cloudColor() }
                MetricTile { Layout.fillWidth: true; label: "待上传事件"; value: Number(backend.cloudStatus.pendingEventCount || 0) === 0 ? "已同步" : backend.cloudStatus.pendingEventCount + " 条" }
                MetricTile {
                    Layout.fillWidth: true
                    label: "当前任务"
                    value: backend.cloudStatus.activeExecutionId ? backend.cloudStatus.activeExecutionId : "无活动任务"
                    tooltip: backend.cloudStatus.activeExecutionId || ""
                }
            }

            Rectangle {
                Layout.fillWidth: true
                implicitWidth: 320
                implicitHeight: 1
                Layout.preferredHeight: diagnosticColumn.implicitHeight + 32
                radius: 16
                color: Theme.surface
                border.color: Theme.border
                ColumnLayout {
                    id: diagnosticColumn
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.margins: 16
                    spacing: 10
                    Button {
                        text: "连接诊断" + (diagnosticBody.visible ? "  收起" : "  展开")
                        flat: true
                        implicitHeight: 44
                        onClicked: diagnosticBody.visible = !diagnosticBody.visible
                    }
                    ColumnLayout {
                        id: diagnosticBody
                        Layout.fillWidth: true
                        visible: false
                        spacing: 14
                        Label { text: "本地 APP 诊断"; color: Theme.text; font.bold: true }
                        Label {
                            Layout.fillWidth: true
                            color: Theme.muted
                            wrapMode: Text.Wrap
                            text: "raw state: " + (backend.localAppStatus.state || "UNAVAILABLE")
                                  + "    HTTP available: " + (!!backend.localAppStatus.httpAvailable)
                                  + "    APP URL: " + (backend.localAppStatus.appUrl || "-")
                                  + "\nauth required: " + (!!backend.localAppStatus.authRequired)
                                  + "    status WS clients: " + (backend.localAppStatus.activeStatusClients || 0)
                                  + "    map WS clients: " + (backend.localAppStatus.activeMapClients || 0)
                                  + "\nlast changed: " + (backend.localAppStatus.lastChangedAt || "-")
                                  + "    last error: " + (backend.localAppStatus.lastError || "-")
                        }
                        Label { text: "云平台诊断"; color: Theme.text; font.bold: true }
                        Label {
                            Layout.fillWidth: true
                            color: Theme.muted
                            wrapMode: Text.Wrap
                            text: "raw state: " + (backend.cloudStatus.state || "UNCONFIGURED")
                                  + "    serverBaseUrl: " + (backend.cloudStatus.serverBaseUrl || "-")
                                  + "\nlastAttemptAt: " + (backend.cloudStatus.lastAttemptAt || "-")
                                  + "    lastSuccessAt: " + (backend.cloudStatus.lastSuccessAt || "-")
                                  + "    lastServerTime: " + (backend.cloudStatus.lastServerTime || "-")
                                  + "\nnextRetrySec: " + (backend.cloudStatus.nextRetrySec || 0)
                                  + "    pendingEventCount: " + (backend.cloudStatus.pendingEventCount || 0)
                                  + "    pendingCommandCount: " + (backend.cloudStatus.pendingCommandCount || 0)
                                  + "\nlatestLocalEventSequence: " + (backend.cloudStatus.latestLocalEventSequence || 0)
                                  + "    lastUploadedSequence: " + (backend.cloudStatus.lastUploadedSequence || 0)
                                  + "\nlastReceivedCommandId: " + (backend.cloudStatus.lastReceivedCommandId || "-")
                                  + "    executionId: " + (backend.cloudStatus.activeExecutionId || "-")
                                  + "    deploymentId: " + (backend.cloudStatus.activeDeploymentId || "-")
                                  + "\nlastError: " + (backend.cloudStatus.lastError || "-")
                        }
                    }
                }
            }
        }
    }

    Dialog {
        id: localDisableDialog
        anchors.centerIn: Overlay.overlay
        modal: true
        title: "关闭本地 APP 服务？"
        standardButtons: Dialog.Yes | Dialog.No
        Label {
            width: Math.min(440, root.width - 80)
            text: "关闭后，手机 APP 将无法通过局域网连接机器人。\n云平台连接和当前巡检不会受到影响。"
            color: Theme.text
            wrapMode: Text.Wrap
        }
        onAccepted: backend.setLocalAppEnabled(false)
    }

    Dialog {
        id: cloudDisableDialog
        anchors.centerIn: Overlay.overlay
        modal: true
        title: "关闭云平台连接？"
        standardButtons: Dialog.Yes | Dialog.No
        Label {
            width: Math.min(440, root.width - 80)
            text: "关闭后，平台将暂时无法远程控制机器人或查看实时状态。\n本地 APP 和当前巡检不会停止，事件将在重新连接后补传。"
            color: Theme.text
            wrapMode: Text.Wrap
        }
        onAccepted: backend.setCloudEnabled(false)
    }

    Dialog {
        id: coreStopDialog
        anchors.centerIn: Overlay.overlay
        modal: true
        title: "停止网桥核心服务？"
        standardButtons: Dialog.Yes | Dialog.No
        Label {
            width: Math.min(440, root.width - 80)
            text: "此操作会同时中断手机 APP 和云平台连接。"
            color: Theme.danger
            wrapMode: Text.Wrap
        }
        onAccepted: backend.sendSystemCommand("stop_mobile_bridge")
    }
}
