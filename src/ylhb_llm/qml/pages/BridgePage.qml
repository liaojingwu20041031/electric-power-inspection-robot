import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQml 2.15
import "../components"
import ".."

ScrollView {
    id: root
    property bool advancedExpanded: false
    property real uiScale: Math.max(1.0, Math.min(1.22, root.width / 1350.0))
    anchors.fill: parent
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
    ScrollBar.vertical.policy: ScrollBar.AsNeeded

    function localColor() {
        if (!backend.localAppStatusReceived) return Theme.info
        var state = String(backend.localAppStatus.state || "UNAVAILABLE")
        if (state === "ENABLED") return Theme.success
        if (state === "DEGRADED") return Theme.warning
        if (state === "DISABLED") return Theme.muted
        return Theme.danger
    }

    function localSoftColor() {
        if (!backend.localAppStatusReceived) return Theme.infoSoft
        var state = String(backend.localAppStatus.state || "UNAVAILABLE")
        if (state === "ENABLED") return Theme.successSoft
        if (state === "DEGRADED") return Theme.warningSoft
        if (state === "DISABLED") return Theme.surfaceAlt
        return Theme.dangerSoft
    }

    function cloudColor() {
        var state = backend.cloudDisplayState
        if (state === "WAITING") return Theme.info
        if (state === "CONNECTED") return Theme.success
        if (state === "CONNECTING") return Theme.info
        if (state === "BACKOFF") return Theme.warning
        if (state === "DISABLED") return Theme.muted
        return Theme.danger
    }

    function cloudSoftColor() {
        var state = backend.cloudDisplayState
        if (state === "WAITING") return Theme.infoSoft
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
            width: Math.min(parent.width - 40, 1540)
            x: Math.max(20, (parent.width - width) / 2)
            y: 22
            spacing: 18 * root.uiScale

            Label {
                text: "连接与服务"
                color: Theme.text
                font.pixelSize: 32 * root.uiScale
                font.bold: true
            }
            Label {
                Layout.fillWidth: true
                text: "管理手机 APP、云平台与网桥核心服务"
                color: Theme.muted
                font.pixelSize: 16 * root.uiScale
                wrapMode: Text.Wrap
            }

            GridLayout {
                Layout.fillWidth: true
                columns: content.width >= 960 ? 2 : 1
                columnSpacing: 16
                rowSpacing: 16

                ConnectionCard {
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignTop
                    title: "本地 APP 服务"
                    stateTitle: backend.localAppStateText
                    description: backend.localAppDescription
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
                            text: backend.localAppStatus.httpAvailable ? "可用" : "不可用"
                            color: backend.localAppStatus.httpAvailable ? Theme.success : Theme.warning
                            horizontalAlignment: Text.AlignRight
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Label { Layout.fillWidth: true; text: "允许手机通过局域网连接"; color: Theme.text; wrapMode: Text.Wrap }
                        BusyIndicator { running: backend.localAppControlPending; visible: running; implicitWidth: 28; implicitHeight: 28 }
                        Switch {
                            id: localAppSwitch
                            objectName: "localAppSwitch"
                            implicitWidth: 64
                            implicitHeight: 48
                            enabled: backend.localAppControlAvailable && !backend.localAppControlPending
                            onClicked: {
                                if (!checked) localDisableDialog.open()
                                else backend.setLocalAppEnabled(true)
                            }
                        }
                        Binding {
                            target: localAppSwitch
                            property: "checked"
                            value: backend.localAppRequestedEnabled
                            when: !localDisableDialog.visible && !backend.localAppControlPending
                            restoreMode: Binding.RestoreBindingOrValue
                        }
                    }
                    Label {
                        Layout.fillWidth: true
                        text: backend.localAppControlPending ? "控制请求处理中"
                              : !backend.localAppStatusReceived ? "正在等待本地 APP 状态"
                              : !backend.localAppControlAvailable ? (backend.bridgeCoreState === "stopped" ? "网桥核心服务未启动" : "正在等待本地 APP 控制服务")
                              : "本地 APP 控制服务可用"
                        color: backend.localAppControlAvailable && !backend.localAppControlPending ? Theme.success : Theme.warning
                        wrapMode: Text.Wrap
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
                    stateTitle: backend.cloudDisplayStateText
                    description: backend.cloudDisplayDescription
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
                            objectName: "cloudSwitch"
                            implicitWidth: 64
                            implicitHeight: 48
                            enabled: backend.cloudControlAvailable && !backend.cloudControlPending && !!backend.cloudStatus.configured
                            onClicked: {
                                if (!checked) cloudDisableDialog.open()
                                else backend.setCloudEnabled(true)
                            }
                        }
                        Binding {
                            target: cloudSwitch
                            property: "checked"
                            value: backend.cloudRequestedEnabled
                            when: !cloudDisableDialog.visible && !backend.cloudControlPending
                            restoreMode: Binding.RestoreBindingOrValue
                        }
                    }
                    Label {
                        Layout.fillWidth: true
                        text: backend.cloudControlPending ? "控制请求处理中"
                              : !backend.cloudStatusReceived ? "正在等待云平台状态"
                              : !backend.cloudStatus.configured ? "云平台尚未配置"
                              : !backend.cloudControlAvailable ? (backend.bridgeCoreState === "stopped" ? "网桥核心服务未启动" : "正在等待云平台控制服务")
                              : "云平台控制服务可用"
                        color: backend.cloudControlAvailable && backend.cloudStatus.configured && !backend.cloudControlPending ? Theme.success : Theme.warning
                        wrapMode: Text.Wrap
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        WarmButton {
                            visible: backend.cloudDisplayState === "BACKOFF"
                            enabled: backend.cloudControlAvailable && !backend.cloudControlPending
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
                coreState: backend.bridgeCoreState
            }

            Rectangle {
                Layout.fillWidth: true
                implicitWidth: 320
                implicitHeight: 1
                Layout.preferredHeight: coreColumn.implicitHeight + 40
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
                        Label { text: backend.bridgeCoreStateText; color: backend.bridgeCoreAvailable ? Theme.success : Theme.warning; wrapMode: Text.Wrap }
                        Label { text: "管理方式"; color: Theme.muted }
                        Label { text: backend.systemStatus.mobile_bridge_owner === "systemd" ? "systemd" : "Supervisor"; color: Theme.text }
                        Label { text: "ROS 通信"; color: Theme.muted }
                        Label { text: backend.bridgeCoreAvailable ? "正常" : "未连接"; color: Theme.text }
                        Label { text: "HTTP 进程状态"; color: Theme.muted }
                        Label { text: backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped"); color: Theme.text }
                    }
                    WarmButton {
                        visible: backend.bridgeRecoveryAvailable
                        Layout.fillWidth: true
                        text: "启动网桥核心服务"
                        buttonColor: Theme.info
                        onClicked: backend.sendSystemCommand("start_mobile_bridge")
                    }
                    Label {
                        Layout.fillWidth: true
                        visible: backend.systemStatus.mobile_bridge_owner === "systemd" && !backend.bridgeCoreAvailable
                        text: "只读排障：sudo systemctl restart ylhb-mobile-bridge.service"
                        color: Theme.warning
                        wrapMode: Text.Wrap
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
                MetricTile { Layout.fillWidth: true; label: "本地 APP"; value: backend.localAppStateText.replace("本地 APP 服务", ""); valueColor: root.localColor() }
                MetricTile { Layout.fillWidth: true; label: "云平台"; value: backend.cloudDisplayStateText.replace("云平台", ""); valueColor: root.cloudColor() }
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
                                  + "    display state: " + backend.cloudDisplayState
                                  + "    heartbeatInFlight: " + backend.cloudHeartbeatInFlight
                                  + "    serverBaseUrl: " + (backend.cloudStatus.serverBaseUrl || "-")
                                  + "\nlastAttemptAt: " + (backend.cloudStatus.lastAttemptAt || "-")
                                  + "    lastSuccessAt: " + (backend.cloudStatus.lastSuccessAt || "-")
                                  + "    lastServerTime: " + (backend.cloudStatus.lastServerTime || "-")
                                  + "\nnextHeartbeatSec: " + (backend.cloudStatus.nextHeartbeatSec || 0)
                                  + "    nextRetrySec: " + (backend.cloudStatus.nextRetrySec || 0)
                                  + "    consecutiveFailures: " + (backend.cloudStatus.consecutiveFailures || 0)
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
