import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 14

    Label { text: "本地 APP 网桥"; color: Theme.text; font.pixelSize: 26; font.bold: true }
    StatusCard {
        Layout.fillWidth: true
        title: "局域网服务"
        value: backend.localizedStatus(backend.systemStatus.mobile_bridge || "stopped") + " / " + backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped")
        statusColor: backend.systemStatus.mobile_bridge_http === "http_ok" ? Theme.success : Theme.warning
    }
    Label { text: "Jetson IP: " + backend.jetsonIp + "    APP 地址: " + backend.appUrl; color: Theme.text; wrapMode: Text.Wrap; Layout.fillWidth: true }
    RowLayout {
        Layout.fillWidth: true
        visible: !backend.systemStatus.mobile_bridge_managed_externally
        WarmButton { text: "启动网桥"; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("start_mobile_bridge") }
        WarmButton { text: "重启网桥"; buttonColor: Theme.warning; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("restart_mobile_bridge") }
        WarmButton { text: "停止网桥"; buttonColor: Theme.danger; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("stop_mobile_bridge") }
    }
    Label { visible: !!backend.systemStatus.mobile_bridge_managed_externally; text: "由 systemd 常驻管理"; color: Theme.muted }

    Label { text: "云平台连接"; color: Theme.text; font.pixelSize: 22; font.bold: true }
    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 210
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            RowLayout {
                Layout.fillWidth: true
                Label { text: backend.cloudStatus.state || "UNCONFIGURED"; color: Theme.text; Layout.fillWidth: true }
                Switch {
                    id: cloudSwitch
                    enabled: !!backend.cloudStatus.configured
                    checked: !!backend.cloudStatus.desiredEnabled
                    onToggled: {
                        if (!checked && backend.cloudStatus.activeExecutionId) cloudDisableDialog.open()
                        else backend.setCloudEnabled(checked)
                    }
                }
            }
            Label { text: "服务器: " + (backend.cloudStatus.serverBaseUrl || "未配置"); color: Theme.primary; Layout.fillWidth: true; wrapMode: Text.Wrap }
            Label { text: "最近成功: " + (backend.cloudStatus.lastSuccessAt || "-") + "    下次重试: " + (backend.cloudStatus.nextRetrySec || 0) + " 秒"; color: Theme.text }
            Label { text: "最近错误: " + (backend.cloudStatus.lastError || "-"); color: Theme.warning; Layout.fillWidth: true; wrapMode: Text.Wrap }
            Label { text: "待上传事件: " + (backend.cloudStatus.pendingEventCount || 0) + "    Execution: " + (backend.cloudStatus.activeExecutionId || "-"); color: Theme.text; Layout.fillWidth: true; wrapMode: Text.Wrap }
            Label { text: "Deployment: " + (backend.cloudStatus.activeDeploymentId || "-"); color: Theme.text; Layout.fillWidth: true; wrapMode: Text.Wrap }
        }
    }
    Dialog {
        id: cloudDisableDialog
        modal: true
        title: "确认关闭云连接"
        standardButtons: Dialog.Yes | Dialog.No
        Label { text: "关闭云平台连接不会停止当前巡检，但会暂停状态上报和云端命令。确认关闭？"; wrapMode: Text.Wrap; width: 420 }
        onAccepted: backend.setCloudEnabled(false)
        onRejected: cloudSwitch.checked = Qt.binding(function() { return !!backend.cloudStatus.desiredEnabled })
    }
    Item { Layout.fillHeight: true }
}
