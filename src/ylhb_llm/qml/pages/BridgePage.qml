import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 18

    Label { text: "APP 网桥"; color: Theme.text; font.pixelSize: 26; font.bold: true }
    StatusCard {
        Layout.fillWidth: true
        title: "服务状态"
        value: backend.localizedStatus(backend.systemStatus.mobile_bridge || "stopped") + " / " + backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped")
        statusColor: backend.systemStatus.mobile_bridge_http === "http_ok" ? Theme.success : Theme.warning
    }
    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 110
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 16
            Label { text: "Jetson IP: " + backend.jetsonIp; color: Theme.text; font.pixelSize: 17 }
            Label { text: "APP 地址: " + backend.appUrl; color: Theme.primary; font.pixelSize: 17; wrapMode: Text.Wrap; Layout.fillWidth: true }
        }
    }
    RowLayout {
        Layout.fillWidth: true
        WarmButton { text: "启动网桥"; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("start_mobile_bridge") }
        WarmButton { text: "重启网桥"; buttonColor: Theme.warning; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("restart_mobile_bridge") }
        WarmButton { text: "停止网桥"; buttonColor: Theme.danger; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("stop_mobile_bridge") }
    }
    Item { Layout.fillHeight: true }
}
