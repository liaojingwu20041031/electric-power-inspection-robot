import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    clip: true
    contentWidth: availableWidth

    ColumnLayout {
        width: parent.width
        spacing: 12
        anchors.margins: Theme.pageMargin

        Label {
            text: "系统总览"
            color: Theme.text
            font.pixelSize: 26
            font.bold: true
        }

        GridLayout {
            Layout.fillWidth: true
            columns: width > 760 ? 3 : 2
            columnSpacing: 14
            rowSpacing: 14

            StatusCard { Layout.fillWidth: true; title: "底盘与传感器"; value: backend.systemStatus.bringup || "stopped"; statusColor: value === "running" ? Theme.success : Theme.warning }
            StatusCard { Layout.fillWidth: true; title: "导航"; value: backend.systemStatus.navigation || "stopped"; statusColor: value === "running" ? Theme.success : Theme.warning }
            StatusCard { Layout.fillWidth: true; title: "视觉感知"; value: backend.systemStatus.perception || "stopped"; statusColor: value === "running" ? Theme.success : Theme.warning }
            StatusCard { Layout.fillWidth: true; title: "APP Bridge"; value: backend.systemStatus.mobile_bridge || "stopped"; statusColor: value === "running" ? Theme.success : Theme.warning }
            StatusCard { Layout.fillWidth: true; title: "Bridge HTTP"; value: backend.systemStatus.mobile_bridge_http || "stopped"; statusColor: value === "http_ok" ? Theme.success : Theme.warning }
            StatusCard { Layout.fillWidth: true; title: "Jetson IP"; value: backend.jetsonIp; statusColor: Theme.primary }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 126
            radius: 8
            color: Theme.surface
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                Label { text: "最近状态"; color: Theme.muted }
                Label { text: backend.systemStatus.message || "等待 supervisor 状态"; color: Theme.text; font.pixelSize: 18; wrapMode: Text.Wrap; Layout.fillWidth: true }
                Label { text: "APP: " + backend.appUrl; color: Theme.primary; wrapMode: Text.Wrap; Layout.fillWidth: true }
            }
        }
    }
}
