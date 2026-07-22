import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    color: Theme.surface
    border.color: Theme.border

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: Theme.pageMargin
        anchors.rightMargin: Theme.pageMargin
        spacing: Theme.controlSpacing

        Label {
            text: "机器人助手"
            color: Theme.text
            font.pixelSize: 20
            font.bold: true
        }
        Item { Layout.fillWidth: true }
        Rectangle {
            implicitWidth: appStatus.implicitWidth + 30
            implicitHeight: 34
            radius: Theme.cardRadius
            color: backend.systemStatus.mobile_bridge_http === "http_ok" ? Theme.successSoft : Theme.warningSoft
            Row {
                anchors.centerIn: parent
                spacing: 8
                Rectangle {
                    width: 8
                    height: 8
                    radius: 4
                    anchors.verticalCenter: parent.verticalCenter
                    color: backend.systemStatus.mobile_bridge_http === "http_ok" ? Theme.success : Theme.warning
                }
                Label {
                    id: appStatus
                    text: "APP " + backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped")
                    color: Theme.text
                }
            }
        }
        Label {
            text: "模式 " + backend.localizedStatus(backend.robotMode)
            color: Theme.muted
        }
    }
}
