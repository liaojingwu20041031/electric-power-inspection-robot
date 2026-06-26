import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    color: Theme.surface
    border.color: Theme.border

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 22
        anchors.rightMargin: 22
        spacing: 14

        Label {
            text: "本体操控台"
            color: Theme.text
            font.pixelSize: 20
            font.bold: true
        }
        Item { Layout.fillWidth: true }
        Rectangle {
            width: 10
            height: 10
            radius: 5
            color: backend.systemStatus.mobile_bridge_http === "http_ok" ? Theme.success : Theme.warning
        }
        Label {
            text: "APP " + backend.localizedStatus(backend.systemStatus.mobile_bridge_http || "stopped")
            color: Theme.text
        }
        Label {
            text: "模式 " + backend.localizedStatus(backend.robotMode)
            color: Theme.muted
        }
    }
}
