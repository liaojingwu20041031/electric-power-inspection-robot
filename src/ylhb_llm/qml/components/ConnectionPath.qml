import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    id: root
    property color localColor: Theme.muted
    property color cloudColor: Theme.muted
    property bool coreRunning: false

    implicitHeight: 116
    radius: 16
    color: Theme.surface
    border.color: Theme.border

    RowLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 10

        Label { text: "手机 APP"; color: Theme.text; font.bold: true }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { Layout.alignment: Qt.AlignHCenter; text: "← 局域网 →"; color: root.localColor; font.pixelSize: 12 }
            Rectangle { Layout.fillWidth: true; height: 4; radius: 2; color: root.localColor }
        }
        Rectangle {
            implicitWidth: 132
            implicitHeight: 56
            radius: 12
            color: root.coreRunning ? Theme.successSoft : Theme.warningSoft
            border.color: root.coreRunning ? Theme.success : Theme.warning
            Label { anchors.centerIn: parent; text: "Jetson 网桥"; color: Theme.text; font.bold: true }
        }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { Layout.alignment: Qt.AlignHCenter; text: "← HTTPS →"; color: root.cloudColor; font.pixelSize: 12 }
            Rectangle { Layout.fillWidth: true; height: 4; radius: 2; color: root.cloudColor }
        }
        Label { text: "云平台"; color: Theme.text; font.bold: true }
    }
}
