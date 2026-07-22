import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Item {
    id: root
    property color localColor: Theme.muted
    property color cloudColor: Theme.muted
    property string coreState: "stopped"

    implicitHeight: 140
    Rectangle { x: 0; y: 2; width: parent.width; height: parent.height; radius: Theme.cardRadius; color: "#100F5C8A" }
    Rectangle { anchors.fill: parent; radius: Theme.cardRadius; color: Theme.surface; border.color: Theme.border }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 24
        spacing: 10

        Label { text: "手机 APP"; color: Theme.text; font.bold: true; font.pixelSize: 16 }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { Layout.alignment: Qt.AlignHCenter; text: "← 局域网 →"; color: root.localColor; font.pixelSize: 14 }
            Rectangle { Layout.fillWidth: true; height: 4; radius: 2; color: root.localColor }
        }
        Rectangle {
            implicitWidth: 170
            implicitHeight: 72
            radius: Theme.cardRadius
            color: root.coreState === "running" ? Theme.successSoft : (root.coreState === "starting" ? Theme.infoSoft : Theme.dangerSoft)
            border.color: root.coreState === "running" ? Theme.success : (root.coreState === "starting" ? Theme.info : Theme.danger)
            Label { anchors.centerIn: parent; text: "Jetson 网桥"; color: Theme.text; font.bold: true; font.pixelSize: 17 }
        }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { Layout.alignment: Qt.AlignHCenter; text: "← HTTPS →"; color: root.cloudColor; font.pixelSize: 14 }
            Rectangle { Layout.fillWidth: true; height: 4; radius: 2; color: root.cloudColor }
        }
        Label { text: "云平台"; color: Theme.text; font.bold: true; font.pixelSize: 16 }
    }
}
