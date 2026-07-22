import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Item {
    property string title: ""
    property string value: "-"
    property color statusColor: Theme.muted
    implicitHeight: 104
    Rectangle { x: 0; y: 2; width: parent.width; height: parent.height; radius: Theme.cardRadius; color: "#100F5C8A" }
    Rectangle { anchors.fill: parent; radius: Theme.cardRadius; color: Theme.surface; border.color: Theme.border }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12
        Rectangle {
            width: 12
            height: 12
            radius: 6
            color: statusColor
        }
        ColumnLayout {
            Layout.fillWidth: true
            Label { text: title; color: Theme.muted; font.pixelSize: 13 }
            Label {
                text: value
                color: Theme.text
                font.pixelSize: 19
                font.bold: true
                wrapMode: Text.Wrap
                Layout.fillWidth: true
            }
        }
    }
}
