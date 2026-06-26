import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    property string title: ""
    property string value: "-"
    property color statusColor: Theme.muted
    implicitHeight: 104
    radius: 8
    color: Theme.surface
    border.color: Theme.border

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
