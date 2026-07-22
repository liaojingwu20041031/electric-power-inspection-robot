import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Item {
    id: root
    default property alias contentData: body.data
    property string title: ""
    property string stateTitle: ""
    property string description: ""
    property color statusColor: Theme.muted
    property color softColor: Theme.surfaceAlt

    implicitWidth: 320
    implicitHeight: Math.max(300, body.childrenRect.height + 48)
    Rectangle { x: 0; y: 2; width: parent.width; height: parent.height; radius: Theme.cardRadius; color: "#100F5C8A" }
    Rectangle { anchors.fill: parent; radius: Theme.cardRadius; color: Theme.surface; border.color: Theme.border }

    ColumnLayout {
        id: body
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 24
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            spacing: 10
            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: root.statusColor
            }
            Label {
                Layout.fillWidth: true
                text: root.title
                color: Theme.text
                font.pixelSize: 20
                font.bold: true
            }
        }
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: stateTitleLabel.implicitHeight + stateDescriptionLabel.implicitHeight + 28
            radius: Theme.cardRadius
            color: root.softColor
            ColumnLayout {
                id: stateColumn
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.margins: 10
                spacing: 4
                Label {
                    id: stateTitleLabel
                    Layout.fillWidth: true
                    text: root.stateTitle
                    color: Theme.text
                    font.pixelSize: 21
                    font.bold: true
                    wrapMode: Text.Wrap
                }
                Label {
                    id: stateDescriptionLabel
                    Layout.fillWidth: true
                    text: root.description
                    color: Theme.muted
                    font.pixelSize: 16
                    wrapMode: Text.Wrap
                }
            }
        }
    }
}
