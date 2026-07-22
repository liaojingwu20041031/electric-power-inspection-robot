import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Item {
    id: root
    property string label: ""
    property string value: "-"
    property string tooltip: ""
    property color valueColor: Theme.text

    implicitHeight: 100
    Rectangle { x: 0; y: 2; width: parent.width; height: parent.height; radius: Theme.cardRadius; color: "#0D0F5C8A" }
    Rectangle { anchors.fill: parent; radius: Theme.cardRadius; color: Theme.surfaceAlt; border.color: Theme.border }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 5
        Label { text: root.label; color: Theme.muted; font.pixelSize: 14 }
        Label {
            Layout.fillWidth: true
            text: root.value
            color: root.valueColor
            font.pixelSize: 19
            font.bold: true
            elide: Text.ElideMiddle
        }
    }
    MouseArea { id: hoverArea; anchors.fill: parent; hoverEnabled: true }
    ToolTip.visible: hoverArea.containsMouse && root.tooltip.length > 0
    ToolTip.text: root.tooltip
}
