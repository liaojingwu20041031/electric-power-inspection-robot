import QtQuick 2.12
import QtQuick.Controls 2.12
import ".."

Button {
    id: control
    property color buttonColor: Theme.primary
    property color textColor: Theme.surface
    property color borderColor: "transparent"
    implicitHeight: 44
    font.pixelSize: 14
    font.bold: true
    background: Rectangle {
        radius: Theme.cardRadius
        color: control.enabled ? control.buttonColor : Theme.border
        border.color: control.enabled ? control.borderColor : "transparent"
    }
    contentItem: Text {
        text: control.text
        color: control.enabled ? control.textColor : Theme.muted
        font: control.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        wrapMode: Text.Wrap
    }
}
