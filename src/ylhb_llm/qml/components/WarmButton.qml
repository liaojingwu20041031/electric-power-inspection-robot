import QtQuick 2.12
import QtQuick.Controls 2.12
import ".."

Button {
    id: control
    property color buttonColor: Theme.primary
    property color textColor: Theme.surface
    implicitHeight: 44
    font.pixelSize: 14
    font.bold: true
    background: Rectangle {
        radius: 10
        color: control.enabled ? control.buttonColor : Theme.border
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
