import QtQuick 2.12
import QtQuick.Controls 2.12
import ".."

Button {
    id: control
    text: "紧急停止"
    implicitHeight: 52
    font.pixelSize: 17
    font.bold: true
    background: Rectangle {
        radius: Theme.cardRadius
        color: Theme.danger
    }
    contentItem: Text {
        text: control.text
        color: Theme.surface
        font: control.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}
