import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: Theme.pageMargin
    spacing: 12

    Label { text: "事件日志"; color: Theme.text; font.pixelSize: 26; font.bold: true }
    ListView {
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true
        spacing: 6
        model: backend.logs
        delegate: Rectangle {
            required property var modelData
            width: ListView.view.width
            height: 48
            radius: Theme.cardRadius
            color: Theme.surface
            border.color: Theme.border
            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                Label { text: modelData.timestamp; color: Theme.muted; Layout.preferredWidth: 80 }
                Label { text: modelData.message; color: Theme.text; Layout.fillWidth: true; elide: Text.ElideRight }
            }
        }
    }
}
