import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Item {
    anchors.fill: parent

    ColumnLayout {
        anchors.centerIn: parent
        spacing: 12

        BusyIndicator {
            Layout.alignment: Qt.AlignHCenter
            running: true
        }
        Label {
            text: backend.startupLoadingText
            color: Theme.text
            font.pixelSize: 18
            Layout.alignment: Qt.AlignHCenter
        }
    }
}
