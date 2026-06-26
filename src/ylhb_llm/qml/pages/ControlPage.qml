import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 18

    RowLayout {
        Layout.fillWidth: true
        Label { text: "运动控制"; color: Theme.text; font.pixelSize: 26; font.bold: true }
        Item { Layout.fillWidth: true }
        Label { text: backend.controlUnlocked ? "已解锁，10 秒无操作自动锁定" : "控制已锁定"; color: backend.controlUnlocked ? Theme.success : Theme.danger }
        Switch {
            checked: backend.controlUnlocked
            onToggled: backend.setControlUnlocked(checked)
        }
    }

    GridLayout {
        Layout.alignment: Qt.AlignHCenter
        columns: 3
        rowSpacing: 12
        columnSpacing: 12
        Item { width: 150; height: 58 }
        WarmButton { text: "前进"; width: 150; height: 58; enabled: backend.controlUnlocked; onClicked: backend.moveForward() }
        Item { width: 150; height: 58 }
        WarmButton { text: "左转"; width: 150; height: 58; enabled: backend.controlUnlocked; onClicked: backend.turnLeft() }
        WarmButton { text: "停止"; width: 150; height: 58; buttonColor: Theme.warning; onClicked: backend.stopMotion() }
        WarmButton { text: "右转"; width: 150; height: 58; enabled: backend.controlUnlocked; onClicked: backend.turnRight() }
        Item { width: 150; height: 58 }
        WarmButton { text: "后退"; width: 150; height: 58; enabled: backend.controlUnlocked; onClicked: backend.moveBackward() }
        Item { width: 150; height: 58 }
    }
    SafetyStopButton {
        Layout.preferredWidth: 474
        Layout.alignment: Qt.AlignHCenter
        onClicked: backend.emergencyStop()
    }
    Item { Layout.fillHeight: true }
}
