import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 16

    Label { text: "语音与 AI"; color: Theme.text; font.pixelSize: 26; font.bold: true }

    StatusCard {
        Layout.fillWidth: true
        title: "语音状态"
        value: backend.systemStatus.voice_status || "等待语音状态"
        statusColor: Theme.primary
    }

    StatusCard {
        Layout.fillWidth: true
        title: "AI 状态"
        value: backend.agentStatusText || "等待 Agent 状态"
        statusColor: Theme.accent
    }

    RowLayout {
        Layout.fillWidth: true
        WarmButton { text: "启动语音会话"; Layout.fillWidth: true; onClicked: backend.callVoiceService("start") }
        WarmButton { text: "采集语音"; Layout.fillWidth: true; onClicked: backend.callVoiceService("capture") }
        WarmButton { text: "停止语音会话"; buttonColor: Theme.danger; Layout.fillWidth: true; onClicked: backend.callVoiceService("stop") }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 180
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 16
            Label { text: "文本指令"; color: Theme.muted }
            TextArea {
                id: commandText
                Layout.fillWidth: true
                Layout.fillHeight: true
                wrapMode: TextEdit.Wrap
                placeholderText: "输入巡检或语音控制指令"
            }
            WarmButton {
                text: "发送到 AI"
                Layout.alignment: Qt.AlignRight
                Layout.preferredWidth: 160
                onClicked: {
                    backend.sendTextCommand(commandText.text)
                    commandText.text = ""
                }
            }
        }
    }

    Item { Layout.fillHeight: true }
}
