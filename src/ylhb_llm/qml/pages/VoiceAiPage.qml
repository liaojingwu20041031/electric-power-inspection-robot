import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 14

    Label { text: "语言 Agent 控制台"; color: Theme.text; font.pixelSize: 26; font.bold: true }

    RowLayout {
        Layout.fillWidth: true
        WarmButton { text: "启动"; Layout.fillWidth: true; onClicked: backend.callVoiceService("start") }
        WarmButton { text: "单次采集"; Layout.fillWidth: true; onClicked: backend.callVoiceService("capture") }
        WarmButton { text: "停止"; buttonColor: Theme.danger; Layout.fillWidth: true; onClicked: backend.callVoiceService("stop") }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 64
        radius: 8
        color: backend.voiceActivityTone === "active" ? Theme.success
             : backend.voiceActivityTone === "busy" ? Theme.warning
             : backend.voiceActivityTone === "speaking" ? Theme.accent
             : backend.voiceActivityTone === "wake" ? Theme.primary
             : Theme.muted
        RowLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 12
            Rectangle {
                width: 14
                height: 14
                radius: 7
                color: Theme.surface
                opacity: backend.voiceActivityTone === "active" ? 1.0 : 0.7
            }
            Label {
                text: backend.voiceActivityText || "语音状态未知"
                color: Theme.surface
                font.pixelSize: 22
                font.bold: true
                Layout.fillWidth: true
                elide: Text.ElideRight
            }
        }
    }

    GridLayout {
        Layout.fillWidth: true
        columns: 2
        rowSpacing: 12
        columnSpacing: 12

        StatusCard { Layout.fillWidth: true; title: "会话状态"; value: backend.voiceStatusSummary || "关闭"; statusColor: Theme.primary }
        StatusCard { Layout.fillWidth: true; title: "Agent 状态"; value: backend.agentStatusSummary || "ready"; statusColor: Theme.accent }
        StatusCard { Layout.fillWidth: true; title: "唤醒词"; value: backend.voiceWakePhrase || "-"; statusColor: Theme.primary }
        StatusCard { Layout.fillWidth: true; title: "最近结果"; value: backend.agentLastResult || "-"; statusColor: Theme.accent }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 160
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 6
            Label { text: "语音诊断"; color: Theme.muted }
            Label { text: "最后识别: " + (backend.voiceLastAsrText || "-"); color: Theme.text; elide: Text.ElideRight; Layout.fillWidth: true }
            Label { text: "最后发布: " + (backend.voiceLastPublishedText || "-"); color: Theme.text; elide: Text.ElideRight; Layout.fillWidth: true }
            Label { text: "失败次数: " + backend.voiceAsrFailCount + "  录音: " + backend.voiceRecording + "  播报: " + backend.voiceSpeaking; color: Theme.text; Layout.fillWidth: true }
            Label { text: "最后错误: " + (backend.voiceLastError || "-"); color: Theme.muted; elide: Text.ElideRight; Layout.fillWidth: true }
            Label { text: "服务: " + (backend.voiceServiceStatus || "-"); color: Theme.muted; elide: Text.ElideRight; Layout.fillWidth: true }
        }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 170
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 6
            Label { text: "Agent 诊断"; color: Theme.muted }
            Label { text: "intent: " + (backend.agentLastIntent || "-"); color: Theme.text; Layout.fillWidth: true }
            Label { text: "tool: " + (backend.agentLastTool || "-"); color: Theme.text; Layout.fillWidth: true }
            Label { text: "error: " + (backend.agentLastError || "-"); color: Theme.muted; Layout.fillWidth: true }
            Repeater {
                model: Math.min(backend.agentEvents.length, 5)
                Label {
                    property var eventData: backend.agentEvents[backend.agentEvents.length - model.index - 1] || ({})
                    text: (eventData.status || "-") + " " + (eventData.tool_name || "") + " " + (eventData.message || "")
                    color: Theme.text
                    elide: Text.ElideRight
                    Layout.fillWidth: true
                }
            }
        }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 150
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            Label { text: "文本输入"; color: Theme.muted }
            TextArea {
                id: commandText
                Layout.fillWidth: true
                Layout.fillHeight: true
                wrapMode: TextEdit.Wrap
                placeholderText: "输入巡检、运动或状态问题"
            }
            WarmButton {
                text: "发送到语言 Agent"
                Layout.alignment: Qt.AlignRight
                Layout.preferredWidth: 180
                onClicked: {
                    backend.sendAgentText(commandText.text)
                    commandText.text = ""
                }
            }
        }
    }
}
