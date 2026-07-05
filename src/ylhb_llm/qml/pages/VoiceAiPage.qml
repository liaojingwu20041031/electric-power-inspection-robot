import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 12

    Label { text: "语言 Agent"; color: Theme.text; font.pixelSize: 26; font.bold: true }

    GridLayout {
        Layout.fillWidth: true
        columns: 3
        rowSpacing: 10
        columnSpacing: 10

        StatusCard { Layout.fillWidth: true; title: "能力"; value: backend.agentSpecSummary.name || "inspection_agent"; statusColor: Theme.accent }
        StatusCard { Layout.fillWidth: true; title: "语音"; value: backend.voiceStatusSummary || "关闭"; statusColor: Theme.primary }
        StatusCard { Layout.fillWidth: true; title: "巡逻"; value: backend.patrolStateLabel || "-"; statusColor: Theme.success }
    }

    RowLayout {
        Layout.fillWidth: true
        WarmButton { text: "启动"; Layout.fillWidth: true; onClicked: backend.callVoiceService("start") }
        WarmButton { text: "单次采集"; Layout.fillWidth: true; onClicked: backend.callVoiceService("capture") }
        WarmButton { text: "停止"; buttonColor: Theme.danger; Layout.fillWidth: true; onClicked: backend.callVoiceService("stop") }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: 42
        radius: 8
        color: backend.voiceActivityTone === "active" ? Theme.success
             : backend.voiceActivityTone === "busy" ? Theme.warning
             : backend.voiceActivityTone === "speaking" ? Theme.accent
             : backend.voiceActivityTone === "wake" ? Theme.primary
             : Theme.muted
        Label {
            anchors.fill: parent
            anchors.margins: 10
            text: backend.voiceActivityText || "语音状态未知"
            color: Theme.surface
            font.pixelSize: 18
            font.bold: true
            elide: Text.ElideRight
        }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.fillHeight: true
        radius: 8
        color: Theme.surface
        border.color: Theme.border

        ListView {
            id: chatList
            anchors.fill: parent
            anchors.margins: 12
            spacing: 10
            clip: true
            model: backend.agentMessages
            onCountChanged: positionViewAtEnd()

            delegate: Item {
                width: chatList.width
                height: bubble.height
                property bool mine: modelData.role === "user"
                property int bubbleWidth: Math.min(width * 0.82, 640)

                Rectangle {
                    id: bubble
                    width: parent.bubbleWidth
                    height: textItem.implicitHeight + metaItem.implicitHeight + 22
                    anchors.right: mine ? parent.right : undefined
                    anchors.left: mine ? undefined : parent.left
                    radius: 8
                    color: mine ? Theme.primary : (modelData.role === "system" ? Theme.warning : Theme.background)
                    border.color: Theme.border

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 4
                        Label {
                            id: metaItem
                            text: (modelData.role || "-") + (modelData.tool_name ? " · " + modelData.tool_name : "")
                            color: mine ? Theme.surface : Theme.muted
                            font.pixelSize: 12
                            Layout.fillWidth: true
                            elide: Text.ElideRight
                        }
                        Label {
                            id: textItem
                            text: modelData.text || ""
                            color: mine ? Theme.surface : Theme.text
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                            font.pixelSize: 16
                        }
                    }
                }
            }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 10
        TextArea {
            id: commandText
            Layout.fillWidth: true
            Layout.preferredHeight: 70
            wrapMode: TextEdit.Wrap
            placeholderText: "输入巡检、运动或状态问题"
            Keys.onReturnPressed: {
                if (event.modifiers & Qt.ControlModifier) {
                    backend.sendAgentText(commandText.text)
                    commandText.text = ""
                    event.accepted = true
                }
            }
        }
        WarmButton {
            text: "发送给语言智能体"
            Layout.preferredWidth: 180
            onClicked: {
                backend.sendAgentText(commandText.text)
                commandText.text = ""
            }
        }
        WarmButton {
            text: "清空"
            Layout.preferredWidth: 90
            onClicked: backend.clearAgentMessages()
        }
        CheckBox {
            text: "诊断"
            checked: backend.agentDebugVisible
            onClicked: backend.toggleAgentDebugVisible()
        }
    }

    Rectangle {
        Layout.fillWidth: true
        Layout.preferredHeight: backend.agentDebugVisible ? 150 : 0
        visible: backend.agentDebugVisible
        radius: 8
        color: Theme.surface
        border.color: Theme.border
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 4
            Label { text: "最近 ASR: " + (backend.voiceLastAsrText || "-"); color: Theme.text; wrapMode: Text.Wrap; Layout.fillWidth: true }
            Label { text: "Agent 错误: " + (backend.agentLastError || "-"); color: Theme.muted; wrapMode: Text.Wrap; Layout.fillWidth: true }
            Label { text: "最近工具: " + (backend.agentLastTool || "-") + " / " + (backend.agentLastResult || "-"); color: Theme.text; wrapMode: Text.Wrap; Layout.fillWidth: true }
            Label { text: "TTS 状态: " + (backend.voiceTtsStatus || "-"); color: Theme.text; wrapMode: Text.Wrap; Layout.fillWidth: true }
        }
    }
}
