import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Rectangle {
    id: root
    color: "#F3F6F8"

    property color cardColor: "#FFFFFF"
    property color accentColor: "#0891B2"
    property color accentSoftColor: "#ECFEFF"
    property color borderColor: "#CBD5E1"
    property color textColor: "#0F172A"
    property color mutedColor: "#64748B"

    function safeMarkdown(text) {
        return String(text || "")
            .replace(/!\[[^\]]*\]\([^)]*\)/g, "[图片已隐藏]")
            .replace(/<[^>]+>/g, "")
    }

    function voiceColor() {
        if (backend.voiceActivityTone === "active") return "#059669"
        if (backend.voiceActivityTone === "busy") return "#D97706"
        if (backend.voiceActivityTone === "speaking") return accentColor
        if (backend.voiceActivityTone === "wake") return "#2563EB"
        return mutedColor
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 24
        spacing: 16

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 96
            radius: 12
            color: cardColor
            border.color: borderColor

            Rectangle {
                width: 7
                radius: 4
                color: voiceColor()
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.bottom: parent.bottom
            }

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 26
                anchors.rightMargin: 20
                anchors.topMargin: 14
                anchors.bottomMargin: 14
                spacing: 24

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 3
                    Label {
                        text: "电力巡检机器人 AI Agent"
                        color: textColor
                        font.pixelSize: 28
                        font.bold: true
                    }
                    Label {
                        text: backend.voiceActivityText || "语音状态未知"
                        color: voiceColor()
                        font.pixelSize: 22
                        font.bold: true
                    }
                }

                Rectangle {
                    Layout.preferredWidth: followupText.implicitWidth + 28
                    Layout.preferredHeight: 44
                    radius: 22
                    color: accentSoftColor
                    visible: followupText.remaining > 0
                    Label {
                        id: followupText
                        property int remaining: Number(
                            backend.voiceSessionStatus.followup_remaining_sec || 0)
                        anchors.centerIn: parent
                        text: "可继续讲话 " + Math.ceil(remaining) + " 秒"
                        color: accentColor
                        font.pixelSize: 18
                        font.bold: true
                    }
                }

                Switch {
                    text: checked ? "语音已开启" : "语音已关闭"
                    checked: backend.voiceSessionEnabled
                    font.pixelSize: 18
                    onClicked: backend.callVoiceService(checked ? "start" : "stop")
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 16

            Rectangle {
                Layout.preferredWidth: Math.min(480, root.width * 0.34)
                Layout.minimumWidth: 360
                Layout.fillHeight: true
                radius: 12
                color: cardColor
                border.color: borderColor

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 22
                    spacing: 14

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            text: "当前任务"
                            color: textColor
                            font.pixelSize: 22
                            font.bold: true
                            Layout.fillWidth: true
                        }
                        Rectangle {
                            Layout.preferredWidth: agentStateText.implicitWidth + 20
                            Layout.preferredHeight: 34
                            radius: 17
                            color: accentSoftColor
                            Label {
                                id: agentStateText
                                anchors.centerIn: parent
                                text: backend.agentStatus.state || "idle"
                                color: accentColor
                                font.pixelSize: 15
                                font.bold: true
                            }
                        }
                    }

                    Label {
                        text: "当前目标"
                        color: mutedColor
                        font.pixelSize: 16
                    }
                    Label {
                        text: backend.agentStatus.current_goal || "等待新的语音指令"
                        color: textColor
                        font.pixelSize: 24
                        font.bold: true
                        wrapMode: Text.Wrap
                        Layout.fillWidth: true
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Math.max(120, stepColumn.implicitHeight + 30)
                        radius: 10
                        color: accentSoftColor
                        border.color: "#A5F3FC"
                        ColumnLayout {
                            id: stepColumn
                            anchors.fill: parent
                            anchors.margins: 16
                            spacing: 8
                            Label {
                                text: "当前步骤"
                                color: accentColor
                                font.pixelSize: 16
                                font.bold: true
                            }
                            Label {
                                text: backend.agentStatus.current_step || "等待指令"
                                color: textColor
                                font.pixelSize: 22
                                font.bold: true
                                wrapMode: Text.Wrap
                                Layout.fillWidth: true
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }

                    Label {
                        text: backend.agentStatus.pending_operation_id
                            ? "正在等待机器人真实反馈"
                            : "语音指令和最终结果会显示在右侧"
                        color: mutedColor
                        font.pixelSize: 16
                        wrapMode: Text.Wrap
                        Layout.fillWidth: true
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 12
                color: cardColor
                border.color: borderColor

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 22
                    spacing: 14

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            text: "最终结果"
                            color: textColor
                            font.pixelSize: 24
                            font.bold: true
                            Layout.fillWidth: true
                        }
                        Label {
                            text: backend.agentStatus.state === "waiting_feedback"
                                ? "等待真实反馈" : "完整回答"
                            color: backend.agentStatus.state === "waiting_feedback"
                                ? "#D97706" : accentColor
                            font.pixelSize: 17
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        height: 1
                        color: borderColor
                    }

                    ScrollView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        Label {
                            width: parent.width
                            text: safeMarkdown(
                                backend.agentStatus.final_result || "说出唤醒词后下达指令，完整结果将在这里显示。")
                            textFormat: Text.MarkdownText
                            color: textColor
                            font.pixelSize: 20
                            lineHeight: 1.35
                            wrapMode: Text.Wrap
                        }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 210
            radius: 12
            color: cardColor
            border.color: borderColor

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 10

                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: "最近执行过程"
                        color: textColor
                        font.pixelSize: 20
                        font.bold: true
                        Layout.fillWidth: true
                    }
                    Label {
                        text: "最近 8 条"
                        color: mutedColor
                        font.pixelSize: 15
                    }
                }

                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    ColumnLayout {
                        width: parent.width
                        spacing: 6
                        Repeater {
                            model: (backend.agentStatus.steps || []).slice(-8)
                            delegate: Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: detailButton.checked
                                    ? detailColumn.implicitHeight + 18 : 42
                                radius: 7
                                color: index % 2 === 0 ? "#F8FAFC" : "#F1F5F9"

                                ColumnLayout {
                                    id: detailColumn
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    anchors.margins: 8
                                    spacing: 4
                                    RowLayout {
                                        Layout.fillWidth: true
                                        Label {
                                            text: (index + 1) + ".  " + (modelData.summary || "执行步骤")
                                            color: textColor
                                            font.pixelSize: 16
                                            Layout.fillWidth: true
                                            elide: Text.ElideRight
                                        }
                                        ToolButton {
                                            id: detailButton
                                            text: checked ? "收起" : "查看详情"
                                            checkable: true
                                            visible: Object.keys(modelData.detail || {}).length > 0
                                        }
                                    }
                                    Label {
                                        visible: detailButton.checked
                                        text: JSON.stringify(modelData.detail || {}, null, 2)
                                        color: mutedColor
                                        font.family: "monospace"
                                        font.pixelSize: 14
                                        wrapMode: Text.WrapAnywhere
                                        Layout.fillWidth: true
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
