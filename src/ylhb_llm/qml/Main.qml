import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "components"
import "pages"
import "."

ApplicationWindow {
    id: window
    visible: false
    width: 1280
    height: 800
    minimumWidth: 960
    minimumHeight: 640
    title: "电力巡检机器人操控台"
    color: Theme.background

    property int currentPage: 0
    property var pageSources: [
        "pages/BridgePage.qml",
        "pages/PatrolPage.qml",
        "pages/StatusPage.qml",
        "pages/VoiceAiPage.qml",
        "pages/LogsPage.qml"
    ]

    Image {
        anchors.fill: parent
        source: backend.assetPath("背景图2 (1).png")
        fillMode: Image.PreserveAspectCrop
        cache: true
        opacity: 0.28
    }

    Rectangle {
        anchors.fill: parent
        color: "#EFFFFFFF"
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            Layout.preferredWidth: 210
            Layout.fillHeight: true
            color: "#F8FFFFFF"
            border.color: Theme.border

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 8

                RowLayout {
                    Layout.fillWidth: true
                    Layout.bottomMargin: 14
                    spacing: 10
                    Image {
                        source: backend.assetPath("UI图标.png")
                        Layout.preferredWidth: 36
                        Layout.preferredHeight: 36
                        fillMode: Image.PreserveAspectFit
                        cache: true
                    }
                    Label {
                        text: "电力巡检机器人"
                        color: Theme.text
                        font.pixelSize: 18
                        font.bold: true
                        wrapMode: Text.Wrap
                        Layout.fillWidth: true
                    }
                }

                Repeater {
                    model: ["APP 网桥", "巡逻模式", "本机状态", "语音与 AI", "日志"]
                    delegate: Button {
                        required property int index
                        required property string modelData
                        Layout.fillWidth: true
                        Layout.preferredHeight: 44
                        text: modelData
                        flat: true
                        font.pixelSize: 15
                        background: Rectangle {
                            radius: 6
                            color: window.currentPage === index ? Theme.border : "transparent"
                        }
                        contentItem: Text {
                            text: parent.text
                            color: window.currentPage === index ? Theme.primary : Theme.text
                            font: parent.font
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 12
                        }
                        onClicked: window.currentPage = index
                    }
                }

                Item { Layout.fillHeight: true }
                SafetyStopButton {
                    Layout.fillWidth: true
                    onClicked: backend.emergencyStop()
                }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            TopStatusBar {
                Layout.fillWidth: true
                Layout.preferredHeight: 64
            }

            Loader {
                Layout.fillWidth: true
                Layout.fillHeight: true
                source: window.pageSources[window.currentPage]
            }
        }
    }
}
