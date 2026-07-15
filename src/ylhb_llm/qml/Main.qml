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

    property int currentPage: 1
    property var pageSources: [
        "pages/BridgePage.qml",
        "pages/PatrolPage.qml",
        "pages/Mapping3DPage.qml",
        "pages/StatusPage.qml",
        "pages/VoiceAiPage.qml",
        "pages/LogsPage.qml"
    ]

    onClosing: function(close) {
        if (backend.shutdownPending) {
            close.accepted = true
        } else {
            close.accepted = false
            shutdownDialog.open()
        }
    }

    Image {
        anchors.fill: parent
        source: backend.uiReady ? backend.assetPath("背景图2 (1).png") : ""
        fillMode: Image.PreserveAspectCrop
        cache: true
        opacity: 0.28
    }

    Rectangle {
        anchors.fill: parent
        color: "#EFFFFFFF"
    }

    Item {
        id: safeArea
        objectName: "safeArea"
        anchors.fill: parent
        anchors.leftMargin: backend.uiSafeMarginLeft
        anchors.rightMargin: backend.uiSafeMarginRight
        anchors.topMargin: backend.uiSafeMarginTop
        anchors.bottomMargin: backend.uiSafeMarginBottom
        enabled: !backend.shutdownPending

        RowLayout {
            anchors.fill: parent
            spacing: 0

            Rectangle {
                objectName: "sidebar"
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
                        model: ["连接与服务", "巡逻模式", "三维建模", "本机状态", "语音与 AI", "日志"]
                        delegate: Button {
                            required property int index
                            required property string modelData
                            objectName: "navigationButton" + index
                            Layout.fillWidth: true
                            Layout.preferredHeight: Theme.minimumTouchHeight
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

                    Button {
                        objectName: "closeConsoleButton"
                        Layout.fillWidth: true
                        Layout.preferredHeight: Theme.minimumTouchHeight
                        text: backend.shutdownPending ? "正在关闭…" : "关闭操控台"
                        font.pixelSize: 15
                        font.bold: true
                        background: Rectangle {
                            radius: 8
                            color: Theme.dangerSoft
                            border.color: Theme.danger
                        }
                        contentItem: Text {
                            text: parent.text
                            color: Theme.danger
                            font: parent.font
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        onClicked: shutdownDialog.open()
                    }

                    SafetyStopButton {
                        objectName: "emergencyStopButton"
                        Layout.fillWidth: true
                        Layout.preferredHeight: 52
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
                    source: backend.uiReady ? window.pageSources[window.currentPage] : "pages/StartupLoadingPage.qml"
                }
            }
        }
    }

    Dialog {
        id: shutdownDialog
        objectName: "shutdownDialog"
        parent: safeArea
        anchors.centerIn: safeArea
        width: Math.min(safeArea.width - 40, 620)
        height: Math.min(safeArea.height - 40, 340)
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape
        title: "关闭操控台？"
        contentItem: Label {
            width: shutdownDialog.availableWidth
            text: "关闭后将停止本次操控台、Agent、语音、系统监督，以及由系统监督管理的巡逻、导航、感知和底盘进程。\n\n由systemd管理的Mobile Bridge将继续运行。\n\n确认关闭？"
            color: Theme.text
            font.pixelSize: 16
            wrapMode: Text.Wrap
            padding: 18
        }
        footer: Item {
            implicitHeight: 64
            RowLayout {
                anchors.fill: parent
                spacing: 10
                Item { Layout.fillWidth: true }
                Button {
                    objectName: "cancelShutdownButton"
                    text: "取消"
                    Layout.preferredWidth: 120
                    Layout.preferredHeight: Theme.minimumTouchHeight
                    onClicked: shutdownDialog.close()
                }
                Button {
                    objectName: "confirmShutdownButton"
                    text: "确认关闭"
                    Layout.preferredWidth: 140
                    Layout.preferredHeight: Theme.minimumTouchHeight
                    onClicked: {
                        shutdownDialog.close()
                        backend.requestInspectionShutdown()
                    }
                }
            }
        }
    }
}
