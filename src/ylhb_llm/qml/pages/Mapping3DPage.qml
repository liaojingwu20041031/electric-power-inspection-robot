import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    id: root
    clip: true
    contentWidth: availableWidth
    property var latestCapture: backend.systemStatus.latest_3d_capture || ({})
    property var latestReconstruct: backend.systemStatus.latest_3d_reconstruct || ({})

    ColumnLayout {
        width: parent.width
        anchors.margins: 22
        spacing: 16

        Label { text: "三维建模"; color: Theme.text; font.pixelSize: 26; font.bold: true }

        Label { text: "总览状态"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        GridLayout {
            Layout.fillWidth: true
            columns: root.availableWidth >= 1000 ? 4 : 2
            columnSpacing: 12
            rowSpacing: 10
            StatusCard {
                Layout.fillWidth: true
                title: "采集状态"
                value: backend.mapping3dCaptureText
                statusColor: backend.systemStatus["3d_capture"] === "running" ? Theme.success : Theme.warning
            }
            StatusCard {
                Layout.fillWidth: true
                title: "重建状态"
                value: backend.mapping3dReconstructText
                statusColor: backend.systemStatus["3d_reconstruct"] === "running" ? Theme.primary : Theme.muted
            }
            StatusCard {
                Layout.fillWidth: true
                title: "最新 SVO"
                value: backend.latestSvoFile || "未采集"
                statusColor: backend.latestSvoFile.length > 0 ? Theme.success : Theme.warning
            }
            StatusCard {
                Layout.fillWidth: true
                title: "最新模型"
                value: backend.latestModelFile || "未生成"
                statusColor: backend.latestModelFile.length > 0 ? Theme.success : Theme.muted
            }
        }

        Label { text: "现场采集"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 178
            radius: 8
            color: Theme.surface
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 10
                RowLayout {
                    Layout.fillWidth: true
                    WarmButton {
                        text: "开始采集"
                        enabled: backend.mapping3dCanStartCapture
                        Layout.fillWidth: true
                        onClicked: backend.start3dCapture()
                    }
                    WarmButton {
                        text: "停止并保存 SVO"
                        enabled: backend.mapping3dCanStopCapture
                        buttonColor: Theme.danger
                        Layout.fillWidth: true
                        onClicked: backend.stop3dCapture()
                    }
                }
                Label {
                    text: "帧数: " + (backend.mapping3dStatus.svo_frame_count || backend.mapping3dStatus.success_frames || root.latestCapture.svo_frame_count || 0)
                    color: Theme.text
                    font.pixelSize: 15
                }
                Label {
                    text: "时长: " + (backend.mapping3dStatus.capture_duration_sec || 0) + " s"
                    color: Theme.text
                    font.pixelSize: 15
                }
                Label {
                    text: "目录: " + (backend.mapping3dStatus.output_dir || root.latestCapture.output_dir || "-")
                    color: Theme.muted
                    font.pixelSize: 14
                    wrapMode: Text.Wrap
                    Layout.fillWidth: true
                }
            }
        }

        Label { text: "离线重建"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 178
            radius: 8
            color: Theme.surface
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 10
                RowLayout {
                    Layout.fillWidth: true
                    WarmButton {
                        text: "快速重建"
                        enabled: backend.mapping3dCanReconstruct
                        Layout.fillWidth: true
                        onClicked: backend.reconstructLatest3dMap("fast_check")
                    }
                    WarmButton {
                        text: "高质量重建"
                        enabled: backend.mapping3dCanReconstruct
                        buttonColor: Theme.primary
                        Layout.fillWidth: true
                        onClicked: backend.reconstructLatest3dMap("quality_plus")
                    }
                }
                Label {
                    text: "profile: " + (backend.mapping3dResult.reconstruct_profile || root.latestReconstruct.reconstruct_profile || "quality_safe")
                    color: Theme.text
                    font.pixelSize: 15
                }
                Label {
                    text: "输出文件: " + (backend.latestModelFile || "-")
                    color: Theme.muted
                    font.pixelSize: 14
                    wrapMode: Text.Wrap
                    Layout.fillWidth: true
                }
                Label {
                    text: "点数: " + (backend.mapping3dResult.export_point_count || root.latestReconstruct.export_point_count || 0)
                    color: Theme.text
                    font.pixelSize: 15
                }
            }
        }

        Label { text: "查看说明"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: 132
            radius: 8
            color: Theme.surface
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 8
                Label { text: "Fixed Frame: zed_3d_map"; color: Theme.text; font.pixelSize: 15 }
                Label { text: "Topic: /inspection_ai/mapping3d_pointcloud"; color: Theme.text; font.pixelSize: 15 }
                Label { text: "Color field: z"; color: Theme.text; font.pixelSize: 15 }
                Label {
                    text: "PLY 可用 CloudCompare / MeshLab / Open3D 查看"
                    color: Theme.muted
                    font.pixelSize: 15
                    wrapMode: Text.Wrap
                    Layout.fillWidth: true
                }
            }
        }
    }
}
