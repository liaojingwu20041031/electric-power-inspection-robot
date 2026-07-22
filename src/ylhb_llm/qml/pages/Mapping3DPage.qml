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
    property var mapping3d_assets: backend.systemStatus.mapping3d_assets || ({})
    function openRenameDialog(assetType, sessionId, displayName) {
        renameDialog.assetType = assetType
        renameDialog.sessionId = sessionId
        renameDialog.originalName = displayName || sessionId
        renameField.text = renameDialog.originalName
        renameDialog.open()
    }
    function assetList(kind) {
        var group = root.mapping3d_assets || ({})
        return group[kind] || []
    }
    function fileName(path) {
        var normalized = String(path || "").replace(/\\/g, "/")
        return normalized.substring(normalized.lastIndexOf("/") + 1)
    }
    function uploadText(status) {
        return ({
            "PENDING": "等待上传",
            "UPLOADING": "正在上传",
            "FAILED_RETRYABLE": "网络异常，等待重试",
            "CREDENTIAL_BLOCKED": "凭据异常",
            "FAILED_FINAL": "上传失败",
            "SUCCEEDED": "已上传，待平台审核"
        })[status] || "未创建上传任务"
    }
    function uploadColor(status) {
        if (status === "SUCCEEDED") return Theme.success
        if (status === "PENDING" || status === "UPLOADING") return Theme.warning
        if (status === "FAILED_RETRYABLE") return Theme.warning
        if (status === "FAILED_FINAL" || status === "CREDENTIAL_BLOCKED") return Theme.danger
        return Theme.muted
    }
    function uploadSoftColor(status) {
        if (status === "SUCCEEDED") return Theme.successSoft
        if (status === "FAILED_FINAL" || status === "CREDENTIAL_BLOCKED") return Theme.dangerSoft
        if (status === "PENDING" || status === "UPLOADING" || status === "FAILED_RETRYABLE") return Theme.warningSoft
        return Theme.surfaceAlt
    }
    function formatBytes(value) {
        var bytes = Number(value || 0)
        if (bytes <= 0) return "大小未知"
        if (bytes >= 1073741824) return (bytes / 1073741824).toFixed(1) + " GB"
        if (bytes >= 1048576) return (bytes / 1048576).toFixed(1) + " MB"
        if (bytes >= 1024) return (bytes / 1024).toFixed(1) + " KB"
        return bytes + " B"
    }
    function uploadActionText(status) {
        if (status === "PENDING") return "等待上传"
        if (status === "UPLOADING") return "正在上传"
        if (status === "SUCCEEDED") return "已上传"
        if (status === "FAILED_FINAL") return "重新上传"
        if (status === "CREDENTIAL_BLOCKED") return "凭据修复后重试"
        return "上传到平台"
    }
    function uploadIsActive(status) {
        return status === "PENDING" || status === "UPLOADING"
            || status === "FAILED_RETRYABLE" || status === "CREDENTIAL_BLOCKED"
    }

    ColumnLayout {
        width: parent.width
        anchors.margins: Theme.pageMargin
        spacing: 12

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
                statusColor: Theme.stateColor(backend.mapping3dStatus.state || backend.systemStatus["3d_capture"])
            }
            StatusCard {
                Layout.fillWidth: true
                title: "重建状态"
                value: backend.mapping3dReconstructText
                statusColor: Theme.stateColor(backend.mapping3dResult.state || backend.systemStatus["3d_reconstruct"])
            }
            StatusCard {
                Layout.fillWidth: true
                title: "最新 SVO"
                value: root.fileName(backend.latestSvoFile) || "未采集"
                statusColor: backend.latestSvoFile.length > 0 ? Theme.success : Theme.warning
            }
            StatusCard {
                Layout.fillWidth: true
                title: "最新模型"
                value: root.fileName(backend.latestModelFile) || "未生成"
                statusColor: backend.latestModelFile.length > 0 ? Theme.success : Theme.muted
            }
        }

        Label { text: "作业操作"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        GridLayout {
            Layout.fillWidth: true
            columns: root.availableWidth >= 900 ? 2 : 1
            columnSpacing: 12
            rowSpacing: 12

            Rectangle {
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignTop
                implicitHeight: captureActions.implicitHeight + 32
                radius: Theme.cardRadius
                color: Theme.surface
                border.color: Theme.border
                ColumnLayout {
                    id: captureActions
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 10
                    Label { text: "现场采集"; color: Theme.text; font.pixelSize: 16; font.bold: true }
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

            Rectangle {
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignTop
                implicitHeight: reconstructActions.implicitHeight + 32
                radius: Theme.cardRadius
                color: Theme.surface
                border.color: Theme.border
                ColumnLayout {
                    id: reconstructActions
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 10
                    Label { text: "离线重建"; color: Theme.text; font.pixelSize: 16; font.bold: true }
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
                        text: "输出文件: " + (root.fileName(backend.latestModelFile) || "-")
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
        }

        Label { text: "资源管理"; color: Theme.text; font.pixelSize: 20; font.bold: true }
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: assetsColumn.implicitHeight + 32
            radius: 8
            color: Theme.surface
            border.color: Theme.border
            ColumnLayout {
                id: assetsColumn
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 16
                anchors.verticalCenter: parent.verticalCenter
                spacing: 12

                Label { text: "SVO2 采集记录"; color: Theme.text; font.pixelSize: 16; font.bold: true }
                Repeater {
                    model: root.assetList("captures").slice(0, 10)
                    delegate: Frame {
                        required property var modelData
                        Layout.fillWidth: true
                        padding: 14
                        background: Rectangle {
                            radius: Theme.cardRadius
                            color: Theme.surfaceAlt
                            border.color: Theme.border
                        }

                        contentItem: ColumnLayout {
                            id: captureCardColumn
                            spacing: 10

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 10
                                ColumnLayout {
                                    Layout.fillWidth: true
                                    spacing: 3
                                    Label {
                                        Layout.fillWidth: true
                                        text: modelData.display_name || modelData.session_id
                                        color: Theme.text
                                        font.pixelSize: 15
                                        font.bold: true
                                        wrapMode: Text.Wrap
                                    }
                                    Label {
                                        Layout.fillWidth: true
                                        text: modelData.session_id + "  ·  "
                                            + (modelData.svo_frame_count || 0) + " 帧  ·  "
                                            + (modelData.capture_duration_sec || 0) + " s"
                                        color: Theme.muted
                                        font.pixelSize: 12
                                        wrapMode: Text.Wrap
                                    }
                                }
                                Label {
                                    text: modelData.state || "ready"
                                    color: Theme.stateColor(modelData.state || "ready")
                                    font.pixelSize: 12
                                    font.bold: true
                                }
                            }

                            GridLayout {
                                Layout.fillWidth: true
                                columns: root.availableWidth >= 900 ? 5 : 2
                                columnSpacing: 8
                                rowSpacing: 8
                                Button {
                                    Layout.fillWidth: true
                                    text: "设为最新"
                                    onClicked: backend.setLatest3dCapture(modelData.session_id)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "重命名"
                                    onClicked: root.openRenameDialog("capture", modelData.session_id, modelData.display_name)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "删除"
                                    onClicked: backend.delete3dAsset("capture", modelData.session_id)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "快速重建"
                                    onClicked: backend.reconstruct3dCapture(modelData.session_id, "fast_check")
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "高质量重建"
                                    onClicked: backend.reconstruct3dCapture(modelData.session_id, "quality_plus")
                                }
                            }
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: "重建模型记录"
                        color: Theme.text
                        font.pixelSize: 16
                        font.bold: true
                    }
                    Item { Layout.fillWidth: true }
                    Rectangle {
                        implicitWidth: reconstructCountLabel.implicitWidth + 18
                        implicitHeight: 28
                        radius: 14
                        color: Theme.primarySoft
                        Label {
                            id: reconstructCountLabel
                            anchors.centerIn: parent
                            text: root.assetList("reconstructs").length + " 个模型"
                            color: Theme.primary
                            font.pixelSize: 12
                            font.bold: true
                        }
                    }
                }
                Rectangle {
                    visible: root.assetList("reconstructs").length === 0
                    Layout.fillWidth: true
                    implicitHeight: 118
                    radius: Theme.cardRadius
                    color: Theme.surfaceAlt
                    border.color: Theme.border
                    ColumnLayout {
                        anchors.centerIn: parent
                        width: Math.min(parent.width - 32, 520)
                        spacing: 6
                        Label {
                            Layout.alignment: Qt.AlignHCenter
                            text: "暂无可上传的三维模型"
                            color: Theme.text
                            font.pixelSize: 16
                            font.bold: true
                        }
                        Label {
                            Layout.fillWidth: true
                            text: "完成一次离线重建后，模型上传状态、失败原因和平台资产 ID 会显示在这里。"
                            color: Theme.muted
                            font.pixelSize: 13
                            horizontalAlignment: Text.AlignHCenter
                            wrapMode: Text.Wrap
                        }
                    }
                }
                Repeater {
                    model: root.assetList("reconstructs").slice(0, 10)
                    delegate: Frame {
                        required property var modelData
                        property var upload: backend.sceneUploadStatuses[modelData.session_id] || ({})
                        Layout.fillWidth: true
                        padding: 14
                        background: Rectangle {
                            radius: Theme.cardRadius
                            color: Theme.surface
                            border.color: Theme.border
                        }

                        contentItem: ColumnLayout {
                            id: reconstructCardColumn
                            spacing: 10

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 10
                                ColumnLayout {
                                    Layout.fillWidth: true
                                    spacing: 3
                                    Label {
                                        text: modelData.display_name || modelData.session_id
                                        color: Theme.text
                                        font.pixelSize: 16
                                        font.bold: true
                                        wrapMode: Text.Wrap
                                        Layout.fillWidth: true
                                    }
                                    Label {
                                        text: modelData.session_id + "  ·  "
                                            + (modelData.export_point_count || 0) + " 点  ·  "
                                            + root.formatBytes(modelData.file_size_bytes)
                                        color: Theme.muted
                                        font.pixelSize: 12
                                        wrapMode: Text.Wrap
                                        Layout.fillWidth: true
                                    }
                                }
                                Rectangle {
                                    implicitWidth: reconstructStateLabel.implicitWidth + 18
                                    implicitHeight: 28
                                    radius: 14
                                    color: Theme.surfaceAlt
                                    Label {
                                        id: reconstructStateLabel
                                        anchors.centerIn: parent
                                        text: modelData.state === "succeeded" ? "重建成功" : (modelData.state || "未知")
                                        color: Theme.stateColor(modelData.state || "ready")
                                        font.pixelSize: 12
                                        font.bold: true
                                    }
                                }
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                implicitHeight: uploadColumn.implicitHeight + 20
                                radius: Theme.cardRadius
                                color: root.uploadSoftColor(upload.status)
                                border.color: root.uploadColor(upload.status)

                                ColumnLayout {
                                    id: uploadColumn
                                    anchors.left: parent.left
                                    anchors.right: parent.right
                                    anchors.top: parent.top
                                    anchors.margins: 10
                                    spacing: 7
                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: 8
                                        Rectangle {
                                            width: 9
                                            height: 9
                                            radius: 5
                                            color: root.uploadColor(upload.status)
                                        }
                                        ColumnLayout {
                                            Layout.fillWidth: true
                                            spacing: 1
                                            Label {
                                                text: "平台上传"
                                                color: Theme.text
                                                font.pixelSize: 13
                                                font.bold: true
                                            }
                                            Label {
                                                text: root.uploadText(upload.status)
                                                color: root.uploadColor(upload.status)
                                                font.pixelSize: 12
                                            }
                                        }
                                        Label {
                                            visible: upload.retryCount > 0
                                            text: "已重试 " + upload.retryCount + " 次"
                                            color: Theme.muted
                                            font.pixelSize: 11
                                        }
                                    }
                                    Label {
                                        visible: Boolean(upload.lastError) && upload.lastError.length > 0
                                        text: "失败原因：" + upload.lastError
                                        color: Theme.danger
                                        font.pixelSize: 12
                                        wrapMode: Text.Wrap
                                        Layout.fillWidth: true
                                    }
                                    RowLayout {
                                        visible: Boolean(upload.sceneAssetId) && upload.sceneAssetId.length > 0
                                        Layout.fillWidth: true
                                        spacing: 8
                                        Label {
                                            text: "平台资产 ID"
                                            color: Theme.muted
                                            font.pixelSize: 12
                                        }
                                        Label {
                                            Layout.fillWidth: true
                                            text: upload.sceneAssetId || ""
                                            color: Theme.text
                                            font.pixelSize: 12
                                            font.family: "monospace"
                                            elide: Text.ElideMiddle
                                        }
                                        Button {
                                            text: "复制 ID"
                                            onClicked: backend.copySceneAssetId(upload.sceneAssetId)
                                        }
                                    }
                                }
                            }

                            GridLayout {
                                Layout.fillWidth: true
                                columns: root.availableWidth >= 900 ? 4 : 2
                                columnSpacing: 8
                                rowSpacing: 8
                                WarmButton {
                                    visible: upload.status !== "FAILED_RETRYABLE"
                                    Layout.fillWidth: true
                                    text: root.uploadActionText(upload.status)
                                    enabled: modelData.state === "succeeded"
                                        && Boolean(modelData.output_file)
                                        && Number(modelData.file_size_bytes || 0) > 0
                                        && (!upload.status || upload.status === "FAILED_FINAL"
                                            || upload.status === "CREDENTIAL_BLOCKED")
                                        && (!upload.status || (Boolean(upload.taskId) && upload.taskId.length > 0))
                                    onClicked: upload.status
                                        ? backend.retrySceneUpload(upload.taskId)
                                        : backend.enqueueSceneUpload(modelData.session_id)
                                }
                                WarmButton {
                                    visible: upload.status === "FAILED_RETRYABLE"
                                    Layout.fillWidth: true
                                    text: "立即重试"
                                    enabled: Boolean(upload.taskId) && upload.taskId.length > 0
                                    onClicked: backend.retrySceneUpload(upload.taskId)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "设为最新"
                                    onClicked: backend.setLatest3dReconstruct(modelData.session_id)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "重命名"
                                    onClicked: root.openRenameDialog("reconstruct", modelData.session_id, modelData.display_name)
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "删除"
                                    enabled: !root.uploadIsActive(upload.status)
                                    onClicked: backend.delete3dAsset("reconstruct", modelData.session_id)
                                }
                            }
                        }
                    }
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

    Dialog {
        id: renameDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(parent.width - 40, 480)
        modal: true
        focus: true
        title: "重命名三维资源"
        standardButtons: Dialog.Cancel | Dialog.Ok
        property string assetType: ""
        property string sessionId: ""
        property string originalName: ""
        onOpened: {
            renameField.selectAll()
            renameField.forceActiveFocus()
        }
        onAccepted: {
            var name = renameField.text.trim()
            if (name.length > 0 && name !== originalName)
                backend.rename3dAsset(assetType, sessionId, name)
        }
        contentItem: TextField {
            id: renameField
            selectByMouse: true
            placeholderText: "请输入资源名称"
            onAccepted: renameDialog.accept()
        }
    }
}
