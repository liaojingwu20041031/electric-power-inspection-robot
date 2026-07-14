import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    id: root
    radius: 12
    color: "#F1F5F9"
    border.color: Theme.border
    clip: true

    property alias source: routePreviewImage.source
    property bool previewOk: false
    property bool loading: false
    property string message: ""
    property int imageStatus: routePreviewImage.status
    property real minZoom: 0.05
    property real maxZoom: 6.0
    property real zoom: 1.0
    property real panX: 0
    property real panY: 0
    property real pinchStartZoom: 1.0
    property bool dragging: false
    property bool autoFit: true
    property string imageLoadError: ""
    signal retryRequested()

    function clamp(value, low, high) {
        return Math.max(low, Math.min(high, value))
    }

    function setZoom(value, manual) {
        zoom = clamp(value, minZoom, maxZoom)
        if (manual === true) autoFit = false
    }

    function zoomIn() { setZoom(zoom * 1.2, true) }
    function zoomOut() { setZoom(zoom / 1.2, true) }

    function reset() {
        autoFit = false
        zoom = 1.0
        panX = 0
        panY = 0
    }

    function fit() {
        if (routePreviewImage.status !== Image.Ready) return
        if (routePreviewImage.implicitWidth <= 0 || routePreviewImage.implicitHeight <= 0) return
        autoFit = true
        setZoom(Math.min(
            imageViewport.width / routePreviewImage.implicitWidth,
            imageViewport.height / routePreviewImage.implicitHeight
        ))
        panX = 0
        panY = 0
    }

    function scheduleFit() {
        if (autoFit) fitTimer.restart()
    }

    onWidthChanged: scheduleFit()
    onHeightChanged: scheduleFit()
    onSourceChanged: {
        autoFit = true
        scheduleFit()
    }

    Timer {
        id: fitTimer
        interval: 100
        repeat: false
        onTriggered: root.fit()
    }

    Item {
        id: imageViewport
        anchors.fill: parent
        anchors.margins: 12
        clip: true

        Image {
            id: routePreviewImage
            width: implicitWidth
            height: implicitHeight
            x: (imageViewport.width - width * root.zoom) / 2 + root.panX
            y: (imageViewport.height - height * root.zoom) / 2 + root.panY
            scale: root.zoom
            transformOrigin: Item.TopLeft
            fillMode: Image.PreserveAspectFit
            asynchronous: true
            cache: true
            smooth: !root.dragging
            sourceSize.width: 1600
            visible: root.previewOk && status === Image.Ready
            onStatusChanged: {
                root.imageLoadError = status === Image.Error ? "路线预览图解码失败" : ""
                if (status === Image.Ready) root.scheduleFit()
            }
        }

        PinchArea {
            anchors.fill: parent
            onPinchStarted: root.pinchStartZoom = root.zoom
            onPinchUpdated: root.setZoom(root.pinchStartZoom * pinch.scale, true)

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                property real lastX: 0
                property real lastY: 0
                onPressed: {
                    root.dragging = true
                    root.autoFit = false
                    lastX = mouse.x
                    lastY = mouse.y
                }
                onReleased: root.dragging = false
                onCanceled: root.dragging = false
                onPositionChanged: {
                    if (!pressed) return
                    root.panX += mouse.x - lastX
                    root.panY += mouse.y - lastY
                    lastX = mouse.x
                    lastY = mouse.y
                }
                onWheel: {
                    if (wheel.angleDelta.y > 0) root.zoomIn()
                    else root.zoomOut()
                    wheel.accepted = true
                }
            }
        }
    }

    Column {
        anchors.centerIn: parent
        width: parent.width - 48
        spacing: 10
        visible: root.loading || root.imageLoadError.length > 0 || !root.previewOk || routePreviewImage.status !== Image.Ready

        BusyIndicator {
            anchors.horizontalCenter: parent.horizontalCenter
            running: root.loading
            visible: running
        }
        Label {
            width: parent.width
            horizontalAlignment: Text.AlignHCenter
            text: root.loading
                ? "正在生成路线预览"
                : (root.imageLoadError.length > 0 ? root.imageLoadError : (!root.previewOk ? root.message : "路线预览图未生成"))
            color: root.imageLoadError.length > 0 ? Theme.danger : Theme.muted
            font.pixelSize: 17
            wrapMode: Text.Wrap
        }
        Button {
            anchors.horizontalCenter: parent.horizontalCenter
            visible: !root.loading && (root.imageLoadError.length > 0 || !root.previewOk)
            text: "重新生成"
            implicitHeight: 44
            onClicked: root.retryRequested()
        }
    }

    Rectangle {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 12
        width: toolRow.implicitWidth + 12
        height: 54
        radius: 10
        color: "#EFFFFFFF"
        border.color: Theme.border
        visible: root.previewOk && routePreviewImage.status === Image.Ready

        RowLayout {
            id: toolRow
            anchors.centerIn: parent
            spacing: 4
            Button { text: "−"; implicitWidth: 42; implicitHeight: 42; onClicked: root.zoomOut() }
            Label { text: String(Math.round(root.zoom * 100)) + "%"; color: Theme.text; Layout.preferredWidth: 52; horizontalAlignment: Text.AlignHCenter }
            Button { text: "+"; implicitWidth: 42; implicitHeight: 42; onClicked: root.zoomIn() }
            Button { text: "1:1"; implicitWidth: 48; implicitHeight: 42; onClicked: root.reset() }
            Button { text: "适应"; implicitWidth: 58; implicitHeight: 42; onClicked: root.fit() }
        }
    }

    Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 12
        width: hintLabel.implicitWidth + 24
        height: 34
        radius: 17
        color: "#D91F2937"
        visible: root.previewOk && routePreviewImage.status === Image.Ready
        Label {
            id: hintLabel
            anchors.centerIn: parent
            text: "拖动查看 · 双指缩放 · 点击适应恢复全图"
            color: "white"
            font.pixelSize: 12
        }
    }
}
