import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import ".."

Rectangle {
    id: root
    radius: 8
    color: Theme.surface
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

    function clamp(value, low, high) {
        return Math.max(low, Math.min(high, value))
    }

    function setZoom(value, manual) {
        zoom = clamp(value, minZoom, maxZoom)
        if (manual === true) autoFit = false
    }

    function zoomIn() {
        setZoom(zoom * 1.2, true)
    }

    function zoomOut() {
        setZoom(zoom / 1.2, true)
    }

    function reset() {
        autoFit = false
        zoom = 1.0
        panX = 0
        panY = 0
    }

    function fit() {
        autoFit = true
        if (routePreviewImage.implicitWidth <= 0 || routePreviewImage.implicitHeight <= 0) {
            reset()
            return
        }
        var fitted = Math.min(
            imageViewport.width / routePreviewImage.implicitWidth,
            imageViewport.height / routePreviewImage.implicitHeight
        )
        setZoom(fitted)
        panX = 0
        panY = 0
    }

    onWidthChanged: { if (autoFit) fit() }
    onHeightChanged: { if (autoFit) fit() }

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
                root.imageLoadError = status === Image.Error
                    ? "路线预览图解码失败，请点击重绘预览"
                    : ""
                if (status === Image.Ready && root.autoFit) {
                    root.fit()
                }
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
                    if (!pressed) {
                        return
                    }
                    root.panX += mouse.x - lastX
                    root.panY += mouse.y - lastY
                    lastX = mouse.x
                    lastY = mouse.y
                }
                onWheel: {
                    if (wheel.angleDelta.y > 0) {
                        root.zoomIn()
                    } else {
                        root.zoomOut()
                    }
                    wheel.accepted = true
                }
            }
        }
    }

    Label {
        anchors.centerIn: parent
        width: parent.width - 32
        horizontalAlignment: Text.AlignHCenter
        text: root.loading
            ? "路线预览加载中"
            : (root.imageLoadError.length > 0
                ? root.imageLoadError
                : (!root.previewOk ? root.message : "路线预览图未生成"))
        color: Theme.muted
        font.pixelSize: 18
        wrapMode: Text.Wrap
        visible: root.loading
            || root.imageLoadError.length > 0
            || !root.previewOk
            || routePreviewImage.status !== Image.Ready
    }

    RowLayout {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 10
        spacing: 6
        visible: root.previewOk && routePreviewImage.status === Image.Ready

        Button { text: "-"; onClicked: root.zoomOut() }
        Label {
            text: String(Math.round(root.zoom * 100)) + "%"
            color: Theme.text
            Layout.preferredWidth: 48
            horizontalAlignment: Text.AlignHCenter
        }
        Button { text: "+"; onClicked: root.zoomIn() }
        Button { text: "1:1"; onClicked: root.reset() }
        Button { text: "适应"; onClicked: root.fit() }
    }
}
