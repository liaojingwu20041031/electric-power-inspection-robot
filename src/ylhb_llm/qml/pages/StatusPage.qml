import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ScrollView {
    id: statusPage
    clip: true
    contentWidth: availableWidth
    property var names: ["bringup", "navigation", "zed", "3d_mapping", "perception", "patrol_executor", "llm", "mobile_bridge"]
    property var labels: ["底盘与传感器", "导航", "ZED", "三维建模", "视觉感知", "巡逻执行器", "AI 任务层", "APP 网桥"]
    function cardValue(name) {
        if (name === "3d_mapping" && backend.mapping3dStatus.state) {
            return backend.mapping3dStateText
        }
        return backend.localizedStatus(backend.systemStatus[name] || "stopped")
    }
    function cardRunning(name) {
        if (name === "3d_mapping" && backend.mapping3dStatus.state) {
            return ["running", "recording", "reconstructing", "opening_camera", "tracking_enabled", "mapping_enabled", "extracting", "saving"].indexOf(backend.mapping3dStatus.state) >= 0
        }
        return backend.systemStatus[name] === "running" || backend.systemStatus[name] === "embedded"
    }
    function cardState(name) {
        if (name === "3d_mapping" && backend.mapping3dStatus.state) {
            return backend.mapping3dStatus.state
        }
        return backend.systemStatus[name] || "stopped"
    }

    ColumnLayout {
        width: parent.width
        anchors.margins: Theme.pageMargin
        spacing: 12
        Label { text: "系统状态"; color: Theme.text; font.pixelSize: 26; font.bold: true }
        Repeater {
            model: statusPage.names.length
            delegate: StatusCard {
                required property int index
                Layout.fillWidth: true
                title: statusPage.labels[index]
                value: statusPage.cardValue(statusPage.names[index])
                statusColor: Theme.stateColor(statusPage.cardState(statusPage.names[index]))
            }
        }
    }
}
