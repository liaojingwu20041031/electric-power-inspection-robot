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

    ColumnLayout {
        width: parent.width
        anchors.margins: 22
        spacing: 14
        Label { text: "系统状态"; color: Theme.text; font.pixelSize: 26; font.bold: true }
        Repeater {
            model: statusPage.names.length
            delegate: StatusCard {
                required property int index
                Layout.fillWidth: true
                title: statusPage.labels[index]
                value: backend.localizedStatus(backend.systemStatus[statusPage.names[index]] || "stopped")
                statusColor: backend.systemStatus[statusPage.names[index]] === "running" || backend.systemStatus[statusPage.names[index]] === "embedded" ? Theme.success : Theme.warning
            }
        }
    }
}
