import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../components"
import ".."

ColumnLayout {
    anchors.fill: parent
    anchors.margins: 22
    spacing: 18

    Label { text: "建图管理"; color: Theme.text; font.pixelSize: 26; font.bold: true }
    StatusCard {
        Layout.fillWidth: true
        title: "SLAM Toolbox"
        value: backend.systemStatus.mapping || "stopped"
        statusColor: value === "running" ? Theme.success : Theme.warning
    }
    RowLayout {
        Layout.fillWidth: true
        WarmButton { text: "启动建图"; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("start_mapping") }
        WarmButton { text: "停止建图"; buttonColor: Theme.warning; Layout.fillWidth: true; onClicked: backend.sendSystemCommand("stop_mapping") }
    }
    RowLayout {
        Layout.fillWidth: true
        TextField {
            id: mapName
            Layout.fillWidth: true
            placeholderText: "地图名称，例如 inspection_map_01"
            selectByMouse: true
        }
        WarmButton { text: "保存地图"; onClicked: backend.saveMap(mapName.text) }
    }
    Item { Layout.fillHeight: true }
}
