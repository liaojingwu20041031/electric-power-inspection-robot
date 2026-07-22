pragma Singleton
import QtQuick 2.12

QtObject {
    readonly property color background: "#F5FAFF"
    readonly property color surface: "#FFFFFF"
    readonly property color surfaceAlt: "#EAF5FF"
    readonly property color primary: "#3B8EF3"
    readonly property color primarySoft: "#EAF5FF"
    readonly property color accent: "#67C5F8"
    readonly property color success: "#22C55E"
    readonly property color successSoft: "#DCFCE7"
    readonly property color warning: "#F59E0B"
    readonly property color warningSoft: "#FEF3C7"
    readonly property color danger: "#DC2626"
    readonly property color dangerSoft: "#FEE2E2"
    readonly property color info: "#3B8EF3"
    readonly property color infoSoft: "#EAF5FF"
    readonly property color text: "#243447"
    readonly property color muted: "#7B8A9A"
    readonly property color border: "#DDEAF4"
    readonly property int cardRadius: 8
    readonly property int pageMargin: 24
    readonly property int controlSpacing: 10
    readonly property int minimumTouchHeight: 48

    function stateColor(state) {
        var value = String(state || "")
        if (["recording", "running", "reconstructing", "extracting", "saving", "opening_camera", "tracking_enabled", "mapping_enabled"].indexOf(value) >= 0) {
            return primary
        }
        if (["succeeded", "ready", "embedded"].indexOf(value) >= 0) {
            return success
        }
        if (["failed", "error", "fault"].indexOf(value) >= 0) {
            return danger
        }
        if (["stopped", "idle", "unavailable"].indexOf(value) >= 0) {
            return muted
        }
        if (value === "waiting" || value === "exporting" || value.indexOf("waiting_") === 0) {
            return warning
        }
        return warning
    }
}
