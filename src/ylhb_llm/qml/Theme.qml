pragma Singleton
import QtQuick 2.12

QtObject {
    readonly property color background: "#FFF7ED"
    readonly property color surface: "#FFFFFF"
    readonly property color primary: "#F97316"
    readonly property color accent: "#FACC15"
    readonly property color success: "#22C55E"
    readonly property color warning: "#F59E0B"
    readonly property color danger: "#DC2626"
    readonly property color text: "#1F2937"
    readonly property color muted: "#6B7280"
    readonly property color border: "#FED7AA"

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
