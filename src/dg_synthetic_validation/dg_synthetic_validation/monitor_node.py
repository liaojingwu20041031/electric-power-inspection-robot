"""Read-only live monitor for DG synthetic validation outputs.

The node only ever creates subscriptions.  It has ZERO publishers of its own,
so it cannot influence the navigation graph, the safety arbiter or the
recorded results in any way.  ``create_publisher`` is locked out after
construction to make that guarantee structural rather than a promise.

One caveat, stated because ``ros2 node info`` would otherwise appear to
contradict the line above: rclpy's own ``Node.__init__`` unconditionally creates
a ``/parameter_events`` publisher (``rclpy/node.py:199``), and the lock is armed
after ``super().__init__`` precisely so that construction still works.  So the
node does show exactly one publisher on the graph, ``/parameter_events``, which
rclpy created and which carries no navigation, command or diagnostic data.
``enable_rosout=False`` does suppress the ``/rosout`` publisher.  No publisher
originating from this module exists, and none can be added at runtime.

Everything rendered here comes from diagnostics that the real nodes already
publish.  Nothing is synthesised: a field that the upstream message does not
carry is printed as ``N/A``, and a topic that has not produced a message yet
is printed as ``NO_DATA``.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Any

# Exactly the states defined in ylhb_base/scripts/active_relocalization_core.py.
# "LOST" and "SEARCHING" are NOT part of this state machine and must never be
# rendered by this monitor.
RELOCALIZATION_STATES = (
    "NORMAL",
    "SUSPECTED",
    "TRIGGERED",
    "STOPPING",
    "ACTIVE_SCAN",
    "WAITING_CANDIDATE",
    "VERIFYING",
    "RECOVERED",
    "FAILED",
    "MANUAL_REQUIRED",
)

# active_relocalization_node.py declares verification_samples=3 as its default.
# The supervisor does not publish this parameter on any topic, so the monitor
# uses the documented default and labels it as such.
DEFAULT_VERIFICATION_SAMPLES = 3

NO_DATA = "NO_DATA"
NA = "N/A"
LABEL_WIDTH = 14
BANNER = (
    "SYNTHETIC SOFTWARE VALIDATION | NOT_REAL_ROBOT_DATA "
    "| NOT_COMPETITION_PERFORMANCE_EVIDENCE"
)
ZERO_EPSILON = 1e-9
EQUAL_EPSILON = 1e-6


def _values(status: Any) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in status.values}


def _level(raw: Any) -> int:
    if isinstance(raw, (bytes, bytearray, memoryview)):
        return int(raw[0]) if raw else 0
    try:
        return int(raw)
    except (TypeError, ValueError):
        return -1


_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def _level_text(level: int) -> str:
    return f"level={level}({_LEVEL_NAMES.get(level, 'UNKNOWN')})"


def _pick(message: Any, *hints: str) -> Any | None:
    """Return the DiagnosticStatus whose name matches one of ``hints``.

    Matching is done on status.name instead of taking status[0] blindly, so a
    multi-status array stays unambiguous.
    """
    statuses = list(message.status)
    if not statuses:
        return None
    for hint in hints:
        needle = hint.lower()
        for status in statuses:
            if needle in str(status.name).lower():
                return status
    return statuses[0]


def _text(values: dict[str, str], key: str) -> str:
    value = values.get(key)
    if value is None or not value.strip() or value.strip().lower() == "none":
        return NA
    return value.strip()


def _number(values: dict[str, str], key: str) -> float | None:
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return None


def _integer(values: dict[str, str], key: str) -> int | None:
    try:
        return int(float(values[key]))
    except (KeyError, TypeError, ValueError):
        return None


def _fmt(value: float | None, digits: int = 3) -> str:
    if value is None:
        return NA
    if not math.isfinite(value):
        return NA
    if abs(value) >= 1e4:
        return f"{value:.3g}"
    return f"{value:.{digits}f}"


def _relocalization_state(raw: str) -> str:
    """Never render a state name that the supervisor cannot produce."""
    candidate = raw.strip().upper()
    if candidate in RELOCALIZATION_STATES:
        return candidate
    return NA if candidate in ("", NA) else "UNRECOGNIZED"


def _yaw_degrees(orientation: Any) -> float:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _progress_bar(done: int, total: int) -> str:
    total = max(1, total)
    filled = max(0, min(total, done))
    return "[" + "#" * filled + "." * (total - filled) + "]"


def _twist_text(cmd: tuple[float, float] | None) -> str:
    if cmd is None:
        return NO_DATA
    return f"linear.x={cmd[0]:+.3f}  angular.z={cmd[1]:+.3f}"


def _is_zero(cmd: tuple[float, float]) -> bool:
    return abs(cmd[0]) <= ZERO_EPSILON and abs(cmd[1]) <= ZERO_EPSILON


def _same(left: tuple[float, float], right: tuple[float, float]) -> bool:
    return (
        abs(left[0] - right[0]) <= EQUAL_EPSILON
        and abs(left[1] - right[1]) <= EQUAL_EPSILON
    )


def _cmd_source(
    test: tuple[float, float] | None,
    recovery: tuple[float, float] | None,
) -> str:
    """Infer the command source numerically.

    geometry_msgs/Twist carries no source metadata whatsoever, so this can only
    ever be an inference from the numbers and is always labelled as such.
    """
    if test is None and recovery is None:
        return NO_DATA
    if test is None or recovery is None:
        return "UNKNOWN (inferred, only one stream seen)"
    if _is_zero(test) and _is_zero(recovery):
        return "ZERO"
    if _same(test, recovery):
        return "RECOVERY (inferred)"
    return "UNKNOWN (inferred)"


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read-only DG ROS2 runtime monitor")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl-C")
    parser.add_argument("--refresh", type=float, default=1.0)
    parsed = parser.parse_args(args)

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    class Monitor(Node):
        """Subscribe-only dashboard.  Creating a publisher here is a bug."""

        def __init__(self) -> None:
            try:
                super().__init__(
                    "dg_synthetic_monitor",
                    enable_rosout=False,
                    start_parameter_services=False,
                )
            except TypeError:  # pragma: no cover - older rclpy signatures
                super().__init__("dg_synthetic_monitor")

            self.started = time.monotonic()
            self.gnss: dict[str, str] | None = None
            self.gnss_level = -1
            self.lidar: dict[str, str] | None = None
            self.lidar_level = -1
            self.fusion: dict[str, str] | None = None
            self.nav: dict[str, str] | None = None
            self.reloc: dict[str, str] | None = None
            self.match: dict[str, str] | None = None
            self.match_level = -1
            self.test_cmd: tuple[float, float] | None = None
            self.recovery_cmd: tuple[float, float] | None = None
            self.odom_yaw: float | None = None

            self.create_subscription(DiagnosticArray, "/dg/gnss/quality", self.on_gnss, 10)
            self.create_subscription(DiagnosticArray, "/dg/lidar/quality", self.on_lidar, 10)
            self.create_subscription(DiagnosticArray, "/dg/fusion/status", self.on_fusion, 10)
            self.create_subscription(DiagnosticArray, "/dg/navigation/status", self.on_nav, 10)
            self.create_subscription(DiagnosticArray, "/dg/relocalization/status", self.on_reloc, 10)
            self.create_subscription(
                DiagnosticArray, "/dg/relocalization/match_quality", self.on_match, 10
            )
            self.create_subscription(Twist, "/dg/test_cmd_vel", self.on_test_cmd, 10)
            self.create_subscription(Twist, "/cmd_vel_recovery", self.on_recovery_cmd, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_timer(max(0.2, parsed.refresh), self.render)

            # From here on the node is sealed: no publisher may ever be made.
            self._publishers_locked = True

        def create_publisher(self, *args: Any, **kwargs: Any) -> Any:
            if getattr(self, "_publishers_locked", False):
                raise RuntimeError(
                    "dg_synthetic_monitor is strictly read-only and must not publish"
                )
            return super().create_publisher(*args, **kwargs)

        # -- subscription callbacks ---------------------------------------
        def on_gnss(self, message: Any) -> None:
            status = _pick(message, "gnss quality", "gnss")
            if status is None:
                return
            self.gnss = _values(status)
            self.gnss_level = _level(status.level)

        def on_lidar(self, message: Any) -> None:
            status = _pick(message, "dg/lidar/quality", "lidar")
            if status is None:
                return
            self.lidar = _values(status)
            self.lidar_level = _level(status.level)

        def on_fusion(self, message: Any) -> None:
            status = _pick(message, "multi-source fusion", "fusion")
            if status is None:
                return
            self.fusion = _values(status)

        def on_nav(self, message: Any) -> None:
            status = _pick(message, "navigation health", "navigation")
            if status is None:
                return
            self.nav = _values(status)

        def on_reloc(self, message: Any) -> None:
            status = _pick(message, "active relocalization", "relocalization supervisor")
            if status is None:
                return
            self.reloc = _values(status)

        def on_match(self, message: Any) -> None:
            status = _pick(message, "scan-to-map match", "match", "scan-to-map")
            if status is None:
                return
            self.match = _values(status)
            self.match_level = _level(status.level)

        def on_test_cmd(self, message: Any) -> None:
            self.test_cmd = (float(message.linear.x), float(message.angular.z))

        def on_recovery_cmd(self, message: Any) -> None:
            self.recovery_cmd = (float(message.linear.x), float(message.angular.z))

        def on_odom(self, message: Any) -> None:
            self.odom_yaw = _yaw_degrees(message.pose.pose.orientation)

        # -- row builders --------------------------------------------------
        def _gnss_row(self) -> str:
            if self.gnss is None:
                return NO_DATA
            return f"{_text(self.gnss, 'current_state')}  {_level_text(self.gnss_level)}"

        def _lidar_row(self) -> str:
            if self.lidar is None:
                return NO_DATA
            # /dg/lidar/quality carries no explicit state key; the state is
            # derived from level+geometry_score the same way evaluator_node does.
            geometry = _number(self.lidar, "geometry_score")
            if self.lidar_level >= 2 or (geometry is not None and geometry <= 0.0):
                state = "REJECTED"
            elif geometry is not None and geometry < 0.2:
                state = "DEGRADED"
            elif geometry is None:
                state = NA
            else:
                state = "GOOD"
            return (
                f"{state} (derived)  geometry_score={_fmt(geometry)}  "
                f"{_level_text(self.lidar_level)}"
            )

        def _fusion_row(self) -> str:
            if self.fusion is None:
                return NO_DATA
            return (
                f"{_text(self.fusion, 'fusion_mode')}  measurement_confidence="
                f"{_fmt(_number(self.fusion, 'measurement_confidence'))}"
            )

        def _nav_row(self) -> str:
            if self.nav is None:
                return NO_DATA
            return _text(self.nav, "overall_state")

        def _reloc_row(self) -> str:
            if self.reloc is None:
                return NO_DATA
            state = _relocalization_state(_text(self.reloc, "state"))
            return (
                f"{state}  attempt_id={_text(self.reloc, 'attempt_id')}  "
                f"active_segment={_text(self.reloc, 'active_segment')}"
            )

        def _reloc_field(self, key: str) -> str:
            if self.reloc is None:
                return NO_DATA
            return _text(self.reloc, key)

        def _candidate_row(self) -> str:
            if self.match is None:
                return NO_DATA
            return (
                f"accepted={_text(self.match, 'accepted')}  "
                f"reason={_text(self.match, 'reason')}  {_level_text(self.match_level)}"
            )

        def _candidate_quality_row(self) -> str:
            if self.match is None:
                return NO_DATA
            return (
                f"score={_fmt(_number(self.match, 'score'))}  "
                f"inlier_ratio={_fmt(_number(self.match, 'inlier_ratio'))}  "
                f"mean_distance={_fmt(_number(self.match, 'mean_distance'))}"
            )

        def _verification_row(self) -> str:
            if self.reloc is None:
                return NO_DATA
            count = _integer(self.reloc, "verification_count")
            if count is None:
                return NA
            total = DEFAULT_VERIFICATION_SAMPLES
            return (
                f"{count}/{total} {_progress_bar(count, total)}  "
                f"(verification_samples={total}, node default; not published)"
            )

        def _odom_row(self) -> str:
            if self.odom_yaw is None:
                return NO_DATA
            return f"yaw={self.odom_yaw:+8.2f} deg  (from /odom quaternion)"

        def _safety_row(self) -> str:
            try:
                count = int(self.count_publishers("/cmd_vel"))
            except Exception:  # pragma: no cover - API unavailable
                return f"/cmd_vel publishers: {NA} (count_publishers unavailable)"
            if count:
                return (
                    f"!! SAFETY ALERT !! /cmd_vel publishers: {count} "
                    "!! REAL CHASSIS COMMAND PATH IS LIVE !!"
                )
            return f"/cmd_vel publishers: {count}  (no chassis command path active)"

        # -- rendering ------------------------------------------------------
        def render(self) -> None:
            elapsed = time.monotonic() - self.started
            if parsed.duration > 0 and elapsed >= parsed.duration:
                rclpy.shutdown()
                return

            def row(label: str, value: str) -> str:
                return f"{label:<{LABEL_WIDTH}}: {value}"

            rule = "-" * 78
            rows = [
                f"DG-202611 READ-ONLY MONITOR   elapsed={elapsed:7.1f}s",
                BANNER,
                rule,
                # The scenario runner does not tell the monitor which scenario or
                # phase is active, and no topic carries it.  Reporting N/A here
                # instead of guessing is deliberate.
                row("SCENARIO", f"{NA}  (not published to the monitor)"),
                row("PHASE", f"{NA}  (not published to the monitor)"),
                row("ELAPSED", f"{elapsed:.1f}s"),
                rule,
                row("GNSS", self._gnss_row()),
                row("LIDAR", self._lidar_row()),
                row("FUSION", self._fusion_row()),
                row("NAV HEALTH", self._nav_row()),
                rule,
                row("RELOCALIZATION", self._reloc_row()),
                row("TRIGGER REASON", self._reloc_field("trigger_reason")),
                row("FAILURE REASON", self._reloc_field("failure_reason")),
                row("CANDIDATE", self._candidate_row()),
                row("CANDIDATE Q", self._candidate_quality_row()),
                row("VERIFICATION", self._verification_row()),
                row("SEED SOURCE", self._reloc_field("seed_source")),
                row("AMCL HEALTH", self._reloc_field("amcl_health")),
                rule,
                row("ODOM YAW", self._odom_row()),
                row("TEST CMD_VEL", _twist_text(self.test_cmd)),
                row("RECOVERY CMD", _twist_text(self.recovery_cmd)),
                row("CMD SOURCE", _cmd_source(self.test_cmd, self.recovery_cmd)),
                row("SAFETY", self._safety_row()),
                rule,
                "monitor: subscriptions only; this node creates no publisher of its own",
                "(rclpy itself adds /parameter_events to every node; /rosout is disabled)",
                "Twist carries no source field, so CMD SOURCE is an inference",
            ]
            print("\033[2J\033[H" + "\n".join(rows), flush=True)

    rclpy.init(args=None)
    node = Monitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
