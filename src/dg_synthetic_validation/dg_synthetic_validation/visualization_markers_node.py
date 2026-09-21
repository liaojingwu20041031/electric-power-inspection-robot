"""Visualization-only marker node for the DG-202611 synthetic validation runs.

This node exists so that a single RViz2 window can document what the algorithms
under test actually published.  It is strictly read-only with respect to the
system under test:

* it creates exactly ONE publisher, ``/dg_validation_viz/markers``
  (``visualization_msgs/MarkerArray``), which lives outside the ``/dg/*``
  namespace owned by the algorithms under test;
* it publishes no TF, no pose, no command, no diagnostics and no
  ``map -> odom`` transform;
* ``/cmd_vel`` and ``/cmd_vel_nav`` are never published.  ``/cmd_vel`` appears in
  this file only as the argument of ``count_publishers``, a graph query used to
  report whether a real chassis command path is live;
* it renders only values that a real topic actually delivered, and only while
  those values are still live.  Anything that was never observed renders as
  ``NO_DATA``; anything that arrived once and then stopped arriving renders as
  ``STALE <age>``; no state is ever synthesised and no slot is ever filled with
  a default that reads like success.

A frozen value is not a live observation
----------------------------------------
The first version of this node latched the last message seen on each topic and
held it on screen forever.  A screenshot taken after a scenario had finished
therefore showed the teardown state while still looking live: after the S05
final run the view read ``RELOCALIZATION: FAILED`` and ``NAV HEALTH: FAILED``,
because once the injector stopped publishing, ``/odom`` went stale, the
supervisor hit ``_fail("ODOM_STALE_OR_MISSING")`` and retried.  All of that was
real, and all of it happened outside the measured window that the recorded
evidence covers (which ends at ``STOPPING`` with result PASS).  A screenshot
that contradicts the recorded result is unusable as evidence, so it must not be
possible to take one by accident.

Every observed source therefore carries the monotonic time of its last message
(``SOURCE_TOPICS`` names them; ``source_ages`` turns those stamps into ages),
and every rendered field distinguishes three states that are never collapsed
into one another:

    NO_DATA        nothing was ever received on that topic
    STALE 12.4s    something was received, 12.4 s ago, which is older than
                   ``STALE_AFTER_S`` and therefore not a live observation
    <the value>    received within ``STALE_AFTER_S``: live right now

The notes region carries a one-line run-state summary built from the same ages
(``DATA: LIVE`` versus ``DATA: STALE - NO SCENARIO RUNNING``), so an
after-the-fact screenshot is self-evidently not live at a glance instead of
requiring the reader to check every field.

This is the same family of trap as the black screenshot and the empty plot: the
image looks like evidence, so it has to say what it really is.

One marker per line
-------------------
Every text line is its own ``TEXT_VIEW_FACING`` marker with its own id and its
own position.  Multi-line marker text was the previous cause of severe visual
overlap: RViz centres a text marker on its position (``MovableText`` is created
with ``H_CENTER``/``V_CENTER``), so a four-line block grows symmetrically in
both directions and collides with whatever sits above and below it.

The layout is therefore a fixed grid of single-line slots, defined by the
module-level constants below and audited statically:

    TOP BANNER      x  0.0   y  6.90 / 6.15 / 5.40   (mandatory self-caption)
    LEFT COLUMN     x -9.3   y  4.00 header, rows 3.30 .. -2.30
    RIGHT COLUMN    x +9.3   y  4.00 header, rows 3.30 .. -1.60
    BOTTOM BLOCK A  x -5.8   y -5.20 header, rows -5.90 .. -9.40
    BOTTOM BLOCK B  x +5.8   y -5.20 header, rows -5.90 .. -9.40
    FOOTER NOTES    x  0.0   y -10.10 / -10.70 / -11.30

The synthetic map is 80x80 cells at 0.1 m with origin (-4.0, -4.0), so the walls
and the scan live inside x,y in [-4, +4].  Every text region is kept clear of
that box, allowing for the fact that a text marker is centred on its position:
a body row is capped at ``MAX_LINE_CHARS`` characters, which at
``BODY_HEIGHT`` is at most ``MAX_LINE_CHARS * BODY_HEIGHT * CHAR_ASPECT`` metres
wide, so the columns sit far enough out that even a full-width row stops short
of the walls.  Values longer than one row are wrapped onto continuation lines
inside the same region instead of running across another region.

A slot that has no line to show this cycle is published as a DELETE marker, so a
stale value can never survive on screen as if it were still being observed.

Frame handling and the deliberately missing ``map -> odom`` transform
--------------------------------------------------------------------
There is no ``map -> odom`` transform in this system.  It was formally denied so
that no ground-truth pose can leak into the algorithms under test.  The only
transforms that exist are ``odom -> base_footprint`` (dynamic, published by the
synthetic kinematic plant) and ``base_footprint -> laser`` (static).  The RViz
fixed frame is therefore ``odom``, and an ``OccupancyGrid`` whose header frame is
``map`` cannot be rendered by the ordinary Map display.

To keep the occupancy walls visible anyway, this node re-emits the occupied
cells of ``/map`` as a ``CUBE_LIST`` marker expressed in ``odom``.

VISUALIZATION ONLY: the synthetic scenario places the odom origin at the map
origin by construction, so for display purposes the two frames coincide.  That
assumption is used here for display and nowhere else.  It is never published as
TF, never written back onto any topic, and therefore never reaches any algorithm
under test.  The footer notes state it on screen so a screenshot carries the
caveat itself.

Division of labour with the monitor
-----------------------------------
``monitor_node`` stays the detailed text source.  This view shows the spatial
evidence plus a key-state summary: the long low-value strings (failure_reason,
health_reasons, covariances, the full per-topic age table) are left to the
monitor.  This view reports an age only where it changes what the reader should
believe -- when a value has gone stale, and for the freshness of a scan-to-map
candidate.
"""

from __future__ import annotations

from typing import Any, NamedTuple


# The single topic this node is allowed to publish.  Deliberately outside /dg/*.
MARKER_TOPIC = "/dg_validation_viz/markers"

# Every marker is emitted in this frame; see the module docstring.
MARKER_FRAME = "odom"

# ---------------------------------------------------------------------------
# Read-only inputs: real algorithm outputs plus synthetic plant inputs.  The
# topic names and the DiagnosticStatus key names below were read from the real
# publishers, not guessed:
#   /dg/gnss/quality                  gnss_quality_node.py:177
#   /dg/lidar/quality                 lidar_robust_node.py:162
#   /dg/fusion/status                 multisource_fusion_node.py:394
#   /dg/navigation/status             navigation_health_node.py:230
#   /dg/relocalization/status         active_relocalization_node.py:465
#   /dg/relocalization/match_quality  scan_map_relocalization_node.py:553
#   /dg/test_cmd_vel                  cmd_vel_arbiter_node output_topic in
#                                     offline test mode
#   /cmd_vel_recovery                 active_relocalization_node cmd_vel_topic
# ---------------------------------------------------------------------------
MAP_TOPIC = "/map"
ODOM_TOPIC = "/odom"
AMCL_POSE_TOPIC = "/amcl_pose"
SCAN_MATCH_POSE_TOPIC = "/scan_match_pose"
RELOCALIZATION_STATUS_TOPIC = "/dg/relocalization/status"
MATCH_QUALITY_TOPIC = "/dg/relocalization/match_quality"
GNSS_QUALITY_TOPIC = "/dg/gnss/quality"
LIDAR_QUALITY_TOPIC = "/dg/lidar/quality"
FUSION_STATUS_TOPIC = "/dg/fusion/status"
NAVIGATION_STATUS_TOPIC = "/dg/navigation/status"
TEST_CMD_VEL_TOPIC = "/dg/test_cmd_vel"
RECOVERY_CMD_VEL_TOPIC = "/cmd_vel_recovery"

# Counted with count_publishers() only.  This node NEVER publishes it, and it
# never subscribes to it either; a non-zero count means a real chassis command
# path is live, which is worth shouting about inside an evidence screenshot.
CHASSIS_CMD_VEL_TOPIC = "/cmd_vel"

# OccupancyGrid cells at or above this value are drawn as walls.
OCCUPIED_THRESHOLD = 50

NO_DATA = "NO_DATA"
# Distinguishes "no publisher for this exists anywhere" from "nothing received
# yet".  Neither is ever hidden.
NO_TOPIC = "NO_DATA (no topic)"

# ---------------------------------------------------------------------------
# Staleness.  A FROZEN VALUE IS NOT A LIVE OBSERVATION.
#
# The scenario publishes every observed source at 10 Hz, and the core's own
# signal_timeout is 1.0 s, so 2.0 s sits comfortably above normal jitter while
# still making a finished run obvious on screen within about two seconds.  It is
# a module constant, not a ROS parameter, so that what a screenshot means cannot
# be changed from a launch file after the fact.
# ---------------------------------------------------------------------------
STALE_AFTER_S = 2.0

# Rendered as e.g. "STALE 12.4s": the age is part of the text so a reader can
# see how long ago the value was last live, not merely that it is not live.
STALE_PREFIX = "STALE"

# Logical source name -> the topic whose arrival time it ages.  Every entry is a
# topic this node really subscribes to; nothing here is inferred.
SOURCE_TOPICS = {
    "gnss": GNSS_QUALITY_TOPIC,
    "lidar": LIDAR_QUALITY_TOPIC,
    "fusion": FUSION_STATUS_TOPIC,
    "navigation": NAVIGATION_STATUS_TOPIC,
    "relocalization": RELOCALIZATION_STATUS_TOPIC,
    "match_quality": MATCH_QUALITY_TOPIC,
    "odom": ODOM_TOPIC,
    "amcl_pose": AMCL_POSE_TOPIC,
    "scan_match_pose": SCAN_MATCH_POSE_TOPIC,
    "test_cmd_vel": TEST_CMD_VEL_TOPIC,
    "recovery_cmd": RECOVERY_CMD_VEL_TOPIC,
}

# The sources a running scenario publishes continuously.  The run-state summary
# is built from these only: /dg/relocalization/match_quality, both cmd_vel
# streams and the two pose topics are intermittent by design, so treating their
# silence as "no run" would cry wolf during a perfectly live scenario.
LIVENESS_SOURCES = (
    "gnss",
    "lidar",
    "fusion",
    "navigation",
    "relocalization",
    "odom",
)

# Run-state summary line, in the notes region.  Four wordings for four honestly
# different situations; "LIVE" is never printed for anything but all-fresh.
RUN_STATE_LIVE = "DATA: LIVE"
RUN_STATE_STALE = "DATA: STALE - NO SCENARIO RUNNING"
RUN_STATE_NEVER = "DATA: NO_DATA - NOTHING RECEIVED YET"
RUN_STATE_PARTIAL = "DATA: PARTIAL - %d/%d SOURCES LIVE"

# A stale pose arrow is a frozen value too, so it is drawn faded instead of at
# full strength.  The arrow is not deleted: /amcl_pose and /scan_match_pose are
# legitimately intermittent, so its last known position stays visible, just
# visibly not current.
STALE_ARROW_ALPHA = 0.35

# active_relocalization_node.py declares verification_samples=3 as its default
# and does not publish it, exactly as monitor_node documents.
VERIFICATION_SAMPLES = 3

# The ten and only Active Relocalization state names, taken from
# ylhb_base/scripts/active_relocalization_core.py.  "LOST" and "SEARCHING" are
# not part of this state machine and must never be rendered.
VALID_STATES = (
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

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

# Exact DiagnosticStatus KeyValue keys, read from the nodes under test.
STATUS_KEY_STATE = "state"
STATUS_KEY_TRIGGER_REASON = "trigger_reason"
STATUS_KEY_VERIFICATION_COUNT = "verification_count"
STATUS_KEY_SEED_SOURCE = "seed_source"
STATUS_KEY_AMCL_HEALTH = "amcl_health"

QUALITY_KEY_SCORE = "score"
QUALITY_KEY_INLIER_RATIO = "inlier_ratio"
QUALITY_KEY_MEAN_DISTANCE = "mean_distance"
QUALITY_KEY_ACCEPTED = "accepted"
QUALITY_KEY_REASON = "reason"

GNSS_KEY_STATE = "current_state"
LIDAR_KEY_GEOMETRY_SCORE = "geometry_score"
FUSION_KEY_MODE = "fusion_mode"
FUSION_KEY_CONFIDENCE = "measurement_confidence"
NAV_KEY_OVERALL_STATE = "overall_state"


# ===========================================================================
# Layout.  These are module constants rather than ROS parameters because the
# clearances between the regions, and between the text and the map walls, are
# verified statically against exactly these numbers.  A runtime knob would
# silently invalidate that audit.
# ===========================================================================

# Mandatory self-labelling banner.  Three separate lines, three separate
# markers.  Never remove these for aesthetics: they are what stops a screenshot
# from being mistaken for real robot or competition data.
BANNER_LINES = (
    "SYNTHETIC SOFTWARE VALIDATION",
    "NOT REAL ROBOT DATA",
    "NOT COMPETITION PERFORMANCE EVIDENCE",
)

BODY_HEIGHT = 0.40          # one uniform height for every state line
BANNER_HEIGHT = 0.60        # banner is deliberately larger
NOTE_HEIGHT = 0.34          # footer caveats, below every state region
ROW_STEP = 0.70             # vertical pitch inside a region
BANNER_STEP = 0.75
NOTE_STEP = 0.60
TEXT_Z = 0.60               # above the 0.2 m wall cubes, so text stays on top

# A text marker is centred on its position, so a row's half-width has to be
# accounted for when keeping text off the map.  CHAR_ASPECT is a deliberately
# generous advance-to-height ratio for RViz's Liberation Sans (uppercase is
# nearer 0.53), so the audited boxes are wider than the real glyphs.
CHAR_ASPECT = 0.62

LABEL_WIDTH = 14
LABEL_GAP = "  "
MAX_LINE_CHARS = 35
VALUE_BUDGET = MAX_LINE_CHARS - LABEL_WIDTH - len(LABEL_GAP)
CONTINUATION_INDENT = " " * (LABEL_WIDTH + len(LABEL_GAP))
# Upper bound on the lines one field may take, so one very long reason string
# can never crowd the rest of its region out of the view.
ROW_MAX_LINES = 3

# A value never just runs off the end of a row.  A token too long for one row is
# broken with a trailing HARD_SPLIT_MARKER and continued on the next row, and a
# value with more rows than its field is allowed ends in ELISION_MARKER.  Either
# way the reader can see that what is on screen is not the whole string; silent
# mid-token clipping (the old "AMCL_COVARIANCE_HIG") is what both markers exist
# to prevent.
HARD_SPLIT_MARKER = "-"
ELISION_MARKER = "..."

# The synthetic map spans x,y in [-4, +4]; keep every text box out of this,
# margin included, so no line ever sits on the walls, the scan or the robot.
MAP_KEEPOUT = 4.45

COLUMN_X = 9.30             # left column at -COLUMN_X, right column at +COLUMN_X
BLOCK_X = 5.80              # bottom block A at -BLOCK_X, block B at +BLOCK_X
BANNER_Y = 6.90
COLUMN_Y_TOP = 4.00
BLOCK_Y_TOP = -5.20
NOTE_Y = -10.10

COLOR_BANNER = (1.0, 0.88, 0.12)
COLOR_BANNER_WARN = (1.0, 0.42, 0.18)
COLOR_HEADER = (0.95, 0.95, 0.95)
COLOR_LEFT = (0.55, 0.92, 0.85)
COLOR_RIGHT = (0.45, 0.80, 1.00)
COLOR_BLOCK_A = (1.00, 0.75, 0.25)
COLOR_BLOCK_B = (0.62, 0.95, 0.55)
COLOR_IDLE = (0.66, 0.66, 0.66)
COLOR_ALERT = (1.00, 0.20, 0.20)
COLOR_OVERFLOW = (0.95, 0.60, 0.30)
COLOR_NOTE = (0.72, 0.72, 0.72)
# Own colour for a stale field, so "stopped publishing" is visible from across
# the room and never reads like a value that is still being observed.
COLOR_STALE = (0.98, 0.62, 0.20)

# Notes region.  Three lines, three markers, unchanged positions.  The third one
# carries the run-state summary in front of the legend, which is why it is built
# per cycle in footer_lines() instead of being a constant.  Each line is kept at
# or under 129 characters: at NOTE_HEIGHT that is the 13.64 m half-width the
# 35-character column rows already set, so the notes never widen the view.
FOOTER_LINE_FRAME = (
    "FIXED FRAME odom - deliberately NO map->odom TF; /map cubes and "
    "map-frame arrows drawn in odom, DISPLAY ONLY; stale arrows dim"
)
FOOTER_LINE_SOURCES = (
    "SOURCES /dg/{gnss,lidar}/quality /dg/{fusion,navigation}/status "
    "/dg/relocalization/{status,match_quality} /odom;vx,wz=lin.x,ang.z"
)
FOOTER_LINE_LEGEND = (
    "NO_DATA=never seen; %s t=no msg for t (>%.1fs); ~=derived; %s=elided; "
    "%s=split" % (STALE_PREFIX, STALE_AFTER_S, ELISION_MARKER, HARD_SPLIT_MARKER)
)
FOOTER_LINES = (FOOTER_LINE_FRAME, FOOTER_LINE_SOURCES, FOOTER_LINE_LEGEND)


class TextLine(NamedTuple):
    """One rendered line.  ``text is None`` means: publish a DELETE marker."""

    ns: str
    marker_id: int
    x: float
    y: float
    height: float
    text: str | None
    color: tuple[float, float, float]


class Region(NamedTuple):
    """A header line plus ``rows`` single-line slots below it."""

    ns: str
    id_base: int
    x: float
    y_top: float
    rows: int
    title: str
    color: tuple[float, float, float]


BANNER_NS = "dg_banner"
BANNER_ID_BASE = 1000
NOTE_NS = "dg_view_notes"
NOTE_ID_BASE = 1500

LEFT_REGION = Region(
    "dg_left_column", 1100, -COLUMN_X, COLUMN_Y_TOP, 9,
    "SCENARIO / SENSOR HEALTH", COLOR_LEFT,
)
RIGHT_REGION = Region(
    "dg_right_column", 1200, COLUMN_X, COLUMN_Y_TOP, 8,
    "ACTIVE RELOCALIZATION", COLOR_RIGHT,
)
CANDIDATE_REGION = Region(
    "dg_candidate_block", 1300, -BLOCK_X, BLOCK_Y_TOP, 6,
    "SCAN-TO-MAP CANDIDATE", COLOR_BLOCK_A,
)
MOTION_REGION = Region(
    "dg_motion_block", 1400, BLOCK_X, BLOCK_Y_TOP, 6,
    "MOTION / SAFETY (read-only)", COLOR_BLOCK_B,
)

# Geometry markers keep their own namespaces and their own ids, so no id is
# shared with a text line.
WALL_NS = "map_walls"
WALL_ID = 1
SCAN_MATCH_NS = "scan_match_pose"
SCAN_MATCH_ID = 20
AMCL_NS = "amcl_pose"
AMCL_ID = 21


# ===========================================================================
# Pure helpers.  No ROS import in this section, so the static layout audit can
# import this module and collect every marker position without a running graph.
# ===========================================================================


def _values(status: Any) -> dict[str, str]:
    """Flatten a DiagnosticStatus KeyValue list into a plain dict."""
    return {str(item.key): str(item.value) for item in status.values}


def _level(raw: Any) -> int:
    """DiagnosticStatus.level is a one-byte ``bytes`` value under Humble Python."""
    if isinstance(raw, (bytes, bytearray, memoryview)):
        as_bytes = bytes(raw)
        return int(as_bytes[0]) if as_bytes else 0
    try:
        return int(raw)
    except (TypeError, ValueError):
        return -1


def _level_name(level: int) -> str:
    """Name of a DiagnosticStatus level.

    Level 3 is called STALE by ``diagnostic_msgs`` itself.  That is a publisher's
    own verdict about its inputs and is rendered inside brackets after a state,
    e.g. ``RECOVERING [STALE]``; it is not this node's freshness verdict, which
    always renders as a bare ``STALE <age>`` in place of the whole value.
    """
    return _LEVEL_NAMES.get(level, "UNKNOWN")


# ---------------------------------------------------------------------------
# Staleness aging.  Pure functions over a plain {topic: monotonic stamp} dict and
# an explicit ``now``, so the three states can be tested against a fake clock
# without a ROS graph, a scenario or a running node.
# ---------------------------------------------------------------------------


def source_ages(received: dict[str, float], now: float) -> dict[str, float | None]:
    """Age in seconds of every tracked source, ``None`` when never received.

    ``None`` is deliberately not zero and not infinity: it is the third state,
    and collapsing it into either of the other two is the whole defect this
    function exists to prevent.
    """
    ages: dict[str, float | None] = {}
    for source, topic in SOURCE_TOPICS.items():
        stamp = received.get(topic)
        ages[source] = None if stamp is None else max(0.0, float(now) - float(stamp))
    return ages


def age_text(age: float) -> str:
    """Short, honest age string: 12.4s, 340s, 12m.  Never long enough to wrap."""
    age = max(0.0, float(age))
    if age < 100.0:
        return "%.1fs" % age
    if age < 6000.0:
        return "%.0fs" % age
    return "%.0fm" % (age / 60.0)


def stale_text(age: float) -> str:
    """The rendered form of a value that stopped being published."""
    return "%s %s" % (STALE_PREFIX, age_text(age))


def is_live(age: float | None, threshold: float = STALE_AFTER_S) -> bool:
    """True only for a value that really did arrive within ``threshold``."""
    return age is not None and float(age) <= float(threshold)


def observed(age: float | None, render: Any, threshold: float = STALE_AFTER_S) -> str:
    """Render a field as exactly one of the three states, never collapsed.

    ``NO_DATA`` when the topic never delivered anything, ``STALE <age>`` when it
    delivered something too old to be a live observation, and otherwise whatever
    ``render()`` observed.  ``render`` is only called in the live case, so a
    latched value cannot reach the screen once it has gone stale.
    """
    if age is None:
        return NO_DATA
    if float(age) > float(threshold):
        return stale_text(age)
    return render()


def _freshest_age(*ages: float | None) -> float | None:
    """Age of the most recent of several sources; ``None`` if none ever arrived."""
    known = [float(age) for age in ages if age is not None]
    return min(known) if known else None


def run_state_text(ages: dict[str, float | None]) -> tuple[str, bool]:
    """One-line liveness summary plus a flag: is every core source live?

    Returns the text for the notes region and ``True`` only when all of
    ``LIVENESS_SOURCES`` arrived within ``STALE_AFTER_S``.  A partly live graph
    is reported as partly live rather than being rounded up to LIVE or down to
    STALE, because both roundings would misdescribe the screenshot.
    """
    total = len(LIVENESS_SOURCES)
    live = [name for name in LIVENESS_SOURCES if is_live(ages.get(name))]
    if len(live) == total:
        return RUN_STATE_LIVE, True
    if live:
        return RUN_STATE_PARTIAL % (len(live), total), False
    if any(ages.get(name) is not None for name in LIVENESS_SOURCES):
        return RUN_STATE_STALE, False
    return RUN_STATE_NEVER, False


def _pick_status(message: Any, *hints: str) -> Any | None:
    """Return the DiagnosticStatus whose name matches one of ``hints``.

    Matching is done on ``status.name`` rather than taking ``status[0]`` blindly,
    so a multi-status DiagnosticArray can never cause one component's values to
    be rendered under another component's label -- which in this node would put
    unrelated numbers on screen inside a captioned evidence screenshot.
    Mirrors monitor_node._pick.
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


def _field(values: dict[str, str] | None, key: str) -> str:
    """Return an observed field, or NO_DATA when it was never published."""
    if values is None:
        return NO_DATA
    value = values.get(key)
    if value is None:
        return NO_DATA
    value = value.strip()
    if not value or value.lower() == "none":
        return NO_DATA
    return value


def _number_field(
    values: dict[str, str] | None, key: str, digits: int = 3
) -> str:
    if values is None:
        return NO_DATA
    raw = values.get(key)
    if raw is None:
        return NO_DATA
    try:
        return f"{float(raw):.{digits}f}"
    except (TypeError, ValueError):
        return raw.strip() or NO_DATA


def _state_field(status: Any, values: dict[str, str]) -> str:
    """Render the Active Relocalization state, never inventing one.

    Only the ten documented state names are shown verbatim.  Anything else is
    surfaced as UNEXPECTED[...] so a screenshot can never claim a state the
    state machine does not have, and so a typo upstream is visible rather than
    silently coerced into something that reads like success.
    """
    raw = (values.get(STATUS_KEY_STATE) or str(status.message) or "").strip()
    if not raw:
        return NO_DATA
    if raw not in VALID_STATES:
        return f"UNEXPECTED[{raw}]"
    return raw


def _breakable(text: str) -> str:
    """Give the wrapper somewhere to break inside ``a;b;c`` reason strings."""
    return text.replace(";", "; ").replace(",", ", ")


def wrap_value(value: str, budget: int = VALUE_BUDGET) -> list[str]:
    """Split a value into rows of at most ``budget`` characters.

    Wrapping happens on word boundaries where there is one; a single token that
    is longer than a row is hard-split rather than truncated, because dropping
    the tail of an observed string would misreport it.  A hard split ends its row
    with ``HARD_SPLIT_MARKER``, so a reader can tell a broken token from a
    complete one: ``AMCL_COVARIANCE_HIGH`` renders as ``AMCL_COVARIANCE_HI-``
    plus ``GH``, never as a bare ``AMCL_COVARIANCE_HIG`` that looks like the
    whole value.
    """
    text = " ".join(str(value).split())
    if not text:
        return [NO_DATA]
    budget = max(4, int(budget))
    head = max(1, budget - len(HARD_SPLIT_MARKER))
    rows: list[str] = []
    current = ""
    for word in text.split(" "):
        while len(word) > budget:
            if current:
                rows.append(current)
                current = ""
            rows.append(word[:head] + HARD_SPLIT_MARKER)
            word = word[head:]
        if not word:
            continue
        candidate = word if not current else current + " " + word
        if len(candidate) <= budget:
            current = candidate
        else:
            rows.append(current)
            current = word
    if current:
        rows.append(current)
    return rows or [NO_DATA]


def _row_color(value: str, region_color: tuple[float, float, float]) -> tuple:
    if value.startswith("!!"):
        return COLOR_ALERT
    if value.startswith(NO_DATA):
        return COLOR_IDLE
    if value.startswith(STALE_PREFIX + " "):
        return COLOR_STALE
    if value.startswith("UNEXPECTED[") or value.startswith("N/A"):
        return COLOR_OVERFLOW
    return region_color


def row_lines(label: str, value: str, limit: int) -> list[str]:
    """Lay one label/value pair out over at most ``limit`` physical lines.

    A value that needs more lines than it is allowed to take is cut and the cut
    is marked with " ...", never left silent; the monitor still prints the field
    in full.  A trailing hard-split marker is dropped before the elision marker
    goes on, so a cut row ends in exactly one marker rather than in ``-...``.
    The label always survives, so no field can be pushed off screen by a
    neighbour's long string.
    """
    chunks = wrap_value(value)
    limit = max(1, int(limit))
    if len(chunks) > limit:
        chunks = chunks[:limit]
        last = chunks[-1]
        if last.endswith(HARD_SPLIT_MARKER):
            last = last[: -len(HARD_SPLIT_MARKER)]
        tail = " " + ELISION_MARKER
        if len(last) + len(tail) > VALUE_BUDGET:
            last = last[: max(0, VALUE_BUDGET - len(tail))].rstrip()
        chunks[-1] = (last + tail).strip()
    lines = []
    for index, chunk in enumerate(chunks):
        if index == 0:
            lines.append(f"{label:<{LABEL_WIDTH}}{LABEL_GAP}{chunk}")
        else:
            lines.append(CONTINUATION_INDENT + chunk)
    return lines


def region_lines(region: Region, rows: list[tuple[str, str]]) -> list[TextLine]:
    """Render one region: a header line plus one marker per physical line.

    Every field is guaranteed its own first line; the slots left over are handed
    out to continuation lines in field order, so a long trigger_reason wraps into
    the spare space instead of starving the fields below it.
    """
    lines = [
        TextLine(
            region.ns, region.id_base, region.x, region.y_top,
            BODY_HEIGHT, region.title, COLOR_HEADER,
        )
    ]
    rendered: list[tuple[str, tuple]] = []
    spare = max(0, region.rows - len(rows))
    for label, value in rows:
        color = _row_color(value, region.color)
        allowed = 1 + min(spare, ROW_MAX_LINES - 1)
        texts = row_lines(label, value, allowed)
        spare -= len(texts) - 1
        for text in texts:
            rendered.append((text, color))
    if len(rendered) > region.rows:
        keep = max(0, region.rows - 1)
        hidden = len(rendered) - keep
        rendered = rendered[:keep]
        rendered.append((f"+{hidden} MORE (see monitor)", COLOR_OVERFLOW))
    for slot in range(region.rows):
        marker_id = region.id_base + 1 + slot
        y = region.y_top - ROW_STEP * (slot + 1)
        if slot < len(rendered):
            text, color = rendered[slot]
            lines.append(
                TextLine(region.ns, marker_id, region.x, y, BODY_HEIGHT, text, color)
            )
        else:
            # Nothing observed for this slot: clear it instead of leaving a
            # stale line on screen.
            lines.append(
                TextLine(region.ns, marker_id, region.x, y, BODY_HEIGHT, None, COLOR_IDLE)
            )
    return lines


def banner_lines() -> list[TextLine]:
    colors = (COLOR_BANNER, COLOR_BANNER_WARN, COLOR_BANNER_WARN)
    return [
        TextLine(
            BANNER_NS, BANNER_ID_BASE + index, 0.0,
            BANNER_Y - BANNER_STEP * index, BANNER_HEIGHT, text, colors[index],
        )
        for index, text in enumerate(BANNER_LINES)
    ]


def footer_lines(observation: dict[str, Any] | None = None) -> list[TextLine]:
    """The three notes markers, with the run-state summary on the third.

    The run state reuses the existing third notes slot -- same namespace, same
    id, same position, same height -- rather than adding a line, so the audited
    layout is untouched.  It is coloured COLOR_STALE whenever the core sources
    are not all live, which is what makes an after-the-fact screenshot obvious
    without reading any of the individual fields.
    """
    ages = (observation or {}).get("ages") or {}
    state_text, all_live = run_state_text(ages)
    texts = (
        FOOTER_LINE_FRAME,
        FOOTER_LINE_SOURCES,
        "%s | %s" % (state_text, FOOTER_LINE_LEGEND),
    )
    colors = (COLOR_NOTE, COLOR_NOTE, COLOR_NOTE if all_live else COLOR_STALE)
    return [
        TextLine(
            NOTE_NS, NOTE_ID_BASE + index, 0.0, NOTE_Y - NOTE_STEP * index,
            NOTE_HEIGHT, text, colors[index],
        )
        for index, text in enumerate(texts)
    ]


# ---------------------------------------------------------------------------
# Row content.  Every builder takes the plain observation snapshot produced by
# the node, so the values on screen can only be values a topic delivered.
# ---------------------------------------------------------------------------


def _gnss_value(observation: dict[str, Any]) -> str:
    values = observation.get("gnss")
    if values is None:
        return NO_DATA
    level = int(observation.get("gnss_level", -1))
    return f"{_field(values, GNSS_KEY_STATE)} [{_level_name(level)}]"


def _lidar_value(observation: dict[str, Any]) -> str:
    """/dg/lidar/quality carries no state key; the state is derived.

    The derivation is the same one monitor_node and evaluator_node use
    (level + geometry_score), and the value is marked with ``~`` so the screen
    never presents a derived label as a published one.
    """
    values = observation.get("lidar")
    if values is None:
        return NO_DATA
    level = int(observation.get("lidar_level", -1))
    raw = values.get(LIDAR_KEY_GEOMETRY_SCORE)
    try:
        geometry = None if raw is None else float(raw)
    except (TypeError, ValueError):
        geometry = None
    if level >= 2 or (geometry is not None and geometry <= 0.0):
        state = "REJECTED~"
    elif geometry is None:
        state = "N/A~"
    elif geometry < 0.2:
        state = "DEGRADED~"
    else:
        state = "GOOD~"
    return f"{state} geom={_number_field(values, LIDAR_KEY_GEOMETRY_SCORE)}"


def _fusion_value(observation: dict[str, Any]) -> str:
    values = observation.get("fusion")
    if values is None:
        return NO_DATA
    return (
        f"{_field(values, FUSION_KEY_MODE)} "
        f"conf={_number_field(values, FUSION_KEY_CONFIDENCE, 2)}"
    )


def _verification_value(values: dict[str, str] | None) -> str:
    if values is None:
        return NO_DATA
    raw = values.get(STATUS_KEY_VERIFICATION_COUNT)
    if raw is None:
        return NO_DATA
    try:
        count = int(float(raw))
    except (TypeError, ValueError):
        return f"UNEXPECTED[{raw.strip()}]"
    total = VERIFICATION_SAMPLES
    filled = max(0, min(total, count))
    bar = "#" * filled + "." * (total - filled)
    return f"{count}/{total} [{bar}] {total}=default"


def _quality_class(values: dict[str, str] | None) -> str:
    """ACCEPTED / REJECTED exactly as the matcher published it."""
    if values is None:
        return NO_DATA
    raw = values.get(QUALITY_KEY_ACCEPTED)
    if raw is None:
        return NO_DATA
    token = raw.strip().lower()
    if token in ("true", "1", "yes"):
        state = "ACCEPTED"
    elif token in ("false", "0", "no"):
        state = "REJECTED"
    else:
        return f"UNEXPECTED[{raw.strip()}]"
    reason = _field(values, QUALITY_KEY_REASON)
    if reason == NO_DATA:
        return state
    return _breakable(f"{state}; {reason}")


def _twist_value(command: tuple[float, float] | None) -> str:
    if command is None:
        return NO_DATA
    return f"vx={command[0]:+.3f} wz={command[1]:+.3f}"


def _cmd_source_value(
    test: tuple[float, float] | None, recovery: tuple[float, float] | None
) -> str:
    """geometry_msgs/Twist carries no source field, so this is an inference.

    Mirrors monitor_node._cmd_source, and is labelled as inferred on screen.
    """
    zero = 1e-9
    equal = 1e-6
    if test is None and recovery is None:
        return NO_DATA
    if test is None or recovery is None:
        return "UNKNOWN (1 stream)"
    if abs(test[0]) <= zero and abs(test[1]) <= zero:
        if abs(recovery[0]) <= zero and abs(recovery[1]) <= zero:
            return "ZERO"
    if abs(test[0] - recovery[0]) <= equal and abs(test[1] - recovery[1]) <= equal:
        return "RECOVERY (inferred)"
    return "UNKNOWN (inferred)"


def _safety_value(publishers: int | None) -> str:
    """Whether a real chassis command path is live, from a graph query only."""
    if publishers is None:
        return "N/A (no graph API)"
    if publishers > 0:
        return f"!! {CHASSIS_CMD_VEL_TOPIC} pubs={publishers} CHASSIS PATH LIVE !!"
    return f"{CHASSIS_CMD_VEL_TOPIC} pubs=0"


def _uptime_value(seconds: float) -> str:
    """How long THIS NODE has been running.  Nothing else.

    The row used to be labelled ELAPSED with "(viz uptime)" in the value, which
    is honest but reads like scenario elapsed time in a screenshot.  The label is
    now VIZ UPTIME outright, so the number cannot be mistaken for a run duration.
    Scenario elapsed time is not published to this node by anything, so it stays
    absent rather than being reconstructed from this clock.
    """
    if seconds < 1000.0:
        return "%.1fs" % seconds
    return "%.0fs" % seconds


def left_column_rows(observation: dict[str, Any]) -> list[tuple[str, str]]:
    # SCENARIO and PHASE are NO_TOPIC on purpose: no node in this repo
    # publishes them, so there is nothing to observe and nothing to invent.
    # VIZ UPTIME is this node's own monotonic clock, so it is always live and is
    # never aged; every other row here is only shown while its topic is live.
    ages = observation.get("ages") or {}
    return [
        ("SCENARIO", NO_TOPIC),
        ("PHASE", NO_TOPIC),
        ("VIZ UPTIME", _uptime_value(float(observation.get("elapsed", 0.0)))),
        ("GNSS", observed(ages.get("gnss"), lambda: _gnss_value(observation))),
        ("LIDAR", observed(ages.get("lidar"), lambda: _lidar_value(observation))),
        ("FUSION", observed(ages.get("fusion"), lambda: _fusion_value(observation))),
        (
            "NAV HEALTH",
            observed(
                ages.get("navigation"),
                lambda: _field(observation.get("nav"), NAV_KEY_OVERALL_STATE),
            ),
        ),
    ]


def right_column_rows(observation: dict[str, Any]) -> list[tuple[str, str]]:
    # Every row here comes from one DiagnosticArray, so one age governs them all.
    # This is the row block that showed FAILED after the S05 run had finished:
    # the supervisor really did publish FAILED during teardown, and the value was
    # then held on screen with nothing to say it was no longer being observed.
    reloc = observation.get("reloc")
    state = observation.get("reloc_state") or NO_DATA
    age = (observation.get("ages") or {}).get("relocalization")
    return [
        ("RELOCALIZATION", observed(age, lambda: state if reloc is not None else NO_DATA)),
        (
            "TRIGGER",
            observed(age, lambda: _breakable(_field(reloc, STATUS_KEY_TRIGGER_REASON))),
        ),
        ("VERIFICATION", observed(age, lambda: _verification_value(reloc))),
        ("SEED SOURCE", observed(age, lambda: _field(reloc, STATUS_KEY_SEED_SOURCE))),
        ("AMCL HEALTH", observed(age, lambda: _field(reloc, STATUS_KEY_AMCL_HEALTH))),
    ]


def candidate_rows(observation: dict[str, Any]) -> list[tuple[str, str]]:
    match = observation.get("match")
    age = (observation.get("ages") or {}).get("match_quality")
    return [
        ("CANDIDATE", observed(age, lambda: "OBSERVED age=%s" % age_text(age))),
        ("MATCH SCORE", observed(age, lambda: _number_field(match, QUALITY_KEY_SCORE))),
        (
            "INLIER RATIO",
            observed(age, lambda: _number_field(match, QUALITY_KEY_INLIER_RATIO)),
        ),
        (
            "MEAN DISTANCE",
            observed(age, lambda: _number_field(match, QUALITY_KEY_MEAN_DISTANCE)),
        ),
        ("QUALITY CLASS", observed(age, lambda: _quality_class(match))),
    ]


def motion_rows(observation: dict[str, Any]) -> list[tuple[str, str]]:
    ages = observation.get("ages") or {}
    yaw = observation.get("odom_yaw")
    test = observation.get("test_cmd")
    recovery = observation.get("recovery_cmd")
    test_age = ages.get("test_cmd_vel")
    recovery_age = ages.get("recovery_cmd")
    return [
        (
            "ODOM YAW",
            observed(
                ages.get("odom"),
                lambda: NO_DATA if yaw is None else "%+.2f deg" % float(yaw),
            ),
        ),
        ("TEST CMD_VEL", observed(test_age, lambda: _twist_value(test))),
        ("RECOVERY CMD", observed(recovery_age, lambda: _twist_value(recovery))),
        # The source inference is only as fresh as the streams it compares, and a
        # stale stream is withheld from it rather than being compared as if live:
        # one live stream honestly yields "UNKNOWN (1 stream)".
        (
            "CMD SOURCE",
            observed(
                _freshest_age(test_age, recovery_age),
                lambda: _cmd_source_value(
                    test if is_live(test_age) else None,
                    recovery if is_live(recovery_age) else None,
                ),
            ),
        ),
        # SAFETY is a live graph query every cycle, not a latched message, so it
        # has no age to report.
        ("SAFETY", _safety_value(observation.get("cmd_vel_publishers"))),
    ]


def build_text_lines(observation: dict[str, Any]) -> list[TextLine]:
    """Every text marker of one publish cycle, in one flat list."""
    lines: list[TextLine] = []
    lines.extend(banner_lines())
    lines.extend(region_lines(LEFT_REGION, left_column_rows(observation)))
    lines.extend(region_lines(RIGHT_REGION, right_column_rows(observation)))
    lines.extend(region_lines(CANDIDATE_REGION, candidate_rows(observation)))
    lines.extend(region_lines(MOTION_REGION, motion_rows(observation)))
    lines.extend(footer_lines(observation))
    return lines


def line_width(text: str, height: float) -> float:
    """Conservative rendered width of one line, in metres."""
    longest = max((len(part) for part in str(text).split("\n")), default=0)
    return longest * height * CHAR_ASPECT


def line_box(line: TextLine) -> tuple[float, float, float, float]:
    """(x_min, x_max, y_min, y_max) of a centred text line, in metres."""
    half_width = line_width(line.text or "", line.height) * 0.5
    half_height = line.height * 0.5
    return (
        line.x - half_width,
        line.x + half_width,
        line.y - half_height,
        line.y + half_height,
    )


# ===========================================================================
# Marker construction.  ROS imports stay local, as elsewhere in this package.
# ===========================================================================


def _new_marker(namespace: str, marker_id: int, marker_type: int, stamp: Any) -> Any:
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = MARKER_FRAME
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = int(marker_type)
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.frame_locked = False
    return marker


def _delete_marker(namespace: str, marker_id: int, stamp: Any) -> Any:
    """Clear a marker instead of drawing an unobserved value."""
    from visualization_msgs.msg import Marker

    marker = _new_marker(namespace, marker_id, Marker.ARROW, stamp)
    marker.action = Marker.DELETE
    return marker


def _set_color(marker: Any, red: float, green: float, blue: float, alpha: float = 1.0) -> None:
    marker.color.r = float(red)
    marker.color.g = float(green)
    marker.color.b = float(blue)
    marker.color.a = float(alpha)


def _text_marker(line: TextLine, stamp: Any) -> Any:
    """One marker for one line, or a DELETE marker for an empty slot."""
    from visualization_msgs.msg import Marker

    if line.text is None:
        return _delete_marker(line.ns, line.marker_id, stamp)
    marker = _new_marker(line.ns, line.marker_id, Marker.TEXT_VIEW_FACING, stamp)
    marker.pose.position.x = float(line.x)
    marker.pose.position.y = float(line.y)
    marker.pose.position.z = float(TEXT_Z)
    marker.scale.z = float(line.height)
    marker.text = line.text
    _set_color(marker, *line.color)
    return marker


def _arrow_marker(
    namespace: str,
    marker_id: int,
    stamp: Any,
    pose: Any,
    length: float,
    color: tuple[float, float, float],
    alpha: float = 1.0,
) -> Any:
    """One pose arrow.  ``alpha`` below 1.0 marks a pose that has gone stale."""
    from visualization_msgs.msg import Marker

    marker = _new_marker(namespace, marker_id, Marker.ARROW, stamp)
    marker.pose.position.x = float(pose.position.x)
    marker.pose.position.y = float(pose.position.y)
    marker.pose.position.z = float(pose.position.z)
    marker.pose.orientation = pose.orientation
    marker.scale.x = float(length)
    marker.scale.y = float(length) * 0.16
    marker.scale.z = float(length) * 0.24
    _set_color(marker, *color, alpha=float(alpha))
    return marker


def _yaw_degrees(orientation: Any) -> float:
    import math

    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def occupied_cell_points(grid: Any, stride: int, max_cells: int) -> tuple[list[Any], bool]:
    """Return the centres of the occupied cells of ``grid``, in metres.

    The cell centres are computed from ``info.resolution`` and ``info.origin``.
    The grid origin rotation is not applied: the synthetic scenario publishes an
    identity origin orientation, and applying an unverified rotation would be a
    guess rather than observed data.
    """
    from geometry_msgs.msg import Point

    info = grid.info
    resolution = float(info.resolution)
    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)
    width = int(info.width)
    height = int(info.height)
    data = grid.data
    step = max(1, int(stride))
    points: list[Any] = []
    truncated = False
    for row in range(0, height, step):
        row_base = row * width
        for col in range(0, width, step):
            index = row_base + col
            if index >= len(data):
                continue
            if int(data[index]) < OCCUPIED_THRESHOLD:
                continue
            if len(points) >= max_cells:
                return points, True
            point = Point()
            point.x = origin_x + (col + 0.5) * resolution
            point.y = origin_y + (row + 0.5) * resolution
            point.z = 0.0
            points.append(point)
    return points, truncated


def build_node() -> Any:
    """Create the visualization-only node (ROS imports stay local, as elsewhere)."""
    import time

    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray

    class VisualizationOnlyNode(Node):
        """Read-only RViz evidence renderer.  One publisher, twelve subscriptions."""

        def __init__(self) -> None:
            super().__init__("dg_validation_visualization_only_node")
            self.declare_parameter("publish_rate_hz", 5.0)
            self.declare_parameter("wall_stride", 1)
            self.declare_parameter("wall_height", 0.2)
            self.declare_parameter("max_wall_cells", 60000)
            self.declare_parameter("arrow_length", 0.9)

            self._started = time.monotonic()
            self._map: Any | None = None
            self._map_warned_origin = False
            self._map_warned_truncated = False
            self._received: dict[str, float] = {}

            self._odom_yaw: float | None = None
            self._amcl_pose: Any | None = None
            self._scan_match_pose: Any | None = None
            self._status_values: dict[str, str] | None = None
            self._status_state = NO_DATA
            self._quality_values: dict[str, str] | None = None
            self._gnss_values: dict[str, str] | None = None
            self._gnss_level = -1
            self._lidar_values: dict[str, str] | None = None
            self._lidar_level = -1
            self._fusion_values: dict[str, str] | None = None
            self._nav_values: dict[str, str] | None = None
            self._test_cmd: tuple[float, float] | None = None
            self._recovery_cmd: tuple[float, float] | None = None

            # The ONLY publisher in this node.  Outside the /dg/* namespace, which
            # belongs to the algorithms under test.
            self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

            # Default (volatile) QoS on /map: it matches both the synthetic
            # injector, which republishes the grid continuously, and a
            # transient_local map_server.  A transient_local *subscription* would
            # not match the injector's volatile publisher.
            self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, 1)
            self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
            self.create_subscription(
                PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._on_amcl_pose, 10
            )
            self.create_subscription(
                PoseStamped, SCAN_MATCH_POSE_TOPIC, self._on_scan_match_pose, 10
            )
            self.create_subscription(
                DiagnosticArray, RELOCALIZATION_STATUS_TOPIC, self._on_status, 10
            )
            self.create_subscription(
                DiagnosticArray, MATCH_QUALITY_TOPIC, self._on_match_quality, 10
            )
            self.create_subscription(
                DiagnosticArray, GNSS_QUALITY_TOPIC, self._on_gnss, 10
            )
            self.create_subscription(
                DiagnosticArray, LIDAR_QUALITY_TOPIC, self._on_lidar, 10
            )
            self.create_subscription(
                DiagnosticArray, FUSION_STATUS_TOPIC, self._on_fusion, 10
            )
            self.create_subscription(
                DiagnosticArray, NAVIGATION_STATUS_TOPIC, self._on_navigation, 10
            )
            self.create_subscription(Twist, TEST_CMD_VEL_TOPIC, self._on_test_cmd, 10)
            self.create_subscription(
                Twist, RECOVERY_CMD_VEL_TOPIC, self._on_recovery_cmd, 10
            )

            rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
            self._timer = self.create_timer(1.0 / rate, self._publish_markers)
            self.get_logger().info(
                "visualization-only node: publishes %s and nothing else; "
                "no TF is published and no map->odom transform is created"
                % MARKER_TOPIC
            )

        # ------------------------------------------------------------------
        # Read-only callbacks.  They only record what really arrived.
        # ------------------------------------------------------------------
        def _mark(self, topic: str) -> None:
            """Record when a message really arrived, for the staleness aging."""
            self._received[topic] = time.monotonic()

        def _on_map(self, message: Any) -> None:
            self._map = message
            self._mark(MAP_TOPIC)
            orientation = message.info.origin.orientation
            identity = abs(float(orientation.w)) > 0.9999
            if not identity and not self._map_warned_origin:
                self._map_warned_origin = True
                self.get_logger().warn(
                    "/map origin orientation is not identity; wall cubes are drawn "
                    "from the origin translation only (display convenience)"
                )

        def _on_odom(self, message: Any) -> None:
            self._odom_yaw = _yaw_degrees(message.pose.pose.orientation)
            self._mark(ODOM_TOPIC)

        def _on_amcl_pose(self, message: Any) -> None:
            self._amcl_pose = message
            self._mark(AMCL_POSE_TOPIC)

        def _on_scan_match_pose(self, message: Any) -> None:
            self._scan_match_pose = message
            self._mark(SCAN_MATCH_POSE_TOPIC)

        def _on_status(self, message: Any) -> None:
            status = _pick_status(
                message, "active relocalization", "relocalization supervisor"
            )
            if status is None:
                return
            self._status_values = _values(status)
            self._status_state = _state_field(status, self._status_values)
            self._mark(RELOCALIZATION_STATUS_TOPIC)

        def _on_match_quality(self, message: Any) -> None:
            status = _pick_status(
                message, "scan-to-map match", "match quality", "scan-to-map"
            )
            if status is None:
                return
            self._quality_values = _values(status)
            self._mark(MATCH_QUALITY_TOPIC)

        def _on_gnss(self, message: Any) -> None:
            status = _pick_status(message, "gnss quality", "gnss")
            if status is None:
                return
            self._gnss_values = _values(status)
            self._gnss_level = _level(status.level)
            self._mark(GNSS_QUALITY_TOPIC)

        def _on_lidar(self, message: Any) -> None:
            status = _pick_status(message, "dg/lidar/quality", "lidar")
            if status is None:
                return
            self._lidar_values = _values(status)
            self._lidar_level = _level(status.level)
            self._mark(LIDAR_QUALITY_TOPIC)

        def _on_fusion(self, message: Any) -> None:
            status = _pick_status(message, "multi-source fusion", "fusion")
            if status is None:
                return
            self._fusion_values = _values(status)
            self._mark(FUSION_STATUS_TOPIC)

        def _on_navigation(self, message: Any) -> None:
            status = _pick_status(message, "navigation health", "navigation")
            if status is None:
                return
            self._nav_values = _values(status)
            self._mark(NAVIGATION_STATUS_TOPIC)

        def _on_test_cmd(self, message: Any) -> None:
            self._test_cmd = (float(message.linear.x), float(message.angular.z))
            self._mark(TEST_CMD_VEL_TOPIC)

        def _on_recovery_cmd(self, message: Any) -> None:
            self._recovery_cmd = (float(message.linear.x), float(message.angular.z))
            self._mark(RECOVERY_CMD_VEL_TOPIC)

        # ------------------------------------------------------------------
        # Observation snapshot and marker construction
        # ------------------------------------------------------------------
        def _chassis_publishers(self) -> int | None:
            """Graph query only.  This node never publishes /cmd_vel."""
            try:
                return int(self.count_publishers(CHASSIS_CMD_VEL_TOPIC))
            except Exception:  # pragma: no cover - API unavailable
                return None

        def _observation(self) -> dict[str, Any]:
            # "ages" is the only clock reading the renderers get.  Every latched
            # value below is published together with the age of the message it
            # came from, so nothing can be drawn as live without its own evidence
            # that it still is.
            return {
                "elapsed": time.monotonic() - self._started,
                "ages": source_ages(self._received, time.monotonic()),
                "gnss": self._gnss_values,
                "gnss_level": self._gnss_level,
                "lidar": self._lidar_values,
                "lidar_level": self._lidar_level,
                "fusion": self._fusion_values,
                "nav": self._nav_values,
                "reloc": self._status_values,
                "reloc_state": self._status_state,
                "match": self._quality_values,
                "odom_yaw": self._odom_yaw,
                "test_cmd": self._test_cmd,
                "recovery_cmd": self._recovery_cmd,
                "cmd_vel_publishers": self._chassis_publishers(),
            }

        def _wall_marker(self, stamp: Any) -> Any:
            """Occupied /map cells as a CUBE_LIST expressed in odom.

            Display-only frame substitution: the synthetic scenario places the
            odom origin at the map origin by construction, so the cells are drawn
            with their map coordinates in the odom frame.  Nothing here is
            published as TF or fed back to any algorithm under test.
            """
            stride = int(self.get_parameter("wall_stride").value)
            max_cells = int(self.get_parameter("max_wall_cells").value)
            height = float(self.get_parameter("wall_height").value)
            marker = _new_marker(WALL_NS, WALL_ID, Marker.CUBE_LIST, stamp)
            points, truncated = occupied_cell_points(self._map, stride, max_cells)
            resolution = float(self._map.info.resolution) * max(1, stride)
            marker.scale.x = resolution
            marker.scale.y = resolution
            marker.scale.z = height
            marker.pose.position.z = height * 0.5
            marker.points = points
            _set_color(marker, 0.75, 0.75, 0.78, 0.85)
            if truncated and not self._map_warned_truncated:
                self._map_warned_truncated = True
                self.get_logger().warn(
                    "/map has more occupied cells than max_wall_cells=%d; "
                    "the wall marker is truncated for display" % max_cells
                )
            return marker

        def _publish_markers(self) -> None:
            stamp = self.get_clock().now().to_msg()
            array = MarkerArray()
            observation = self._observation()
            ages = observation["ages"]

            # One marker per line: banner, both columns, both bottom blocks and
            # the footer notes.  Empty slots arrive as DELETE markers.
            for line in build_text_lines(observation):
                array.markers.append(_text_marker(line, stamp))

            if self._map is not None:
                array.markers.append(self._wall_marker(stamp))

            # /scan_match_pose and /amcl_pose carry header.frame_id "map".  They are
            # drawn in odom under the same display-only origin assumption as the
            # walls; no transform is published and nothing is fed back.
            #
            # A pose that has stopped arriving is drawn faded rather than deleted.
            # Both topics are legitimately intermittent -- AMCL only republishes
            # when its filter updates, and a scan match only exists when one
            # succeeded -- so the last known arrow stays useful, while the fade
            # stops a teardown-time pose from reading as the current estimate.
            arrow_length = float(self.get_parameter("arrow_length").value)
            if self._scan_match_pose is not None:
                array.markers.append(
                    _arrow_marker(
                        SCAN_MATCH_NS,
                        SCAN_MATCH_ID,
                        stamp,
                        self._scan_match_pose.pose,
                        arrow_length,
                        (1.0, 0.1, 1.0),
                        1.0 if is_live(ages.get("scan_match_pose")) else STALE_ARROW_ALPHA,
                    )
                )
            else:
                array.markers.append(_delete_marker(SCAN_MATCH_NS, SCAN_MATCH_ID, stamp))

            if self._amcl_pose is not None:
                amcl_marker = _arrow_marker(
                    AMCL_NS,
                    AMCL_ID,
                    stamp,
                    self._amcl_pose.pose.pose,
                    arrow_length * 0.8,
                    (0.1, 1.0, 0.25),
                    1.0 if is_live(ages.get("amcl_pose")) else STALE_ARROW_ALPHA,
                )
                amcl_marker.pose.position.z += 0.12
                array.markers.append(amcl_marker)
            else:
                array.markers.append(_delete_marker(AMCL_NS, AMCL_ID, stamp))

            self._marker_pub.publish(array)

    return VisualizationOnlyNode()


def main(args: list[str] | None = None) -> int:
    import rclpy

    rclpy.init(args=args)
    node = build_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
