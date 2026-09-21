"""Generate conservative static PNG curves from recorded evaluator CSV data."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any


# The mandatory banner, as an exact ASCII literal.  Do not "typographically
# improve" the separator: an em dash breaks any grep/CI check for the mandated
# string "SYNTHETIC SOFTWARE VALIDATION - NOT REAL ROBOT DATA".
PLOT_LABEL = "SYNTHETIC SOFTWARE VALIDATION - NOT REAL ROBOT DATA"

# Reference values copied verbatim from the real algorithm defaults.  They are
# drawn as dashed threshold lines only; no recorded sample is ever rescaled,
# clamped or interpolated so that it agrees with them.
SCAN_MATCH_SCORE_THRESHOLD = 0.45
SCAN_MATCH_INLIER_RATIO_THRESHOLD = 0.45
SCAN_MATCH_MEAN_DISTANCE_THRESHOLD = 0.30
VERIFICATION_SAMPLES_DEFAULT = 3
MAX_COVARIANCE_THRESHOLD = 0.5

# Exact Active Relocalization state names.  LOST and SEARCHING do not exist.
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

NAVIGATION_STATES = (
    "NOMINAL",
    "DEGRADED",
    "LOCALIZATION_SUSPECT",
    "RECOVERING",
    "FAILED",
    "MANUAL_REQUIRED",
)

RAD_TO_DEG = 180.0 / math.pi

# FILE_EXISTS != VALID_EVIDENCE.
#
# A PNG is only evidence of something if it actually carries a recorded data
# series.  A figure whose source columns were all absent or all empty renders as
# a correctly labelled, perfectly blank pair of axes -- and a blank chart filed
# as a generated plot is the plotting twin of a black screenshot filed as a
# photo.  So every artist this module draws is tagged at creation time, and the
# single write choke point counts the tagged artists that hold real data before
# it is allowed to call savefig().
#
# Tagging is what makes the count trustworthy.  Counting ``len(ax.lines)`` would
# be wrong: ``axhline`` threshold markers are Line2D objects too, and they are
# drawn from hard-coded algorithm defaults rather than from samples.csv, so a
# figure containing nothing but reference lines would otherwise self-certify as
# populated.
DATA_SERIES_GID = "dg-data-series"
REFERENCE_LINE_GID = "dg-reference-line"


def _finite_point_count(artist: Any) -> int:
    """Count points on an artist whose x and y are both finite.

    ``get_xydata`` resolves whatever matplotlib actually stored, so a series
    handed an all-``None`` timestamp column comes back as NaN and scores zero:
    the column held numbers, but no point was ever placeable on the axes.
    """
    try:
        data = artist.get_xydata()
    except Exception:
        return 0
    total = 0
    for point in data:
        try:
            x_value = float(point[0])
            y_value = float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x_value) and math.isfinite(y_value):
            total += 1
    return total


def count_data_series(fig: Any) -> int:
    """Number of tagged data series on a figure that hold at least one point.

    ``fig.axes`` includes every twin axis created with ``twinx``, so a series
    parked on a secondary scale is counted exactly like one on the primary.
    """
    total = 0
    for ax in fig.axes:
        for artist in ax.lines:
            if artist.get_gid() != DATA_SERIES_GID:
                continue
            if _finite_point_count(artist) > 0:
                total += 1
    return total


def _tag(artists: Any, gid: str) -> Any:
    """Label freshly created artists so the series count can recognise them."""
    if artists is None:
        return artists
    items = artists if isinstance(artists, (list, tuple)) else [artists]
    for artist in items:
        try:
            artist.set_gid(gid)
        except AttributeError:
            continue
    return artists


def plot_series(ax: Any, *args: Any, **kwargs: Any) -> Any:
    """``ax.plot`` for recorded samples; the result counts as evidence."""
    return _tag(ax.plot(*args, **kwargs), DATA_SERIES_GID)


def step_series(ax: Any, *args: Any, **kwargs: Any) -> Any:
    """``ax.step`` for recorded samples; the result counts as evidence."""
    return _tag(ax.step(*args, **kwargs), DATA_SERIES_GID)


def reference_line(ax: Any, value: float, **kwargs: Any) -> Any:
    """A hard-coded algorithm threshold.  Never counted as recorded data."""
    return _tag(ax.axhline(value, **kwargs), REFERENCE_LINE_GID)


def _float(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def _text(row: dict[str, str], key: str) -> str:
    value = row.get(key, "")
    return value.strip() if isinstance(value, str) else ""


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def generate_plots(run_dir: Path) -> dict[str, Any]:
    """Write plots from existing samples only; never synthesize missing values."""
    samples_path = run_dir / "samples.csv"
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    if not samples_path.exists():
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text("samples.csv is absent; no plot data were available.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS", "skipped": {}, "partial": {}, "plots": {}}
    rows = _read(samples_path)
    if len(rows) < 2:
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text("fewer than two recorded samples; no curve was invented.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS", "skipped": {}, "partial": {}, "plots": {}}
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        marker = plots_dir / "NOT_ENOUGH_DATA_FOR_PLOTS"
        marker.write_text(f"matplotlib unavailable ({type(exc).__name__}); no plot was invented.\n", encoding="utf-8")
        return {"files": [], "warning": "NOT_ENOUGH_DATA_FOR_PLOTS", "skipped": {}, "partial": {}, "plots": {}}

    x = [_float(row, "elapsed_time") for row in rows]
    files: list[str] = []
    skipped: dict[str, str] = {}
    partial: dict[str, str] = {}
    # Per-figure verdict: generated or not, how many real series were counted,
    # and for a skip exactly which columns were at fault.
    report: dict[str, dict[str, Any]] = {}
    columns = {name for row in rows for name in row.keys() if isinstance(name, str)}

    def numeric(key: str, scale: float = 1.0) -> tuple[list[float], list[float]]:
        """Pair a numeric column with its timestamps, dropping unusable cells."""
        xs: list[float] = []
        ys: list[float] = []
        for stamp, row in zip(x, rows):
            value = _float(row, key)
            if stamp is not None and value is not None:
                xs.append(stamp)
                ys.append(value * scale)
        return xs, ys

    def categorical(key: str, order: tuple[str, ...]) -> tuple[list[float], list[int], list[str]]:
        """Map a recorded state column onto ordinals using the real state names."""
        observed = [_text(row, key) for row in rows]
        # Tick labels in contract order, then anything else that was recorded.
        # A value the state machine cannot produce (LOST, SEARCHING, a typo, a
        # truncated diagnostic message) is kept -- dropping recorded data would
        # be dishonest -- but it is labelled UNEXPECTED[...] so the axis can
        # never present it as a legitimate state of the machine.
        levels: list[str] = []
        index: dict[str, int] = {}
        for name in order:
            if name in observed:
                index[name] = len(levels)
                levels.append(name)
        for name in sorted({item for item in observed if item and item not in order}):
            index[name] = len(levels)
            levels.append(f"UNEXPECTED[{name}]")
        xs: list[float] = []
        ys: list[int] = []
        for stamp, row in zip(x, rows):
            name = _text(row, key)
            if stamp is not None and name in index:
                xs.append(stamp)
                ys.append(index[name])
        return xs, ys, levels

    def populated(key: str) -> bool:
        """True when the column carries at least one non-blank recorded cell."""
        return any(_text(row, key) for row in rows)

    def skip(name: str, keys: tuple[str, ...]) -> None:
        """Record why a figure was not drawn instead of inventing a curve.

        The reason names the offending columns individually and separates the
        three ways a figure can end up with nothing to show, because "no data"
        and "data that could not be placed on a time axis" are different
        defects and the reader has to be able to tell them apart.
        """
        absent = [key for key in keys if key not in columns]
        empty = [key for key in keys if key in columns and not populated(key)]
        unplottable = [key for key in keys if key in columns and populated(key)]
        reasons = []
        if absent:
            reasons.append("column(s) absent from samples.csv: " + ", ".join(absent))
        if empty:
            reasons.append("column(s) present but every cell empty: " + ", ".join(empty))
        if unplottable:
            reasons.append(
                "column(s) present but not plottable, no sample paired a usable "
                "elapsed_time with a usable value: " + ", ".join(unplottable)
            )
        skipped[f"{name}.png"] = "; ".join(reasons) or "no usable source column"
        report[f"{name}.png"] = {
            "generated": False,
            "series_plotted": 0,
            "reason": skipped[f"{name}.png"],
        }

    def note_omitted(name: str, keys: tuple[str, ...]) -> None:
        """Document traces left out of a rendered figure; nothing is faked in."""
        absent = [key for key in keys if key not in columns]
        empty = [key for key in keys if key in columns]
        reasons = []
        if absent:
            reasons.append("trace(s) omitted, column absent: " + ", ".join(absent))
        if empty:
            reasons.append("trace(s) omitted, column has no finite value: " + ", ".join(empty))
        if reasons:
            existing = partial.get(f"{name}.png")
            merged = "; ".join(reasons)
            partial[f"{name}.png"] = f"{existing}; {merged}" if existing else merged

    def save_figure(fig: Any, name: str, keys: tuple[str, ...]) -> bool:
        """The one place a plot file may be created, and the gate on doing it.

        Every figure in this module is written through here, so the
        at-least-one-real-series rule cannot be bypassed by a figure that is
        assembled outside ``save`` -- the step/state-machine figures included.
        A figure that counts zero data series is closed undrawn: no savefig, no
        entry in ``files``, and a structured skip naming the columns at fault.
        """
        drawn = count_data_series(fig)
        if drawn == 0:
            plt.close(fig)
            skip(name, keys)
            return False
        fig.savefig(plots_dir / f"{name}.png", dpi=140)
        plt.close(fig)
        files.append(str(plots_dir / f"{name}.png"))
        report[f"{name}.png"] = {"generated": True, "series_plotted": drawn, "reason": None}
        return True

    def stacked_panels(name: str, title: str, keys: tuple[str, ...], panels: list[tuple[str, Any]]) -> None:
        """One panel per unit family, so a threshold cannot appear to sit on
        another series that merely shares the frame through a second scale."""
        count = len(panels)
        fig, raw_axes = plt.subplots(
            count, 1, sharex=True, figsize=(9, 3.0 * count + 1.2), constrained_layout=True
        )
        axes = list(raw_axes) if count > 1 else [raw_axes]
        for ax, (ylabel, draw) in zip(axes, panels):
            draw(ax)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(loc="best", fontsize=8)
        axes[-1].set_xlabel("elapsed time (s)")
        fig.suptitle(f"{title}\n{PLOT_LABEL}", fontsize=10)
        save_figure(fig, name, keys)

    def finish(fig: Any, ax: Any, name: str, title: str, keys: tuple[str, ...], twin: Any = None) -> None:
        ax.set_xlabel("elapsed time (s)")
        ax.set_title(f"{title}\n{PLOT_LABEL}", fontsize=10)
        ax.grid(True, alpha=0.3)
        handles, labels = ax.get_legend_handles_labels()
        if twin is not None:
            extra_handles, extra_labels = twin.get_legend_handles_labels()
            handles += extra_handles
            labels += extra_labels
        if handles:
            ax.legend(handles, labels, loc="best", fontsize=8)
        save_figure(fig, name, keys)

    def save(name: str, series: list[tuple[str, str]]) -> None:
        """Draw a multi-series figure, or record why it could not be drawn.

        The figure is written only when ``save_figure`` counts at least one
        plotted data series on it.  Writing an empty pair of axes and returning
        its path as a generated file would make a completely unobserved
        quantity look like a successful measurement, so an all-empty figure is
        routed to SKIPPED_PLOTS.txt instead.

        Values stay on the shared ``x`` with ``None`` rendered as a gap: nothing
        is interpolated across a sample the evaluator did not record.
        """
        keys = tuple(key for _, key in series)
        drawn: list[str] = []
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        for label, key in series:
            ys = [_float(row, key) for row in rows]
            if not any(value is not None for value in ys):
                continue
            lines = plot_series(ax, x, ys, label=label, linewidth=1.8)
            # Holding numbers is not the same as being plottable: a point exists
            # only where a usable elapsed_time met a usable value.  A trace with
            # no such point is removed rather than left as an invisible line,
            # so it cannot claim a legend entry it does not illustrate.
            if any(_finite_point_count(line) > 0 for line in lines):
                drawn.append(key)
            else:
                for line in lines:
                    line.remove()
        if not drawn:
            plt.close(fig)
            skip(name, keys)
            return
        note_omitted(name, tuple(key for key in keys if key not in drawn))
        finish(fig, ax, name, name.replace("_", " ").title(), keys)

    save("gnss_quality", [("HDOP", "gnss_hdop"), ("satellites", "gnss_satellites"), ("differential_age", "gnss_differential_age")])
    save("lidar_quality", [("geometry score", "geometry_score"), ("temporal match", "temporal_match_ratio"), ("angular coverage", "angular_coverage")])
    save("fusion_confidence", [("measurement confidence", "measurement_confidence"), ("position uncertainty", "position_uncertainty"), ("adaptive position uncertainty", "adaptive_position_uncertainty")])

    # Single-glance overview of all four state machines on one shared ordinal
    # axis.  The dedicated per-machine figures below are the authoritative ones;
    # this figure exists only so a whole run reads at a glance.
    state_keys = [
        ("gnss_state", "GNSS"),
        ("lidar_state", "LiDAR"),
        ("navigation_state", "Navigation"),
        ("relocalization_state", "Relocalization"),
    ]

    def tick_label(key: str, state: str) -> str:
        """A relocalization value outside the contract must not look valid."""
        if key == "relocalization_state" and state not in RELOCALIZATION_STATES:
            return f"UNEXPECTED[{state}]"
        return state

    timeline_keys = tuple(key for key, _ in state_keys)
    observed_states = sorted({
        tick_label(key, row.get(key, ""))
        for key, _ in state_keys
        for row in rows
        if row.get(key, "")
    })
    if observed_states:
        ymap = {state: idx for idx, state in enumerate(observed_states)}
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        for key, label in state_keys:
            xx: list[float] = []
            yy: list[int] = []
            for t, row in zip(x, rows):
                state = row.get(key, "")
                if t is None or not state:
                    continue
                xx.append(t)
                yy.append(ymap[tick_label(key, state)])
            if xx:
                step_series(ax, xx, yy, where="post", label=label, linewidth=1.6)
        ax.set_yticks(list(ymap.values()), list(ymap.keys()), fontsize=8)
        # Recorded state names alone do not make a timeline: without a usable
        # elapsed_time there is no step to draw, and save_figure refuses the
        # blank frame rather than filing it as a generated plot.
        finish(fig, ax, "state_timeline", "State timeline (all four state machines)", timeline_keys)
    else:
        skip("state_timeline", timeline_keys)

    # --- Active Relocalization supervisor state -----------------------------
    reloc_keys = ("relocalization_state", "reloc_active_segment", "reloc_attempt_id")
    reloc_x, reloc_y, reloc_levels = categorical("relocalization_state", RELOCALIZATION_STATES)
    segment_x, segment_y = numeric("reloc_active_segment")
    attempt_x, attempt_y = numeric("reloc_attempt_id")
    if reloc_x or segment_x or attempt_x:
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        twin = None
        if reloc_x:
            step_series(ax, reloc_x, reloc_y, where="post", label="relocalization_state", linewidth=1.8, color="tab:blue")
            ax.set_yticks(list(range(len(reloc_levels))), reloc_levels, fontsize=8)
            ax.set_ylim(-0.5, len(reloc_levels) - 0.5)
        else:
            ax.set_ylabel("relocalization_state not recorded")
        if segment_x or attempt_x:
            twin = ax.twinx()
            if segment_x:
                step_series(twin, segment_x, segment_y, where="post", label="reloc_active_segment", linewidth=1.3, linestyle="--", color="tab:orange")
            if attempt_x:
                step_series(twin, attempt_x, attempt_y, where="post", label="reloc_attempt_id", linewidth=1.3, linestyle=":", color="tab:green")
            twin.set_ylabel("active segment / attempt id")
        note_omitted("relocalization_state", tuple(
            key for key, data in (
                ("relocalization_state", reloc_x),
                ("reloc_active_segment", segment_x),
                ("reloc_attempt_id", attempt_x),
            ) if not data
        ))
        finish(fig, ax, "relocalization_state", "Active relocalization state machine", reloc_keys, twin=twin)
    else:
        skip("relocalization_state", reloc_keys)

    # --- Navigation health aggregator state --------------------------------
    nav_x, nav_y, nav_levels = categorical("navigation_state", NAVIGATION_STATES)
    if nav_x:
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        step_series(ax, nav_x, nav_y, where="post", label="navigation_state", linewidth=1.8, color="tab:purple")
        ax.set_yticks(list(range(len(nav_levels))), nav_levels, fontsize=8)
        ax.set_ylim(-0.5, len(nav_levels) - 0.5)
        finish(fig, ax, "navigation_health", "Navigation health state", ("navigation_state",))
    else:
        skip("navigation_health", ("navigation_state",))

    # --- cmd_vel arbiter output versus recovery request ---------------------
    cmd_keys = ("cmd_linear_x", "cmd_angular_z", "recovery_linear_x", "recovery_angular_z")
    cmd_traces = [
        ("cmd_linear_x (arbiter output)", "cmd_linear_x", "-", "tab:blue"),
        ("cmd_angular_z (arbiter output)", "cmd_angular_z", "-", "tab:cyan"),
        ("recovery_linear_x (recovery request)", "recovery_linear_x", "--", "tab:red"),
        ("recovery_angular_z (recovery request)", "recovery_angular_z", "--", "tab:orange"),
    ]
    cmd_data = {key: numeric(key) for _, key, _, _ in cmd_traces}
    if any(cmd_data[key][0] for key in cmd_data):
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        for label, key, style, colour in cmd_traces:
            xs, ys = cmd_data[key]
            if xs:
                step_series(ax, xs, ys, where="post", label=label, linewidth=1.6, linestyle=style, color=colour)
        ax.set_ylabel("linear (m/s) / angular (rad/s)")
        note_omitted("cmd_vel", tuple(key for key in cmd_keys if not cmd_data[key][0]))
        finish(fig, ax, "cmd_vel", "Arbiter cmd_vel versus recovery command", cmd_keys)
    else:
        skip("cmd_vel", cmd_keys)

    # --- Scan-to-map match quality against the real thresholds -------------
    match_keys = ("match_score", "match_inlier_ratio", "match_mean_distance")
    score_x, score_y = numeric("match_score")
    inlier_x, inlier_y = numeric("match_inlier_ratio")
    distance_x, distance_y = numeric("match_mean_distance")
    def draw_match_scores(ax: Any) -> None:
        if score_x:
            plot_series(ax, score_x, score_y, label="match_score", linewidth=1.8, color="tab:blue")
        if inlier_x:
            plot_series(ax, inlier_x, inlier_y, label="match_inlier_ratio", linewidth=1.8, color="tab:green")
        reference_line(
            ax,
            SCAN_MATCH_SCORE_THRESHOLD,
            color="tab:red",
            linestyle="--",
            linewidth=1.2,
            label=f"acceptance threshold {SCAN_MATCH_SCORE_THRESHOLD} (score and inlier_ratio must be >=)",
        )

    def draw_match_distance(ax: Any) -> None:
        plot_series(ax, distance_x, distance_y, label="match_mean_distance", linewidth=1.8, linestyle="-.", color="tab:orange")
        reference_line(
            ax,
            SCAN_MATCH_MEAN_DISTANCE_THRESHOLD,
            color="tab:brown",
            linestyle="--",
            linewidth=1.2,
            label=f"acceptance threshold {SCAN_MATCH_MEAN_DISTANCE_THRESHOLD} m (mean_distance must be <=)",
        )

    match_panels: list[tuple[str, Any]] = []
    if score_x or inlier_x:
        match_panels.append(("score / inlier ratio (unitless)", draw_match_scores))
    if distance_x:
        match_panels.append(("mean distance (m)", draw_match_distance))
    if match_panels:
        note_omitted("scan_map_quality", tuple(
            key for key, data in (
                ("match_score", score_x),
                ("match_inlier_ratio", inlier_x),
                ("match_mean_distance", distance_x),
            ) if not data
        ))
        stacked_panels("scan_map_quality", "Scan-to-map match quality vs acceptance thresholds", match_keys, match_panels)
    else:
        skip("scan_map_quality", match_keys)

    # --- Recovery verification progress and request counters ---------------
    verify_keys = ("reloc_verification_count", "seed_request_count", "match_message_count")
    verify_x, verify_y = numeric("reloc_verification_count")
    seed_x, seed_y = numeric("seed_request_count")
    message_x, message_y = numeric("match_message_count")
    if verify_x or seed_x or message_x:
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        twin = None
        if verify_x:
            step_series(ax, verify_x, verify_y, where="post", label="reloc_verification_count", linewidth=1.8, color="tab:blue")
            reference_line(
                ax,
                VERIFICATION_SAMPLES_DEFAULT,
                color="tab:red",
                linestyle="--",
                linewidth=1.2,
                label=f"default verification_samples = {VERIFICATION_SAMPLES_DEFAULT}",
            )
            ax.set_ylabel("consecutive accepted verifications")
        else:
            ax.set_ylabel("reloc_verification_count not recorded")
        if seed_x or message_x:
            twin = ax.twinx()
            if seed_x:
                step_series(twin, seed_x, seed_y, where="post", label="seed_request_count", linewidth=1.3, linestyle="--", color="tab:green")
            if message_x:
                step_series(twin, message_x, message_y, where="post", label="match_message_count", linewidth=1.3, linestyle=":", color="tab:orange")
            twin.set_ylabel("cumulative message counters")
        note_omitted("recovery_verification", tuple(
            key for key, data in (
                ("reloc_verification_count", verify_x),
                ("seed_request_count", seed_x),
                ("match_message_count", message_x),
            ) if not data
        ))
        finish(fig, ax, "recovery_verification", "Relocalization verification progress", verify_keys, twin=twin)
    else:
        skip("recovery_verification", verify_keys)

    # --- Plant / AMCL surrogate motion -------------------------------------
    motion_keys = ("odom_yaw", "amcl_yaw", "amcl_covariance")
    odom_yaw_x, odom_yaw_y = numeric("odom_yaw", RAD_TO_DEG)
    amcl_yaw_x, amcl_yaw_y = numeric("amcl_yaw", RAD_TO_DEG)
    covariance_x, covariance_y = numeric("amcl_covariance")
    def draw_yaw(ax: Any) -> None:
        if odom_yaw_x:
            plot_series(ax, odom_yaw_x, odom_yaw_y, label="odom_yaw (deg)", linewidth=1.8, color="tab:blue")
        if amcl_yaw_x:
            plot_series(ax, amcl_yaw_x, amcl_yaw_y, label="amcl_yaw (deg)", linewidth=1.8, linestyle="--", color="tab:green")

    def draw_covariance(ax: Any) -> None:
        plot_series(ax, covariance_x, covariance_y, label="amcl_covariance", linewidth=1.8, linestyle="-.", color="tab:orange")
        reference_line(
            ax,
            MAX_COVARIANCE_THRESHOLD,
            color="tab:red",
            linestyle="--",
            linewidth=1.2,
            label=f"max_covariance trigger threshold = {MAX_COVARIANCE_THRESHOLD}",
        )

    motion_panels: list[tuple[str, Any]] = []
    if odom_yaw_x or amcl_yaw_x:
        motion_panels.append(("yaw (deg)", draw_yaw))
    if covariance_x:
        motion_panels.append(("amcl covariance", draw_covariance))
    if motion_panels:
        note_omitted("plant_motion", tuple(
            key for key, data in (
                ("odom_yaw", odom_yaw_x),
                ("amcl_yaw", amcl_yaw_x),
                ("amcl_covariance", covariance_x),
            ) if not data
        ))
        stacked_panels("plant_motion", "Plant motion and AMCL surrogate covariance", motion_keys, motion_panels)
    else:
        skip("plant_motion", motion_keys)

    readme = [
        "Plots use only evaluator samples.csv; missing values are omitted, never fabricated.",
        "FILE_EXISTS != VALID_EVIDENCE: a PNG is written only after the figure was",
        "counted to carry at least one real plotted data series.  Dashed threshold",
        "lines are drawn from algorithm defaults and never count as recorded data,",
        "so a figure holding only threshold lines is skipped, not written.",
        "Dashed reference lines reproduce the real unmodified algorithm defaults:",
        f"  scan match score >= {SCAN_MATCH_SCORE_THRESHOLD}, inlier ratio >= {SCAN_MATCH_INLIER_RATIO_THRESHOLD}, mean distance <= {SCAN_MATCH_MEAN_DISTANCE_THRESHOLD} m",
        f"  verification_samples = {VERIFICATION_SAMPLES_DEFAULT}, max_covariance = {MAX_COVARIANCE_THRESHOLD}",
        "See SKIPPED_PLOTS.txt for any figure that had no recorded source data.",
    ]
    (plots_dir / "README.txt").write_text("\n".join(readme) + "\n", encoding="utf-8")
    if skipped:
        lines = ["Figures not drawn because zero real data series could be plotted.", ""]
        lines += [f"{name}: {reason}" for name, reason in sorted(skipped.items())]
        (plots_dir / "SKIPPED_PLOTS.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    # ``warning`` stays None here on purpose.  scenario_runner appends it into
    # the run's warnings list, and a scenario that simply does not record the
    # optional relocalization/scan-match columns is not a defective run.  A
    # routine skip is reported through ``skipped``/``plots``; ``warning`` is
    # reserved for a genuine failure (absent samples.csv, missing matplotlib).
    return {
        "files": files,
        "warning": None,
        "skipped": skipped,
        "partial": partial,
        "plots": report,
    }
