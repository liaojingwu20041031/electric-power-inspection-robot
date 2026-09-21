"""FILE_EXISTS != VALID_EVIDENCE for the synthetic-validation plot generator.

A PNG only counts as evidence if it actually carries a recorded data series.
These tests therefore always assert on both halves of the claim:

  * the returned structure -- what generate_plots reported to scenario_runner
    and, through it, to result.json and the evidence manifest, and
  * the filesystem -- what a reviewer would actually find in plots/.

A test that only checks "the PNG exists" is exactly the hole being guarded
against here, so no test below is allowed to stop at that.
"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from dg_synthetic_validation.plot_results import (
    DATA_SERIES_GID,
    PLOT_LABEL,
    REFERENCE_LINE_GID,
    count_data_series,
    generate_plots,
)


# Figures whose source columns are optional; a scenario that records none of
# them must skip them rather than file blank frames.
OPTIONAL_PLOTS = (
    "lidar_quality.png",
    "fusion_confidence.png",
    "scan_map_quality.png",
    "recovery_verification.png",
    "plant_motion.png",
    "relocalization_state.png",
)

GNSS_COLUMNS = ("gnss_hdop", "gnss_satellites", "gnss_differential_age")


def write_samples(run_dir: Path, columns: list[str], rows: list[list[str]]) -> Path:
    """Write a throwaway samples.csv; never touches anything under results/."""
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / "samples.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        writer.writerows(rows)
    return path


def names(result: dict) -> set[str]:
    return {Path(item).name for item in result["files"]}


def on_disk(run_dir: Path) -> set[str]:
    return {path.name for path in (run_dir / "plots").glob("*.png")}


@pytest.fixture
def valid_run(tmp_path: Path) -> Path:
    """A run whose GNSS and state columns carry real recorded values."""
    run_dir = tmp_path / "S99_known_valid"
    write_samples(
        run_dir,
        ["elapsed_time", "gnss_hdop", "gnss_satellites", "gnss_differential_age",
         "gnss_state", "lidar_state", "navigation_state", "relocalization_state"],
        [
            ["0.0", "0.85", "18", "1.0", "GOOD", "GOOD", "NOMINAL", "NORMAL"],
            ["0.5", "0.91", "17", "1.2", "GOOD", "GOOD", "NOMINAL", "NORMAL"],
            ["1.0", "1.40", "12", "2.4", "DEGRADED", "GOOD", "DEGRADED", "SUSPECTED"],
            ["1.5", "1.10", "15", "1.8", "GOOD", "GOOD", "NOMINAL", "RECOVERED"],
        ],
    )
    return run_dir


# --------------------------------------------------------------------------
# 1. KNOWN_VALID_SERIES
# --------------------------------------------------------------------------

def test_known_valid_series_is_generated_and_listed(valid_run: Path) -> None:
    result = generate_plots(valid_run)

    expected = valid_run / "plots" / "gnss_quality.png"
    assert "gnss_quality.png" in names(result), result["files"]
    assert expected.exists()
    assert expected.stat().st_size > 0
    assert str(expected) in result["files"]
    assert "gnss_quality.png" not in result["skipped"]

    # The per-plot report must agree with both the list and the disk.
    entry = result["plots"]["gnss_quality.png"]
    assert entry["generated"] is True
    assert entry["series_plotted"] == 3
    assert entry["reason"] is None


def test_state_machine_figures_are_generated_from_real_states(valid_run: Path) -> None:
    """The step-plot figures are built outside save() and need the same gate."""
    result = generate_plots(valid_run)
    for name in ("state_timeline.png", "navigation_health.png"):
        assert name in names(result), (name, result["files"])
        assert (valid_run / "plots" / name).exists()
        assert result["plots"][name]["series_plotted"] >= 1


def test_reported_files_match_the_filesystem_exactly(valid_run: Path) -> None:
    """No phantom entries, and no unreported PNG left lying in plots/."""
    result = generate_plots(valid_run)
    assert names(result) == on_disk(valid_run)


def test_every_written_figure_carries_the_mandatory_marker(
    valid_run: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The banner has to be on the figures actually written, not just declared.

    savefig is intercepted so the assertion reads the title text off each
    figure at the moment it is committed to disk.
    """
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure

    # Both halves the evidence manifest scans for must be in the label, which
    # keeps this test independent of the separator character.
    assert "SYNTHETIC SOFTWARE VALIDATION" in PLOT_LABEL
    assert "NOT REAL ROBOT DATA" in PLOT_LABEL

    seen: dict[str, str] = {}
    original = Figure.savefig

    def spy(self, fname, *args, **kwargs):  # type: ignore[no-untyped-def]
        texts = [text.get_text() for text in self.texts]
        texts += [axes.get_title() for axes in self.axes]
        seen[Path(str(fname)).name] = "\n".join(texts)
        return original(self, fname, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", spy)
    result = generate_plots(valid_run)

    assert seen, "no figure was written at all"
    assert set(seen) == names(result), "savefig calls must match the reported files"
    for name, text in sorted(seen.items()):
        assert PLOT_LABEL in text, f"{name} lost the mandatory marker: {text!r}"


# --------------------------------------------------------------------------
# 2. NO_SERIES -- the optional columns are entirely absent
# --------------------------------------------------------------------------

def test_absent_columns_are_not_listed_and_not_written(tmp_path: Path) -> None:
    run_dir = tmp_path / "S99_no_series"
    write_samples(
        run_dir,
        ["elapsed_time", "phase", "gnss_hdop"],
        [["0.0", "warmup", "0.9"], ["0.5", "run", "1.0"], ["1.0", "run", "1.1"]],
    )

    result = generate_plots(run_dir)

    for name in OPTIONAL_PLOTS:
        assert name not in names(result), f"{name} was reported as generated"
        assert not (run_dir / "plots" / name).exists(), f"{name} was written to disk"
        assert name in result["skipped"], f"{name} was not recorded as skipped"
        assert result["plots"][name]["generated"] is False
        assert result["plots"][name]["series_plotted"] == 0

    # The skip reason has to name the specific columns, not just say "no data".
    reason = result["skipped"]["plant_motion.png"]
    assert "absent" in reason
    for column in ("odom_yaw", "amcl_yaw", "amcl_covariance"):
        assert column in reason, reason

    # A figure whose only content would be a dashed threshold line drawn from
    # an algorithm default must not be written: the threshold is not evidence.
    assert "scan_map_quality.png" not in on_disk(run_dir)

    # The one figure with real data still comes through.
    assert "gnss_quality.png" in names(result)
    assert (run_dir / "plots" / "gnss_quality.png").exists()


def test_no_series_at_all_writes_no_png_whatsoever(tmp_path: Path) -> None:
    run_dir = tmp_path / "S99_time_only"
    write_samples(run_dir, ["elapsed_time", "phase"],
                  [["0.0", "warmup"], ["0.5", "run"], ["1.0", "run"]])

    result = generate_plots(run_dir)

    assert result["files"] == []
    assert on_disk(run_dir) == set()
    assert result["skipped"], "a run with no plottable column must explain itself"
    assert not any(entry["generated"] for entry in result["plots"].values())
    assert (run_dir / "plots" / "SKIPPED_PLOTS.txt").exists()


# --------------------------------------------------------------------------
# 3. ALL_REQUIRED_COLUMNS_EMPTY -- columns present, every cell blank
# --------------------------------------------------------------------------

def test_all_columns_empty_is_skipped_not_written(tmp_path: Path) -> None:
    run_dir = tmp_path / "S99_all_empty"
    write_samples(
        run_dir,
        ["elapsed_time", *GNSS_COLUMNS, "navigation_state"],
        [
            ["0.0", "", "", "", ""],
            ["0.5", "", "", "", ""],
            ["1.0", "", "", "", ""],
        ],
    )

    result = generate_plots(run_dir)

    for name in ("gnss_quality.png", "navigation_health.png", "state_timeline.png"):
        assert name not in names(result), f"{name} was reported despite empty columns"
        assert not (run_dir / "plots" / name).exists(), f"{name} was written blank"
        assert name in result["skipped"]

    reason = result["skipped"]["gnss_quality.png"]
    assert "every cell empty" in reason, reason
    for column in GNSS_COLUMNS:
        assert column in reason, reason

    assert result["files"] == []
    assert on_disk(run_dir) == set()


def test_skipped_figures_never_reach_savefig(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The file must never be written, not written and then cleaned up.

    Intercepting savefig proves the decision happens before the write, so a
    crash midway through cannot leave a blank PNG behind for the manifest to
    pick up.
    """
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure

    run_dir = tmp_path / "S99_never_saved"
    write_samples(
        run_dir,
        ["elapsed_time", *GNSS_COLUMNS],
        [["0.0", "", "", ""], ["0.5", "", "", ""], ["1.0", "", "", ""]],
    )

    calls: list[str] = []

    def spy(self, fname, *args, **kwargs):  # type: ignore[no-untyped-def]
        calls.append(Path(str(fname)).name)
        raise AssertionError(f"savefig was called for a figure with no data: {fname}")

    monkeypatch.setattr(Figure, "savefig", spy)
    result = generate_plots(run_dir)

    assert calls == []
    assert result["files"] == []
    assert on_disk(run_dir) == set()


def test_values_present_but_no_usable_timestamp_is_skipped(tmp_path: Path) -> None:
    """The blank-chart case that a column-only check misses.

    Every value column is populated, so a "does the column hold a number?"
    test passes -- but elapsed_time is blank throughout, so not a single point
    is placeable and both the curve figure and the step figure would render
    empty.  Nothing may be written or reported.
    """
    run_dir = tmp_path / "S99_no_timestamps"
    write_samples(
        run_dir,
        ["elapsed_time", "gnss_hdop", "gnss_state", "navigation_state", "relocalization_state"],
        [
            ["", "0.85", "GOOD", "NOMINAL", "NORMAL"],
            ["", "0.91", "GOOD", "NOMINAL", "NORMAL"],
            ["", "1.40", "DEGRADED", "DEGRADED", "SUSPECTED"],
        ],
    )

    result = generate_plots(run_dir)

    assert result["files"] == []
    assert on_disk(run_dir) == set()
    for name in ("gnss_quality.png", "state_timeline.png", "navigation_health.png"):
        assert name in result["skipped"], f"{name} was not explained"
        assert result["plots"][name]["series_plotted"] == 0

    reason = result["skipped"]["state_timeline.png"]
    assert "elapsed_time" in reason, reason


# --------------------------------------------------------------------------
# 4. A routine skip must never inflate the run's warning count
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "columns, rows",
    [
        # optional columns entirely absent
        (["elapsed_time", "gnss_hdop"], [["0.0", "0.9"], ["0.5", "1.0"], ["1.0", "1.1"]]),
        # optional columns present but every cell empty
        (["elapsed_time", *GNSS_COLUMNS],
         [["0.0", "", "", ""], ["0.5", "", "", ""], ["1.0", "", "", ""]]),
        # values recorded but no usable timestamp
        (["elapsed_time", "gnss_hdop"], [["", "0.9"], ["", "1.0"], ["", "1.1"]]),
    ],
    ids=["absent", "empty", "unplottable"],
)
def test_routine_skips_leave_warning_none(tmp_path: Path, columns: list[str], rows: list[list[str]]) -> None:
    """scenario_runner appends result["warning"] into the run's warnings list.

    A scenario that simply did not record the optional columns is not a
    defective run, so a routine skip must stay out of "warning" entirely.
    """
    run_dir = tmp_path / "S99_warning_free"
    write_samples(run_dir, columns, rows)

    result = generate_plots(run_dir)

    assert result["warning"] is None, result["warning"]
    assert result["skipped"], "the skip still has to be reported somewhere"
    # scenario_runner's exact condition for polluting the warnings list.
    assert not result.get("warning")


def test_genuine_failure_still_warns(tmp_path: Path) -> None:
    """"warning" is reserved for a real problem, and must still fire for one."""
    run_dir = tmp_path / "S99_missing_csv"
    run_dir.mkdir()

    result = generate_plots(run_dir)

    assert result["warning"] == "NOT_ENOUGH_DATA_FOR_PLOTS"
    assert result["files"] == []
    assert (run_dir / "plots" / "NOT_ENOUGH_DATA_FOR_PLOTS").exists()


# --------------------------------------------------------------------------
# The counting mechanism itself
# --------------------------------------------------------------------------

def test_count_data_series_ignores_threshold_lines_and_nan_series() -> None:
    """The gate counts recorded series only, on primary and twin axes alike."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    reference = ax.axhline(0.45)
    reference.set_gid(REFERENCE_LINE_GID)
    assert count_data_series(fig) == 0, "a threshold line is not recorded data"

    unplottable, = ax.plot([None, None], [1.0, 2.0])
    unplottable.set_gid(DATA_SERIES_GID)
    assert count_data_series(fig) == 0, "a series with no placeable point is not data"

    real, = ax.plot([0.0, 1.0], [1.0, 2.0])
    real.set_gid(DATA_SERIES_GID)
    assert count_data_series(fig) == 1

    twin = ax.twinx()
    secondary, = twin.step([0.0, 1.0], [3.0, 4.0], where="post")
    secondary.set_gid(DATA_SERIES_GID)
    assert count_data_series(fig) == 2, "a twin-axis series counts too"

    untagged, = ax.plot([0.0, 1.0], [5.0, 6.0])
    assert untagged.get_gid() is None
    assert count_data_series(fig) == 2, "only tagged series may be counted"

    plt.close(fig)
