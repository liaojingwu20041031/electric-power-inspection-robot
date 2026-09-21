"""Evidence manifest writer for DG-202611 synthetic validation.

ROS-free by design, so the manifest contract is unit-testable without rclpy and
can be regenerated from a plain shell.

Writes both ``evidence_manifest.csv`` (machine-readable) and
``evidence_manifest.md`` (human-readable, disclaimer first) into an evidence
directory, and verifies the two against what is actually on disk.

WHAT A CAPTION MAY SAY -- AND MUST NOT
--------------------------------------
A caption describes ONLY what is literally visible in the image.  It is a
description of a picture, not a result.  The following claims are FORBIDDEN in
any caption, note or field written through this module, because this evidence
class cannot support them:

* any positioning-accuracy claim (for example "20 cm", "decimetre-level",
  "centimetre-level accuracy");
* any feature-repeatability claim (for example ">= 95% repeatability");
* any relocalization success-rate claim (for example "> 95% success");
* any claim that a real robot was involved;
* any claim that Gazebo or any physics simulator was involved;
* any competition-performance or ranking claim.

The reason is structural: these artefacts come from a deterministic synthetic
injector driving software state machines.  There is no robot, no simulator and
no ground truth to measure against, so a number quoted from them would be
fabricated.  ``check_forbidden_claims`` performs a best-effort textual scan for
these patterns, but it is a safety net for accidents, not a substitute for
writing honest captions.

ASSERTION VERSUS DENIAL -- AND WHY THIS SCANNER FAILS CLOSED
------------------------------------------------------------
A disclaimer has to be able to name the very thing it denies: "NOT REAL ROBOT
DATA" and "非真实机器人数据" both contain a real-robot phrase, and both must
pass.  General negation detection is NOT used to tell the two apart, because a
"is there a 'not' somewhere nearby" rule is trivially abused -- "本次不是重点,
定位精度20厘米" would exempt a fabricated number -- and Chinese negation scope
cannot be bounded reliably by character distance.

So the rule is a closed, auditable allowlist: ``DISCLAIMER_EXEMPT_PHRASES``.
A pattern hit is dropped ONLY when it lies WHOLLY inside one of those literal
phrases.  Everything else is reported, including a negated claim phrased in
some way not on the list.  That is deliberate fail-closed behaviour: a false
alarm costs a caption rewrite, a miss ships a fabricated performance claim into
a competition document.  Because exemption is per-occurrence and span-based,
appending a disclaimer never licenses a claim made elsewhere in the same text:
"定位精度20厘米, NOT REAL ROBOT DATA" is still flagged.

STAGES THAT DID NOT HAPPEN
--------------------------
If a stage never occurred, record it as ``NOT_OBSERVED`` and leave the image
absent.  Never stage, re-enact or recreate a stage to produce a picture of it.
``verify_manifest`` reports missing files loudly rather than dropping them, so
an absent image stays visible as a gap instead of disappearing.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any, Iterable


CSV_NAME = "evidence_manifest.csv"
MD_NAME = "evidence_manifest.md"

# Exact column order, shared by the CSV and the Markdown table.
COLUMNS = [
    "filename",
    "scenario",
    "run_id",
    "timestamp",
    "elapsed",
    "phase",
    "event",
    "source",
    "real_state",
    "caption",
    "evidence_level",
    "synthetic_or_real",
    "notes",
]

VALID_SOURCES = ("RVIZ", "MONITOR", "PLOT", "MANUAL", "DESKTOP")

EVIDENCE_LEVEL = "SYNTHETIC_SOFTWARE_VALIDATION"
SYNTHETIC_OR_REAL = "SYNTHETIC"

NA = "N/A"
NOT_OBSERVED = "NOT_OBSERVED"

# Exactly the states in ylhb_base/scripts/active_relocalization_core.py.
# "LOST" and "SEARCHING" are NOT part of this state machine and must never be
# written into a manifest.
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

NON_STATES = ("LOST", "SEARCHING")

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".webp", ".bmp", ".gif")

DISCLAIMER_MARKERS = (
    "SYNTHETIC SOFTWARE VALIDATION",
    "NOT REAL ROBOT DATA",
    "NOT COMPETITION PERFORMANCE EVIDENCE",
)

# Phrases that DENY a claim instead of making one.  A pattern hit is ignored
# only when the hit lies wholly inside one of these literal phrases; see
# "ASSERTION VERSUS DENIAL" above.  Compared lower-cased, so write them lower
# case.  Keep this list short, literal and obviously a denial -- every entry is
# a hole punched in a safety net, so it must be readable as one at a glance.
DISCLAIMER_EXEMPT_PHRASES = (
    # The three mandatory English markers, verbatim.
    "synthetic software validation",
    "not real robot data",
    "not competition performance evidence",
    # The three mandatory Chinese markers, verbatim.
    "合成软件验证",
    "非真实机器人数据",
    "非竞赛性能证据",
    # Explicit denials, each carrying its own negation token.
    "not real-robot data",
    "not a real robot",
    "no real robot",
    "no physical robot",
    "not competition performance",
    "no competition performance",
    "not gazebo",
    "no gazebo",
    "without gazebo",
    "no physics simulator",
    "no physics engine",
    "非真实机器人",
    "非实车",
    "无真实机器人",
    "未使用真实机器人",
    "不涉及真实机器人",
    "无gazebo",
    "未使用gazebo",
    "不涉及gazebo",
    "无物理仿真",
    "未使用物理仿真",
    "不涉及物理仿真",
    "非竞赛性能",
    "不构成竞赛性能",
    "不代表竞赛性能",
    "不构成竞赛成绩",
)

# Word-edge anchors that survive a Latin/CJK junction.
#
# `\b` is NOT usable here.  Chinese characters are Unicode word characters, so
# there is no word boundary between a Latin letter/digit and a Chinese
# character.  A `\b`-anchored pattern therefore silently never fires inside
# Chinese prose, which is exactly the prose these captions are written in:
#
#   `\b(gazebo)\b`          misses "使用Gazebo仿真采集"
#   `\b20\s*(cm|厘米)\b`    misses "定位精度20厘米"
#   `...(cm|m)\b...(精度)`  misses "20cm精度"
#   `\b(real\s+robot)\b`    misses "在real robot上测试通过"
#
# So: CJK terms below are matched as plain substrings, with no edge assertion at
# all, and the two anchors below assert only against ASCII word characters.
# That keeps the useful half of `\b` for pure-ASCII text ("gazebos" still does
# not match, "20 cm" does not fire inside "120 cm3") while firing correctly
# where a Latin term abuts a Chinese one.  Nothing in this module may use `\b`
# next to a CJK alternative; ``test_evidence_manifest.py`` asserts that.
_EDGE_L = r"(?<![A-Za-z0-9_])"
_EDGE_R = r"(?![A-Za-z0-9_])"

# Length units, ASCII and Chinese.  Longest alternatives first; bare "m" last so
# it cannot shadow "cm"/"mm".
_UNIT = (
    r"(?:centimet(?:er|re)s?|millimet(?:er|re)s?|decimet(?:er|re)s?"
    r"|cm|mm|厘米|毫米|分米|米|m)"
)
_ACCURACY = r"(?:精度|精确度|准确度|准确率|误差|偏差|accuracy|precision|error)"

# Repeatability, written 重复性 / 重复率 / 重复度 / 复现率 / 重现率 in practice.
# The original pattern carried only 重复性, so "特征重复率95%" -- the phrasing
# actually used -- walked straight through.
_REPEATABILITY = r"(?:重复性|重复率|重复度|复现率|重现率|repeatab)"
# "rate" is optional on the ASCII side and 率 on the Chinese side: "重定位成功
# 95%" claims exactly what "success rate 95%" claims.
_SUCCESS_RATE = r"(?:成功率|成功比例|成功|success(?:\s*rate)?)"
# Rates quoted as a percentage rather than as a length: "定位准确率99%".  These
# are separate from _ACCURACY, which only fires next to a length unit.
_RATE = (
    r"(?:准确率|正确率|精确率|达标率|通过率|命中率"
    r"|accuracy\s*rate|pass\s*rate|hit\s*rate)"
)
_PERCENT = r"\d{2,3}\s*%"

# Best-effort scan for claims this evidence class cannot support.
FORBIDDEN_CLAIM_PATTERNS = (
    (rf"\d+(?:\.\d+)?\s*{_UNIT}{_EDGE_R}.{{0,24}}{_ACCURACY}", "positioning-accuracy claim"),
    (rf"{_ACCURACY}.{{0,24}}\d+(?:\.\d+)?\s*{_UNIT}{_EDGE_R}", "positioning-accuracy claim"),
    # Accuracy quoted as a percentage instead of a length: "定位精度99%".
    (rf"{_ACCURACY}.{{0,24}}{_PERCENT}", "positioning-accuracy claim"),
    (rf"{_PERCENT}.{{0,24}}{_ACCURACY}", "positioning-accuracy claim"),
    (rf"{_EDGE_L}20\s*(?:cm|厘米){_EDGE_R}", "20 cm positioning claim"),
    # "decimetre-level" / "厘米级" quote no number but claim the same thing.
    (r"(?:厘米级|毫米级|分米级|亚米级)", "positioning-accuracy claim"),
    (rf"{_EDGE_L}(?:centimet(?:er|re)|decimet(?:er|re)|millimet(?:er|re))[\s-]*level",
     "positioning-accuracy claim"),
    (rf"{_REPEATABILITY}.{{0,24}}{_PERCENT}", "feature-repeatability claim"),
    (rf"{_PERCENT}.{{0,24}}{_REPEATABILITY}", "feature-repeatability claim"),
    (rf"{_SUCCESS_RATE}.{{0,24}}{_PERCENT}", "success-rate claim"),
    (rf"{_PERCENT}.{{0,24}}{_SUCCESS_RATE}", "success-rate claim"),
    (rf"{_RATE}.{{0,24}}{_PERCENT}", "performance-rate claim"),
    (rf"{_PERCENT}.{{0,24}}{_RATE}", "performance-rate claim"),
    (r"(?:真实机器人|实物机器人|实际机器人|真机|实车)", "real-robot claim"),
    (rf"{_EDGE_L}(?:real|physical|actual)[\s-]+(?:robot|machine|vehicle|hardware)"
     rf"{_EDGE_R}", "real-robot claim"),
    (r"(?:物理仿真|物理引擎)", "simulator claim"),
    (rf"{_EDGE_L}(?:gazebo|ignition|isaac[\s-]*sim|webots|mujoco|coppeliasim)"
     rf"{_EDGE_R}", "simulator claim"),
    (r"(?:竞赛成绩|比赛成绩|参赛成绩|竞赛排名|比赛排名|竞赛名次|竞赛性能|比赛性能)",
     "competition-performance claim"),
    (rf"{_EDGE_L}(?:competition[\s-]+performance|competition[\s-]+result"
     rf"|benchmark[\s-]+result){_EDGE_R}", "competition-performance claim"),
)


def _exempt_spans(lowered: str) -> list[tuple[int, int]]:
    """Half-open ``[start, end)`` spans of every disclaimer phrase occurrence."""
    spans: list[tuple[int, int]] = []
    for phrase in DISCLAIMER_EXEMPT_PHRASES:
        start = lowered.find(phrase)
        while start != -1:
            spans.append((start, start + len(phrase)))
            start = lowered.find(phrase, start + 1)
    return spans


def check_forbidden_claims(text: str) -> list[str]:
    """Best-effort scan for claims this evidence class cannot support.

    Returns a list of human-readable problem labels; empty means nothing was
    matched.  A clean result is NOT a guarantee of honesty -- it only means the
    known bad patterns are absent.

    A hit is suppressed only when it falls WHOLLY inside a
    ``DISCLAIMER_EXEMPT_PHRASES`` occurrence, so a disclaimer may name what it
    denies without the denial covering for a claim made elsewhere in the text.
    A hit that merely overlaps the edge of a disclaimer is still reported.
    """
    if not text:
        return []
    lowered = text.lower()
    spans = _exempt_spans(lowered)
    problems: list[str] = []
    for pattern, label in FORBIDDEN_CLAIM_PATTERNS:
        if label in problems:
            continue
        for match in re.finditer(pattern, lowered, re.IGNORECASE):
            if any(start <= match.start() and match.end() <= end
                   for start, end in spans):
                continue
            problems.append(label)
            break
    return problems


@dataclass
class EvidenceEntry:
    """One row of the evidence manifest.

    ``evidence_level`` and ``synthetic_or_real`` are fixed literals and are not
    caller-tunable: every row in this manifest is synthetic software validation.

    ``real_state`` is the Active Relocalization / Navigation Health state that
    was ACTUALLY OBSERVED at capture time.  Use ``N/A`` when the image has no
    such state (a plot, or a desktop shot), or ``NOT_OBSERVED`` when the stage
    never happened.  It must never be back-filled from what a scenario was
    *expected* to do.

    ``caption`` is Chinese and describes ONLY what is literally visible.
    """

    filename: str
    scenario: str = ""
    run_id: str = ""
    timestamp: str = ""
    elapsed: str = ""
    phase: str = ""
    event: str = ""
    source: str = "MANUAL"
    real_state: str = NA
    caption: str = ""
    notes: str = ""
    evidence_level: str = field(default=EVIDENCE_LEVEL, init=False)
    synthetic_or_real: str = field(default=SYNTHETIC_OR_REAL, init=False)

    def to_row(self) -> dict[str, str]:
        """Ordered, all-string row ready for csv.DictWriter."""
        data = asdict(self)
        data["evidence_level"] = EVIDENCE_LEVEL
        data["synthetic_or_real"] = SYNTHETIC_OR_REAL
        return {column: _text(data.get(column, "")) for column in COLUMNS}

    def validate(self) -> list[str]:
        """Structural problems with this entry, as human-readable strings."""
        problems: list[str] = []
        if not self.filename or not str(self.filename).strip():
            problems.append("filename is empty")
        if self.source not in VALID_SOURCES:
            problems.append(
                f"source {self.source!r} is not one of {', '.join(VALID_SOURCES)}"
            )
        state = (self.real_state or "").strip()
        if not state:
            problems.append(
                "real_state is empty; use N/A when not applicable or "
                "NOT_OBSERVED when the stage did not occur"
            )
        elif state.upper() in NON_STATES:
            problems.append(
                f"real_state {state!r} does not exist in the Active "
                "Relocalization state machine"
            )
        elif state not in RELOCALIZATION_STATES and state not in (NA, NOT_OBSERVED):
            problems.append(
                f"real_state {state!r} is not a known state; expected one of "
                f"{', '.join(RELOCALIZATION_STATES)}, or {NA}/{NOT_OBSERVED}"
            )
        if not (self.caption or "").strip():
            problems.append("caption is empty")
        for label in check_forbidden_claims(f"{self.caption} {self.notes}"):
            problems.append(f"forbidden {label} in caption/notes")
        return problems


def _text(value: Any) -> str:
    """Everything lands in the manifest as a trimmed string."""
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value).strip()


class EvidenceManifest:
    """Collects entries and writes the CSV + Markdown pair."""

    def __init__(self, directory: str | os.PathLike[str], strict: bool = False) -> None:
        self.directory = Path(directory)
        self.strict = strict
        self.entries: list[EvidenceEntry] = []

    def add_entry(self, entry: EvidenceEntry | None = None, **kwargs: Any) -> EvidenceEntry:
        """Append an entry, either prebuilt or from keyword fields.

        Raises ValueError when the entry is structurally invalid and the
        manifest was constructed with ``strict=True``; otherwise the problems
        are recorded and surfaced by ``write_manifest``/``verify_manifest`` so
        nothing is silently accepted.
        """
        if entry is None:
            allowed = {f.name for f in fields(EvidenceEntry) if f.init}
            unknown = set(kwargs) - allowed
            if unknown:
                raise TypeError(
                    f"unknown EvidenceEntry field(s): {', '.join(sorted(unknown))}"
                )
            entry = EvidenceEntry(**kwargs)
        problems = entry.validate()
        if problems and self.strict:
            raise ValueError(
                f"invalid evidence entry for {entry.filename!r}: " + "; ".join(problems)
            )
        self.entries.append(entry)
        return entry

    def rows(self) -> list[dict[str, str]]:
        return [entry.to_row() for entry in self.entries]

    def problems(self) -> list[str]:
        """Structural problems across all entries, including duplicates."""
        found: list[str] = []
        for entry in self.entries:
            for problem in entry.validate():
                found.append(f"{entry.filename}: {problem}")
        seen: dict[str, int] = {}
        for entry in self.entries:
            seen[entry.filename] = seen.get(entry.filename, 0) + 1
        for name, count in seen.items():
            if count > 1:
                found.append(f"{name}: listed {count} times in the manifest")
        return found

    def write_manifest(self, directory: str | os.PathLike[str] | None = None) -> dict[str, Any]:
        """Write both evidence_manifest.csv and evidence_manifest.md.

        Returns a dict with the two paths, the entry count, any structural
        problems, and the ``verify_manifest`` result for the same directory, so
        a caller sees missing files immediately.
        """
        target_dir = Path(directory) if directory is not None else self.directory
        target_dir.mkdir(parents=True, exist_ok=True)
        csv_path = target_dir / CSV_NAME
        md_path = target_dir / MD_NAME

        rows = self.rows()
        with open(csv_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        md_path.write_text(_render_markdown(rows, target_dir, self.problems()), encoding="utf-8")

        return {
            "csv_path": str(csv_path),
            "md_path": str(md_path),
            "entries": len(rows),
            "problems": self.problems(),
            "verification": verify_manifest(target_dir),
        }


def add_entry(manifest: EvidenceManifest, **kwargs: Any) -> EvidenceEntry:
    """Module-level convenience wrapper around ``EvidenceManifest.add_entry``."""
    return manifest.add_entry(**kwargs)


def write_manifest(
    directory: str | os.PathLike[str],
    entries: Iterable[EvidenceEntry] | None = None,
    strict: bool = False,
) -> dict[str, Any]:
    """Write a manifest for ``directory`` from ``entries`` in one call."""
    manifest = EvidenceManifest(directory, strict=strict)
    for entry in entries or []:
        manifest.add_entry(entry)
    return manifest.write_manifest()


def _md_cell(value: str) -> str:
    """Escape a value so it cannot break out of a Markdown table cell."""
    text = _text(value).replace("|", "\\|")
    return text.replace("\r", " ").replace("\n", "<br>") or "-"


def _render_markdown(
    rows: list[dict[str, str]], directory: Path, problems: list[str]
) -> str:
    """Markdown document, disclaimer first.

    The disclaimer carries all three mandatory literal markers.
    """
    lines: list[str] = []
    lines.append("# DG-202611 Evidence Manifest")
    lines.append("")
    lines.append("> **SYNTHETIC SOFTWARE VALIDATION**")
    lines.append("> **NOT REAL ROBOT DATA**")
    lines.append("> **NOT COMPETITION PERFORMANCE EVIDENCE**")
    lines.append(">")
    lines.append(
        # Worded so the rendered disclaimer passes check_forbidden_claims
        # itself: each denial keeps its negation adjacent to the term it
        # denies, and the closing list avoids the bare phrase "competition
        # performance".  A disclaimer someone copies into a caption must not
        # trip the scanner.
        "> Every image listed below was produced by a deterministic synthetic "
        "injector driving DG-202611 software state machines. No physical robot "
        "was involved. No Gazebo or other physics simulator was involved. "
        "There is no ground truth here, so these artefacts cannot and do not "
        "support any claim about positioning accuracy, feature repeatability, "
        "relocalization success rate, or standing in a competition."
    )
    lines.append(">")
    lines.append(
        "> Captions describe only what is literally visible in each image. A "
        f"stage that did not occur is recorded as `{NOT_OBSERVED}`; it is never "
        "staged, re-enacted or recreated to produce a picture."
    )
    lines.append("")
    lines.append(f"- Evidence directory: `{directory}`")
    lines.append(f"- Entries listed: {len(rows)}")
    lines.append(f"- Evidence level: `{EVIDENCE_LEVEL}`")
    lines.append(f"- Data class: `{SYNTHETIC_OR_REAL}`")
    lines.append(f"- Generated: {time.strftime('%Y-%m-%dT%H:%M:%S%z')}")
    lines.append("")

    lines.append("## Entries")
    lines.append("")
    if rows:
        lines.append("| " + " | ".join(COLUMNS) + " |")
        lines.append("|" + "|".join(["---"] * len(COLUMNS)) + "|")
        for row in rows:
            lines.append("| " + " | ".join(_md_cell(row.get(c, "")) for c in COLUMNS) + " |")
    else:
        lines.append("_No entries recorded._")
    lines.append("")

    verification = verify_manifest(directory, rows=rows)
    lines.append("## Verification")
    lines.append("")
    lines.append(f"- Listed entries: {verification['listed']}")
    lines.append(f"- Present on disk: {len(verification['present'])}")
    lines.append(f"- **Missing from disk: {len(verification['missing'])}**")
    lines.append(
        f"- **Images on disk but absent from this manifest: "
        f"{len(verification['unlisted_images'])}**"
    )
    lines.append("")
    if verification["missing"]:
        lines.append(
            "The following manifest entries have NO file on disk. They are "
            "reported, not dropped:"
        )
        lines.append("")
        for name in verification["missing"]:
            lines.append(f"- `{name}` — MISSING")
        lines.append("")
    if verification["unlisted_images"]:
        lines.append("The following images exist on disk but are not listed above:")
        lines.append("")
        for name in verification["unlisted_images"]:
            lines.append(f"- `{name}` — UNLISTED")
        lines.append("")
    if verification["not_captured_notes"]:
        lines.append(
            "Capture-failure notes found in this directory (each explains why an "
            "image is absent):"
        )
        lines.append("")
        for name in verification["not_captured_notes"]:
            lines.append(f"- `{name}`")
        lines.append("")
    if problems:
        lines.append("## Structural problems")
        lines.append("")
        for problem in problems:
            lines.append(f"- {problem}")
        lines.append("")
    if not verification["missing"] and not verification["unlisted_images"] and not problems:
        lines.append("No missing files, no unlisted images, no structural problems.")
        lines.append("")
    return "\n".join(lines)


def _read_csv_rows(csv_path: Path) -> list[dict[str, str]]:
    with open(csv_path, "r", encoding="utf-8", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def verify_manifest(
    directory: str | os.PathLike[str], rows: list[dict[str, str]] | None = None
) -> dict[str, Any]:
    """Cross-check a manifest against the files actually on disk.

    Reports, in both directions and never silently:

    * ``missing`` -- listed in the manifest but NOT present on disk;
    * ``unlisted_images`` -- an image sits in the directory but no manifest row
      mentions it, so it would otherwise travel as undocumented evidence.

    ``rows`` may be passed to verify an in-memory manifest; otherwise
    ``evidence_manifest.csv`` is read from ``directory``.
    """
    target_dir = Path(directory)
    result: dict[str, Any] = {
        "directory": str(target_dir),
        "csv_path": str(target_dir / CSV_NAME),
        "csv_exists": (target_dir / CSV_NAME).exists(),
        "md_exists": (target_dir / MD_NAME).exists(),
        "listed": 0,
        "present": [],
        "missing": [],
        "unlisted_images": [],
        "not_captured_notes": [],
        "problems": [],
        "ok": False,
    }
    if not target_dir.is_dir():
        result["problems"].append(f"evidence directory does not exist: {target_dir}")
        return result

    if rows is None:
        if not result["csv_exists"]:
            result["problems"].append(f"{CSV_NAME} not found in {target_dir}")
            return result
        try:
            rows = _read_csv_rows(target_dir / CSV_NAME)
        except (OSError, csv.Error) as exc:
            result["problems"].append(f"cannot read {CSV_NAME}: {exc}")
            return result

    result["listed"] = len(rows)
    listed_names: set[str] = set()
    for row in rows:
        name = _text(row.get("filename", ""))
        if not name:
            result["problems"].append("a manifest row has an empty filename")
            continue
        listed_names.add(name)
        if (target_dir / name).is_file():
            result["present"].append(name)
        else:
            result["missing"].append(name)
            note = target_dir / (Path(name).stem + ".NOT_CAPTURED.txt")
            result["problems"].append(
                f"MISSING FILE: {name} is listed in the manifest but not on disk"
                + (f" (see {note.name})" if note.exists() else "")
            )

    for child in sorted(target_dir.iterdir()):
        if not child.is_file():
            continue
        if child.name.endswith(".NOT_CAPTURED.txt"):
            result["not_captured_notes"].append(child.name)
            continue
        if child.suffix.lower() in IMAGE_SUFFIXES and child.name not in listed_names:
            result["unlisted_images"].append(child.name)
            result["problems"].append(
                f"UNLISTED IMAGE: {child.name} is present on disk but has no "
                "manifest entry"
            )

    for row in rows:
        state = _text(row.get("real_state", ""))
        if state.upper() in NON_STATES:
            result["problems"].append(
                f"{row.get('filename')}: real_state {state!r} is not a real "
                "Active Relocalization state"
            )
        source = _text(row.get("source", ""))
        if source and source not in VALID_SOURCES:
            result["problems"].append(
                f"{row.get('filename')}: source {source!r} is not one of "
                f"{', '.join(VALID_SOURCES)}"
            )
        level = _text(row.get("evidence_level", ""))
        if level and level != EVIDENCE_LEVEL:
            result["problems"].append(
                f"{row.get('filename')}: evidence_level must be {EVIDENCE_LEVEL}"
            )
        for label in check_forbidden_claims(
            f"{row.get('caption', '')} {row.get('notes', '')}"
        ):
            result["problems"].append(f"{row.get('filename')}: forbidden {label}")

    if result["md_exists"]:
        try:
            markdown = (target_dir / MD_NAME).read_text(encoding="utf-8")
        except OSError as exc:
            result["problems"].append(f"cannot read {MD_NAME}: {exc}")
        else:
            absent = [m for m in DISCLAIMER_MARKERS if m not in markdown]
            if absent:
                result["problems"].append(
                    f"{MD_NAME} is missing mandatory disclaimer marker(s): "
                    + "; ".join(absent)
                )

    result["ok"] = not result["missing"] and not result["unlisted_images"] and not result["problems"]
    return result


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="evidence_manifest",
        description=(
            "Write and verify the DG-202611 evidence manifest "
            "(SYNTHETIC SOFTWARE VALIDATION only)."
        ),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    verify_cmd = sub.add_parser(
        "verify", help="cross-check an existing manifest against files on disk"
    )
    verify_cmd.add_argument("directory", help="evidence directory to verify")

    write_cmd = sub.add_parser(
        "write", help="write a manifest from a JSON list of entry dicts"
    )
    write_cmd.add_argument("directory", help="evidence directory to write into")
    write_cmd.add_argument(
        "--entries-json",
        required=True,
        help="path to a JSON file holding a list of entry objects, or '-' for stdin",
    )
    write_cmd.add_argument(
        "--strict",
        action="store_true",
        help="fail instead of recording problems when an entry is invalid",
    )

    sub.add_parser("columns", help="print the manifest column order as JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "columns":
        print(json.dumps({"columns": COLUMNS, "sources": list(VALID_SOURCES),
                          "relocalization_states": list(RELOCALIZATION_STATES)}, indent=2))
        return 0

    if args.command == "verify":
        result = verify_manifest(args.directory)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result["ok"] else 1

    raw = sys.stdin.read() if args.entries_json == "-" else Path(args.entries_json).read_text(
        encoding="utf-8"
    )
    payload = json.loads(raw)
    if not isinstance(payload, list):
        print("entries JSON must be a list of objects", file=sys.stderr)
        return 2

    manifest = EvidenceManifest(args.directory, strict=args.strict)
    try:
        for item in payload:
            manifest.add_entry(**item)
    except (TypeError, ValueError) as exc:
        print(f"invalid entry: {exc}", file=sys.stderr)
        return 2

    result = manifest.write_manifest()
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["verification"]["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
