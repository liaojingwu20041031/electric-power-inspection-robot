"""Desktop evidence capture helper for DG-202611 synthetic validation.

ROS-free on purpose: the module is importable from plain unit tests and from
the scenario tooling without pulling in ``rclpy``, and it runs as a CLI on a
machine where the workspace has not been sourced.

Design rule 1 -- REPORT, NEVER ASSUME
-------------------------------------
Nothing here guesses at the graphical environment.  Every candidate DISPLAY,
every ``XAUTHORITY`` candidate and every external tool is probed, and the raw
observation is returned.  ``XAUTHORITY`` files are checked for *existence
only*; their contents are never read, logged or returned, because they hold
live X11 session cookies.

Design rule 2 -- NEVER FABRICATE AN IMAGE
-----------------------------------------
A missing screenshot is an acceptable outcome.  An invented one is not.  So
this module

* never draws, synthesises, or substitutes a placeholder picture;
* captures to a temporary sibling file and promotes it to the requested path
  only after the frame passes inspection, so a previously good screenshot is
  never clobbered by a failed attempt; and
* treats a BLANK frame as a FAILURE, not a success -- both a flat single-colour
  frame and a frame that is merely black plus noise.  Under a GNOME/Wayland
  session ``ffmpeg -f x11grab`` exits 0 while handing back an all-black frame,
  because the rootless Xwayland root window is covered by mutter's guard window
  and holds no composited desktop content.  Measured on this machine that frame
  has mean RGB 0.02 yet a channel spread of 11, so a pure uniformity test is not
  enough and a brightness floor is applied as well.  Passing such a frame off as
  a screenshot would be fabricating evidence, so it is rejected with an
  explanation.

External tools
--------------
Only ``ffmpeg`` and ``xwd`` are ever invoked to capture.  ``xdpyinfo``,
``xrandr``, ``loginctl`` and ``systemctl`` are used for *reporting* only, are
strictly optional, and their absence merely reduces the detail of the report.
Nothing is ever installed.  ImageMagick is not present on the target machine,
so ``ffmpeg`` is also the only available converter for the ``xwd`` byte stream.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

X11_SOCKET_DIR = Path("/tmp/.X11-unix")

# Tools this module may actually capture with.
CAPTURE_TOOLS = ("ffmpeg", "xwd")

# Optional, report-only helpers.  Never required.
REPORT_TOOLS = ("xdpyinfo", "xrandr", "loginctl", "systemctl")

METHODS = ("auto", "ffmpeg", "xwd")

# A 32x32 downscale of a real desktop always shows spread between the darkest
# and brightest sub-block.  Anything flatter than this is a blank frame.
BLANK_PROBE_SIZE = 32
BLANK_TOLERANCE = 6

# A flat-colour test alone is not enough.  A guarded Xwayland root grab comes
# back essentially black but with a few stray near-black sub-blocks, which is
# sufficient spread to sneak past a pure uniformity check while still holding no
# content whatsoever (observed on this machine: mean RGB 0.02, spread 11).
# So a frame is also blank when nothing in it is brighter than this ceiling.
# Real screenshots clear it easily: even a dark RViz view has light text,
# widgets and window decoration well above it.
NEAR_BLACK_CEILING = 32

DEFAULT_TIMEOUT = 25.0

NOT_CAPTURED_SUFFIX = ".NOT_CAPTURED.txt"

PLACEHOLDER_BANNER = (
    "SYNTHETIC SOFTWARE VALIDATION | NOT_REAL_ROBOT_DATA "
    "| NOT_COMPETITION_PERFORMANCE_EVIDENCE"
)


# --------------------------------------------------------------------------
# process helpers
# --------------------------------------------------------------------------


def _tool_path(name: str) -> str | None:
    """Absolute path of ``name``, or None when it is not installed."""
    return shutil.which(name)


def _stderr_tail(text: str, lines: int = 12) -> str:
    """Last few stderr lines, which is where ffmpeg/xwd put the real cause."""
    stripped = [line for line in (text or "").splitlines() if line.strip()]
    return "\n".join(stripped[-lines:])


def _run(
    argv: list[str],
    env: dict[str, str] | None = None,
    timeout: float = DEFAULT_TIMEOUT,
    stdin_bytes: bytes | None = None,
) -> dict[str, Any]:
    """Run ``argv`` and return rc/stdout/stderr without ever raising."""
    try:
        completed = subprocess.run(
            argv,
            env=env,
            input=stdin_bytes,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except FileNotFoundError:
        return {"rc": 127, "stdout": b"", "stderr": f"not installed: {argv[0]}"}
    except subprocess.TimeoutExpired:
        return {"rc": 124, "stdout": b"", "stderr": f"timeout after {timeout}s: {argv[0]}"}
    except OSError as exc:  # pragma: no cover - defensive
        return {"rc": 126, "stdout": b"", "stderr": f"{type(exc).__name__}: {exc}"}
    return {
        "rc": completed.returncode,
        "stdout": completed.stdout or b"",
        "stderr": (completed.stderr or b"").decode("utf-8", "replace"),
    }


# --------------------------------------------------------------------------
# environment discovery
# --------------------------------------------------------------------------


def list_x11_sockets() -> list[dict[str, Any]]:
    """Every socket in /tmp/.X11-unix, with its owner uid.

    On the DG-202611 VM this reports X0, X1, X1024 and X1025.  X1024/X1025
    belong to the gdm greeter and are not openable by the workspace user, so
    ownership is reported instead of being assumed away.
    """
    found: list[dict[str, Any]] = []
    try:
        names = sorted(p.name for p in X11_SOCKET_DIR.iterdir())
    except OSError as exc:
        return [{"error": f"cannot list {X11_SOCKET_DIR}: {exc}"}]
    for name in names:
        match = re.fullmatch(r"X(\d+)", name)
        if not match:
            continue
        socket_path = X11_SOCKET_DIR / name
        entry: dict[str, Any] = {
            "socket": str(socket_path),
            "display": f":{match.group(1)}",
            "owner_uid": None,
            "owned_by_current_user": None,
        }
        try:
            stat = socket_path.stat()
            entry["owner_uid"] = stat.st_uid
            entry["owned_by_current_user"] = stat.st_uid == os.getuid()
        except OSError as exc:
            entry["stat_error"] = str(exc)
        found.append(entry)
    return found


def xauthority_candidates() -> list[dict[str, Any]]:
    """XAUTHORITY candidates, reported by EXISTENCE ONLY.

    The files hold live X11 session cookies, so this function deliberately
    never opens them: no contents are read, returned or logged.
    """
    uid = os.getuid()
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{uid}"
    home = Path(os.path.expanduser("~"))

    candidates: list[str] = []
    env_value = os.environ.get("XAUTHORITY")
    if env_value:
        candidates.append(env_value)
    candidates.append(str(home / ".Xauthority"))
    candidates.append(f"{runtime_dir}/gdm/Xauthority")
    candidates.append(f"/run/user/{uid}/gdm/Xauthority")
    # GNOME/Wayland: mutter spawns Xwayland with a per-session random suffix,
    # so this must be globbed and can never be hard-coded.
    candidates.extend(sorted(glob.glob(f"{runtime_dir}/.mutter-Xwaylandauth.*")))
    candidates.append("/var/lib/gdm3/.config/Xauthority")

    reported: list[dict[str, Any]] = []
    seen: set[str] = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        entry: dict[str, Any] = {"path": path, "exists": os.path.exists(path)}
        if entry["exists"]:
            entry["readable"] = os.access(path, os.R_OK)
            try:
                entry["size_bytes"] = os.stat(path).st_size
            except OSError:
                entry["size_bytes"] = None
        reported.append(entry)
    return reported


def _systemd_user_environment() -> dict[str, str]:
    """Selected vars from ``systemctl --user show-environment``.

    An SSH shell reports ``XDG_SESSION_TYPE=tty``, which says nothing about the
    desktop.  The systemd user manager holds the *graphical* session's values,
    so this is where the real DISPLAY/XAUTHORITY/session type come from.
    Optional: an empty dict simply means less detail in the report.
    """
    if not _tool_path("systemctl"):
        return {}
    result = _run(["systemctl", "--user", "show-environment"], timeout=10.0)
    if result["rc"] != 0:
        return {}
    wanted = {
        "DISPLAY",
        "GNOME_SETUP_DISPLAY",
        "WAYLAND_DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "XDG_SESSION_TYPE",
        "XDG_SESSION_DESKTOP",
        "XDG_CURRENT_DESKTOP",
    }
    values: dict[str, str] = {}
    for line in result["stdout"].decode("utf-8", "replace").splitlines():
        key, _, value = line.partition("=")
        if key in wanted:
            values[key] = value
    return values


def _graphical_sessions() -> list[dict[str, str]]:
    """Seat sessions and their type, via loginctl.  Optional/report-only."""
    if not _tool_path("loginctl"):
        return []
    listing = _run(["loginctl", "list-sessions", "--no-legend"], timeout=10.0)
    if listing["rc"] != 0:
        return []
    sessions: list[dict[str, str]] = []
    for line in listing["stdout"].decode("utf-8", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        session_id = parts[0]
        detail = _run(
            [
                "loginctl", "show-session", session_id,
                "-p", "Id", "-p", "Type", "-p", "Class",
                "-p", "Active", "-p", "TTY", "-p", "Remote",
            ],
            timeout=10.0,
        )
        if detail["rc"] != 0:
            continue
        info: dict[str, str] = {}
        for entry in detail["stdout"].decode("utf-8", "replace").splitlines():
            key, _, value = entry.partition("=")
            if key:
                info[key] = value
        if info:
            sessions.append(info)
    return sessions


def _xwayland_processes() -> list[str]:
    """Xwayland command lines, to expose ``-rootless`` when it is in play."""
    lines: list[str] = []
    for proc in sorted(glob.glob("/proc/[0-9]*")):
        try:
            with open(f"{proc}/cmdline", "rb") as handle:
                raw = handle.read()
        except OSError:
            continue
        if not raw:
            continue
        argv = raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        if "Xwayland" in argv or re.search(r"(^|/)(Xorg|X)\s", argv + " "):
            lines.append(argv)
    return lines


def build_x_env(display: str | None, xauthority: str | None) -> dict[str, str]:
    """Environment for an X client, with DISPLAY/XAUTHORITY forced."""
    env = dict(os.environ)
    if display:
        env["DISPLAY"] = display
    if xauthority:
        env["XAUTHORITY"] = xauthority
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    return env


def resolve_screen_geometry(
    display: str, xauthority: str | None = None
) -> dict[str, Any]:
    """Screen geometry for ``display``.

    Tries xdpyinfo then xrandr.  Both are optional; when neither is available
    the result reports ``source='unresolved'`` and the caller omits
    ``-video_size`` so that x11grab falls back to its own detection.
    """
    env = build_x_env(display, xauthority)
    if _tool_path("xdpyinfo"):
        result = _run(["xdpyinfo"], env=env, timeout=15.0)
        if result["rc"] == 0:
            match = re.search(
                r"dimensions:\s*(\d+)x(\d+)", result["stdout"].decode("utf-8", "replace")
            )
            if match:
                return {
                    "width": int(match.group(1)),
                    "height": int(match.group(2)),
                    "video_size": f"{match.group(1)}x{match.group(2)}",
                    "source": "xdpyinfo",
                }
    if _tool_path("xrandr"):
        result = _run(["xrandr"], env=env, timeout=15.0)
        if result["rc"] == 0:
            match = re.search(
                r"current\s+(\d+)\s*x\s*(\d+)", result["stdout"].decode("utf-8", "replace")
            )
            if match:
                return {
                    "width": int(match.group(1)),
                    "height": int(match.group(2)),
                    "video_size": f"{match.group(1)}x{match.group(2)}",
                    "source": "xrandr",
                }
    return {
        "width": None,
        "height": None,
        "video_size": None,
        "source": "unresolved",
    }


def _display_reachable(display: str, xauthority: str | None) -> dict[str, Any]:
    """Can we actually open ``display``?

    Socket reachability is tested in pure Python so the check works with no
    extra tooling.  When xdpyinfo exists it is also used, because a listening
    socket does not prove the auth cookie is accepted.
    """
    info: dict[str, Any] = {"display": display}
    match = re.fullmatch(r":(\d+)(\.\d+)?", display)
    socket_path = X11_SOCKET_DIR / f"X{match.group(1)}" if match else None
    info["socket"] = str(socket_path) if socket_path else None
    info["socket_exists"] = bool(socket_path and socket_path.exists())

    if _tool_path("xdpyinfo"):
        result = _run(["xdpyinfo"], env=build_x_env(display, xauthority), timeout=15.0)
        info["xdpyinfo_rc"] = result["rc"]
        info["opened"] = result["rc"] == 0
        if result["rc"] != 0:
            info["error"] = _stderr_tail(result["stderr"], 3) or _stderr_tail(
                result["stdout"].decode("utf-8", "replace"), 3
            )
    else:
        info["xdpyinfo_rc"] = None
        info["opened"] = info["socket_exists"]
        info["error"] = None if info["socket_exists"] else "socket missing"
        info["note"] = "xdpyinfo absent: socket presence only, auth not verified"
    return info


# --------------------------------------------------------------------------
# frame inspection -- the anti-fabrication gate
# --------------------------------------------------------------------------


def inspect_frame(path: str | os.PathLike[str], tolerance: int = BLANK_TOLERANCE) -> dict[str, Any]:
    """Decide whether ``path`` holds real desktop content.

    ffmpeg downscales the image to a small RGB grid and the result is judged on
    two independent grounds.  ``blank=True`` means the file holds no desktop
    content and must NOT be presented as evidence:

    * ``flat_colour`` -- the whole frame is one colour (spread <= tolerance);
    * ``near_black``  -- nothing in the frame is brighter than
      ``NEAR_BLACK_CEILING``, so it is black plus noise.  This second test is
      what catches a guarded rootless-Xwayland root grab, which carries just
      enough stray spread to pass a uniformity check alone.
    """
    target = Path(path)
    verdict: dict[str, Any] = {
        "path": str(target),
        "exists": target.exists(),
        "bytes": target.stat().st_size if target.exists() else 0,
        "blank": None,
        "flat_colour": None,
        "near_black": None,
        "channel_spread": None,
        "mean_rgb": None,
        "max_rgb": None,
        "analyzed": False,
    }
    if not verdict["exists"] or verdict["bytes"] == 0:
        verdict["reason"] = "file missing or empty"
        return verdict
    if not _tool_path("ffmpeg"):
        verdict["reason"] = "ffmpeg absent: frame content not analysable"
        return verdict

    size = f"{BLANK_PROBE_SIZE}:{BLANK_PROBE_SIZE}"
    result = _run(
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-i", str(target),
            "-vf", f"scale={size}",
            "-frames:v", "1",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-",
        ],
        timeout=DEFAULT_TIMEOUT,
    )
    raw = result["stdout"]
    expected = BLANK_PROBE_SIZE * BLANK_PROBE_SIZE * 3
    if result["rc"] != 0 or len(raw) < expected:
        verdict["reason"] = (
            "ffmpeg could not decode the frame: " + (_stderr_tail(result["stderr"], 4) or "no output")
        )
        return verdict

    channels = []
    for offset in range(3):
        samples = raw[offset:expected:3]
        channels.append((min(samples), max(samples), sum(samples) / len(samples)))
    verdict["analyzed"] = True
    verdict["mean_rgb"] = [round(channel[2], 2) for channel in channels]
    verdict["max_rgb"] = [channel[1] for channel in channels]
    verdict["channel_spread"] = [channel[1] - channel[0] for channel in channels]
    verdict["tolerance"] = tolerance
    verdict["near_black_ceiling"] = NEAR_BLACK_CEILING

    spread = max(verdict["channel_spread"])
    brightest = max(verdict["max_rgb"])
    verdict["flat_colour"] = spread <= tolerance
    verdict["near_black"] = brightest <= NEAR_BLACK_CEILING
    verdict["blank"] = bool(verdict["flat_colour"] or verdict["near_black"])

    if verdict["flat_colour"]:
        verdict["reason"] = (
            "frame is a single flat colour (max channel spread "
            f"{spread} <= {tolerance}); it carries no desktop content"
        )
    elif verdict["near_black"]:
        verdict["reason"] = (
            f"frame is black plus noise: its brightest sub-block is {brightest}, "
            f"at or below the {NEAR_BLACK_CEILING} content floor (mean RGB "
            f"{verdict['mean_rgb']}). The small spread of {spread} is noise, not "
            "content, so this is not a usable screenshot"
        )
    return verdict


# --------------------------------------------------------------------------
# probe
# --------------------------------------------------------------------------


def probe_gui_environment(deep: bool = True) -> dict[str, Any]:
    """Report the GUI capture situation.  Assumes nothing.

    With ``deep=True`` a throwaway single frame is grabbed into a temporary
    directory and inspected, then deleted.  That trial grab is the only honest
    way to answer "does capture work here", because ``ffmpeg -f x11grab``
    returns exit code 0 on a Wayland desktop while producing a black frame.

    Returns a dict including ``capture_available`` (bool) and, when that is
    False, a human-readable ``reason``.
    """
    systemd_env = _systemd_user_environment()
    sessions = _graphical_sessions()
    sockets = list_x11_sockets()
    xauth = xauthority_candidates()

    graphical = [
        session for session in sessions
        if session.get("Type") in {"wayland", "x11"} and session.get("Remote") == "no"
    ]
    session_type = (
        systemd_env.get("XDG_SESSION_TYPE")
        or (graphical[0].get("Type") if graphical else None)
        or os.environ.get("XDG_SESSION_TYPE")
    )

    report: dict[str, Any] = {
        "banner": PLACEHOLDER_BANNER,
        "probed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "uid": os.getuid(),
        "shell_env": {
            "DISPLAY": os.environ.get("DISPLAY") or None,
            "XAUTHORITY": os.environ.get("XAUTHORITY") or None,
            "XDG_RUNTIME_DIR": os.environ.get("XDG_RUNTIME_DIR") or None,
            "XDG_SESSION_TYPE": os.environ.get("XDG_SESSION_TYPE") or None,
            "WAYLAND_DISPLAY": os.environ.get("WAYLAND_DISPLAY") or None,
        },
        "systemd_user_environment": systemd_env,
        "logind_sessions": sessions,
        "graphical_session_type": session_type,
        "is_wayland": session_type == "wayland",
        "x_servers": _xwayland_processes(),
        "x11_sockets": sockets,
        "xauthority_candidates": xauth,
        "tools": {
            name: _tool_path(name) for name in CAPTURE_TOOLS + REPORT_TOOLS
        },
    }
    report["ffmpeg_available"] = bool(report["tools"].get("ffmpeg"))
    report["xwd_available"] = bool(report["tools"].get("xwd"))

    # Candidate displays: the session manager's DISPLAY first, then every
    # socket we could plausibly open.
    candidates: list[str] = []
    for value in (systemd_env.get("DISPLAY"), os.environ.get("DISPLAY")):
        if value and value not in candidates:
            candidates.append(value)
    for entry in sockets:
        display = entry.get("display")
        if display and display not in candidates:
            candidates.append(display)
    report["candidate_displays"] = candidates

    preferred_xauth = systemd_env.get("XAUTHORITY") or next(
        (item["path"] for item in xauth if item["exists"] and item.get("readable")), None
    )
    report["preferred_xauthority"] = preferred_xauth

    checks: list[dict[str, Any]] = []
    for display in candidates:
        check = _display_reachable(display, preferred_xauth)
        if check.get("opened"):
            check["geometry"] = resolve_screen_geometry(display, preferred_xauth)
        checks.append(check)
    report["display_checks"] = checks

    openable = [check for check in checks if check.get("opened")]
    report["openable_displays"] = [check["display"] for check in openable]
    report["resolved_display"] = openable[0]["display"] if openable else None
    report["resolved_geometry"] = openable[0].get("geometry") if openable else None
    report["resolved_xauthority"] = preferred_xauth
    return _finalize_probe(report, deep=deep)


def _finalize_probe(report: dict[str, Any], deep: bool) -> dict[str, Any]:
    """Set ``capture_available``/``reason``, optionally via a trial grab."""
    report["trial_capture"] = None

    if not report["ffmpeg_available"]:
        report["capture_available"] = False
        report["reason"] = (
            "ffmpeg is not installed, and it is the only converter available on "
            "this machine (ImageMagick is absent). Cannot capture."
        )
        return report
    if not report["resolved_display"]:
        report["capture_available"] = False
        report["reason"] = (
            "no usable DISPLAY could be opened. Candidates checked: "
            + ", ".join(report["candidate_displays"] or ["<none>"])
            + ". Sockets owned by another user (for example the gdm greeter's "
            ":1024/:1025) are not openable by this account."
        )
        return report

    if not deep:
        report["capture_available"] = True
        report["reason"] = (
            "shallow probe only: a DISPLAY opened, but frame content was NOT "
            "verified. Run with deep=True to confirm the grab is not blank."
        )
        return report

    # Trial grab into a temp dir, then inspect and discard.  Calls the private
    # backends directly so this never recurses back into probe_gui_environment.
    display = report["resolved_display"]
    xauthority = report["resolved_xauthority"]
    geometry = report.get("resolved_geometry") or {}
    trial: dict[str, Any] = {"display": display, "attempts": []}
    verdict = None
    with tempfile.TemporaryDirectory(prefix="dg_capture_probe_") as tmpdir:
        target = Path(tmpdir) / "trial.png"
        for method, backend in (("ffmpeg", _capture_ffmpeg), ("xwd", _capture_xwd)):
            if method == "xwd" and not report["xwd_available"]:
                trial["attempts"].append({"method": "xwd", "ok": False, "error": "xwd absent"})
                continue
            attempt = backend(target, display, xauthority, geometry)
            attempt["method"] = method
            if attempt.get("ok"):
                verdict = inspect_frame(target)
                attempt["frame"] = verdict
            trial["attempts"].append(attempt)
            if attempt.get("ok") and verdict and verdict.get("blank") is False:
                break
            if target.exists():
                target.unlink()
            verdict = None

    report["trial_capture"] = trial
    good = [
        attempt for attempt in trial["attempts"]
        if attempt.get("ok") and (attempt.get("frame") or {}).get("blank") is False
    ]
    if good:
        report["capture_available"] = True
        report["reason"] = None
        report["working_method"] = good[0]["method"]
        return report

    blank = [
        attempt for attempt in trial["attempts"]
        if attempt.get("ok") and (attempt.get("frame") or {}).get("blank") is True
    ]
    report["capture_available"] = False
    report["working_method"] = None
    if blank:
        report["reason"] = (
            f"a grab from {display} succeeded technically but returned a BLANK "
            "frame, so there is no desktop content to capture. "
            + str((blank[0].get("frame") or {}).get("reason"))
            + (
                " This machine runs a GNOME/Wayland session: the desktop is "
                "composited by mutter on Wayland, and the rootless Xwayland "
                "root window that x11grab reads is covered by mutter's guard "
                "window. x11grab cannot capture a Wayland surface."
                if report.get("is_wayland")
                else ""
            )
            + " Refusing to emit this frame as evidence; screenshots must be "
            "taken manually from the desktop session."
        )
    else:
        errors = "; ".join(
            f"{attempt.get('method')}: {attempt.get('error') or attempt.get('stderr_tail')}"
            for attempt in trial["attempts"]
        )
        report["reason"] = f"every capture backend failed on {display}. {errors}"
    return report


# --------------------------------------------------------------------------
# capture backends
# --------------------------------------------------------------------------


def _ffmpeg_has_demuxer(name: str) -> bool:
    if not _tool_path("ffmpeg"):
        return False
    result = _run(["ffmpeg", "-hide_banner", "-demuxers"], timeout=15.0)
    if result["rc"] != 0:
        return False
    return re.search(
        rf"^\s*\S*D\S*\s+{re.escape(name)}\s",
        result["stdout"].decode("utf-8", "replace"),
        re.MULTILINE,
    ) is not None


def _capture_ffmpeg(
    target: Path,
    display: str,
    xauthority: str | None,
    geometry: dict[str, Any] | None,
) -> dict[str, Any]:
    """ffmpeg -f x11grab -video_size WxH -i DISPLAY -frames:v 1 -y out.png"""
    argv = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "x11grab"]
    video_size = (geometry or {}).get("video_size")
    if video_size:
        argv += ["-video_size", str(video_size)]
    argv += ["-i", display, "-frames:v", "1", "-y", str(target)]
    result = _run(argv, env=build_x_env(display, xauthority))
    ok = result["rc"] == 0 and target.exists() and target.stat().st_size > 0
    return {
        "ok": ok,
        "argv": argv,
        "rc": result["rc"],
        "stderr_tail": _stderr_tail(result["stderr"]),
        "error": None if ok else (_stderr_tail(result["stderr"], 4) or f"rc={result['rc']}"),
    }


def _capture_xwd(
    target: Path,
    display: str,
    xauthority: str | None,
    geometry: dict[str, Any] | None,
) -> dict[str, Any]:
    """xwd -root -display DISPLAY | ffmpeg -i - -frames:v 1 -y out.png

    ffmpeg is the only converter on this machine, so the xwd byte stream is fed
    to it on stdin.  ``-f xwd_pipe`` is used when the build advertises it;
    plain ``-f xwd`` is not a demuxer name and is rejected by ffmpeg 4.4.
    """
    if not _tool_path("xwd"):
        return {"ok": False, "argv": [], "rc": 127, "stderr_tail": "", "error": "xwd absent"}

    dump = _run(["xwd", "-root", "-display", display], env=build_x_env(display, xauthority))
    if dump["rc"] != 0 or not dump["stdout"]:
        return {
            "ok": False,
            "argv": ["xwd", "-root", "-display", display],
            "rc": dump["rc"],
            "stderr_tail": _stderr_tail(dump["stderr"]),
            "error": (
                _stderr_tail(dump["stderr"], 4)
                or f"xwd produced no data (rc={dump['rc']})"
            ),
        }

    argv = ["ffmpeg", "-hide_banner", "-loglevel", "error"]
    if _ffmpeg_has_demuxer("xwd_pipe"):
        argv += ["-f", "xwd_pipe"]
    argv += ["-i", "-", "-frames:v", "1", "-y", str(target)]
    result = _run(argv, stdin_bytes=dump["stdout"])
    ok = result["rc"] == 0 and target.exists() and target.stat().st_size > 0
    return {
        "ok": ok,
        "argv": ["xwd", "-root", "-display", display, "|"] + argv,
        "rc": result["rc"],
        "xwd_bytes": len(dump["stdout"]),
        "stderr_tail": _stderr_tail(result["stderr"]),
        "error": None if ok else (_stderr_tail(result["stderr"], 4) or f"rc={result['rc']}"),
    }


# --------------------------------------------------------------------------
# public capture API
# --------------------------------------------------------------------------


def capture_screenshot(
    output_path: str | os.PathLike[str],
    display: str | None = None,
    xauthority: str | None = None,
    method: str = "auto",
    allow_blank: bool = False,
) -> dict[str, Any]:
    """Capture one screenshot to ``output_path``.

    ``method`` is 'ffmpeg', 'xwd' or 'auto' (ffmpeg first, then xwd).

    Refuses to run and returns ``success=False`` when no usable DISPLAY is
    found.  The frame is written to a temporary sibling file and only moved to
    ``output_path`` once it is confirmed to be non-blank, so a failed attempt
    never overwrites an existing good screenshot and no invented image is ever
    produced.  Set ``allow_blank=True`` only if you deliberately want a flat
    frame kept; it defaults to False precisely so that a black Wayland grab is
    reported as a failure.
    """
    if method not in METHODS:
        return {
            "success": False,
            "method_used": None,
            "output_path": str(output_path),
            "bytes": 0,
            "stderr_tail": "",
            "reason": f"unknown method {method!r}; expected one of {', '.join(METHODS)}",
        }

    target = Path(output_path)
    result: dict[str, Any] = {
        "success": False,
        "method_used": None,
        "output_path": str(target),
        "bytes": 0,
        "stderr_tail": "",
        "reason": None,
        "attempts": [],
    }

    # Resolve DISPLAY/XAUTHORITY when not given.  Shallow probe: the trial grab
    # would be redundant because we are about to grab for real.
    probe: dict[str, Any] | None = None
    if display is None or xauthority is None:
        probe = probe_gui_environment(deep=False)
        display = display or probe.get("resolved_display")
        xauthority = xauthority or probe.get("resolved_xauthority")
        result["probe_summary"] = {
            "candidate_displays": probe.get("candidate_displays"),
            "openable_displays": probe.get("openable_displays"),
            "graphical_session_type": probe.get("graphical_session_type"),
            "is_wayland": probe.get("is_wayland"),
        }

    if not display:
        result["reason"] = (
            "refusing to capture: no usable DISPLAY was found. "
            + str((probe or {}).get("reason") or "")
        ).strip()
        return result
    result["display"] = display
    result["xauthority"] = xauthority

    if not _tool_path("ffmpeg"):
        result["reason"] = (
            "refusing to capture: ffmpeg is absent and is the only converter "
            "available on this machine."
        )
        return result

    geometry = resolve_screen_geometry(display, xauthority)
    result["geometry"] = geometry

    order = ("ffmpeg", "xwd") if method == "auto" else (method,)
    backends = {"ffmpeg": _capture_ffmpeg, "xwd": _capture_xwd}

    try:
        target.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        result["reason"] = f"cannot create output directory {target.parent}: {exc}"
        return result

    handle, tmp_name = tempfile.mkstemp(
        prefix=f".{target.stem}.partial-", suffix=target.suffix or ".png", dir=str(target.parent)
    )
    os.close(handle)
    tmp_path = Path(tmp_name)

    try:
        for name in order:
            attempt = backends[name](tmp_path, display, xauthority, geometry)
            attempt["method"] = name
            if attempt.get("ok"):
                frame = inspect_frame(tmp_path)
                attempt["frame"] = frame
                if frame.get("blank") is False or allow_blank:
                    os.replace(tmp_path, target)
                    result["success"] = True
                    result["method_used"] = name
                    result["bytes"] = target.stat().st_size
                    result["frame"] = frame
                    result["blank_frame_accepted"] = bool(
                        allow_blank and frame.get("blank")
                    )
                    result["attempts"].append(attempt)
                    return result
                attempt["rejected"] = "blank frame"
            result["attempts"].append(attempt)
            result["stderr_tail"] = attempt.get("stderr_tail") or result["stderr_tail"]
            if tmp_path.exists():
                tmp_path.write_bytes(b"")
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

    blank = next(
        (a for a in result["attempts"] if (a.get("frame") or {}).get("blank") is True),
        None,
    )
    if blank:
        result["reason"] = (
            f"capture from {display} produced a BLANK frame via "
            f"{blank['method']}: {(blank.get('frame') or {}).get('reason')}. "
            "No image was written, because emitting a flat frame would be "
            "fabricating evidence. Take this screenshot manually instead."
        )
    else:
        result["reason"] = "all capture backends failed: " + "; ".join(
            f"{a.get('method')}: {a.get('error')}" for a in result["attempts"]
        )
    return result


def not_captured_path(output_path: str | os.PathLike[str]) -> Path:
    """Sibling note path for a screenshot that could not be taken."""
    target = Path(output_path)
    return target.with_name(target.stem + NOT_CAPTURED_SUFFIX)


def capture_or_note(
    output_path: str | os.PathLike[str],
    display: str | None = None,
    xauthority: str | None = None,
    method: str = "auto",
    context: str = "",
) -> dict[str, Any]:
    """Capture, or leave a note saying why there is no image.

    A missing screenshot is then always self-documenting rather than silently
    absent: on failure a ``<name>.NOT_CAPTURED.txt`` is written next to where
    the image would have gone.  That text file is explicitly NOT evidence and
    is not an image; nothing is ever synthesised in place of the screenshot.
    """
    result = capture_screenshot(
        output_path, display=display, xauthority=xauthority, method=method
    )
    note = not_captured_path(output_path)

    if result.get("success"):
        result["note_path"] = None
        # A stale note from an earlier failed run would contradict the image.
        if note.exists():
            try:
                note.unlink()
                result["stale_note_removed"] = str(note)
            except OSError:
                result["stale_note_removed"] = None
        return result

    lines = [
        PLACEHOLDER_BANNER,
        "",
        "SCREENSHOT NOT CAPTURED",
        "=======================",
        "This file is a note, NOT evidence, and NOT an image.",
        "No picture was generated, synthesised or substituted.",
        "",
        f"intended image : {Path(output_path)}",
        f"attempted at   : {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
        f"method         : {method}",
        f"display        : {result.get('display') or '<none resolved>'}",
        f"reason         : {result.get('reason')}",
    ]
    if context:
        lines.append(f"context        : {context}")
    if result.get("stderr_tail"):
        lines += ["", "stderr tail:", result["stderr_tail"]]
    attempts = result.get("attempts") or []
    if attempts:
        lines += ["", "backend attempts:"]
        for attempt in attempts:
            lines.append(
                f"  - {attempt.get('method')}: rc={attempt.get('rc')} "
                f"error={attempt.get('error')} rejected={attempt.get('rejected')}"
            )
    lines += [
        "",
        "Next step: capture this view manually from the desktop session and",
        "store it under the same filename, then re-run the manifest writer so",
        "the entry stops being reported as missing.",
        "",
    ]
    try:
        note.parent.mkdir(parents=True, exist_ok=True)
        note.write_text("\n".join(lines), encoding="utf-8")
        result["note_path"] = str(note)
    except OSError as exc:
        result["note_path"] = None
        result["note_error"] = f"could not write note: {exc}"
    return result


# --------------------------------------------------------------------------
# deferred re-probe: call this later, while RViz is actually on screen
# --------------------------------------------------------------------------


def attempt_capture_with_validation(
    output_path: str | os.PathLike[str],
    display: str | None = None,
    xauthority: str | None = None,
    method: str = "auto",
    context: str = "",
    write_note: bool = True,
) -> dict[str, Any]:
    """One validated capture attempt, intended to be run LATER.

    Why this exists
    ---------------
    On this machine the desktop is GNOME on Wayland and a grab of the rootless
    Xwayland *root* window comes back blank (see the module docstring).  That
    closes automatic full-desktop capture, and the standing verdict is
    MANUAL_ONLY.

    It does NOT necessarily close capture of an individual X11 client.  RViz2 is
    a Qt application and, unless it is started with the Wayland QPA backend, it
    runs as an Xwayland client.  In that case its window is a real X11 window
    inside the same rootless Xwayland server, and a root grab taken *while RViz
    is mapped and visible* may legitimately contain it even though the bare
    desktop root is blank.

    So this function is a capability, not a promise.  Call it at evidence time,
    with RViz already on screen, and it will:

    * take exactly one grab,
    * accept the result ONLY if it passes the non-blank frame check (neither a
      flat colour nor black-plus-noise), and
    * otherwise return a clean, explicit failure and keep no image.

    The verdict therefore degrades safely: MANUAL_ONLY today, and if a real grab
    ever passes validation it will have been *verified* rather than assumed.

    UNTESTED ON THIS MACHINE, DELIBERATELY.  It has never been executed with an
    RViz window present, because launching RViz or running a scenario was out of
    scope for the change that introduced it.  Treat the first successful return
    as new evidence about the environment, not as confirmation of a known-good
    path, and sanity-check that first image by eye before using it.

    Returns the ``capture_screenshot`` result dict, plus ``validated`` (True only
    when an image was written AND passed the frame check) and ``note_path``.
    """
    if write_note:
        result = capture_or_note(
            output_path,
            display=display,
            xauthority=xauthority,
            method=method,
            context=context or "attempt_capture_with_validation (deferred re-probe)",
        )
    else:
        result = capture_screenshot(
            output_path, display=display, xauthority=xauthority, method=method
        )
        result["note_path"] = None

    frame = result.get("frame") or {}
    result["validated"] = bool(
        result.get("success")
        and frame.get("analyzed")
        and frame.get("blank") is False
    )
    if result["validated"]:
        result["validation_note"] = (
            "frame passed the non-blank check: it holds varying, non-black "
            f"content (channel spread {frame.get('channel_spread')}, mean RGB "
            f"{frame.get('mean_rgb')}, brightest sub-block "
            f"{frame.get('max_rgb')}). Confirm by eye that the intended window "
            "is the thing actually shown before citing this as evidence."
        )
    else:
        result["validation_note"] = (
            "NOT validated. No image is being offered as evidence. "
            + str(result.get("reason") or "capture did not pass validation")
        )
    return result


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def _json_default(value: Any) -> str:
    return str(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="evidence_capture",
        description=(
            "Probe the GUI environment and capture desktop evidence. "
            "Never fabricates an image."
        ),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    probe_cmd = sub.add_parser("probe", help="print the environment report as JSON")
    probe_cmd.add_argument(
        "--shallow",
        action="store_true",
        help="skip the throwaway trial grab (frame content will NOT be verified)",
    )

    shot_cmd = sub.add_parser("shot", help="capture one screenshot to a path")
    shot_cmd.add_argument("path", help="output image path, for example /tmp/shot.png")
    shot_cmd.add_argument("--display", default=None, help="X display, for example :0")
    shot_cmd.add_argument("--xauthority", default=None, help="XAUTHORITY file to use")
    shot_cmd.add_argument("--method", default="auto", choices=list(METHODS))
    shot_cmd.add_argument(
        "--note",
        action="store_true",
        help="on failure write a sibling <name>.NOT_CAPTURED.txt",
    )
    shot_cmd.add_argument(
        "--context", default="", help="short text recorded in the failure note"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "probe":
        report = probe_gui_environment(deep=not args.shallow)
        print(json.dumps(report, indent=2, sort_keys=False, default=_json_default))
        return 0 if report.get("capture_available") else 1

    if args.note:
        result = capture_or_note(
            args.path,
            display=args.display,
            xauthority=args.xauthority,
            method=args.method,
            context=args.context,
        )
    else:
        result = capture_screenshot(
            args.path,
            display=args.display,
            xauthority=args.xauthority,
            method=args.method,
        )
    print(json.dumps(result, indent=2, sort_keys=False, default=_json_default))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
