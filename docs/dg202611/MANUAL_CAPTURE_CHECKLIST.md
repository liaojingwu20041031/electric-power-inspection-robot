# DG-202611 Manual Evidence Capture Checklist

> **SYNTHETIC SOFTWARE VALIDATION**
> **NOT REAL ROBOT DATA**
> **NOT COMPETITION PERFORMANCE EVIDENCE**
>
> Everything captured under this checklist is a picture of software running against a
> deterministic synthetic injector. No physical robot is involved. No Gazebo or other
> physics simulator is involved. These images cannot support any claim about positioning
> accuracy, feature repeatability, relocalization success rate, or competition performance.

This document tells you how to capture desktop evidence **on this machine specifically**
(`ubuntu103`, Ubuntu 22.04.5, GNOME Shell 42.9). It is written from probe results taken on
the box, not from generic advice.

---

## 1. Why automatic capture does not work here

`evidence_capture.py probe` reports `capture_available: false`. That is a correct result, not
a bug, and it will keep reporting false until the session type changes. The concrete evidence:

| Observation | Measured value |
|---|---|
| Graphical session type | `wayland` (logind session 2, seat0/tty2, `Active=yes`) |
| Session `DISPLAY` | `:0` |
| `XAUTHORITY` | `/run/user/1000/.mutter-Xwaylandauth.EEX7U3` (random suffix, per session) |
| X server | `Xwayland :0 -rootless` — the only X server for this user |
| Screen geometry | 1280x800 (via `xdpyinfo`) |
| `ffmpeg -f x11grab` result | exit code **0**, PNG written, but content is **black**: mean RGB `[0.02, 0.02, 0.02]`, brightest 32x32 sub-block `11`, `blackframe` reports `pblack:99` |
| Root window obstruction | `xwininfo -root -children` shows `mutter guard window` at `1280x800+0+0` covering the root |
| `xwd -root -display :0` | fails: `BadMatch (invalid parameter attributes)`, `Major opcode 73 (X_GetImage)` |
| `xwd -id <any window>` | same `BadMatch` failure |
| GNOME D-Bus screenshot | `org.gnome.Shell.Screenshot` → `GDBus.Error:org.freedesktop.DBus.Error.AccessDenied: Screenshot is not allowed` |

The mechanism: GNOME composites the desktop on **Wayland**, via `gnome-shell`/mutter. The
rootless Xwayland root window that `x11grab` and `xwd` read is not the desktop — it is
covered by mutter's guard window and holds no composited content. `x11grab` cannot capture a
Wayland surface. This is why an exit code of 0 is not proof of a screenshot.

**`evidence_capture.py` therefore refuses to emit that black frame.** It applies two checks to
every grab — a flat-colour test and a brightness floor (`NEAR_BLACK_CEILING = 32`) — and treats
a blank frame as a failure. A frame that is black plus a little noise still fails, which is
exactly the case here (spread 11, mean 0.02). Instead of an image you get an explicit failure
and a `<name>.NOT_CAPTURED.txt` note. Do not try to work around this. A black PNG presented as
a screenshot is fabricated evidence.

Not installed, and must not be installed to work around this: `gnome-screenshot`, `scrot`,
ImageMagick (`import`/`convert`), `maim`, `flameshot`. Only `ffmpeg` and `xwd` are available,
and neither can reach the Wayland desktop.

### What still might work later, and how to check honestly

RViz2 is a Qt application. Unless it is started with the Wayland QPA backend, it runs as an
**Xwayland client**, and its window is a real X11 window inside the same `:0` server. A root
grab taken *while RViz is mapped and on screen* may therefore contain the RViz window even
though the bare desktop root is blank. This has **not** been tested — no RViz window has ever
been on screen during a probe.

To test it at evidence time, with RViz already visible:

```python
# Import through the package, not by bare module name: `evidence_capture` only
# resolves if your CWD happens to be the package source directory.
from dg_synthetic_validation.evidence_capture import attempt_capture_with_validation
result = attempt_capture_with_validation("/tmp/rviz_probe.png")
print(result["validated"], result["validation_note"])
```

It performs exactly one grab and accepts the result **only** if it passes the non-blank check;
otherwise it fails cleanly and writes no image. If `validated` is `True`, still look at the
image with your own eyes before using it — a passing check proves the frame is not blank, not
that it shows the thing you wanted. If it fails, fall back to the manual path below.

---

## 2. The manual path that does work

The D-Bus *API* is refused to remote callers, but the GNOME screenshot *UI* is still fully
available to a human sitting at the console. These keybindings were read from gsettings on this
machine (`org.gnome.shell.keybindings`), not assumed:

| Keys | gsettings key | Effect |
|---|---|---|
| `Print` | `show-screenshot-ui` | Opens the interactive screenshot UI (select area / window / whole screen, then click to save) |
| `Shift`+`Print` | `screenshot` | Whole screen, saved immediately, no prompt |
| `Alt`+`Print` | `screenshot-window` | The focused window only, saved immediately |

Note these are `Shift`+`Print` and `Alt`+`Print` on this box, not the `Ctrl`+`Shift`+`Print`
variant seen on some other distributions. `Ctrl`+`Shift`+`Alt`+`R` starts screen recording
(`show-screen-recording-ui`) — not needed for still evidence.

You must be at the **physical VMware console** of the VM (the GUI desktop session on tty2). These
keys are handled by `gnome-shell` itself; they cannot be triggered over SSH.

### Where GNOME writes the files

Verified read-only on this machine:

- `XDG_PICTURES_DIR="$HOME/Pictures"` (from `~/.config/user-dirs.dirs`)
- `/home/weiyang/Pictures` **exists**
- `/home/weiyang/Pictures/Screenshots` **does not exist yet**

GNOME 42 saves to `~/Pictures/Screenshots/` and creates that directory on the first screenshot,
so expect it to appear once you take shot 01. Do not pre-create it and do not assume it is
already there — confirm after your first capture:

```bash
ls -l ~/Pictures/Screenshots/
```

Filenames arrive as `Screenshot from YYYY-MM-DD HH-MM-SS.png`. You will rename them per the
table in section 4.

---

## 3. Rules that are not negotiable

1. **A stage that did not occur is recorded as `NOT_OBSERVED`.** Put `NOT_OBSERVED` in the
   `real_state` column, leave the image absent, and say so in `notes`.
2. **Never stage, re-enact, re-run, or recreate a stage to produce a picture of it.** If the
   software did not reach `TRIGGERED`, there is no `TRIGGERED` screenshot. Manufacturing one by
   forcing the state by hand, or by photographing a different moment and labelling it
   `TRIGGERED`, is fabricated evidence.
3. **Record the state you actually see on screen**, read from the monitor output in that same
   image. Do not fill `real_state` from what the scenario was *expected* to do.
4. **Only these ten Active Relocalization states exist:** `NORMAL`, `SUSPECTED`, `TRIGGERED`,
   `STOPPING`, `ACTIVE_SCAN`, `WAITING_CANDIDATE`, `VERIFYING`, `RECOVERED`, `FAILED`,
   `MANUAL_REQUIRED`. **`LOST` and `SEARCHING` do not exist** — never write them.
5. **Captions describe only what is literally visible**, in Chinese. No accuracy figures, no
   repeatability or success-rate percentages, no mention of a real robot or Gazebo. The manifest
   writer scans for these and will flag them.
6. **Never write into `/home/weiyang/dg202611_ws/results/`** while capturing. Stage new images in
   a fresh evidence directory.

### A timing reality you need to plan around

The scenarios are short: S01 is 8 s, S03 is 12 s, S02 and S04 are 16 s, with individual phases
lasting about 4 s each. **You cannot reliably hand-capture twelve live transitions inside a 16
second run.** Do not speed-run the keyboard and do not extend a scenario to make capturing
easier. Capture from durable artefacts instead:

- **PLOT** shots: from the generated plots in the run's results directory — static, capture at leisure.
- **MONITOR** shots: `monitor_scenario` output; scroll back in the terminal after the run, or capture the
  terminal showing the recorded timeline.
- **RVIZ** shots: RViz displaying the recorded run state.

If a transient state was never visible long enough to photograph, that shot is `NOT_OBSERVED`.
That is an acceptable and expected outcome. An incomplete honest set beats a complete staged one.

---

## 4. Per-shot checklist

For each row: foreground the window named in **Foreground**, press the key from section 2
(`Alt`+`Print` for a single window is usually cleanest), then rename the file from
`~/Pictures/Screenshots/` to the **Filename** shown and move it into your evidence directory.

Phase IDs below are the real ones from `config/S0*.yaml`.

> **Before you take any `RVIZ` shot, read this.** The banner, the Active
> Relocalization state text, the match-quality text and the map wall cubes are drawn by
> `visualization_markers_node`. That node **now has an installed executable**
> (confirmed with `ros2 pkg executables dg_synthetic_validation`), so it can be started
> directly:
>
> ```bash
> ros2 run dg_synthetic_validation visualization_markers_node
> ```
>
> Confirm the topic is actually live before capturing:
>
> ```bash
> ros2 topic hz /dg_validation_viz/markers
> ```
>
> If nothing is publishing, the `Validation Markers` display is empty and an RViz
> screenshot will contain no state text and no map. Take the shot as a `MONITOR` shot
> instead and record it as such.
>
> **The overlap is fixed, and two limits remain for you to check on screen.** The text
> markers used to overlap badly enough that the default view was unusable as evidence.
> The layout fix has landed and reached the installed config
> (`MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED`): one marker per line, unique
> position, separated regions kept outside the map box, `TopDownOrtho` at `Scale: 32`.
>
> ```text
> RVIZ_DISPLAY_FIELD_ACCEPTANCE:        NOT_YET_VERIFIED
> EVIDENCE_TEXT_LEGIBILITY_AT_1280x800: USER_JUDGEMENT_REQUIRED
> ```
>
> Field acceptance is only settled by opening RViz — a display whose fields RViz silently
> drops still parses fine — so look at the window, not at the config. Legibility is your
> call: at 1280x800 with `Scale: 32` body text is roughly 9-10 px cap height and the
> banner about 1.7x larger, which reads on screen but is small for a printed proposal
> figure. Maximise the RViz window before capturing, or raise the VM display resolution.
> The visible extent was deliberately **not** cropped to enlarge the text, because
> cropping out the text regions defeats the layout's purpose.
>
> If any text in your frame is unreadable, that frame is not evidence — do not caption it
> from memory or from what you expect the state to be.
>
> **The monitor is the authoritative source for detailed state text. RViz carries
> spatial evidence plus a key-state summary.** This division exists because an earlier
> version of this checklist told you to read the relocalization state off an RViz
> screenshot while the node drawing that text was not installed — an evidence
> instruction asking you to describe something that was not on screen. Read detailed
> state from the monitor terminal, in the same frame if possible.
>
> Also note what an RViz shot legitimately cannot show: `/dg/fusion/pose`, `/amcl_pose`
> and `/scan_match_pose` are published in the `map` frame, and there is deliberately no
> `map -> odom` transform, so no display can render them against the `odom` fixed
> frame. Two displays that tried were removed this round because they were dead: both
> topics are `PoseWithCovarianceStamped` and had been configured as
> `rviz_default_plugins/Pose`, which consumes `PoseStamped`, so they rendered nothing
> while appearing configured. The marker node redraws `/amcl_pose` and
> `/scan_match_pose` as arrows in `odom` under an explicit display-only map-origin
> assumption, which the in-view banner states. `/dg/fusion/pose` is not drawn at all.
>
> ```text
> RULE: a .rviz file parsing as valid YAML proves neither that RViz accepts a
> display's field set nor that the message types match.
> ```


| # | Filename | Foreground | Scenario / phase | Expected `source` | `real_state` to record |
|---|---|---|---|---|---|
| 1 | `01_nominal_before_failure.png` | RViz2 | S01 `NOMINAL` | `RVIZ` | whatever the monitor shows, typically `NORMAL` |
| 2 | `02_nominal_monitor_baseline.png` | monitor terminal | S01 `NOMINAL` | `MONITOR` | as displayed |
| 3 | `03_gnss_degraded_onset.png` | monitor terminal | S02 `DEGRADED` | `MONITOR` | as displayed |
| 4 | `04_gnss_rejected_fusion_switch.png` | monitor terminal | S02 `REJECTED` | `MONITOR` | as displayed |
| 5 | `05_gnss_outage_inputs_stopped.png` | monitor terminal | S02 `OUTAGE` | `MONITOR` | as displayed |
| 6 | `06_lidar_narrow_geometry.png` | RViz2 | S03 `NARROW_GEOMETRY` | `RVIZ` | as displayed |
| 7 | `07_lidar_empty_scan.png` | RViz2 | S03 `EMPTY_SCAN` | `RVIZ` | as displayed |
| 8 | `08_concurrent_both_degraded.png` | monitor terminal | S04 `BOTH_DEGRADED` | `MONITOR` | as displayed |
| 9 | `09_relocalization_triggered.png` | monitor terminal | S04 `BOTH_REJECTED` | `MONITOR` | only if a trigger is actually shown, else `NOT_OBSERVED` |
| 10 | `10_active_scan_or_verifying.png` | monitor terminal | S04 `BOTH_OUTAGE` | `MONITOR` | only the state actually shown, else `NOT_OBSERVED` |
| 11 | `11_timeline_plot_overview.png` | image viewer showing the run plot | any completed run | `PLOT` | `N/A` |
| 12 | `12_post_recovery_stable.png` | monitor terminal or RViz2 | end of a run that recovered | `MONITOR` or `RVIZ` | `RECOVERED` only if displayed, else `NOT_OBSERVED` |

Shots 9, 10 and 12 are the ones most likely to be legitimately unobtainable: these scenarios
degrade sensors and may never drive the state machine to `TRIGGERED`, `ACTIVE_SCAN` or
`RECOVERED` at all. If so, record `NOT_OBSERVED` and move on. Use `DESKTOP` as `source` only for
a whole-desktop shot, and `MANUAL` for anything captured by other means.

### Caption examples

Good — describes only what is on screen:

```
RViz 窗口显示地图与激光点云，机器人位姿箭头位于走廊中部，重定位状态栏显示 NORMAL。
监控终端文本显示 GNSS 状态为 REJECTED，卫星数为 2，融合来源一栏显示 LIDAR。
折线图显示重定位状态时间线最后停留在 RECOVERED，图标题标注为合成软件验证。
```

Forbidden — asserts performance the evidence cannot support:

```
定位精度达到 20 cm。                 <- accuracy claim
特征重复性达到 95%。                  <- repeatability claim
重定位成功率超过 95%。                 <- success-rate claim
真实机器人现场运行结果。                 <- real-robot claim
Gazebo 仿真截图。                    <- simulator claim
```

---

## 5. After capturing

1. Put every image in one evidence directory.
2. Build the manifest with `evidence_manifest.py`, one `add_entry` per shot, using the exact
   filenames above and the states you actually saw.
3. Verify:

   ```bash
   # Run it as a module of the installed package. A bare
   # `python3 -m evidence_manifest` only works from inside the package source
   # directory, and will fail everywhere else.
   source /home/weiyang/dg202611_ws/install/setup.bash
   python3 -m dg_synthetic_validation.evidence_manifest verify <evidence_dir>
   ```

   Exit code 0 means every listed file exists and no unlisted image is sitting in the directory.
   Exit code 1 lists what is missing and what is undocumented. Missing files are reported, never
   dropped — fix the manifest or capture the shot, do not delete the row to silence it.
4. Leave any `*.NOT_CAPTURED.txt` notes in place. They document why an image is absent, which is
   the point.

---

## 6. Why the automatic checker is trusted only after being caught out

`evidence_capture.py` applies two checks, not one, and the second exists because the
first was not enough.

The first implementation judged frames by flat colour and channel spread only. The
mutter guard-window black frame carries a channel spread of about **11** — enough noise
to pass a spread test — so the helper reported `capture_available=true` for a frame with
**no content**. That is a false positive on the exact frame the check exists to reject.

A brightness floor (`NEAR_BLACK_CEILING = 32`) was added and validated against inputs
known to be good and inputs known to be blank.

```text
FILE_EXISTS       != VALID_SCREENSHOT
CAPTURE_AVAILABLE != VALID_EVIDENCE
```

Practical consequence for you: a `validated: True` result means the frame is not blank.
It does not mean the frame shows what you wanted. Look at every image with your own
eyes before filing it. The same reasoning applies to plots — a figure is now written
only when a tagged data series with at least one finite point was drawn, because blank
charts were previously entering the manifest.

## 7. Host facts that change across reboots

The VM is rebooted between sessions. Disk state persists, and no long-running process
is lost because none is started before a session.

```text
XAUTHORITY: /run/user/1000/.mutter-Xwaylandauth.*
```

The suffix is **random and changes on every reboot**. The value in the table in section
1 was read during one session and is not the value present after a reboot. Glob it;
never hard-code it, and never copy it out of this document into a script.

The screenshot directory is subject to the same discipline for a different reason:
`~/Pictures/Screenshots` did not exist at audit time, and GNOME 42 creates it on the
first capture.

```text
RULE: confirm the screenshot directory AFTER your first capture. Never assume it.
```

```bash
ls -l ~/Pictures/Screenshots/
```

## 8. Run class, before you caption anything

```text
RUN_CLASS: PRECHECK          -> NOT evidence. Not citable anywhere.
RUN_CLASS: FINAL_EVIDENCE    -> requires the VISIBLE_EVIDENCE_CHECKPOINT
```

The four S05–S08 runs completed so far are **prechecks**: headless, no screenshots, no
manifest. Their `metadata.json`, `result.json` and `result.md` all carry
`RUN_CLASS=PRECHECK`, `NOT_FINAL_EVIDENCE=TRUE` and `NOT_FOR_DOCUMENT_CLAIM=TRUE`.

If you are capturing images of a precheck run for your own inspection, say so in the
`notes` column. A screenshot of a precheck does not become final evidence by being
photographed carefully.
