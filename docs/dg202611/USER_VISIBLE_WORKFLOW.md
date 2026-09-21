# DG-202611 — User-Visible Workflow

How a human actually watches a DG scenario run. Every command below is real. Nothing
is invented; unverified items are marked.

## 1. Environment setup, required in every terminal

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
```

Do not rely on `.bashrc`. It sources `/opt/ros/humble/setup.bash` and two unrelated
workspaces, but it does **not** source the DG workspace.

## 2. Terminal layout

| Terminal | Role | Where it must run |
|---|---|---|
| A | monitor | VMware desktop, so the user can see it |
| B | RViz | VMware desktop, mandatory |
| C | live log tail | either |

```text
tmux is NOT installed and must NOT be installed.
```

The three terminals are three real terminals. Do not install a multiplexer to
combine them.

### Terminal A — monitor

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
ros2 run dg_synthetic_validation monitor_scenario
```

Read-only: it subscribes and nothing else. It never publishes algorithm inputs,
never commands the robot, and never changes state.

It displays elapsed time plus GNSS, LiDAR, Fusion, Navigation, Relocalization, and
`/dg/test_cmd_vel`. Scenario and phase come from the external run and are **not**
live monitor fields; they read `N/A` and must be reported as `N/A`, never inferred.
Missing values appear as `NO_DATA`.

### Terminal B — RViz

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
rviz2 -d /home/weiyang/dg202611_ws/install/dg_synthetic_validation/share/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz
```

```text
RViz belongs on the VMware desktop. The SSH session has no DISPLAY.
```

A Qt/xcb error from an SSH session is a headless-session limitation, not an RViz
configuration failure. Do not "fix" the config in response to it.

The fixed frame is `odom`, not `map` (verified:
`src/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz`, `Fixed Frame: odom`).
This is deliberate: there is no `map -> odom` transform. See
`MAP_ODOM_TF_JUSTIFICATION.md`. The map appears as a visualization-only `MarkerArray`
on `/dg_validation_viz/markers`, outside `/dg/*`.

Verified displays in the shipped config, after the repair described in
`04_debug_and_failures.md` section 13:

```text
Grid                Raw Scan        /scan
Filtered Scan       /dg/lidar/filtered_scan
Odometry            /odom
TF                  Validation Markers   /dg_validation_viz/markers
Default view        rviz_default_plugins/TopDownOrtho
```

Three displays were removed because they could never render: `/dg/fusion/pose` and
`/amcl_pose` are `PoseWithCovarianceStamped` and had been configured as
`rviz_default_plugins/Pose`, which consumes `PoseStamped`, and a third targeted the
unreachable `map` frame.

The marker node now has an installed executable, so it can be started directly:

```bash
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash
ros2 run dg_synthetic_validation visualization_markers_node
```

```text
MARKER_LAYOUT_FIX_ON_TARGET: TARGET_WORKTREE_VERIFIED
```

The text markers previously overlapped badly enough that the default view was unusable
as evidence. The layout fix has landed and reached the installed config, verified by
read-back: 6 displays, `Fixed Frame: odom`, `TopDownOrtho`, `Scale: 32`, `Y: -2.1`, one
marker per line across separated regions kept outside the map box. RViz started from the
install path therefore gets the fixed layout.

Two limits remain, and neither is a reason to skip the view:

```text
RVIZ_DISPLAY_FIELD_ACCEPTANCE:         NOT_YET_VERIFIED
EVIDENCE_TEXT_LEGIBILITY_AT_1280x800:  USER_JUDGEMENT_REQUIRED
```

Field acceptance can only be settled by opening RViz: a YAML parse cannot tell an accepted
field set from one RViz silently drops. Legibility is a judgement call for you at the
screen — body text is roughly 9-10 px cap height at `Scale: 32` on 1280x800, with the
banner about 1.7x larger. Maximise the window, or raise the VM resolution, if you want
bigger text for a printed figure. The visible extent was deliberately not cropped to
enlarge the text, because that would push the text regions out of frame.

```text
AUTHORITATIVE FOR DETAILED STATE TEXT: monitor_scenario
AUTHORITATIVE FOR SPATIAL EVIDENCE:    RViz (plus a key-state summary)
```
```

### Terminal C — live log tail

Tails the log of the active scenario run under the results root.

```text
LOG_LOCATION: each run writes its own logs/ directory inside its results directory.
S01-S04 under /home/weiyang/dg202611_ws/results/synthetic/<run>/logs/
S05-S08 precheck under
  /home/weiyang/dg202611_ws/results/synthetic/precheck/<run>/logs/
S05-S08 FINAL EVIDENCE under
  /home/weiyang/dg202611_ws/results/synthetic/final/<run>/logs/
  (CORRECTED: this block previously said EXACT_LOG_PATH was NOT_YET_VERIFIED
  "because no final-evidence run has produced a directory yet". Seven FINAL
  directories exist. S06 canonical = ..._20260829-165043.)
```

## 3. The scenario run itself

```text
PRECHECK RUNS:        DONE for S05-S08 (headless, RUN_CLASS=PRECHECK)
FINAL EVIDENCE RUNS:  DONE — 7 FINAL runs under results/synthetic/final/, all PASS
                      (CORRECTED: this line read "NOT_YET_RUN, pending the
                      VISIBLE_EVIDENCE_CHECKPOINT in section 4")
                      S06 canonical ..._20260829-165043; manual capture landed
                      for S06 canonical, S07 and S08. S05 GUI: NOT_CAPTURED,
                      which is a capture omission, not NOT_OBSERVED.
```

All eight scenario files exist and install to share, verified by read-back of
`install/dg_synthetic_validation/share/dg_synthetic_validation/config/`: `S01.yaml`
through `S08.yaml`.

```bash
cd /home/weiyang/dg202611_ws
source /opt/ros/humble/setup.bash
source /home/weiyang/dg202611_ws/install/setup.bash

ros2 run dg_synthetic_validation run_scenario S05 \
  --repo-root /home/weiyang/dg202611_ws/src/electric-power-inspection-robot \
  --results-root /home/weiyang/dg202611_ws/results/synthetic
```

The four precheck runs used this invocation shape and wrote under
`results/synthetic/precheck/`. Those runs are **not** evidence: they were headless, so
nobody was watching terminals A and B, and no screenshot exists. See
`06_results_and_evidence.md` section 1.

A final-evidence run is the same command **plus** a human at the VMware console with
the checkpoint below satisfied.

Run scenarios individually. `run_all_s01_s04` exists but individual runs are
preferred when diagnosing a specific scenario.

## 4. `VISIBLE_EVIDENCE_CHECKPOINT`

Before any S05–S08 run is treated as **final evidence**, this checkpoint must pass. A
passing precheck does not substitute for it.

```text
GUI_CAPTURE_READY: MANUAL_ONLY   (frozen)
```

The desktop session is GNOME on Wayland (seat0/tty2) with `Xwayland :0 -rootless` at
1280x800, and automatic capture does not work:

| Method | Result |
|---|---|
| `ffmpeg -f x11grab` | exits `0`, yields `pblack:99` — mutter's guard window covers the rootless root |
| `xwd -root` | `BadMatch` on `X_GetImage` |
| GNOME Shell D-Bus Screenshot | `AccessDenied` |

Checkpoint steps:

1. Terminals A and B are open on the VMware desktop and visibly updating
2. RViz text markers are legible and not overlapping. The layout fix is
   `TARGET_WORKTREE_VERIFIED`, but `RVIZ_DISPLAY_FIELD_ACCEPTANCE` is `NOT_YET_VERIFIED`
   and legibility at 1280x800 is `USER_JUDGEMENT_REQUIRED`, so confirm both with your own
   eyes on this run's window before capturing
3. `/cmd_vel` publisher count is `0` before the run starts:
   ```bash
   ros2 topic info /cmd_vel -v
   ros2 topic info /dg/test_cmd_vel -v
   ```
4. Screenshots are captured **manually** by the user from the VMware desktop, per
   `MANUAL_CAPTURE_CHECKLIST.md`
5. Every captured frame passes both the flat-colour test and the brightness floor
   before being filed. A frame that is black plus noise fails
6. The screenshot directory is confirmed **after** the first capture, never assumed
7. `/cmd_vel` publisher count is `0` again after the run ends
8. The evidence manifest is built and verified, and any `*.NOT_CAPTURED.txt` notes are
   left in place

```text
A uniform or black frame is a FAILURE. It is never filed as a screenshot,
because that would be fabricated evidence.

FILE_EXISTS       != VALID_SCREENSHOT
CAPTURE_AVAILABLE != VALID_EVIDENCE
```

The second line is not decoration. The capture helper's first implementation judged
frames by flat colour and channel spread only, and the guard-window black frame carries
a spread of about 11, so it reported `capture_available=true` for a frame with no
content. A brightness floor now backs it up.

Automatic data plots are unaffected by the Wayland limitation and are still produced —
subject to the write choke point that refuses a figure with zero data series, so a
blank plot is no longer filed either.


## 5. Safety, restated

```text
TEST_OUTPUT:     /dg/test_cmd_vel
REAL_ROBOT_CMD:  /cmd_vel
```

If any part of the synthetic or test chain is found publishing to `/cmd_vel`, stop
immediately and report it. A non-zero `/cmd_vel` publisher count at scenario start or
end is a safety failure, not a warning.

Nav2 is disabled for this phase, so after `RECOVERED` the arbiter emits zero with
`NAV_SOURCE_STALE`. On screen this looks like the robot stopping and staying stopped.
That is the correct and expected outcome. Describe it as "safe control handoff
verified in synthetic software validation", never as navigation resuming or a mission
continuing.

```text
SAFE_CONTROL_HANDOFF_EVIDENCED:  YES
NAVIGATION_RESUMED_EVIDENCED:    NO
HANDOFF_EVIDENCE_STRENGTH:       DERIVED_ASSERTION
```

The last line matters when you write a caption: `cmd_vel_source` and
`safety_cmd_vel_publishers` were empty in every sample of the FINAL runs, so no
published source field says which controller held the output. What you see on
screen plus the state transitions support the handoff claim; they do not measure
it directly.

Gazebo is not part of this phase and must not be installed or started.

## 6. Idle behaviour, and what must never be "fixed"

With no scenario running, nothing publishes `/scan`, `/dg/lidar/filtered_scan` or
`/odom`. So:

```text
RViz: no Raw Scan data, no Filtered Scan data, no Odometry data
Monitor: NO_DATA
STATUS: CORRECT AND EXPECTED IDLE BEHAVIOUR
```

This is what a correctly wired read-only monitor and a correctly wired RViz look like
when there is nothing to show.

```text
RULE: an empty idle view is never repaired by fabricating a publisher.
```

A publisher added to make the idle view look alive would put fabricated data on a real
topic. That is the same failure `DECISION_006` forbids for `/cmd_vel_nav`, for the same
reason: the observation would then be of the fabrication, not of the system.

Other idle observations that are expected rather than broken:

| Observation | Why it is correct |
|---|---|
| Scenario and phase read `N/A` in the monitor | they come from the external run, not from live monitor fields |
| the TF tree shows two edges and no `map` frame | `DECISION_005`; a complete-looking tree would be the warning sign |
| `/cmd_vel` has 0 publishers | the required safety state, at idle and at every other time |

## 7. Environment note across reboots

The VM was rebooted between sessions. All disk state persisted, and no long-running
process was lost because none had been started.

```text
XAUTHORITY: /run/user/1000/.mutter-Xwaylandauth.*
```

The suffix is **random and changes across reboots**, so any command that needs it must
glob rather than hard-code. This was observed directly: the suffix recorded during the
earlier capture probe is not the one present after the reboot.
