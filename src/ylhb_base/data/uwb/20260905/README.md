# 2026-09-05 UWB measured-data replay set

This directory contains copies of the measured three-anchor data selected for
robot-side replay. The preserved source directory on Windows is not modified.

Surveyed anchor coordinates, in metres:

- physical 0 / A2: `(0.0, 0.0)`
- physical 1 / A3: `(2.4, 0.0)`
- physical 2 / A4: `(1.2, 1.8)`

The software exposes six logical slots using the mapping
`A0:0,A1:1,A2:2,A3:0,A4:1,A5:2`. Mirrored slots retain the same physical source
and correlation group, so geometry and confidence calculations still use three
independent measurements.

Replay scenarios:

- `static_raw_20260905_165208.csv`: default stable-data replay.
- `move_raw_20260905_170425.csv`: measured movement sequence.
- `anchor_fault.csv`: measured partial-anchor failure sequence.

These captures do not contain an independent position ground truth or host-side
timestamps. They support parsing, quality-gate, rejection/recovery and MODEL-1
consumption checks; they do not establish absolute localization accuracy or
end-to-end latency.
