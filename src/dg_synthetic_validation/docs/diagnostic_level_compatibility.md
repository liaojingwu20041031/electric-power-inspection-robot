# DiagnosticStatus.level compatibility evidence

Verified in the target VM before applying the fix:

```text
Ubuntu 22.04.5 LTS
ROS_DISTRO=humble
message=diagnostic_msgs/msg/DiagnosticStatus
runtime Python type=bytes
DiagnosticStatus.OK=b'\x00'
DiagnosticStatus.WARN=b'\x01'
DiagnosticStatus.ERROR=b'\x02'
STALE convention=b'\x03'
```

The generated ROS2 Humble Python binding represents the `uint8 level` field as
a one-byte `bytes` value. The previous adapter code called `int(status.level)`;
Python cannot parse `b"\\x00"` as a decimal integer, so a real DiagnosticArray
callback raised `ValueError` and terminated the node.

The minimal compatibility strategy is
`ylhb_base/scripts/diagnostic_level.py:normalize_diagnostic_level`. It accepts
the actual `bytes`/`bytearray` representation, ordinary integer test doubles,
and a decimal string, returning the same uint8 integer. Invalid values raise
`ValueError` for strict callers; ROS callbacks pass a fail-closed default of 3
(ERROR/stale) so malformed diagnostics cannot be treated as healthy data.

Only the representation conversion changed. GNSS, LiDAR, fusion, navigation
health, relocalization, and arbiter thresholds/state rules were not changed.
