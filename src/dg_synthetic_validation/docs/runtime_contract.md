# DG-202611 ROS2 runtime contract

This contract is derived from the existing ROS2 nodes at the second-round
runtime-integration commit (`096bf66377e0dbaa3ed812782222ea95a2439c7a`). The
synthetic package publishes only the left-hand/input side. The right-hand
outputs below must be produced by the real nodes under test.

| Component | Input topics | Output topic | Message | Key output fields used by evaluator |
| --- | --- | --- | --- | --- |
| GNSS quality gate | `/gps/fix`, `/gps/rtk_status` | `/dg/gnss/quality`, `/dg/gnss/accepted_fix` | `NavSatFix`, `DiagnosticArray` | `current_state`, `decision`, `accepted`, `satellites`, `hdop`, `differential_age` |
| LiDAR robust features | `/scan`, `/odom` | `/dg/lidar/filtered_scan`, `/dg/lidar/stable_features`, `/dg/lidar/quality` | `LaserScan`, `PoseArray`, `DiagnosticArray` | `raw_point_count`, `valid_point_count`, `stable_point_count`, `dynamic_candidate_count`, `temporal_match_ratio`, `angular_coverage`, `geometry_score` |
| Scan-to-map | `/map`, `/dg/lidar/filtered_scan`, `/initialpose`, `/dg/relocalization/seed` | `/scan_match_pose`, `/dg/relocalization/match_quality` | `PoseStamped`, `DiagnosticArray`, `PoseWithCovarianceStamped` | `accepted`, `score`, `inlier_ratio`, `mean_distance`, `reason` |
| Active relocalization | `/dg/lidar/filtered_scan`, `/odom`, `/amcl_pose`, health diagnostics, manual takeover | `/cmd_vel_recovery`, `/dg/relocalization/status`, `/dg/relocalization/seed` | `Twist`, `DiagnosticArray`, `PoseWithCovarianceStamped` | `state`, `trigger_reason`, `request_candidate`, `failure_reason` |
| Multi-source fusion | `/odom`, accepted GNSS, GNSS/LiDAR diagnostics, `/amcl_pose`, `/scan_match_pose`, UWB pose/quality | `/dg/fusion/odom`, `/dg/fusion/pose`, `/dg/fusion/status` | `Odometry`, `PoseWithCovarianceStamped`, `DiagnosticArray` | `fusion_mode`, `accepted_source`, `measurement_confidence`, uncertainty fields |
| Navigation health | LiDAR/GNSS/UWB/relocalization diagnostics, `/amcl_pose`, `/odom`, `/scan` | `/dg/navigation/status` | `DiagnosticArray` | `overall_state`, source states, `post_recovery_elapsed`, `post_recovery_complete`, `reasons` |
| cmd_vel arbiter | `/cmd_vel_nav`, `/cmd_vel_recovery`, navigation status, manual takeover | `/dg/test_cmd_vel` in this test | `Twist` | numeric command only; source is intentionally blank because `Twist` has no source metadata |

## Relevant thresholds

- GNSS: good requires at least 8 satellites and HDOP no more than 1.5;
  degraded allows at least 4 satellites and HDOP no more than 3.0; stale and
  hard quality reasons are rejected.
- LiDAR: the existing node computes `geometry_score` from stable points and
  angular coverage. Navigation health treats a score below `0.2` as degraded.
- Fusion emits the explicit status field
  `uncertainty_model=POC_HEURISTIC_NOT_CALIBRATED_COVARIANCE`; this validation
  records it but does not turn it into a calibrated accuracy claim.
- The integration launch is always run with Nav2 disabled and
  `cmd_vel_output_topic:=/dg/test_cmd_vel`. A non-zero publisher count on the
  real `/cmd_vel` topic is a safety failure.

## Synthetic-input boundary

The injector publishes `/gps/fix`, `/gps/rtk_status`, `/scan`, `/odom`,
`/amcl_pose`, `/map`, and `/initialpose`. It never publishes `/dg/*`,
`/cmd_vel`, or a fabricated quality/fusion result. Missing observations remain
empty in `samples.csv` rather than being inferred.
