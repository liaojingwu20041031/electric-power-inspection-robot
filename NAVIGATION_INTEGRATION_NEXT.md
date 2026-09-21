# Navigation Integration Next

This file records follow-up integration work for the DG-202611 navigation and
localization line.  The current active-relocalization supervisor is a
side-channel POC and is not part of the default bringup.

## Next integration tasks

1. Verify the runtime topic contracts for `/scan`, `/odom`, `/amcl_pose`,
   `/dg/lidar/quality`, `/dg/gnss/quality`, and
   `/dg/relocalization/match_quality` on the target ROS2 image.
2. Replay recorded or synthetic data through the LiDAR and GNSS quality
   adapters and confirm that health freshness and diagnostic values are
   interpreted consistently by the supervisor.
3. Validate the existing Scan-to-Map candidate and `/initialpose` handoff in a
   ROS2 launch test; do not replace the existing matcher in this step.
4. Decide how `/cmd_vel` ownership will be arbitrated before enabling active
   sensing beside Nav2 or another motion publisher.
5. Add a ROS2 end-to-end replay test covering trigger, safe stop, bounded
   sensing, candidate verification, retry, timeout, and manual takeover.
6. Run the supervisor with real rosbag data and then on the robot, recording
   failure cases before any competition metric claim is made.

No default bringup or navigation input is changed by the current POC.
