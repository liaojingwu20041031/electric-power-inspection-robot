# 05 — Read-Only Verification Plan

```
CONSTRAINT  No real-vehicle motion. No motor command. No /cmd_vel publication.
            No CAN driver start. The ZLAC8015D driver is NOT to be launched:
            starting it opens can1 and can energise the drive.
```

## 1. Static checks — safe now, no node started

```bash
R=/home/weiyang/dg202611_ws/src/electric-power-inspection-robot

# backend default
grep -n -A2 "'base_backend'" $R/src/ylhb_base/launch/bringup.launch.py

# node-name / YAML-key correspondence
grep -n 'name=' $R/src/ylhb_base/launch/bringup.launch.py | grep -E 'base_controller|zlac'
head -1 $R/src/ylhb_base/config/base_kinematics.yaml

# is base_kinematics_path in each node's parameter list?
grep -n -A12 'stm32_base_node = Node' $R/src/ylhb_base/launch/bringup.launch.py
grep -n -A8  'zlac_base_node = Node'  $R/src/ylhb_base/launch/bringup.launch.py

# no kinematics override hiding in the second YAML
grep -c 'wheel_track\|wheel_radius' $R/src/ylhb_base/config/zlac8015d.yaml   # expect 0
```

Expected results, as measured during this audit: default `zlac`; YAML key
`zlac8015d_canopen_controller` matching the zlac node name only; `base_kinematics_path`
present for zlac and absent for stm32; second YAML count 0.

## 2. Parameter-load verification WITHOUT the real driver

The zlac node opens `can1` on start, so do not run it to inspect parameters.
Verify the load path with a node that has no hardware side effect instead:

```bash
# Confirm ROS 2 applies a YAML by node-name key, using any harmless node.
# This validates the MECHANISM, not the chassis.
ros2 run demo_nodes_cpp talker --ros-args --params-file /tmp/probe.yaml
ros2 param dump /talker
```

Where `/tmp/probe.yaml` carries a deliberately mismatched top-level key in one run
and a matching key in another. A mismatched key must produce no parameter. That
reproduces the exact failure mode of `base_controller` without touching the base.

## 3. If and only if MASTER later authorises a bench check

Preconditions before any chassis node is started:
- drive power physically disconnected, or wheels off the ground
- `ros2 topic info /cmd_vel -v` shows **0 publishers** before start
- no teleop, no Nav2, no mobile bridge running

Then:
```bash
ros2 param get /base_controller wheel_track      # expect the fix to show 0.4008
ros2 param dump /base_controller > /tmp/bc_params.yaml
ros2 topic info /cmd_vel -v                     # re-check publisher count
```

`ros2 param get` reads a live node and sends no motor command. It still requires
the node to be running, which for the stm32 backend means the serial port is open,
so it stays gated behind the preconditions above.

## 4. Unit / config consistency test worth adding with the fix

A test in `src/ylhb_base/test/` asserting that for every chassis node named in
`bringup.launch.py`, either the node has a matching top-level key in the parameter
file passed to it, or it declares no kinematic parameters at all. That converts the
present defect into a build-time failure.

There is precedent in the repository: `test_nav2_localization_config.py` already
asserts on controller source content, so this fits existing practice.

## 5. Measurement the owner must take by hand

`wheel_track` appears in exactly one place in the repository and has no second
corroborating source. Measure the distance between the two drive wheels' ground
contact points and record it with its source class. Until then `0.4008` remains
`REPOSITORY_CONFIG`, not `MEASURED`.
