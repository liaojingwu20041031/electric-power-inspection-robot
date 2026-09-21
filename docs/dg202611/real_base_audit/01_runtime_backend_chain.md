# 01 — Real Base Runtime Backend Chain

```
AUDIT_CLASS          READ_ONLY_STATIC_AUDIT
REAL_HARDWARE_ACCESSED   NO
MOTOR_COMMAND_SENT       NO
CMD_VEL_PUBLISHED        NO
REAL_BASE_CONTROLLER_MODIFIED  NO
```

## 1. Backend selection

`src/ylhb_base/launch/bringup.launch.py` offers two mutually exclusive chassis
backends, chosen by the `base_backend` launch argument:

```python
# line 296-297
base_backend_arg = DeclareLaunchArgument(
    'base_backend', default_value='zlac',
...
# line 353-354
use_zlac  = IfCondition(PythonExpression(["'", base_backend, "' == 'zlac'"]))
use_stm32 = IfCondition(PythonExpression(["'", base_backend, "' == 'stm32'"]))
```

```
CURRENT_REAL_BASE_BACKEND       zlac8015d_canopen_controller
BACKEND_RESOLUTION_CONFIDENCE   HIGH
```

Evidence for HIGH confidence: the default value is the literal string `'zlac'`,
the condition is a direct string comparison, and `bringup.launch.py` is the only
file in the repository that launches either executable. Verified by grep across
`src/` for `base_controller` and `zlac8015d_canopen_controller`: the only other
hits are `ylhb_mobile_bridge` status reporting and unit tests, neither of which
launches a controller.

The `stm32` backend (`base_controller`) is a fallback, reachable only by passing
`base_backend:=stm32` explicitly.

## 2. Chain for the active backend (zlac)

```
CONFIG_SOURCE                          LOAD_PATH                         NODE                            RUNTIME_USE
config/base_kinematics.yaml            parameters=[base_kinematics_path, node name:                      line 480 builds ONE
  top key: zlac8015d_canopen_controller              zlac_config_path,  zlac8015d_canopen_controller     DifferentialDriveKinematics
config/zlac8015d.yaml                                {publish_tf:False}]  (launch line 361)               line 704 cmd -> wheel rpm
  top key: zlac8015d_canopen_controller  (launch lines 364-368)                                          line 803 wheel rpm -> odom
```

Node-name match is the load-bearing detail: ROS 2 matches a parameter file by
node name, and the YAML top-level key `zlac8015d_canopen_controller` equals the
node's `name=` in the launch. So the file is genuinely consumed.

```
BASE_KINEMATICS_YAML_ACTUALLY_CONSUMED   YES  (for the zlac backend)
```

Override order check: `parameters=` is evaluated left to right, so
`zlac8015d.yaml` could shadow `base_kinematics.yaml`. It does not —
`grep -c 'wheel_track\|wheel_radius' config/zlac8015d.yaml` returns **0**. That
file carries CAN, rate and protocol settings only.

## 3. Chain for the fallback backend (stm32)

```
CONFIG_SOURCE   LOAD_PATH                                   NODE             RUNTIME_USE
(none)          parameters=[{serial_port}, {publish_tf,      base_controller  line 169-170 cmd -> wheel m/s
                cmd_vel_topic, scan_topic,                   (launch line 375) line 221 wheel -> odom yaw
                require_fresh_scan, scan_timeout_sec,
                cmd_timeout_sec}]   (launch lines 378-388)
```

`base_kinematics_path` is **absent** from this node's parameter list. There is no
YAML in the chain at all, so every kinematic value falls back to the C++ default.

Two independent reasons the YAML would not help even if added:

1. it is not passed
2. its top-level key is `zlac8015d_canopen_controller`, while this node is named
   `base_controller`, so ROS 2 would not apply it

Both must be fixed together for a config-level repair to take effect. Fixing only
the launch line would look correct and change nothing.
