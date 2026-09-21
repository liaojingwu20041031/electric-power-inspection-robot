# dg_synthetic_validation

Deterministic, software-only ROS2 Humble validation for DG-202611 scenarios
S01–S04. It is a repeatable input-injection and evidence-collection harness,
not a Gazebo world and not a robot test.

## Run

From the workspace root after sourcing ROS2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dg_synthetic_validation run_scenario S01
ros2 run dg_synthetic_validation run_all_s01_s04
```

Artifacts are written below `/home/weiyang/dg202611_ws/results/synthetic/` by
default. Override with `--results-root` or `--scenario-dir` when needed. Each
run contains `metadata.json`, `scenario.yaml`, `samples.csv`, `timeline.csv`,
`result.json`, `result.md`, `logs/`, a scoped `rosbag/`, and (when matplotlib is
available) a `plots/` directory containing static PNG curves. `summary.csv` and
`summary.md` include duration, failure/warning counts, important transitions,
and artifact paths.

The batch command runs S01, S02, S03, and S04 sequentially and writes
`summary.csv` and `summary.md`. A failing scenario does not prevent later
scenarios from running.

## Authenticity and safety labels

Every result is labelled:

- `SYNTHETIC_SOFTWARE_VALIDATION`
- `NOT_REAL_ROBOT_DATA`
- `NOT_GAZEBO_DATA`
- `NOT_COMPETITION_PERFORMANCE_EVIDENCE`

The runner starts only `ylhb_base`'s integration launch with Nav2 disabled. It
routes arbiter output to `/dg/test_cmd_vel`, checks that `/cmd_vel` has zero
publishers, and stops child process groups after each scenario. No ZED, CUDA,
TensorRT, hardware driver, or real-robot bringup is started.

`cmd_vel_source` is intentionally empty in the CSV: `geometry_msgs/Twist` has  
no source field and the evaluator does not guess ownership from a numeric       
command. See [`docs/runtime_contract.md`](docs/runtime_contract.md) for the     
topic and diagnostic contract.

## Read-only observation tools

The live monitor subscribes to diagnostics and `/dg/test_cmd_vel` only; it never
publishes:

```bash
ros2 run dg_synthetic_validation monitor_scenario
```

An RViz2 layout is installed at
`share/dg_synthetic_validation/rviz/dg_synthetic_validation.rviz` and can be
opened with `rviz2 -d <path>`. It displays `/map`, `/scan`, the filtered scan,
`/odom`, `/amcl_pose`, `/dg/fusion/pose`, and TF. Both the monitor and RViz are
optional observers and are not required for headless validation.
