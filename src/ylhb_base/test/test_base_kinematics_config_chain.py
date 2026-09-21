"""Config-chain consistency tests for the two chassis backends.

These are static tests: they read the YAML and the launch source and never start
a node, open a CAN interface, publish to /cmd_vel or touch hardware.

They exist because the STM32 fallback silently ran on the C++ default
wheel_track of 0.25 m instead of the repository value 0.4008 m. Two independent
faults caused it: the parameter file was not passed to the node, and the file
keyed only on the other backend's node name, so it would not have applied even if
it had been passed. Either fault alone makes a fix look correct while changing
nothing, which is exactly the kind of defect a test should pin down.
"""

from pathlib import Path
import re

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
KINEMATICS_YAML = PACKAGE_DIR / "config" / "base_kinematics.yaml"
BRINGUP_LAUNCH = PACKAGE_DIR / "launch" / "bringup.launch.py"

ZLAC_NODE = "zlac8015d_canopen_controller"
STM32_NODE = "base_controller"


def _kinematics() -> dict:
    with KINEMATICS_YAML.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _launch_source() -> str:
    return BRINGUP_LAUNCH.read_text(encoding="utf-8")


def _node_block(source: str, variable: str) -> str:
    """Return the Node(...) call text assigned to `variable`."""
    start = source.index(f"{variable} = Node(")
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "(":
            depth += 1
        elif source[index] == ")":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated Node( call for {variable}")


def test_both_backends_have_a_parameter_block():
    """ROS 2 matches a parameter file by node name, so both keys must exist."""
    config = _kinematics()
    assert ZLAC_NODE in config, f"{ZLAC_NODE} block missing from base_kinematics.yaml"
    assert STM32_NODE in config, (
        f"{STM32_NODE} block missing. Without a top-level key matching the node "
        "name, this file does not apply to that node even when passed to it."
    )


def test_shared_wheel_track_is_identical_across_backends():
    """A divergence here means one backend drives on a different geometry."""
    config = _kinematics()
    zlac = config[ZLAC_NODE]["ros__parameters"]["wheel_track"]
    stm32 = config[STM32_NODE]["ros__parameters"]["wheel_track"]
    assert zlac == stm32, (
        f"wheel_track differs between backends: {ZLAC_NODE}={zlac} "
        f"{STM32_NODE}={stm32}. Both drive the same physical chassis."
    )


def test_stm32_wheel_track_is_not_the_cxx_default():
    """0.25 is base_controller.cpp's declared default and is not the real track."""
    stm32 = _kinematics()[STM32_NODE]["ros__parameters"]["wheel_track"]
    assert stm32 != 0.25, (
        "wheel_track is 0.25, the C++ default in base_controller.cpp. That value "
        "under-rotates the vehicle and over-reports yaw in odometry."
    )


def test_stm32_block_does_not_declare_wheel_radius():
    """base_controller consumes no radius; it sends mm/s and the firmware converts.

    Padding the block with an unused parameter would imply the node honours it.
    """
    stm32 = _kinematics()[STM32_NODE]["ros__parameters"]
    assert "wheel_radius" not in stm32, (
        "base_controller declares no wheel_radius parameter, so listing one here "
        "would be inert and misleading."
    )


def test_stm32_launch_node_loads_the_kinematics_yaml():
    """Fault 1 of the original defect: the file was never passed to the node."""
    block = _node_block(_launch_source(), "stm32_base_node")
    assert "base_kinematics_path" in block, (
        "stm32_base_node does not receive base_kinematics_path, so it falls back "
        "to the C++ default wheel_track."
    )


def test_stm32_launch_node_name_matches_the_yaml_key():
    """Fault 2: node name and YAML top-level key must agree exactly."""
    block = _node_block(_launch_source(), "stm32_base_node")
    match = re.search(r"name\s*=\s*'([^']+)'", block)
    assert match, "could not read name= from stm32_base_node"
    assert match.group(1) == STM32_NODE, (
        f"stm32_base_node name is {match.group(1)!r} but the YAML key is "
        f"{STM32_NODE!r}; the parameter file would not apply."
    )


def test_zlac_launch_node_still_loads_the_kinematics_yaml():
    """Regression guard on the active path, which was already correct."""
    block = _node_block(_launch_source(), "zlac_base_node")
    assert "base_kinematics_path" in block
    match = re.search(r"name\s*=\s*'([^']+)'", block)
    assert match and match.group(1) == ZLAC_NODE


def test_default_backend_is_still_zlac():
    """The fix must not change which backend runs by default."""
    source = _launch_source()
    match = re.search(
        r"DeclareLaunchArgument\(\s*'base_backend'\s*,\s*default_value\s*=\s*'([^']+)'",
        source,
    )
    assert match, "could not read the base_backend default"
    assert match.group(1) == "zlac", (
        f"default backend is {match.group(1)!r}; the audited active path is 'zlac'."
    )


def test_zlac_kinematics_values_unchanged():
    """Pin the audited active-path values so a later edit cannot drift them."""
    zlac = _kinematics()[ZLAC_NODE]["ros__parameters"]
    assert zlac["wheel_track"] == 0.4008
    assert zlac["wheel_radius"] == 0.0865


def test_wheel_track_is_repository_config_not_a_measurement():
    """Documentation guard.

    0.4008 appears in exactly one place with no corroborating source. This test
    does not validate the number; it records that the value is still awaiting a
    physical measurement between the drive wheels' ground contact points.
    """
    zlac = _kinematics()[ZLAC_NODE]["ros__parameters"]["wheel_track"]
    assert zlac == 0.4008, (
        "If the physical measurement changes this value, update both backend "
        "blocks together and re-run this suite."
    )
