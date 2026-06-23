import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


URDF_PATH = Path(__file__).resolve().parents[1] / "urdf" / "ylhb.urdf.xacro"
NAV2_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
CYLINDER_RADIUS = 0.27675
CYLINDER_LENGTH = 0.3255
CYLINDER_ORIGIN = (-0.09025, 0.0, 0.07625)
EXPECTED_CIRCULAR_FOOTPRINT = [
    [0.18650, 0.00000],
    [0.16543, 0.10591],
    [0.10544, 0.19569],
    [0.01566, 0.25568],
    [-0.09025, 0.27675],
    [-0.19616, 0.25568],
    [-0.28594, 0.19569],
    [-0.34593, 0.10591],
    [-0.36700, 0.00000],
    [-0.34593, -0.10591],
    [-0.28594, -0.19569],
    [-0.19616, -0.25568],
    [-0.09025, -0.27675],
    [0.01566, -0.25568],
    [0.10544, -0.19569],
    [0.16543, -0.10591],
]


def parse_xyz(text):
    return tuple(float(value) for value in text.split())


def find_joint(root, name):
    return root.find(f"./joint[@name='{name}']")


def rounded_points(points):
    return {tuple(round(value, 5) for value in point) for point in points}


def body_cylinder(root, element_name):
    body_element = root.find(f"./link[@name='base_link']/{element_name}")
    cylinder = body_element.find("geometry/cylinder")

    return (
        parse_xyz(body_element.find("origin").attrib["xyz"]),
        float(cylinder.attrib["radius"]),
        float(cylinder.attrib["length"]),
    )


def costmap_footprint(config, name):
    params = config[name][name]["ros__parameters"]
    return ast.literal_eval(params["footprint"])


def test_laser_x_axis_points_forward():
    root = ET.parse(URDF_PATH).getroot()
    laser_joint = find_joint(root, "base_to_laser_joint")

    assert laser_joint is not None
    assert laser_joint.find("origin").attrib["rpy"] == "0 0 0"


def test_gps_link_is_fixed_to_base_with_provisional_zero_offset():
    root = ET.parse(URDF_PATH).getroot()
    gps_link = root.find("./link[@name='gps_link']")
    gps_joint = find_joint(root, "base_to_gps_joint")

    assert gps_link is not None
    assert gps_joint is not None
    assert gps_joint.find("parent").attrib["link"] == "base_footprint"
    assert gps_joint.find("child").attrib["link"] == "gps_link"
    assert parse_xyz(gps_joint.find("origin").attrib["xyz"]) == (0.0, 0.0, 0.0)
    assert gps_joint.find("origin").attrib["rpy"] == "0 0 0"


def test_body_visual_and_collision_use_matching_cylinder_geometry():
    root = ET.parse(URDF_PATH).getroot()
    visual = body_cylinder(root, "visual")
    collision = body_cylinder(root, "collision")

    assert visual == collision
    assert visual == (CYLINDER_ORIGIN, CYLINDER_RADIUS, CYLINDER_LENGTH)


def test_body_geometry_center_relative_to_wheel_center():
    root = ET.parse(URDF_PATH).getroot()
    base_joint = find_joint(root, "footprint_to_base_joint")
    body_visual = root.find("./link[@name='base_link']/visual")

    base_offset = parse_xyz(base_joint.find("origin").attrib["xyz"])
    visual_offset = parse_xyz(body_visual.find("origin").attrib["xyz"])
    geometry_center = tuple(a + b for a, b in zip(base_offset, visual_offset))

    assert base_offset == (0.0, 0.0, 0.0865)
    assert geometry_center == (-0.09025, 0.0, 0.16275)


def test_costmap_footprints_use_matching_circular_body_outline():
    config = yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))
    local_footprint = costmap_footprint(config, "local_costmap")
    global_footprint = costmap_footprint(config, "global_costmap")

    assert local_footprint == global_footprint
    assert local_footprint == EXPECTED_CIRCULAR_FOOTPRINT
    assert len(local_footprint) == 16
    assert min(x for x, _ in local_footprint) == -0.367
    assert max(x for x, _ in local_footprint) == 0.1865
    assert min(y for _, y in local_footprint) == -0.27675
    assert max(y for _, y in local_footprint) == 0.27675
    assert rounded_points(local_footprint) == rounded_points(
        (x, -y) for x, y in local_footprint
    )
