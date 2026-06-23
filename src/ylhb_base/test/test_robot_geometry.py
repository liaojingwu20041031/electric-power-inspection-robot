import xml.etree.ElementTree as ET
import ast
from pathlib import Path

import yaml


URDF_PATH = Path(__file__).resolve().parents[1] / "urdf" / "ylhb.urdf.xacro"
NAV2_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
EXPECTED_CHAMFERED_FOOTPRINT = [
    [0.104, 0.235],
    [0.144, 0.195],
    [0.144, -0.195],
    [0.104, -0.235],
    [-0.281, -0.235],
    [-0.321, -0.195],
    [-0.321, 0.195],
    [-0.281, 0.235],
]


def parse_xyz(text):
    return tuple(float(value) for value in text.split())


def find_joint(root, name):
    return root.find(f"./joint[@name='{name}']")


def rounded_points(points):
    return {tuple(round(value, 4) for value in point) for point in points}


def body_box_xy_bounds(root, element_name):
    base_joint = find_joint(root, "footprint_to_base_joint")
    body_element = root.find(f"./link[@name='base_link']/{element_name}")

    base_offset = parse_xyz(base_joint.find("origin").attrib["xyz"])
    body_offset = parse_xyz(body_element.find("origin").attrib["xyz"])
    box_size = parse_xyz(body_element.find("geometry").find("box").attrib["size"])

    center_x = base_offset[0] + body_offset[0]
    center_y = base_offset[1] + body_offset[1]
    half_x = box_size[0] / 2.0
    half_y = box_size[1] / 2.0

    return (
        round(center_x - half_x, 4),
        round(center_x + half_x, 4),
        round(center_y - half_y, 4),
        round(center_y + half_y, 4),
    )


def costmap_footprint(config, name):
    params = config[name][name]["ros__parameters"]
    return ast.literal_eval(params["footprint"])


def test_laser_x_axis_points_forward():
    root = ET.parse(URDF_PATH).getroot()
    laser_joint = find_joint(root, "base_to_laser_joint")

    assert laser_joint is not None
    assert laser_joint.find("origin").attrib["rpy"] == "0 0 0"


def test_body_geometry_center_relative_to_wheel_center():
    root = ET.parse(URDF_PATH).getroot()
    base_joint = find_joint(root, "footprint_to_base_joint")
    body_visual = root.find("./link[@name='base_link']/visual")

    base_offset = parse_xyz(base_joint.find("origin").attrib["xyz"])
    visual_offset = parse_xyz(body_visual.find("origin").attrib["xyz"])
    geometry_center = tuple(a + b for a, b in zip(base_offset, visual_offset))

    assert geometry_center == (-0.0885, 0.0, 0.15825)


def test_costmap_footprints_use_matching_chamfered_body_outline():
    root = ET.parse(URDF_PATH).getroot()
    visual_bounds = body_box_xy_bounds(root, "visual")
    collision_bounds = body_box_xy_bounds(root, "collision")

    config = yaml.safe_load(NAV2_CONFIG_PATH.read_text(encoding="utf-8"))
    local_footprint = costmap_footprint(config, "local_costmap")
    global_footprint = costmap_footprint(config, "global_costmap")

    assert visual_bounds == collision_bounds
    assert local_footprint == global_footprint
    assert local_footprint == EXPECTED_CHAMFERED_FOOTPRINT
    assert len(local_footprint) == 8

    min_x, max_x, min_y, max_y = visual_bounds
    assert all(
        min_x <= x <= max_x and min_y <= y <= max_y
        for x, y in local_footprint
    )
    assert min(x for x, _ in local_footprint) == min_x
    assert max(x for x, _ in local_footprint) == max_x
    assert min(y for _, y in local_footprint) == min_y
    assert max(y for _, y in local_footprint) == max_y
    assert rounded_points(local_footprint) == rounded_points(
        (x, -y) for x, y in local_footprint
    )
