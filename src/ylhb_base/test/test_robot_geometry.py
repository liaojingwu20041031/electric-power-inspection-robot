import xml.etree.ElementTree as ET
from pathlib import Path


URDF_PATH = Path(__file__).resolve().parents[1] / "urdf" / "ylhb.urdf.xacro"


def parse_xyz(text):
    return tuple(float(value) for value in text.split())


def find_joint(root, name):
    return root.find(f"./joint[@name='{name}']")


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
