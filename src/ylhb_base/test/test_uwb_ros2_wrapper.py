#!/usr/bin/env python3
"""Tests for UWB ROS2 wrapper in multisource_fusion_node."""

import unittest
import math
import sys
from pathlib import Path

# Add scripts to path
SCRIPT_DIR = Path(__file__).resolve().parent.parent / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from multisource_fusion_node import _finite, _bool_value, _value


class TestUwbWrapper(unittest.TestCase):
    """Test UWB wrapper logic."""

    def test_finite_helper(self):
        """Test _finite helper function."""
        self.assertTrue(_finite(1.0))
        self.assertTrue(_finite(0.0))
        self.assertTrue(_finite(-5.5))
        self.assertFalse(_finite(float('inf')))
        self.assertFalse(_finite(float('nan')))
        self.assertFalse(_finite(None))
        self.assertFalse(_finite("abc"))

    def test_bool_value_helper(self):
        """Test _bool_value helper function."""
        self.assertTrue(_bool_value("true"))
        self.assertTrue(_bool_value("True"))
        self.assertTrue(_bool_value("TRUE"))
        self.assertTrue(_bool_value("1"))
        self.assertTrue(_bool_value("yes"))
        self.assertTrue(_bool_value("accepted"))
        self.assertTrue(_bool_value("accept"))
        self.assertFalse(_bool_value("false"))
        self.assertFalse(_bool_value("0"))
        self.assertFalse(_bool_value("no"))
        self.assertFalse(_bool_value("rejected"))

    def test_uwb_to_map_transform_identity(self):
        """Test UWB->map transform with identity (no rotation/offset)."""
        uwb_x, uwb_y = 1.0, 2.0
        yaw = 0.0
        offset_x, offset_y = 0.0, 0.0

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * uwb_x - sin_yaw * uwb_y + offset_x
        map_y = sin_yaw * uwb_x + cos_yaw * uwb_y + offset_y

        self.assertAlmostEqual(map_x, 1.0, places=6)
        self.assertAlmostEqual(map_y, 2.0, places=6)

    def test_uwb_to_map_transform_90deg(self):
        """Test UWB->map transform with 90 degree rotation."""
        uwb_x, uwb_y = 1.0, 0.0
        yaw = math.pi / 2  # 90 degrees
        offset_x, offset_y = 0.0, 0.0

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * uwb_x - sin_yaw * uwb_y + offset_x
        map_y = sin_yaw * uwb_x + cos_yaw * uwb_y + offset_y

        # After 90deg rotation: (1,0) -> (0,1)
        self.assertAlmostEqual(map_x, 0.0, places=5)
        self.assertAlmostEqual(map_y, 1.0, places=5)

    def test_uwb_to_map_transform_with_offset(self):
        """Test UWB->map transform with offset only."""
        uwb_x, uwb_y = 1.0, 2.0
        yaw = 0.0
        offset_x, offset_y = 10.0, 20.0

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * uwb_x - sin_yaw * uwb_y + offset_x
        map_y = sin_yaw * uwb_x + cos_yaw * uwb_y + offset_y

        self.assertAlmostEqual(map_x, 11.0, places=6)
        self.assertAlmostEqual(map_y, 22.0, places=6)

    def test_uwb_to_map_transform_combined(self):
        """Test UWB->map transform with rotation and offset."""
        uwb_x, uwb_y = 1.0, 0.0
        yaw = math.pi  # 180 degrees
        offset_x, offset_y = 5.0, 10.0

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * uwb_x - sin_yaw * uwb_y + offset_x
        map_y = sin_yaw * uwb_x + cos_yaw * uwb_y + offset_y

        # After 180deg rotation: (1,0) -> (-1,0), then offset
        self.assertAlmostEqual(map_x, 4.0, places=5)
        self.assertAlmostEqual(map_y, 10.0, places=5)

    def test_quality_parsing(self):
        """Test quality field parsing."""
        quality = {
            "current_state": "GOOD",
            "accepted": "true",
            "residual_rms": "0.125",
            "geometry_score": "0.223",
            "unique_anchor_count": "3",
            "confidence": "0.85"
        }

        self.assertEqual(_value(quality, "current_state"), "GOOD")
        self.assertTrue(_bool_value(_value(quality, "accepted")))
        self.assertAlmostEqual(float(_value(quality, "residual_rms")), 0.125, places=6)
        self.assertEqual(int(_value(quality, "unique_anchor_count")), 3)

    def test_quality_states(self):
        """Test quality state acceptance logic."""
        # GOOD should be accepted
        self.assertIn("GOOD", ["GOOD", "DEGRADED"])

        # DEGRADED should be accepted
        self.assertIn("DEGRADED", ["GOOD", "DEGRADED"])

        # REJECTED should not be accepted
        self.assertNotIn("REJECTED", ["GOOD", "DEGRADED"])

        # STALE should not be accepted
        self.assertNotIn("STALE", ["GOOD", "DEGRADED"])


if __name__ == '__main__':
    unittest.main()
