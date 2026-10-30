#!/usr/bin/env python3
import importlib.util
import sys
import unittest
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PKG_DIR / "scripts" / "shift_waypoints_origin.py"


def import_shift_module():
    sys.path.insert(0, str(PKG_DIR / "scripts"))
    spec = importlib.util.spec_from_file_location("shift_waypoints_origin", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WaypointOriginShiftTest(unittest.TestCase):
    def test_shift_waypoint_positions_by_origin_delta_without_mutating_input(self):
        module = import_shift_module()
        data = {
            "waypoints": [
                {"id": "wp_001", "position": {"x": 10.0, "y": 20.0, "z": 0.0}},
                {"id": "wp_002", "position": {"x": -1.0, "y": 2.5, "z": 0.0}},
            ]
        }

        shifted = module.shift_waypoint_data(data, -4.135013, -2.523704)

        self.assertAlmostEqual(shifted["waypoints"][0]["position"]["x"], 5.864987)
        self.assertAlmostEqual(shifted["waypoints"][0]["position"]["y"], 17.476296)
        self.assertAlmostEqual(shifted["waypoints"][1]["position"]["x"], -5.135013)
        self.assertAlmostEqual(shifted["waypoints"][1]["position"]["y"], -0.023704)
        self.assertEqual(data["waypoints"][0]["position"]["x"], 10.0)


if __name__ == "__main__":
    unittest.main()
