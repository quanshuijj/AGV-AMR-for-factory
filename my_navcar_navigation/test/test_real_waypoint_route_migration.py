#!/usr/bin/env python3
import importlib.util
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parents[1]
NAVCAR_SERVER_DIR = PKG_DIR.parents[0] / "navcar_project_server"
WAYPOINT_ROUTE_MANAGER_SCRIPT = PKG_DIR / "scripts" / "waypoint_route_manager.py"


def import_route_manager_module():
    spec = importlib.util.spec_from_file_location("waypoint_route_manager", WAYPOINT_ROUTE_MANAGER_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RealWaypointRouteMigrationTest(unittest.TestCase):
    def test_runtime_waypoint_stack_is_present(self):
        expected = [
            PKG_DIR / "scripts" / "waypoint_io.py",
            PKG_DIR / "scripts" / "waypoint_marker_publisher.py",
            PKG_DIR / "scripts" / "waypoint_route_manager.py",
            PKG_DIR / "launch" / "ad_waypoints.launch",
            PKG_DIR / "maps" / "ADmap" / "AD_waypoints_left.yaml",
            PKG_DIR / "maps" / "ADmap" / "AD_waypoints_right.yaml",
        ]
        for path in expected:
            self.assertTrue(path.exists(), str(path))

    def test_mynav_wires_waypoint_router_without_sim_speed_overrides(self):
        root = ET.parse(PKG_DIR / "launch" / "mynav.launch").getroot()
        nodes = [node for node in root.findall("node") if node.attrib.get("name") == "waypoint_route_manager"]
        self.assertEqual(len(nodes), 1)
        params = {param.attrib["name"]: param.attrib.get("value") for param in nodes[0].findall("param")}
        self.assertEqual(params["goal_topic"], "/waypoint_route_goal")
        self.assertEqual(params["route_window_size"], "4")
        self.assertEqual(params["route_lateral_tolerance"], "1.5")
        launch_text = (PKG_DIR / "launch" / "mynav.launch").read_text(encoding="utf-8")
        self.assertNotIn('TebLocalPlannerROS/max_vel_x" value="4.0', launch_text)
        self.assertNotIn('TebLocalPlannerROS/max_vel_theta" value="1.2', launch_text)
        self.assertNotIn('SBPLLatticePlanner/nominalvel_mpersecs" value="3.6', launch_text)

    def test_move_base_loads_common_costmap_params_in_private_namespace(self):
        for launch_file in ("mynav.launch", "sim_planning.launch"):
            root = ET.parse(PKG_DIR / "launch" / launch_file).getroot()
            move_base_nodes = [
                node for node in root.findall("node") if node.attrib.get("name") == "move_base_node"
            ]
            self.assertEqual(len(move_base_nodes), 1, launch_file)
            rosparams = [
                (rp.attrib.get("file"), rp.attrib.get("ns"))
                for rp in move_base_nodes[0].findall("rosparam")
            ]
            common_refs = [
                ns for file_ref, ns in rosparams
                if file_ref and "costmap_common_params.yaml" in file_ref
            ]
            self.assertIn("global_costmap", common_refs, launch_file)
            self.assertIn("local_costmap", common_refs, launch_file)

    def test_operator_goal_inputs_use_route_goal_topic(self):
        mainwindow = (NAVCAR_SERVER_DIR / "src" / "mainwindow.cpp").read_text(encoding="utf-8")
        rviz = (PKG_DIR / "rviz" / "sim_planning.rviz").read_text(encoding="utf-8")
        self.assertIn('goalPub = nh.advertise<geometry_msgs::PoseStamped>("/waypoint_route_goal", 1);', mainwindow)
        self.assertIn("Topic: /waypoint_route_goal", rviz)
        for path in (PKG_DIR / "rviz").glob("*.rviz"):
            self.assertNotIn("Topic: /move_base_simple/goal", path.read_text(encoding="utf-8"), str(path))

    def test_ros_work_manager_tracks_action_goals_not_raw_operator_goals(self):
        source = (NAVCAR_SERVER_DIR / "src" / "rosworkmanager.cpp").read_text(encoding="utf-8")
        header = (NAVCAR_SERVER_DIR / "include" / "navcar_project_server" / "rosworkmanager.h").read_text(encoding="utf-8")
        self.assertIn('n.subscribe("/move_base/goal", 1, &ROSWorkManager::moveBaseGoalCallback, this)', source)
        self.assertNotIn('n.subscribe("/waypoint_route_goal", 1, &ROSWorkManager::simpleGoalCallback, this)', source)
        self.assertNotIn('n.subscribe("/move_base_simple/goal", 1, &ROSWorkManager::simpleGoalCallback, this)', source)
        self.assertNotIn("simpleGoalSub", header)
        action_block = source.split("void ROSWorkManager::moveBaseGoalCallback", 1)[1].split(
            "void ROSWorkManager::simpleGoalCallback", 1
        )[0]
        self.assertIn("currentNavMode == NAV_IDLE || currentNavMode == NAV_REACHED", action_block)

    def test_lane_switch_uses_transition_before_following_target_lane(self):
        module = import_route_manager_module()
        lanes = {
            "right": {
                "waypoints": [
                    {"id": "r1", "position": {"x": 0.0, "y": 0.0}},
                    {"id": "r2", "position": {"x": 10.0, "y": 0.0}},
                ]
            },
            "left": {
                "waypoints": [
                    {"id": "l1", "position": {"x": 10.0, "y": 5.0}},
                    {"id": "l2", "position": {"x": 0.0, "y": 5.0}},
                ]
            },
        }
        transition = module.build_lane_transition_plan("right", "left", lanes)
        self.assertEqual([wp["id"] for wp in transition], ["r2", "l1"])

    def test_stale_current_lane_does_not_pull_return_route_back_to_outbound_endpoint(self):
        module = import_route_manager_module()
        lanes = {
            "right": {
                "waypoints": [
                    {"id": "r1", "position": {"x": 0.0, "y": 0.0}},
                    {"id": "r2", "position": {"x": 10.0, "y": 0.0}},
                ]
            },
            "left": {
                "waypoints": [
                    {"id": "l1", "position": {"x": 10.0, "y": 5.0}},
                    {"id": "l2", "position": {"x": 0.0, "y": 5.0}},
                ]
            },
        }

        self.assertFalse(
            module.should_use_lane_transition(
                "right",
                "left",
                lanes,
                start_x=8.0,
                start_y=5.1,
            )
        )
        self.assertTrue(
            module.should_use_lane_transition(
                "right",
                "left",
                lanes,
                start_x=8.0,
                start_y=0.1,
            )
        )

    def test_route_progress_can_resync_to_nearest_forward_waypoint(self):
        module = import_route_manager_module()
        route = [
            {"id": "wp_001", "position": {"x": 0.0, "y": 0.0}},
            {"id": "wp_002", "position": {"x": 2.0, "y": 0.0}},
            {"id": "wp_003", "position": {"x": 4.0, "y": 0.0}},
            {"id": "wp_004", "position": {"x": 6.0, "y": 0.0}},
        ]

        self.assertEqual(module.nearest_route_index(route, 4.1, 0.1, start_index=0), 2)
        self.assertEqual(module.nearest_route_index(route, 1.9, 0.1, start_index=2), 2)

    def test_route_window_starts_from_forward_waypoint_not_first_waypoint(self):
        module = import_route_manager_module()
        self.assertEqual(module.route_window_goal_index(route_length=10, active_index=0, window_size=4), 3)
        self.assertEqual(module.route_window_goal_index(route_length=10, active_index=2, window_size=4), 5)
        self.assertEqual(module.route_window_goal_index(route_length=3, active_index=0, window_size=4), 2)

    def test_route_drift_reacquire_only_arms_after_entering_route_corridor(self):
        module = import_route_manager_module()
        self.assertFalse(
            module.route_drift_requires_reacquire(
                has_entered_route_corridor=False,
                is_reacquiring_route=False,
                current_lateral_error=20.0,
                reacquire_tolerance=3.0,
            )
        )
        self.assertTrue(
            module.route_drift_requires_reacquire(
                has_entered_route_corridor=True,
                is_reacquiring_route=False,
                current_lateral_error=20.0,
                reacquire_tolerance=3.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
