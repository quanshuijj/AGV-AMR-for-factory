#!/usr/bin/env python3
import math
import sys
from pathlib import Path

import actionlib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from waypoint_io import load_waypoint_file


def squared_distance(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


def nearest_waypoint_index(waypoints, x, y):
    if not waypoints:
        raise ValueError("waypoints must not be empty")
    return min(
        range(len(waypoints)),
        key=lambda index: squared_distance(
            float(waypoints[index]["position"]["x"]),
            float(waypoints[index]["position"]["y"]),
            x,
            y,
        ),
    )


def nearest_waypoint_distance_sq(waypoints, x, y):
    if not waypoints:
        return float("inf")
    index = nearest_waypoint_index(waypoints, x, y)
    position = waypoints[index]["position"]
    return squared_distance(
        float(position["x"]),
        float(position["y"]),
        x,
        y,
    )


def nearest_route_index(route, x, y, start_index=0):
    if not route:
        raise ValueError("route must not be empty")
    start_index = max(0, min(int(start_index), len(route) - 1))
    return min(
        range(start_index, len(route)),
        key=lambda index: squared_distance(
            float(route[index]["position"]["x"]),
            float(route[index]["position"]["y"]),
            x,
            y,
        ),
    )


def build_route_waypoints(waypoints, start_x, start_y, goal_x, goal_y):
    if not waypoints:
        return []
    start_index = nearest_waypoint_index(waypoints, start_x, start_y)
    goal_index = nearest_waypoint_index(waypoints, goal_x, goal_y)
    if start_index <= goal_index:
        return waypoints[start_index : goal_index + 1]
    return list(reversed(waypoints[goal_index : start_index + 1]))


def choose_directional_route(lanes, start_x, start_y, goal_x, goal_y):
    if not lanes:
        raise ValueError("lanes must not be empty")

    forward_candidates = []
    fallback_candidates = []
    for lane_name, lane_data in lanes.items():
        waypoints = lane_data["waypoints"]
        if not waypoints:
            continue
        start_index = nearest_waypoint_index(waypoints, start_x, start_y)
        goal_index = nearest_waypoint_index(waypoints, goal_x, goal_y)
        route = build_route_waypoints(waypoints, start_x, start_y, goal_x, goal_y)
        entry = waypoints[start_index]["position"]
        exit_wp = waypoints[goal_index]["position"]
        connection_cost = squared_distance(start_x, start_y, entry["x"], entry["y"]) + squared_distance(
            goal_x, goal_y, exit_wp["x"], exit_wp["y"]
        )
        candidate = (connection_cost, lane_name, lane_data, route)
        fallback_candidates.append(candidate)
        if start_index <= goal_index:
            forward_candidates.append(candidate)

    candidates = forward_candidates or fallback_candidates
    if not candidates:
        raise ValueError("no lane contains waypoints")
    _cost, lane_name, lane_data, route = min(candidates, key=lambda item: item[0])
    return lane_name, lane_data, route


def build_lane_transition_plan(current_lane, target_lane, lanes):
    if current_lane == target_lane:
        return []
    current_waypoints = lanes[current_lane]["waypoints"]
    target_waypoints = lanes[target_lane]["waypoints"]
    if not current_waypoints or not target_waypoints:
        return []
    if current_lane == "right" and target_lane == "left":
        return [current_waypoints[-1], target_waypoints[0]]
    if current_lane == "left" and target_lane == "right":
        return [current_waypoints[-1], target_waypoints[0]]
    return []


def should_use_lane_transition(current_lane, target_lane, lanes, start_x, start_y):
    if current_lane == target_lane:
        return False
    if current_lane not in lanes or target_lane not in lanes:
        return False

    current_distance_sq = nearest_waypoint_distance_sq(
        lanes[current_lane]["waypoints"],
        start_x,
        start_y,
    )
    target_distance_sq = nearest_waypoint_distance_sq(
        lanes[target_lane]["waypoints"],
        start_x,
        start_y,
    )
    return current_distance_sq < target_distance_sq


def route_window_goal_index(route_length, active_index, window_size):
    if route_length <= 0:
        raise ValueError("route_length must be positive")
    if active_index < 0 or active_index >= route_length:
        raise ValueError("active_index outside route")
    return min(route_length - 1, active_index + max(1, int(window_size)) - 1)


def route_lateral_error(route, waypoint_index, vehicle_x, vehicle_y):
    if waypoint_index < 0 or waypoint_index >= len(route):
        raise IndexError("waypoint_index outside route")
    if waypoint_index >= len(route) - 1:
        current = route[waypoint_index]["position"]
        return math.hypot(
            vehicle_x - float(current["x"]),
            vehicle_y - float(current["y"]),
        )

    current = route[waypoint_index]["position"]
    nxt = route[waypoint_index + 1]["position"]
    dx_route = float(nxt["x"]) - float(current["x"])
    dy_route = float(nxt["y"]) - float(current["y"])
    route_norm = math.hypot(dx_route, dy_route)
    if route_norm < 1e-9:
        return math.hypot(
            vehicle_x - float(current["x"]),
            vehicle_y - float(current["y"]),
        )

    dx_vehicle = vehicle_x - float(current["x"])
    dy_vehicle = vehicle_y - float(current["y"])
    return abs(dx_vehicle * dy_route - dy_vehicle * dx_route) / route_norm


def waypoint_has_been_passed(
    route,
    waypoint_index,
    vehicle_x,
    vehicle_y,
    tolerance,
    lateral_tolerance,
):
    if waypoint_index < 0 or waypoint_index >= len(route):
        raise IndexError("waypoint_index outside route")

    current = route[waypoint_index]["position"]
    dx_vehicle = vehicle_x - float(current["x"])
    dy_vehicle = vehicle_y - float(current["y"])
    distance_sq = dx_vehicle * dx_vehicle + dy_vehicle * dy_vehicle
    tolerance_sq = float(tolerance) * float(tolerance)

    if waypoint_index == len(route) - 1:
        return distance_sq <= tolerance_sq

    nxt = route[waypoint_index + 1]["position"]
    dx_route = float(nxt["x"]) - float(current["x"])
    dy_route = float(nxt["y"]) - float(current["y"])
    route_norm_sq = dx_route * dx_route + dy_route * dy_route
    if route_norm_sq < 1e-9:
        return distance_sq <= tolerance_sq

    forward_projection = dx_vehicle * dx_route + dy_vehicle * dy_route
    lateral_error = route_lateral_error(route, waypoint_index, vehicle_x, vehicle_y)
    return forward_projection >= 0.0 and lateral_error <= float(lateral_tolerance)


def route_drift_requires_reacquire(
    has_entered_route_corridor,
    is_reacquiring_route,
    current_lateral_error,
    reacquire_tolerance,
):
    return (
        has_entered_route_corridor
        and not is_reacquiring_route
        and current_lateral_error > float(reacquire_tolerance)
    )


def pose_from_waypoint(frame_id, waypoint):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(waypoint["position"]["x"])
    pose.pose.position.y = float(waypoint["position"]["y"])
    pose.pose.position.z = float(waypoint["position"].get("z", 0.0))
    yaw = float(waypoint.get("yaw", 0.0))
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def build_route_path(frame_id, route):
    path = NavPath()
    path.header.frame_id = frame_id
    for waypoint in route:
        path.poses.append(pose_from_waypoint(frame_id, waypoint))
    return path


class WaypointRouteManager:
    def __init__(self):
        default_dir = Path(__file__).resolve().parents[1] / "maps" / "ADmap"
        self.goal_topic = rospy.get_param("~goal_topic", "/waypoint_route_goal")
        self.route_path_topic = rospy.get_param("~route_path_topic", "/waypoint_route_path")
        self.route_window_topic = rospy.get_param("~route_window_topic", "/waypoint_route_window")
        self.route_window_goal_topic = rospy.get_param(
            "~route_window_goal_topic", "/waypoint_route_window_goal"
        )
        self.route_final_goal_topic = rospy.get_param(
            "~route_final_goal_topic", "/waypoint_route_final_goal"
        )
        self.right_waypoint_file = Path(
            rospy.get_param("~right_waypoint_file", str(default_dir / "AD_waypoints_right.yaml"))
        )
        self.left_waypoint_file = Path(
            rospy.get_param("~left_waypoint_file", str(default_dir / "AD_waypoints_left.yaml"))
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.active_lane = rospy.get_param("~default_lane", "right")
        self.route_goal_tolerance = float(rospy.get_param("~route_goal_tolerance", 0.8))
        self.route_lateral_tolerance = float(rospy.get_param("~route_lateral_tolerance", 1.5))
        self.route_reacquire_tolerance = float(rospy.get_param("~route_reacquire_tolerance", 3.0))
        self.route_recovered_tolerance = float(rospy.get_param("~route_recovered_tolerance", 1.5))
        self.route_window_size = int(rospy.get_param("~route_window_size", 4))
        self.route_goal_clearance_radius = float(rospy.get_param("~route_goal_clearance_radius", 1.2))
        self.route_goal_obstacle_threshold = int(rospy.get_param("~route_goal_obstacle_threshold", 80))
        self.route_goal_costmap_topic = rospy.get_param(
            "~route_goal_costmap_topic",
            "/move_base_node/global_costmap/costmap",
        )
        self.route_update_rate = float(rospy.get_param("~route_update_rate", 5.0))
        self.final_goal_timeout = float(rospy.get_param("~final_goal_timeout", 180.0))
        self.active_route = []
        self.active_route_frame_id = self.map_frame
        self.active_route_index = 0
        self.active_goal_index = None
        self.final_goal = None
        self.latest_route_goal_costmap = None
        self.is_reacquiring_route = False
        self.has_entered_route_corridor = False
        self.current_lane = self.active_lane
        self.target_lane = self.active_lane
        self.route_mode = "lane_following"
        self.transition_route = []
        self.transition_route_index = 0
        self.pending_lane_route = []

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.on_goal, queue_size=1)
        self.route_goal_costmap_sub = rospy.Subscriber(
            self.route_goal_costmap_topic,
            OccupancyGrid,
            self.on_route_goal_costmap,
            queue_size=1,
        )
        self.route_path_pub = rospy.Publisher(self.route_path_topic, NavPath, queue_size=1, latch=True)
        self.route_window_pub = rospy.Publisher(self.route_window_topic, NavPath, queue_size=1, latch=True)
        self.route_window_goal_pub = rospy.Publisher(
            self.route_window_goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self.final_goal_pub = rospy.Publisher(
            self.route_final_goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self.use_right_srv = rospy.Service("~use_right", Trigger, self.on_use_right)
        self.use_left_srv = rospy.Service("~use_left", Trigger, self.on_use_left)
        self.route_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.route_update_rate, 1e-3)),
            self.on_route_timer,
        )

        rospy.loginfo("Waypoint route manager waiting for move_base action server")
        self.move_base.wait_for_server()
        rospy.loginfo("Waypoint route manager active lane: %s", self.active_lane)

    def on_route_goal_costmap(self, msg):
        self.latest_route_goal_costmap = msg

    def on_use_right(self, _req):
        self.active_lane = "right"
        rospy.loginfo("Waypoint route manager switched to right lane")
        return TriggerResponse(True, "using right lane route")

    def on_use_left(self, _req):
        self.active_lane = "left"
        rospy.loginfo("Waypoint route manager switched to left lane")
        return TriggerResponse(True, "using left lane route")

    def on_goal(self, msg):
        try:
            start_x, start_y = self.current_position()
            lane_data_by_name = self.load_lane_data()
            lane_name, lane_data, route = choose_directional_route(
                lane_data_by_name,
                start_x,
                start_y,
                msg.pose.position.x,
                msg.pose.position.y,
            )
        except Exception as exc:
            rospy.logerr("Failed to build waypoint route: %s", exc)
            return

        rospy.loginfo(
            "Received final goal x=%.3f y=%.3f via %s lane with %d route waypoints",
            msg.pose.position.x,
            msg.pose.position.y,
            lane_name,
            len(route),
        )
        self.active_lane = lane_name
        self.target_lane = lane_name
        self.final_goal = msg
        self.active_route_frame_id = lane_data["frame_id"]
        self.route_path_pub.publish(build_route_path(lane_data["frame_id"], route))
        current_lane = getattr(self, "current_lane", self.active_lane)
        self.current_lane = current_lane
        if should_use_lane_transition(
            current_lane,
            lane_name,
            lane_data_by_name,
            start_x,
            start_y,
        ):
            transition_route = build_lane_transition_plan(
                self.current_lane,
                lane_name,
                lane_data_by_name,
            )
            if not transition_route:
                rospy.logwarn(
                    "No lane transition route from %s to %s; following %s lane directly",
                    self.current_lane,
                    lane_name,
                    lane_name,
                )
                self.current_lane = lane_name
                self._activate_lane_route(lane_data["frame_id"], route)
                return

            self.route_mode = "lane_transition"
            self.transition_route = transition_route
            self.transition_route_index = 0
            self.pending_lane_route = route
            self.active_route = []
            self.active_route_index = 0
            self.active_goal_index = None
            self.is_reacquiring_route = False
            self.has_entered_route_corridor = False
            self.send_pose_goal(
                pose_from_waypoint(
                    lane_data_by_name[current_lane]["frame_id"],
                    self.transition_route[self.transition_route_index],
                ),
                is_final=False,
            )
            return

        if current_lane != lane_name:
            rospy.loginfo(
                "Vehicle is already closer to %s lane than stale %s lane; following selected lane directly",
                lane_name,
                current_lane,
            )
        self.current_lane = lane_name
        self._activate_lane_route(lane_data["frame_id"], route)

    def load_lane_data(self):
        return {
            "right": load_waypoint_file(self.right_waypoint_file),
            "left": load_waypoint_file(self.left_waypoint_file),
        }

    def current_position(self):
        self.tf_listener.waitForTransform(
            self.map_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        translation, _rotation = self.tf_listener.lookupTransform(
            self.map_frame,
            self.base_frame,
            rospy.Time(0),
        )
        return translation[0], translation[1]

    def on_route_timer(self, _event):
        self.update_route_progress()

    def update_route_progress(self):
        if getattr(self, "route_mode", "lane_following") == "lane_transition":
            self.update_transition_progress()
            return
        if not self.active_route:
            return

        try:
            vehicle_x, vehicle_y = self.current_position()
        except Exception as exc:
            rospy.logwarn("Failed to update waypoint route progress: %s", exc)
            return

        synced_index = nearest_route_index(
            self.active_route,
            vehicle_x,
            vehicle_y,
            self.active_route_index,
        )
        if synced_index > self.active_route_index:
            synced_position = self.active_route[synced_index]["position"]
            synced_distance = math.hypot(
                vehicle_x - float(synced_position["x"]),
                vehicle_y - float(synced_position["y"]),
            )
            if synced_distance <= self.route_reacquire_tolerance:
                self.active_route_index = synced_index
                restored_goal_index = self.select_window_goal_index(self.active_route_index)
                if restored_goal_index != self.active_goal_index:
                    self.active_goal_index = restored_goal_index
                    rospy.loginfo(
                        "Synced route progress to waypoint %s",
                        self.active_route[self.active_route_index]["id"],
                    )
                    self.publish_route_visualization()
                    self.send_pose_goal(
                        pose_from_waypoint(
                            self.active_route_frame_id,
                            self.active_route[self.active_goal_index],
                        ),
                        is_final=False,
                    )

        current_lateral_error = route_lateral_error(
            self.active_route,
            self.active_route_index,
            vehicle_x,
            vehicle_y,
        )
        if (
            not self.has_entered_route_corridor
            and current_lateral_error <= self.route_recovered_tolerance
        ):
            self.has_entered_route_corridor = True
            rospy.loginfo("Entered route corridor; route drift monitoring armed")

        if route_drift_requires_reacquire(
            self.has_entered_route_corridor,
            self.is_reacquiring_route,
            current_lateral_error,
            self.route_reacquire_tolerance,
        ):
            self.is_reacquiring_route = True
            if self.active_goal_index != self.active_route_index:
                self.active_goal_index = self.active_route_index
                rospy.logwarn(
                    "Route drift %.2fm detected; reacquiring waypoint %s",
                    current_lateral_error,
                    self.active_route[self.active_goal_index]["id"],
                )
                self.publish_route_visualization()
                self.send_pose_goal(
                    pose_from_waypoint(
                        self.active_route_frame_id,
                        self.active_route[self.active_goal_index],
                    ),
                    is_final=False,
                )
            return

        if self.is_reacquiring_route:
            if current_lateral_error > self.route_recovered_tolerance:
                return
            self.is_reacquiring_route = False
            restored_goal_index = self.select_window_goal_index(
                self.active_route_index,
            )
            if restored_goal_index != self.active_goal_index:
                self.active_goal_index = restored_goal_index
                rospy.loginfo(
                    "Route reacquired; restoring window goal %s",
                    self.active_route[self.active_goal_index]["id"],
                )
                self.publish_route_visualization()
                self.send_pose_goal(
                    pose_from_waypoint(
                        self.active_route_frame_id,
                        self.active_route[self.active_goal_index],
                    ),
                    is_final=False,
                )

        while self.active_route and waypoint_has_been_passed(
            self.active_route,
            self.active_route_index,
            vehicle_x,
            vehicle_y,
            self.route_goal_tolerance,
            self.route_lateral_tolerance,
        ):
            if self.active_route_index >= len(self.active_route) - 1:
                final_goal = self.final_goal
                self.active_route = []
                self.active_goal_index = None
                self.is_reacquiring_route = False
                self.has_entered_route_corridor = False
                if final_goal is not None:
                    rospy.loginfo("Exited waypoint route; sending final goal")
                    self.publish_route_visualization()
                    self.send_pose_goal(final_goal, is_final=True)
                return

            self.active_route_index += 1
            next_goal_index = self.select_window_goal_index(self.active_route_index)
            if next_goal_index != self.active_goal_index:
                self.active_goal_index = next_goal_index
                waypoint_goal = pose_from_waypoint(
                    self.active_route_frame_id,
                    self.active_route[self.active_goal_index],
                )
                rospy.loginfo(
                    "Advancing route window to waypoint %s",
                    self.active_route[self.active_goal_index]["id"],
                )
                self.publish_route_visualization()
                self.send_pose_goal(waypoint_goal, is_final=False)

    def update_transition_progress(self):
        if not self.transition_route:
            return
        try:
            vehicle_x, vehicle_y = self.current_position()
        except Exception as exc:
            rospy.logwarn("Failed to update lane transition progress: %s", exc)
            return

        current_transition_wp = self.transition_route[self.transition_route_index]
        current_pos = current_transition_wp["position"]
        if math.hypot(
            vehicle_x - float(current_pos["x"]),
            vehicle_y - float(current_pos["y"]),
        ) > self.route_goal_tolerance:
            return

        if self.transition_route_index < len(self.transition_route) - 1:
            self.transition_route_index += 1
            next_wp = self.transition_route[self.transition_route_index]
            rospy.loginfo("Advancing lane transition to waypoint %s", next_wp["id"])
            self.send_pose_goal(
                pose_from_waypoint(self.active_route_frame_id, next_wp),
                is_final=False,
            )
            return

        self.current_lane = self.target_lane
        rospy.loginfo("Lane transition complete; following %s lane", self.current_lane)
        self._activate_lane_route(self.active_route_frame_id, self.pending_lane_route)

    def _activate_lane_route(self, frame_id, route):
        self.route_mode = "lane_following"
        self.transition_route = []
        self.transition_route_index = 0
        self.pending_lane_route = []
        self.active_route = route
        self.active_route_frame_id = frame_id
        self.active_route_index = 0
        self.active_goal_index = self.select_window_goal_index(0)
        self.is_reacquiring_route = False
        self.has_entered_route_corridor = False
        self.publish_route_visualization()
        self.send_pose_goal(
            pose_from_waypoint(frame_id, route[self.active_goal_index]),
            is_final=False,
        )

    def select_window_goal_index(self, active_index):
        nominal_index = route_window_goal_index(
            len(self.active_route),
            active_index,
            self.route_window_size,
        )
        for candidate_index in range(nominal_index, len(self.active_route)):
            if not self.is_waypoint_goal_blocked(self.active_route[candidate_index]):
                if candidate_index != nominal_index:
                    rospy.logwarn(
                        "Route window goal %s is blocked; skipping to %s",
                        self.active_route[nominal_index]["id"],
                        self.active_route[candidate_index]["id"],
                    )
                return candidate_index
        return nominal_index

    def is_waypoint_goal_blocked(self, waypoint):
        costmap = self.latest_route_goal_costmap
        if costmap is None or costmap.info.resolution <= 0.0:
            return False
        if costmap.header.frame_id and costmap.header.frame_id != self.active_route_frame_id:
            return False

        position = waypoint["position"]
        x = float(position["x"])
        y = float(position["y"])
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        radius_cells = max(1, int(math.ceil(self.route_goal_clearance_radius / resolution)))
        center_x = int((x - origin_x) / resolution)
        center_y = int((y - origin_y) / resolution)

        if center_x < 0 or center_x >= costmap.info.width or center_y < 0 or center_y >= costmap.info.height:
            return False

        for cell_y in range(max(0, center_y - radius_cells), min(costmap.info.height, center_y + radius_cells + 1)):
            for cell_x in range(max(0, center_x - radius_cells), min(costmap.info.width, center_x + radius_cells + 1)):
                dx = (cell_x - center_x) * resolution
                dy = (cell_y - center_y) * resolution
                if dx * dx + dy * dy > self.route_goal_clearance_radius * self.route_goal_clearance_radius:
                    continue
                index = cell_y * costmap.info.width + cell_x
                if index < len(costmap.data) and costmap.data[index] >= self.route_goal_obstacle_threshold:
                    return True
        return False

    def publish_route_visualization(self):
        if self.final_goal is not None:
            self.final_goal_pub.publish(self.final_goal)

        if not self.active_route or self.active_goal_index is None:
            self.route_window_pub.publish(build_route_path(self.active_route_frame_id, []))
            return

        current_window = self.active_route[self.active_route_index : self.active_goal_index + 1]
        self.route_window_pub.publish(build_route_path(self.active_route_frame_id, current_window))
        self.route_window_goal_pub.publish(
            pose_from_waypoint(
                self.active_route_frame_id,
                self.active_route[self.active_goal_index],
            )
        )

    def send_pose_goal(self, pose, is_final):
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.move_base.send_goal(goal)
        if is_final:
            self.move_base.wait_for_result(rospy.Duration(self.final_goal_timeout))


if __name__ == "__main__":
    rospy.init_node("waypoint_route_manager")
    WaypointRouteManager()
    rospy.spin()
