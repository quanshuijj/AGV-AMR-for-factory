#!/usr/bin/env python3
import math
from pathlib import Path
import sys

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
from waypoint_io import (
    dump_waypoint_file,
    load_waypoint_file,
    normalize_vector,
    snap_heading_to_cardinal,
    yaw_from_vector,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorder:
    def __init__(self):
        default_path = Path(__file__).resolve().parents[1] / "maps" / "ADmap" / "AD_waypoints.yaml"
        self.waypoint_file = Path(rospy.get_param("~waypoint_file", str(default_path)))
        self.clicked_topic = rospy.get_param("~clicked_topic", "/clicked_point")
        self.pose_topic = rospy.get_param("~pose_topic", "/waypoint_pose")
        self.input_mode = rospy.get_param("~input_mode", "pose")
        self.id_prefix = rospy.get_param("~id_prefix", "wp")
        self.default_heading = rospy.get_param("~default_heading", [1.0, 0.0])

        self.data = load_waypoint_file(self.waypoint_file)
        if self.input_mode == "point":
            self.sub = rospy.Subscriber(self.clicked_topic, PointStamped, self.on_clicked_point, queue_size=10)
            rospy.loginfo("Recording point waypoints from %s into %s", self.clicked_topic, self.waypoint_file)
        else:
            self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=10)
            rospy.loginfo("Recording oriented waypoints from %s into %s", self.pose_topic, self.waypoint_file)

    def on_pose(self, msg):
        frame_id = self.data.get("frame_id", "map")
        if msg.header.frame_id and msg.header.frame_id != frame_id:
            rospy.logwarn("Ignoring waypoint pose in frame '%s'; expected '%s'", msg.header.frame_id, frame_id)
            return

        yaw = yaw_from_quaternion(msg.pose.orientation)
        hx, hy = snap_heading_to_cardinal(math.cos(yaw), math.sin(yaw))
        self.append_waypoint(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            hx,
            hy,
            update_sequence_headings=False,
        )

    def on_clicked_point(self, msg):
        frame_id = self.data.get("frame_id", "map")
        if msg.header.frame_id and msg.header.frame_id != frame_id:
            rospy.logwarn("Ignoring clicked point in frame '%s'; expected '%s'", msg.header.frame_id, frame_id)
            return

        hx, hy = normalize_vector(float(self.default_heading[0]), float(self.default_heading[1]))
        self.append_waypoint(
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
            hx,
            hy,
            update_sequence_headings=True,
        )

    def append_waypoint(self, x, y, z, hx, hy, update_sequence_headings):
        waypoints = self.data.setdefault("waypoints", [])
        wp_id = "{}_{:03d}".format(self.id_prefix, len(waypoints) + 1)
        waypoint = {
            "id": wp_id,
            "position": {"x": x, "y": y, "z": z},
            "heading": {"x": hx, "y": hy},
            "yaw": yaw_from_vector(hx, hy),
        }
        waypoints.append(waypoint)
        if update_sequence_headings:
            self.update_headings_from_sequence(waypoints)
        dump_waypoint_file(self.waypoint_file, self.data)
        rospy.loginfo("Saved waypoint %s at x=%.3f y=%.3f heading=(%.3f, %.3f)", wp_id, x, y, hx, hy)

    @staticmethod
    def update_headings_from_sequence(waypoints):
        for index in range(len(waypoints) - 1):
            current = waypoints[index]["position"]
            nxt = waypoints[index + 1]["position"]
            hx, hy = normalize_vector(float(nxt["x"]) - float(current["x"]), float(nxt["y"]) - float(current["y"]))
            waypoints[index]["heading"] = {"x": hx, "y": hy}
            waypoints[index]["yaw"] = yaw_from_vector(hx, hy)
        if len(waypoints) >= 2:
            waypoints[-1]["heading"] = dict(waypoints[-2]["heading"])
            waypoints[-1]["yaw"] = waypoints[-2]["yaw"]


if __name__ == "__main__":
    rospy.init_node("waypoint_recorder")
    WaypointRecorder()
    rospy.spin()
