#!/usr/bin/env python3
import math
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from waypoint_io import load_waypoint_file


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))


class WaypointMarkerPublisher:
    def __init__(self):
        default_path = Path(__file__).resolve().parents[1] / "maps" / "ADmap" / "AD_waypoints.yaml"
        self.waypoint_file = Path(rospy.get_param("~waypoint_file", str(default_path)))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.use_wall_rate = bool(rospy.get_param("~use_wall_rate", False))
        self.arrow_length = float(rospy.get_param("~arrow_length", 2.0))
        self.arrow_width = float(rospy.get_param("~arrow_width", 0.18))

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("~poses", PoseArray, queue_size=1, latch=True)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                data = load_waypoint_file(self.waypoint_file)
                self.publish(data)
            except Exception as exc:
                rospy.logerr_throttle(5.0, "Failed to publish waypoints from %s: %s", self.waypoint_file, exc)
            if self.use_wall_rate:
                time.sleep(1.0 / self.publish_rate)
            else:
                rate.sleep()

    def publish(self, data):
        frame_id = data.get("frame_id", "map")
        stamp = rospy.Time.now()
        waypoints = data.get("waypoints", [])

        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = stamp

        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker(frame_id, stamp))

        for index, wp in enumerate(waypoints):
            pose = self.pose_from_waypoint(wp)
            pose_array.poses.append(pose)
            markers.markers.extend(self.markers_for_waypoint(frame_id, stamp, index, wp, pose))

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(markers)

    def pose_from_waypoint(self, wp):
        pose = Pose()
        pose.position.x = float(wp["position"]["x"])
        pose.position.y = float(wp["position"]["y"])
        pose.position.z = float(wp["position"].get("z", 0.0))
        pose.orientation = quaternion_from_yaw(float(wp["yaw"]))
        return pose

    def markers_for_waypoint(self, frame_id, stamp, index, wp, pose):
        waypoint_id = wp["id"]
        heading = wp["heading"]
        hx = float(heading["x"])
        hy = float(heading["y"])
        start = Point(pose.position.x, pose.position.y, pose.position.z + 0.25)
        end = Point(
            pose.position.x + hx * self.arrow_length,
            pose.position.y + hy * self.arrow_length,
            pose.position.z + 0.25,
        )

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "waypoint_heading"
        arrow.id = index
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [start, end]
        arrow.scale.x = self.arrow_width
        arrow.scale.y = self.arrow_width * 2.5
        arrow.scale.z = self.arrow_width * 2.5
        arrow.color = ColorRGBA(0.0, 0.55, 1.0, 0.95)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "waypoint_label"
        text.id = index
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = pose.position.z + 1.2
        text.scale.z = 0.8
        text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.text = waypoint_id

        return [arrow, text]

    @staticmethod
    def delete_all_marker(frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker


if __name__ == "__main__":
    rospy.init_node("waypoint_marker_publisher")
    WaypointMarkerPublisher().run()
