#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse


class DualWaypointRouter:
    def __init__(self):
        self.input_pose_topic = rospy.get_param("~input_pose_topic", "/waypoint_pose")
        self.right_pose_topic = rospy.get_param("~right_pose_topic", "/right_waypoint_pose")
        self.left_pose_topic = rospy.get_param("~left_pose_topic", "/left_waypoint_pose")
        self.active_lane = rospy.get_param("~default_lane", "right")

        self.right_pub = rospy.Publisher(self.right_pose_topic, PoseStamped, queue_size=10)
        self.left_pub = rospy.Publisher(self.left_pose_topic, PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.input_pose_topic, PoseStamped, self.on_pose, queue_size=10)
        self.use_right_srv = rospy.Service("~use_right", Trigger, self.on_use_right)
        self.use_left_srv = rospy.Service("~use_left", Trigger, self.on_use_left)
        rospy.loginfo("Dual waypoint router active lane: %s", self.active_lane)

    def on_pose(self, msg):
        if self.active_lane == "left":
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)
        rospy.loginfo(
            "Forwarded waypoint pose x=%.3f y=%.3f to %s lane",
            msg.pose.position.x,
            msg.pose.position.y,
            self.active_lane,
        )

    def on_use_right(self, _req):
        self.active_lane = "right"
        rospy.loginfo("Dual waypoint router switched to right lane")
        return TriggerResponse(True, "recording right lane waypoints")

    def on_use_left(self, _req):
        self.active_lane = "left"
        rospy.loginfo("Dual waypoint router switched to left lane")
        return TriggerResponse(True, "recording left lane waypoints")


if __name__ == "__main__":
    rospy.init_node("dual_waypoint_router")
    DualWaypointRouter()
    rospy.spin()
