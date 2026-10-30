#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("static_odom_publisher")

    odom_topic = rospy.get_param("~odom_topic", "/local_odometry")
    global_odom_topic = rospy.get_param("~global_odom_topic", "/global_odometry")
    frame_id = rospy.get_param("~frame_id", "odom")
    child_frame_id = rospy.get_param("~child_frame_id", "base_footprint")
    rate_hz = rospy.get_param("~rate", 20.0)

    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    global_odom_pub = rospy.Publisher(global_odom_topic, Odometry, queue_size=1)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.01
        msg.twist.covariance[35] = 0.01
        odom_pub.publish(msg)

        global_msg = Odometry()
        global_msg.header.stamp = msg.header.stamp
        global_msg.header.frame_id = "map"
        global_msg.child_frame_id = child_frame_id
        global_msg.pose.pose.orientation.w = 1.0
        global_odom_pub.publish(global_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
