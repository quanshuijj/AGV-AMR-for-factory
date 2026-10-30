#!/usr/bin/env python3
import rospy

from agv_nav_evaluation.online_status import OnlineStatusMonitor


def main():
    rospy.init_node("agv_nav_evaluation_online_monitor", anonymous=False)
    monitor = OnlineStatusMonitor(
        topics_path=rospy.get_param("~topics_config"),
        vehicle_path=rospy.get_param("~vehicle_config"),
        metrics_path=rospy.get_param("~metrics_config"),
    )
    monitor.start()


if __name__ == "__main__":
    main()
