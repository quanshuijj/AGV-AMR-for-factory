cmake_minimum_required(VERSION 3.0.2)
project(agv_nav_evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  roslaunch
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_msgs
  actionlib_msgs
  diagnostic_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/record_experiment.py
  scripts/online_monitor.py
  scripts/analyze_bag.py
  scripts/compute_metrics.py
  scripts/generate_report.py
  scripts/export_manual_event_template.py
  scripts/eval_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch tools output
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
