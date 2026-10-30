cmake_minimum_required(VERSION 3.0.2)
project(my_nav_assist)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   nav_msgs
   geometry_msgs
   sensor_msgs
   tf2
   tf2_ros
   tf2_geometry_msgs
   chcnav  # 添加chcnav依赖
   message_generation
 )
 
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# 移除hcinspvatzcb.msg的定义，因为它在chcnav包中
# add_message_files(
#   FILES
#   hcinspvatzcb.msg
# )

# 如果没有其他自定义消息，可以注释掉消息生成
# generate_messages(
#   DEPENDENCIES
#   std_msgs   # 基础依赖
#   geometry_msgs # 若消息中包含几何类型
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_nav_assist
  CATKIN_DEPENDS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  chcnav  # 添加chcnav运行时依赖
#  message_runtime  # 如果没有自定义消息可以注释掉
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${GeographicLib_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS} 
)

add_library(lidar_odometry_coordinate_fix src/lidar_odometry_coordinate_fix.cpp)
target_link_libraries(lidar_odometry_coordinate_fix ${catkin_LIBRARIES})

add_executable(stand_alone_lidar_odometry_fix src/stand_alone_lidar_odometry_fix.cpp)
target_link_libraries(stand_alone_lidar_odometry_fix ${catkin_LIBRARIES})

add_executable(gnss_processor src/gnss_processor.cpp)
target_link_libraries(gnss_processor ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(map_orgin_recorder src/map_orgin_recorder.cpp)
target_link_libraries(map_orgin_recorder ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(tf_coordinate_converter src/tf_coordinate_converter.cpp)
target_link_libraries(tf_coordinate_converter ${catkin_LIBRARIES})

add_executable(simple_tf_coordinate_converter src/simple_tf_coordinate_converter.cpp)
target_link_libraries(simple_tf_coordinate_converter ${catkin_LIBRARIES})

add_executable(enhanced_map_origin_recorder src/enhanced_map_origin_recorder.cpp)
target_link_libraries(enhanced_map_origin_recorder ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(rtk_gnss_processor src/rtk_gnss_processor.cpp)
target_link_libraries(rtk_gnss_processor ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

# 添加对chcnav消息的依赖
add_dependencies(enhanced_map_origin_recorder
  ${catkin_EXPORTED_TARGETS}
  chcnav_generate_messages_cpp  # 依赖chcnav的消息生成
)

add_dependencies(rtk_gnss_processor
  ${catkin_EXPORTED_TARGETS}
  chcnav_generate_messages_cpp  # 依赖chcnav的消息生成
)

catkin_install_python(PROGRAMS
   scripts/call_save_pcd.py
   scripts/call_save_pcd_hdl.py
   scripts/pcd_publisher.py
   scripts/nav_monitor.py
   scripts/global_path_metrics_recorder.py
   scripts/global_path_metrics_plotter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
