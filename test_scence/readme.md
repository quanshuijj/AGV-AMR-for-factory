rosrun map_server map_saver map:=/projected_map -f /home/lujiaqi/navCarProject_ws/src/test_scence/map/nav
地图保存
栅格地图运行

pointcloud_to_laserscan


# for noetic
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release



cd /your/catkin_ws/src
git clone https://github.com/koide3/ndt_omp
git clone https://github.com/SMRT-AIST/fast_gicp --recursive
git clone https://github.com/koide3/hdl_localization
git clone https://github.com/koide3/hdl_global_localization

cd /your/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release

# if you want to enable CUDA-accelerated NDT
# catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=ON
