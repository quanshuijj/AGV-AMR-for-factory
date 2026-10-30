---调试及环境配置说明--
ROS包
ROS NAV
ROS Teb
sudo apt install ros-noetic-teb-local-planner
 H.264
 ffmpeg-4.2.7
Draco版本-1.5.7
mkdir build_dir && cd build_dir
cmake ../
make 
sudo make install
 pcl版本1.10
 
 定位模块插件
 sudo apt install ros-noetic-jsk-visualization
 
 GeographicLib
cd GeographicLib-2.3 
 
mkdir BUILD
cd BUILD 
 
../configure 
 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
 
sudo make install

# 查找模块路径（通常为 /usr/share/cmake/geographiclib/）
locate FindGeographicLib.cmake

# 创建符号链接（以 CMake 3.16 为例）
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/
