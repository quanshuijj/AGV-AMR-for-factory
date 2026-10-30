cmake_minimum_required(VERSION 3.0.2)
project(navcar_project_server)

## Compile as C++17
add_compile_options(-std=c++17)

# Qt Creator环境检测
if(DEFINED ENV{QTCREATOR_VERSION})
    set(QTCREATOR_BUILD TRUE)
    message(STATUS "Detected Qt Creator build environment")
else()
    set(QTCREATOR_BUILD FALSE)
endif()

# ROS环境检测
find_package(catkin QUIET)
if(catkin_FOUND)
    set(ROS_BUILD TRUE)
    message(STATUS "ROS environment detected")
else()
    set(ROS_BUILD FALSE)
    message(WARNING "ROS environment not found, building as Qt-only project")
endif()

# Draco点云压缩库环境检测
find_package(draco QUIET)
if(draco_FOUND)
    set(ENABLE_DRACO TRUE)
    message(STATUS "Draco点云压缩库检测成功 (版本: ${draco_VERSION})")
else()
    set(ENABLE_DRACO FALSE)
    message(WARNING "Draco点云压缩库未找到，点云压缩功能将被禁用")
endif()

# ffmpeg库环境检测
option(ENABLE_FFMPEG "Enable FFmpeg multimedia support" ON)
if(ENABLE_FFMPEG)
    if(WIN32)
        # Windows环境手动配置
        set(FFMPEG_ROOT "D:/ffmpeg-5.0.1-full_build-shared" CACHE PATH "FFmpeg安装路径")
        if(EXISTS ${FFMPEG_ROOT})
            set(FFMPEG_INCLUDE_DIRS ${FFMPEG_ROOT}/include)
            set(FFMPEG_LIB_DIR ${FFMPEG_ROOT}/lib)
            set(FFMPEG_BIN_DIR ${FFMPEG_ROOT}/bin)

            # 定义核心库
            set(FFMPEG_LIBS
                avcodec avformat avutil swscale swresample avfilter avdevice)

            include_directories(${FFMPEG_INCLUDE_DIRS})
            link_directories(${FFMPEG_LIB_DIR})
            message(STATUS "FFmpeg Windows路径: ${FFMPEG_ROOT}")
        else()
            message(WARNING "FFmpeg路径未找到，禁用多媒体功能")
            set(ENABLE_FFMPEG OFF)
        endif()
    else()
        # Linux/macOS使用pkg-config
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
            libavcodec libavformat libavutil libswscale libswresample)

        if(FFMPEG_FOUND)
            message(STATUS "FFmpeg版本: ${FFMPEG_VERSION}")
            set(FFMPEG_LIBS PkgConfig::FFMPEG)
        else()
            set(ENABLE_FFMPEG OFF)
        endif()
    endif()
endif()

# Qt版本检测和配置
if(ROS_BUILD AND NOT QTCREATOR_BUILD)
    # ROS catkin_make环境，使用Qt5
    message(STATUS "Using Qt5 for ROS catkin build")
    find_package(Qt5 REQUIRED COMPONENTS
        Core
        Widgets
        Network
        SerialPort
        Concurrent
    )
    set(QT_VERSION_MAJOR 5)
    set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Network Qt5::SerialPort Qt5::Concurrent)
else()
    # Qt Creator环境或非ROS环境，尝试Qt6，失败则降级到Qt5
    find_package(Qt6 QUIET COMPONENTS Core Widgets Network SerialPort Concurrent)
    if(Qt6_FOUND)
        message(STATUS "Using Qt6")
        set(QT_VERSION_MAJOR 6)
        set(QT_LIBRARIES Qt6::Core Qt6::Widgets Qt6::Network Qt6::SerialPort Qt6::Concurrent)
    else()
        message(STATUS "Qt6 not found, falling back to Qt5")
        find_package(Qt5 REQUIRED COMPONENTS
            Core
            Widgets
            Network
            SerialPort
            Concurrent
        )
        set(QT_VERSION_MAJOR 5)
        set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Network Qt5::SerialPort Qt5::Concurrent)
    endif()
endif()

# ROS依赖 - 只在ROS环境中启用
if(ROS_BUILD)
    # 设置 chcnav 的显式路径
    set(chcnav_DIR "/home/c106/navCarProject_ws/devel/share/chcnav/cmake")
    message(STATUS "Setting chcnav_DIR to: ${chcnav_DIR}")

    # 验证文件存在
    if(EXISTS "${chcnav_DIR}/chcnavConfig.cmake" OR EXISTS "${chcnav_DIR}/chcnav-config.cmake")
        message(STATUS "Found chcnav config files")
    else()
        file(GLOB chcnav_files "${chcnav_DIR}/*")
        message(WARNING "chcnav config missing! Files in directory: ${chcnav_files}")
    endif()
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      std_srvs
      sensor_msgs
      nav_msgs
      geometry_msgs
      cv_bridge
      tf2
      tf2_ros
      tf2_geometry_msgs
      chcnav
    )
endif()

# Enable Qt MOC, UIC, and RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt Creator需要的UI文件路径
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 根据构建环境选择配置
if(ROS_BUILD)
    ## catkin specific configuration
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        std_srvs
        sensor_msgs
        nav_msgs
        geometry_msgs
        cv_bridge
        tf2
        tf2_ros
        tf2_geometry_msgs
        chcnav
      DEPENDS Qt${QT_VERSION_MAJOR}Core Qt${QT_VERSION_MAJOR}Widgets Qt${QT_VERSION_MAJOR}Network Qt${QT_VERSION_MAJOR}SerialPort
    )

    ## Build
    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )
else()
    # Qt Creator构建时的include目录
    include_directories(
      include
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
    )
endif()

# Qt source files
set(QT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/lininterface.cpp
  src/socketcanmanager.cpp
  src/navcarmanagernode.cpp
  src/canlinhandle.cpp
  src/mylininterface.cpp
  src/calikcontrolmodel.cpp
  src/compressionmanager.cpp
  src/networkmanager.cpp
  src/rosworkmanager.cpp
  src/globalpathnavigator.cpp
  src/mylininterface_hw_mi.cpp
  src/draco_helper.cpp
  src/h264_cache_manager.cpp
  src/data_broadcast_manager.cpp
  src/data_broadcast_worker.cpp
  src/datalogger.cpp
  src/mapdisplaywidget.cpp       # 新增
  src/pointmanager.cpp           # 新增
)

set(QT_HEADERS
  include/${PROJECT_NAME}/mainwindow.h
  include/${PROJECT_NAME}/lininterface.h
  include/${PROJECT_NAME}/socketcanmanager.h
  include/${PROJECT_NAME}/navcarmanagernode.h
  include/${PROJECT_NAME}/canlinhandle.h
  include/${PROJECT_NAME}/mylininterface.h
  include/${PROJECT_NAME}/calikcontrolmodel.h
  include/${PROJECT_NAME}/compressionmanager.h
  include/${PROJECT_NAME}/networkmanager.h
  include/${PROJECT_NAME}/rosworkmanager.h
  include/${PROJECT_NAME}/globalpathnavigator.h
  include/${PROJECT_NAME}/mylininterface_hw_mi.h
  include/${PROJECT_NAME}/networkprotocol.h
  include/${PROJECT_NAME}/draco_helper.h
  include/${PROJECT_NAME}/h264_cache_manager.h
  include/${PROJECT_NAME}/qt_version_compat.h
  include/${PROJECT_NAME}/data_broadcast_manager.h
  include/${PROJECT_NAME}/data_broadcast_worker.h
  include/${PROJECT_NAME}/datalogger.h
  include/${PROJECT_NAME}/mapdisplaywidget.h    # 新增
  include/${PROJECT_NAME}/pointmanager.h        # 新增
)

set(QT_UI_FILES
  ui/mainwindow.ui
)

# Add executable
add_executable(${PROJECT_NAME}_node
  ${QT_SOURCES}
  ${QT_HEADERS}
  ${QT_UI_FILES}
)

# Add dependencies
if(ROS_BUILD)
    add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${QT_LIBRARIES}
)

if(ROS_BUILD)
    target_link_libraries(${PROJECT_NAME}_node
      ${catkin_LIBRARIES}
      yaml-cpp
    )
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE ROS_BUILD)
endif()

# Draco相关依赖
if(ENABLE_DRACO)
    target_include_directories(${PROJECT_NAME}_node PRIVATE ${draco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_node
        draco::draco
        ${draco_LIBRARIES}
    )
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE ENABLE_DRACO)
endif()

# ffmpeg相关依赖
if(ENABLE_FFMPEG)
    target_link_libraries(${PROJECT_NAME}_node PkgConfig::FFMPEG ${FFMPEG_LIBS})
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE ENABLE_FFMPEG)

    # Windows下自动复制DLL到输出目录
    if(WIN32)
        add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${FFMPEG_BIN_DIR}/"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}_node>"
            COMMENT "Copying FFmpeg DLLs to output directory"
        )
    endif()
endif()

# 为Qt Creator设置目标属性
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_node
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Install (只在ROS环境中)
if(ROS_BUILD)
    install(TARGETS ${PROJECT_NAME}_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
      PATTERN ".svn" EXCLUDE
    )

    install(DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      PATTERN ".svn" EXCLUDE
    )

    install(DIRECTORY config/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
      PATTERN ".svn" EXCLUDE
    )
endif()

# Qt Creator项目文件信息
if(QTCREATOR_BUILD)
    # 添加资源文件到项目树
    file(GLOB_RECURSE RESOURCE_FILES
        "launch/*.launch"
        "config/*.yaml"
        "config/*.xml"
        "*.md"
        "*.txt"
    )

    # 创建一个虚拟目标来显示这些文件
    add_custom_target(project_files SOURCES ${RESOURCE_FILES})
endif()
