/**
 * @file map_origin_recorder.cpp
 * @brief 地图原点记录器 - 建图时运行一次，保存GPS原点到文件
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <GeographicLib/UTMUPS.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp> 
#include <fstream>
#include <iomanip>
#include <ctime>

class MapOriginRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Publisher status_pub_;
    
    // 配置参数
    std::string map_name_;
    std::string maps_database_path_;
    int min_fix_type_;
    double min_satellites_;
    
    // 状态变量
    bool origin_saved_;
    int gps_message_count_;
    
public:
    MapOriginRecorder() : origin_saved_(false), gps_message_count_(0) {
        // 加载参数
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("map_name", map_name_, "");
        pnh.param<std::string>("maps_database_path", maps_database_path_, 
                              std::string(getenv("HOME")) + "/.ros/maps");
        pnh.param<int>("min_fix_type", min_fix_type_, 4);  // RTK Fixed
        pnh.param<double>("min_satellites", min_satellites_, 8.0);
        
        // 验证参数
        if (map_name_.empty()) {
            ROS_ERROR("map_name parameter is required!");
            ros::shutdown();
            return;
        }
        
        // 创建地图数据库目录
        boost::filesystem::create_directories(maps_database_path_);
        
        // 初始化发布订阅
        gps_sub_ = nh_.subscribe("/gnss/fix", 10, &MapOriginRecorder::gpsCallback, this);
        status_pub_ = nh_.advertise<std_msgs::String>("/map_origin_status", 1, true);
        
        ROS_INFO("Map Origin Recorder started");
        ROS_INFO("  Map name: %s", map_name_.c_str());
        ROS_INFO("  Database path: %s", maps_database_path_.c_str());
        ROS_INFO("  Min fix type: %d", min_fix_type_);
        ROS_INFO("  Min satellites: %.0f", min_satellites_);
        ROS_INFO("Waiting for high-quality GPS fix...");
        
        publishStatus("Waiting for GPS fix");
    }

private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        gps_message_count_++;
        
        // 如果已经保存过原点，直接返回
        if (origin_saved_) {
            return;
        }
        
        // 检查GPS质量
        if (!isHighQualityGPS(*msg)) {
            // 每10条消息输出一次状态
            if (gps_message_count_ % 10 == 0) {
                ROS_INFO("Waiting for high-quality GPS fix... (received %d messages)", gps_message_count_);
                ROS_INFO("  Current status: %d, Required: %d", msg->status.status, min_fix_type_);
            }
            return;
        }
        
        // 保存原点
        if (saveOrigin(*msg)) {
            origin_saved_ = true;
            publishStatus("Origin saved successfully");
            
            ROS_INFO("Origin saved successfully! Shutting down recorder...");
            
            // 延迟1秒后关闭节点
            ros::Timer shutdown_timer = nh_.createTimer(ros::Duration(1.0), 
                [this](const ros::TimerEvent&) {
                    ros::shutdown();
                }, true);  // oneshot = true
        } else {
            publishStatus("Failed to save origin");
            ROS_ERROR("Failed to save origin, will try again...");
        }
    }
    
    /**
     * @brief 检查GPS是否为高质量信号
     */
    bool isHighQualityGPS(const sensor_msgs::NavSatFix& msg) {
        // 检查定位状态
        if (msg.status.status < min_fix_type_) {
            return false;
        }
        
        // 检查坐标有效性
        if (std::isnan(msg.latitude) || std::isnan(msg.longitude) || std::isnan(msg.altitude)) {
            return false;
        }
        
        // 检查坐标合理性
        if (std::abs(msg.latitude) > 90.0 || std::abs(msg.longitude) > 180.0) {
            return false;
        }
        
        // 检查协方差 (如果有的话)
        if (msg.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            double xy_var = std::max(msg.position_covariance[0], msg.position_covariance[4]);
            if (xy_var > 0.1) {  // 方差大于10cm认为精度不够
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * @brief 保存原点到YAML文件
     */
    bool saveOrigin(const sensor_msgs::NavSatFix& msg) {
        try {
            // 转换为UTM坐标
            int utm_zone;
            bool utm_north;
            double utm_x, utm_y;
            
            GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude,
                                          utm_zone, utm_north, utm_x, utm_y);
            
            // 创建YAML配置
            YAML::Node config;
            
            // 地图信息
            config["map_info"]["map_name"] = map_name_;
            config["map_info"]["creation_time"] = getCurrentTimeString();
            config["map_info"]["coordinate_system"] = "WGS84";
            config["map_info"]["datum"] = "WGS84";
            
            // GPS原点坐标
            config["gps_origin"]["latitude"] = msg.latitude;
            config["gps_origin"]["longitude"] = msg.longitude;
            config["gps_origin"]["altitude"] = msg.altitude;
            
            // UTM坐标信息
            config["utm_info"]["zone"] = utm_zone;
            config["utm_info"]["north"] = utm_north;
            config["utm_info"]["utm_x"] = utm_x;
            config["utm_info"]["utm_y"] = utm_y;
            
            // GPS质量信息
            config["gps_quality"]["fix_type"] = static_cast<int>(msg.status.status);
            config["gps_quality"]["service"] = static_cast<int>(msg.status.service);
            
            if (msg.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
                config["gps_quality"]["covariance_xx"] = msg.position_covariance[0];
                config["gps_quality"]["covariance_yy"] = msg.position_covariance[4];
                config["gps_quality"]["covariance_zz"] = msg.position_covariance[8];
            }
            
            // 元数据
            config["metadata"]["recorder_version"] = "1.0";
            config["metadata"]["ros_version"] = "noetic";  // 或者获取实际版本
            config["metadata"]["notes"] = "Generated by map_origin_recorder";
            
            // 保存到文件
            std::string file_path = maps_database_path_ + "/" + map_name_ + "_origin.yaml";
            std::ofstream file(file_path);
            if (!file.is_open()) {
                ROS_ERROR("Failed to open file for writing: %s", file_path.c_str());
                return false;
            }
            
            file << "# Map Origin Configuration File" << std::endl;
            file << "# Generated on: " << getCurrentTimeString() << std::endl;
            file << "# Map: " << map_name_ << std::endl;
            file << std::endl;
            file << config << std::endl;
            file.close();
            
            ROS_INFO("Origin saved successfully!");
            ROS_INFO("  File: %s", file_path.c_str());
            ROS_INFO("  GPS: (%.8f, %.8f, %.3f)", msg.latitude, msg.longitude, msg.altitude);
            ROS_INFO("  UTM: (%.3f, %.3f) Zone: %d%c", utm_x, utm_y, utm_zone, utm_north ? 'N' : 'S');
            ROS_INFO("  Fix type: %d", static_cast<int>(msg.status.status));
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while saving origin: %s", e.what());
            return false;
        }
    }
    
    /**
     * @brief 获取当前时间字符串
     */
    std::string getCurrentTimeString() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::gmtime(&time_t);  // 使用UTC时间
        
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }
    
    /**
     * @brief 发布状态信息
     */
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_origin_recorder");
    
    try {
        MapOriginRecorder recorder;
        ros::spin();
        
        ROS_INFO("Map Origin Recorder finished");
        
    } catch (std::exception& e) {
        ROS_ERROR("Map Origin Recorder failed: %s", e.what());
        return -1;
    }
    
    return 0;
}

/*
 * ============================================================================
 * 编译配置 (CMakeLists.txt)
 * ============================================================================
 * 
 * find_package(catkin REQUIRED COMPONENTS
 *   roscpp
 *   sensor_msgs
 *   std_msgs
 * )
 * 
 * find_package(PkgConfig REQUIRED)
 * pkg_check_modules(GeographicLib REQUIRED geographic)
 * find_package(yaml-cpp REQUIRED)
 * 
 * add_executable(map_origin_recorder src/map_origin_recorder.cpp)
 * target_link_libraries(map_origin_recorder 
 *   ${catkin_LIBRARIES} 
 *   ${GeographicLib_LIBRARIES}
 *   yaml-cpp)
 * 
 * ============================================================================
 * 使用方法
 * ============================================================================
 * 
 * 1. 建图时启动 (一次性运行):
 * rosrun your_package map_origin_recorder _map_name:=warehouse_map
 * 
 * 2. 在launch文件中使用:
 * <node pkg="your_package" type="map_origin_recorder" name="origin_recorder">
 *   <param name="map_name" value="warehouse_map"/>
 *   <param name="min_fix_type" value="4"/>  <!-- RTK Fixed -->
 *   <param name="min_satellites" value="8"/>
 * </node>
 * 
 * 3. 生成的文件格式:
 * ~/.ros/maps/warehouse_map_origin.yaml
 * 
 * ============================================================================
 * 工作流程
 * ============================================================================
 * 
 * 建图阶段:
 * 1. 启动建图系统
 * 2. 同时启动 map_origin_recorder
 * 3. 等待RTK固定解
 * 4. 自动保存原点并退出
 * 5. 继续建图过程
 * 
 * 定位阶段:
 * 1. 启动gnss_processor (use_saved_origin:=true)
 * 2. 自动加载保存的原点
 * 3. 确保坐标系一致性
 * 
 * ============================================================================
 */
