/**
 * @file enhanced_map_origin_recorder.cpp
 * @brief 地图原点记录器 v2.0
 * @author Lu Jiaqi
 * @date 2025
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <GeographicLib/UTMUPS.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp> 
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>

// RTK原始数据，屏蔽本身RTK节点
#include "chcnav/hcinspvatzcb.h"

class EnhancedMapOriginRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber rtk_sub_;
    ros::Subscriber laser_odom_sub_;
    ros::Publisher status_pub_;
    
    // 配置参数
    std::string map_name_;
    std::string maps_database_path_;
    int min_rtk_status_;           // 最小RTK状态
    double min_position_accuracy_; // 最小位置精度
    double min_heading_accuracy_;  // 最小航向精度
    int required_stable_samples_;  // 需要的稳定样本数
    bool use_laser_odom_reference_; // 是否使用激光里程计作为参考
    
    // 状态变量
    bool origin_saved_;
    int rtk_message_count_;
    int stable_sample_count_;
    
    // 稳定性检查
    struct RTKSample {
        double latitude, longitude, altitude;
        double heading2;  // 双天线航向角
        double position_accuracy;
        double heading_accuracy;
        ros::Time timestamp;
    };
    
    std::vector<RTKSample> stable_samples_;
    nav_msgs::Odometry last_laser_odom_;
    bool has_laser_odom_;
    
    // 统计信息
    double avg_position_accuracy_;
    double avg_heading_accuracy_;
    
public:
    EnhancedMapOriginRecorder() : 
        origin_saved_(false), 
        rtk_message_count_(0),
        stable_sample_count_(0),
        has_laser_odom_(false),
        avg_position_accuracy_(0.0),
        avg_heading_accuracy_(0.0) {
        
        loadParameters();
        initializePublishersSubscribers();
        
        ROS_INFO("Enhanced Map Origin Recorder started");
        ROS_INFO("  Map name: %s", map_name_.c_str());
        ROS_INFO("  Database path: %s", maps_database_path_.c_str());
        ROS_INFO("  Min RTK status: %d", min_rtk_status_);
        ROS_INFO("  Min position accuracy: %.3f m", min_position_accuracy_);
        ROS_INFO("  Min heading accuracy: %.1f°", min_heading_accuracy_);
        ROS_INFO("  Required stable samples: %d", required_stable_samples_);
        ROS_INFO("  Use laser odometry reference: %s", use_laser_odom_reference_ ? "yes" : "no");
        ROS_INFO("Waiting for high-quality RTK data...");
        
        publishStatus("Waiting for RTK data");
    }

private:
    void loadParameters() {
        ros::NodeHandle pnh("~");
        
        pnh.param<std::string>("map_name", map_name_, "");
        pnh.param<std::string>("maps_database_path", maps_database_path_, 
                              std::string(getenv("HOME")) + "/.ros/maps");
        pnh.param<int>("min_rtk_status", min_rtk_status_, 4);  // RTK固定解
        pnh.param<double>("min_position_accuracy", min_position_accuracy_, 0.02);  // 2cm
        pnh.param<double>("min_heading_accuracy", min_heading_accuracy_, 0.5);     // 0.5°
        pnh.param<int>("required_stable_samples", required_stable_samples_, 10);
        pnh.param<bool>("use_laser_odom_reference", use_laser_odom_reference_, true);
        
        // 验证参数
        if (map_name_.empty()) {
            ROS_ERROR("map_name parameter is required!");
            ros::shutdown();
            return;
        }
        
        // 创建地图数据库目录
        boost::filesystem::create_directories(maps_database_path_);
    }
    
    void initializePublishersSubscribers() {
        // 订阅RTK数据
        rtk_sub_ = nh_.subscribe("/chcnav/devpvt", 10, 
            &EnhancedMapOriginRecorder::rtkCallback, this);
        
        // 激光里程计参考
        if (use_laser_odom_reference_) {
            laser_odom_sub_ = nh_.subscribe("/laser_odom", 10, 
                &EnhancedMapOriginRecorder::laserOdomCallback, this);
        }
        
        status_pub_ = nh_.advertise<std_msgs::String>("/map_origin_status", 1, true);
    }
    
    void rtkCallback(const chcnav::hcinspvatzcb::ConstPtr& msg) {
        rtk_message_count_++;
        
        // 如果已经保存过原点，直接返回
        if (origin_saved_) {
            return;
        }
        
        // 检查RTK数据质量
        RTKSample sample;
        if (!extractRTKSample(*msg, sample)) {
            // 每50条消息输出一次状态
            if (rtk_message_count_ % 50 == 0) {
                ROS_INFO("Waiting for high-quality RTK data... (received %d messages)", rtk_message_count_);
                std::string status_msg = getQualityStatusMessage(*msg);
                publishStatus(status_msg);
                ROS_INFO("  %s", status_msg.c_str());
            }
            return;
        }
        
        // 添加到稳定样本队列
        addStableSample(sample);
        
        // 检查是否有足够的稳定样本
        if (stable_sample_count_ >= required_stable_samples_) {
            if (validateStability() && saveOriginFromStableSamples()) {
                origin_saved_ = true;
                publishStatus("Origin saved successfully");
                
                ROS_INFO("Origin saved successfully! Shutting down recorder...");
                
                // 延迟1秒后关闭节点
                ros::Timer shutdown_timer = nh_.createTimer(ros::Duration(1.0), 
                    [this](const ros::TimerEvent&) {
                        ros::shutdown();
                    }, true);
            } else {
                publishStatus("Stability validation failed, continuing...");
                ROS_WARN("Stability validation failed, clearing samples and continuing...");
                clearStableSamples();
            }
        } else {
            std::stringstream ss;
            ss << "Collecting stable samples: " << stable_sample_count_ << "/" << required_stable_samples_;
            publishStatus(ss.str());
            ROS_INFO("  %s (avg accuracy: pos=%.3fm, heading=%.2f°)", 
                     ss.str().c_str(), avg_position_accuracy_, avg_heading_accuracy_);
        }
    }
    
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        last_laser_odom_ = *msg;
        has_laser_odom_ = true;
    }
    
    bool extractRTKSample(const chcnav::hcinspvatzcb& msg, RTKSample& sample) {
        // 检查RTK状态
        if (msg.stat.size() >= 2 && msg.stat[1] < min_rtk_status_) {
            return false;
        }
        
        // 检查坐标有效性
        if (std::isnan(msg.latitude) || std::isnan(msg.longitude) || std::isnan(msg.altitude)) {
            return false;
        }
        
        // 检查姿态有效性
        if (std::isnan(msg.roll) || std::isnan(msg.pitch) || std::isnan(msg.yaw) || std::isnan(msg.heading2)) {
            return false;
        }
        
        // 提取精度信息
        double position_accuracy = 0.0;
        double heading_accuracy = 0.0;
        
        if (msg.position_stdev.size() >= 3) {
            position_accuracy = std::max(msg.position_stdev[0], msg.position_stdev[1]);
        }
        
        if (msg.euler_stdev.size() >= 3) {
            heading_accuracy = msg.euler_stdev[2];  // yaw标准差(度)
        }
        
        // 检查精度要求
        if (position_accuracy > min_position_accuracy_) {
            return false;
        }
        
        if (heading_accuracy > min_heading_accuracy_) {
            return false;
        }
        
        // 检查坐标合理性
        if (std::abs(msg.latitude) > 90.0 || std::abs(msg.longitude) > 180.0) {
            return false;
        }
        
        // 填充样本
        sample.latitude = msg.latitude;
        sample.longitude = msg.longitude;
        sample.altitude = msg.altitude;
        sample.heading2 = msg.heading2;
        sample.position_accuracy = position_accuracy;
        sample.heading_accuracy = heading_accuracy;
        sample.timestamp = msg.header.stamp;
        
        return true;
    }
    
    std::string getQualityStatusMessage(const chcnav::hcinspvatzcb& msg) {
        std::stringstream ss;
        
        if (msg.stat.size() >= 2) {
            ss << "RTK status: " << static_cast<int>(msg.stat[1]) << " (need " << min_rtk_status_ << ")";
        }
        
        if (msg.position_stdev.size() >= 2) {
            double pos_acc = std::max(msg.position_stdev[0], msg.position_stdev[1]);
            ss << ", pos_acc: " << std::fixed << std::setprecision(3) << pos_acc 
               << "m (need <" << min_position_accuracy_ << "m)";
        }
        
        if (msg.euler_stdev.size() >= 3) {
            ss << ", heading_acc: " << std::fixed << std::setprecision(2) << msg.euler_stdev[2] 
               << "° (need <" << min_heading_accuracy_ << "°)";
        }
        
        return ss.str();
    }
    
    void addStableSample(const RTKSample& sample) {
        stable_samples_.push_back(sample);
        stable_sample_count_++;
        
        // 保持最近N个样本
        if (stable_samples_.size() > static_cast<size_t>(required_stable_samples_)) {
            stable_samples_.erase(stable_samples_.begin());
            stable_sample_count_ = stable_samples_.size();
        }
        
        // 更新平均精度
        updateAverageAccuracy();
    }
    
    void updateAverageAccuracy() {
        if (stable_samples_.empty()) return;
        
        double sum_pos_acc = 0.0;
        double sum_heading_acc = 0.0;
        
        for (const auto& sample : stable_samples_) {
            sum_pos_acc += sample.position_accuracy;
            sum_heading_acc += sample.heading_accuracy;
        }
        
        avg_position_accuracy_ = sum_pos_acc / stable_samples_.size();
        avg_heading_accuracy_ = sum_heading_acc / stable_samples_.size();
    }
    
    bool validateStability() {
        if (stable_samples_.size() < static_cast<size_t>(required_stable_samples_)) {
            return false;
        }
        
        // 计算位置稳定性
        double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0, heading_sum = 0.0;
        for (const auto& sample : stable_samples_) {
            lat_sum += sample.latitude;
            lon_sum += sample.longitude;
            alt_sum += sample.altitude;
            heading_sum += sample.heading2;
        }
        
        double lat_mean = lat_sum / stable_samples_.size();
        double lon_mean = lon_sum / stable_samples_.size();
        double alt_mean = alt_sum / stable_samples_.size();
        double heading_mean = heading_sum / stable_samples_.size();
        
        // 计算标准差
        double lat_var = 0.0, lon_var = 0.0, alt_var = 0.0, heading_var = 0.0;
        for (const auto& sample : stable_samples_) {
            lat_var += (sample.latitude - lat_mean) * (sample.latitude - lat_mean);
            lon_var += (sample.longitude - lon_mean) * (sample.longitude - lon_mean);
            alt_var += (sample.altitude - alt_mean) * (sample.altitude - alt_mean);
            
            double heading_diff = sample.heading2 - heading_mean;
            // 处理角度跨越问题
            if (heading_diff > 180.0) heading_diff -= 360.0;
            if (heading_diff < -180.0) heading_diff += 360.0;
            heading_var += heading_diff * heading_diff;
        }
        
        double lat_std = sqrt(lat_var / stable_samples_.size());
        double lon_std = sqrt(lon_var / stable_samples_.size());
        double alt_std = sqrt(alt_var / stable_samples_.size());
        double heading_std = sqrt(heading_var / stable_samples_.size());
        
        // 转换为米和度
        double lat_std_m = lat_std * 111000.0;  // 纬度1度约111km
        double lon_std_m = lon_std * 111000.0 * cos(lat_mean * M_PI / 180.0);
        
        // 稳定性检查阈值
        const double max_position_std = 0.05;  // 5cm
        const double max_altitude_std = 0.1;   // 10cm  
        const double max_heading_std = 1.0;    // 1°
        
        bool stable = (lat_std_m < max_position_std) && 
                     (lon_std_m < max_position_std) && 
                     (alt_std < max_altitude_std) &&
                     (heading_std < max_heading_std);
        
        ROS_INFO("Stability validation:");
        ROS_INFO("  Lat std: %.3fm (max %.3fm) %s", lat_std_m, max_position_std, lat_std_m < max_position_std ? "OK" : "FAIL");
        ROS_INFO("  Lon std: %.3fm (max %.3fm) %s", lon_std_m, max_position_std, lon_std_m < max_position_std ? "OK" : "FAIL");
        ROS_INFO("  Alt std: %.3fm (max %.3fm) %s", alt_std, max_altitude_std, alt_std < max_altitude_std ? "OK" : "FAIL");
        ROS_INFO("  Heading std: %.2f° (max %.2f°) %s", heading_std, max_heading_std, heading_std < max_heading_std ? "OK" : "FAIL");
        
        return stable;
    }
    
    void clearStableSamples() {
        stable_samples_.clear();
        stable_sample_count_ = 0;
        avg_position_accuracy_ = 0.0;
        avg_heading_accuracy_ = 0.0;
    }
    
    bool saveOriginFromStableSamples() {
        if (stable_samples_.empty()) return false;
        
        try {
            // 计算平均值作为最终原点
            double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0, heading_sum = 0.0;
            for (const auto& sample : stable_samples_) {
                lat_sum += sample.latitude;
                lon_sum += sample.longitude;
                alt_sum += sample.altitude;
                heading_sum += sample.heading2;
            }
            
            double final_lat = lat_sum / stable_samples_.size();
            double final_lon = lon_sum / stable_samples_.size();
            double final_alt = alt_sum / stable_samples_.size();
            double final_heading = heading_sum / stable_samples_.size();
            
            // 计算UTM坐标
            int utm_zone;
            bool utm_north;
            double utm_x, utm_y;
            
            GeographicLib::UTMUPS::Forward(final_lat, final_lon,
                                          utm_zone, utm_north, utm_x, utm_y);
            
            // 处理激光里程计参考
            double laser_offset_x = 0.0, laser_offset_y = 0.0;
            bool has_laser_reference = false;
            
            if (use_laser_odom_reference_ && has_laser_odom_) {
                laser_offset_x = last_laser_odom_.pose.pose.position.x;
                laser_offset_y = last_laser_odom_.pose.pose.position.y;
                has_laser_reference = true;
                
                ROS_INFO("Using laser odometry reference: (%.2f, %.2f)", 
                         laser_offset_x, laser_offset_y);
            }
            
            // 创建YAML配置
            YAML::Node config;
            
            // 地图信息
            config["map_info"]["map_name"] = map_name_;
            config["map_info"]["creation_time"] = getCurrentTimeString();
            config["map_info"]["coordinate_system"] = "WGS84";
            config["map_info"]["rtk_system"] = "dual_antenna";
            
            // GPS原点坐标（平均值）
            config["gps_origin"]["latitude"] = final_lat;
            config["gps_origin"]["longitude"] = final_lon;
            config["gps_origin"]["altitude"] = final_alt;
            config["gps_origin"]["heading"] = final_heading;
            
            // UTM坐标信息
            config["utm_info"]["zone"] = utm_zone;
            config["utm_info"]["north"] = utm_north;
            
            if (has_laser_reference) {
                // 调整UTM原点以对齐激光里程计
                config["utm_info"]["utm_x"] = utm_x - laser_offset_x;
                config["utm_info"]["utm_y"] = utm_y - laser_offset_y;
            } else {
                config["utm_info"]["utm_x"] = utm_x;
                config["utm_info"]["utm_y"] = utm_y;
            }
            
            // 激光里程计参考信息
            if (has_laser_reference) {
                config["laser_reference"]["used"] = true;
                config["laser_reference"]["offset_x"] = laser_offset_x;
                config["laser_reference"]["offset_y"] = laser_offset_y;
            } else {
                config["laser_reference"]["used"] = false;
            }
            
            // RTK质量信息
            config["rtk_quality"]["samples_used"] = static_cast<int>(stable_samples_.size());
            config["rtk_quality"]["avg_position_accuracy"] = avg_position_accuracy_;
            config["rtk_quality"]["avg_heading_accuracy"] = avg_heading_accuracy_;
            config["rtk_quality"]["min_rtk_status"] = min_rtk_status_;
            
            // 时间戳信息
            config["timestamps"]["first_sample"] = stable_samples_.front().timestamp.toSec();
            config["timestamps"]["last_sample"] = stable_samples_.back().timestamp.toSec();
            config["timestamps"]["duration"] = (stable_samples_.back().timestamp - stable_samples_.front().timestamp).toSec();
            
            // 元数据
            config["metadata"]["recorder_version"] = "2.0_enhanced";
            config["metadata"]["ros_version"] = "noetic";
            config["metadata"]["notes"] = "Generated by enhanced_map_origin_recorder with RTK dual-antenna";
            
            // 保存到文件
            std::string file_path = maps_database_path_ + "/" + map_name_ + "_origin.yaml";
            std::ofstream file(file_path);
            if (!file.is_open()) {
                ROS_ERROR("Failed to open file for writing: %s", file_path.c_str());
                return false;
            }
            
            file << "# Enhanced RTK Map Origin Configuration File" << std::endl;
            file << "# Generated on: " << getCurrentTimeString() << std::endl;
            file << "# Map: " << map_name_ << std::endl;
            file << "# RTK System: Dual-antenna with " << stable_samples_.size() << " stable samples" << std::endl;
            file << std::endl;
            file << config << std::endl;
            file.close();
            
            ROS_INFO("High-quality RTK origin saved successfully!");
            ROS_INFO("  File: %s", file_path.c_str());
            ROS_INFO("  Final GPS: (%.8f, %.8f, %.3f)", final_lat, final_lon, final_alt);
            ROS_INFO("  Final Heading: %.2f°", final_heading);
            ROS_INFO("  UTM: (%.3f, %.3f) Zone: %d%c", utm_x, utm_y, utm_zone, utm_north ? 'N' : 'S');
            ROS_INFO("  Quality: pos_acc=%.3fm, heading_acc=%.2f°", avg_position_accuracy_, avg_heading_accuracy_);
            ROS_INFO("  Samples used: %zu over %.1fs", stable_samples_.size(), 
                     (stable_samples_.back().timestamp - stable_samples_.front().timestamp).toSec());
            
            if (has_laser_reference) {
                ROS_INFO("  Laser reference: (%.2f, %.2f)", laser_offset_x, laser_offset_y);
                ROS_INFO("  Adjusted UTM origin: (%.3f, %.3f)", utm_x - laser_offset_x, utm_y - laser_offset_y);
            }
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while saving RTK origin: %s", e.what());
            return false;
        }
    }
    
    std::string getCurrentTimeString() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::gmtime(&time_t);
        
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_map_origin_recorder");
    
    try {
        EnhancedMapOriginRecorder recorder;
        ros::spin();
        
        ROS_INFO("Enhanced Map Origin Recorder finished");
        
    } catch (std::exception& e) {
        ROS_ERROR("Enhanced Map Origin Recorder failed: %s", e.what());
        return -1;
    }
    
    return 0;
}
