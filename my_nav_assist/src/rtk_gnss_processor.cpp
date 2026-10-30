/**
 * @file rtk_gnss_processor.cpp
 * @brief gnss里程计 v2.0
 * @author Lu Jiaqi
 * @date 2025
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <GeographicLib/UTMUPS.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <cmath>

// RTK原始数据，忽略RTK本身节点
#include "chcnav/hcinspvatzcb.h"

class RTKGNSSProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber rtk_sub_;
    ros::Subscriber laser_odom_sub_;
    ros::Publisher gnss_odom_pub_;
    ros::Publisher gnss_pose_pub_;
    ros::Publisher gnss_imu_pub_;
    ros::Publisher status_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 配置参数
    std::string map_frame_;
    std::string base_link_frame_;
    double min_gps_accuracy_;
    bool use_laser_for_origin_init_;
    bool publish_rtk_orientation_;
    int min_rtk_status_;
    
    // 安装补偿参数
    bool enable_installation_compensation_;
    double yaw_offset_deg_;
    double heading_offset_deg_;
    double heading2_offset_deg_;
    bool reverse_yaw_sign_;
    bool reverse_heading_sign_;
    
    // 地图原点和变换参数
    std::string map_name_;
    std::string maps_database_path_;
    bool use_saved_origin_;
    bool auto_initialize_origin_;
    
    // 坐标变换参数
    bool origin_set_;
    double origin_lat_, origin_lon_, origin_alt_;
    double origin_utm_x_, origin_utm_y_;
    int utm_zone_;
    bool utm_north_;
    
    // 坐标系旋转参数
    bool use_heading_alignment_;
    double map_heading_offset_;
    bool heading_offset_set_;
    
    // 状态跟踪
    nav_msgs::Odometry last_gnss_odom_;
    nav_msgs::Odometry last_laser_odom_;
    bool has_last_gnss_;
    bool has_laser_odom_;
    
    // 诊断和监控
    ros::Time last_rtk_time_;
    int rtk_message_count_;
    double last_position_accuracy_;
    double last_heading_accuracy_;
    ros::Timer param_update_timer_;

public:
    RTKGNSSProcessor() : 
        tf_listener_(tf_buffer_),
        origin_set_(false), 
        has_last_gnss_(false),
        has_laser_odom_(false),
        heading_offset_set_(false),
        rtk_message_count_(0),
        last_position_accuracy_(0.0),
        last_heading_accuracy_(0.0) {
        
        loadParameters();
        initializePublishersSubscribers();
        
        // 尝试加载保存的原点和变换
        if (use_saved_origin_ && !map_name_.empty()) {
            loadSavedOriginAndTransform();
        }
        
        // 参数更新定时器
        param_update_timer_ = nh_.createTimer(ros::Duration(1.0), 
            &RTKGNSSProcessor::updateParametersCallback, this);
        
        logConfiguration();
    }

private:
    void loadParameters() {
        ros::NodeHandle pnh("~");
        
        // 基本参数
        pnh.param<std::string>("map_frame", map_frame_, "map");
        pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        pnh.param<double>("min_gps_accuracy", min_gps_accuracy_, 0.05);
        pnh.param<bool>("use_laser_for_origin_init", use_laser_for_origin_init_, true);
        pnh.param<bool>("publish_rtk_orientation", publish_rtk_orientation_, true);
        pnh.param<int>("min_rtk_status", min_rtk_status_, 4);
        pnh.param<bool>("use_heading_alignment", use_heading_alignment_, true);
        
        // 安装补偿参数
        pnh.param<bool>("enable_installation_compensation", enable_installation_compensation_, false);
        pnh.param<double>("yaw_offset_deg", yaw_offset_deg_, 0.0);
        pnh.param<double>("heading_offset_deg", heading_offset_deg_, 0.0);
        pnh.param<double>("heading2_offset_deg", heading2_offset_deg_, 0.0);
        pnh.param<bool>("reverse_yaw_sign", reverse_yaw_sign_, false);
        pnh.param<bool>("reverse_heading_sign", reverse_heading_sign_, false);
        
        // 地图和原点参数
        pnh.param<std::string>("map_name", map_name_, "");
        pnh.param<std::string>("maps_database_path", maps_database_path_, 
                              std::string(getenv("HOME")) + "/.ros/maps");
        pnh.param<bool>("use_saved_origin", use_saved_origin_, false);
        pnh.param<bool>("auto_initialize_origin", auto_initialize_origin_, true);
        
        // 创建目录
        boost::filesystem::create_directories(maps_database_path_);
    }
    
    void initializePublishersSubscribers() {
        // 订阅RTK数据
        rtk_sub_ = nh_.subscribe("/chcnav/devpvt", 10, &RTKGNSSProcessor::rtkCallback, this);
        
        // 激光里程计用于原点初始化参考
        if (use_laser_for_origin_init_) {
            laser_odom_sub_ = nh_.subscribe("/laser_odom", 10, 
                &RTKGNSSProcessor::laserOdomCallback, this);
        }
        
        // 发布器
        gnss_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gnss_odometry", 10);
        gnss_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gnss_pose", 10);
        status_pub_ = nh_.advertise<std_msgs::String>("/gnss_status", 1, true);
        
        if (publish_rtk_orientation_) {
            gnss_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/gnss_imu", 10);
        }
    }
    
    void logConfiguration() {
        ROS_INFO("RTK GNSS Processor started");
        ROS_INFO("  Architecture: Single EKF (map≡odom)");
        ROS_INFO("  Map frame: %s", map_frame_.c_str());
        ROS_INFO("  Map name: %s", map_name_.empty() ? "none" : map_name_.c_str());
        ROS_INFO("  Min RTK status: %d (4=固定解)", min_rtk_status_);
        ROS_INFO("  Min position accuracy: %.3f m", min_gps_accuracy_);
        ROS_INFO("  Use heading alignment: %s", use_heading_alignment_ ? "yes" : "no");
        ROS_INFO("  Publish RTK orientation: %s", publish_rtk_orientation_ ? "yes" : "no");
        
        ROS_INFO("  Installation compensation: %s", enable_installation_compensation_ ? "ENABLED" : "disabled");
        if (enable_installation_compensation_) {
            ROS_INFO("    yaw offset: %.1f°", yaw_offset_deg_);
            ROS_INFO("    heading offset: %.1f°", heading_offset_deg_);
            ROS_INFO("    heading2 offset: %.1f°", heading2_offset_deg_);
            ROS_INFO("    reverse yaw: %s", reverse_yaw_sign_ ? "yes" : "no");
            ROS_INFO("    reverse heading: %s", reverse_heading_sign_ ? "yes" : "no");
        }
    }
    
    void updateParametersCallback(const ros::TimerEvent&) {
        ros::NodeHandle pnh("~");
        
        // 动态更新补偿参数
        bool old_enable = enable_installation_compensation_;
        pnh.getParamCached("enable_installation_compensation", enable_installation_compensation_);
        pnh.getParamCached("yaw_offset_deg", yaw_offset_deg_);
        pnh.getParamCached("heading_offset_deg", heading_offset_deg_);
        pnh.getParamCached("heading2_offset_deg", heading2_offset_deg_);
        pnh.getParamCached("reverse_yaw_sign", reverse_yaw_sign_);
        pnh.getParamCached("reverse_heading_sign", reverse_heading_sign_);
        
        if (old_enable != enable_installation_compensation_) {
            ROS_INFO("Installation compensation %s", 
                     enable_installation_compensation_ ? "ENABLED" : "DISABLED");
        }
    }
    
    void rtkCallback(const chcnav::hcinspvatzcb::ConstPtr& msg) {
        rtk_message_count_++;
        last_rtk_time_ = msg->header.stamp;
        
        // 应用安装补偿
        chcnav::hcinspvatzcb compensated_msg = *msg;
        applyInstallationCompensation(compensated_msg);
        
        // 检查RTK数据质量
        if (!isValidRTKData(compensated_msg)) return;
        
        // 提取精度信息
        double xy_variance, z_variance, heading_variance;
        extractCovarianceFromRTK(compensated_msg, xy_variance, z_variance, heading_variance);
        
        // 原点初始化
        if (!origin_set_) {
            if (auto_initialize_origin_) {
                initializeOriginWithRTK(compensated_msg);
            } else {
                publishStatus("Origin not set and auto initialization disabled");
                return;
            }
            if (!origin_set_) return;
        }
        
        // 航向对齐初始化
        if (use_heading_alignment_ && !heading_offset_set_) {
            initializeHeadingAlignment(compensated_msg);
        }
        
        // 转换到地图坐标系（包含旋转变换）
        double map_x, map_y, map_z;
        double map_roll, map_pitch, map_yaw;
        if (!convertToMapFrameWithRotation(compensated_msg, map_x, map_y, map_z, 
                                          map_roll, map_pitch, map_yaw)) return;
        
        // 创建和发布消息
        nav_msgs::Odometry gnss_odom = createRTKOdometry(compensated_msg, map_x, map_y, map_z,
                                                         map_roll, map_pitch, map_yaw,
                                                         xy_variance, z_variance, heading_variance);
        
        geometry_msgs::PoseWithCovarianceStamped gnss_pose = 
            createRTKPose(compensated_msg, map_x, map_y, map_z, map_roll, map_pitch, map_yaw,
                         xy_variance, z_variance, heading_variance);
        
        publishGNSSOdometry(gnss_odom);
        publishGNSSPose(gnss_pose);
        
        if (publish_rtk_orientation_) {
            sensor_msgs::Imu rtk_imu = createRTKIMU(compensated_msg, map_roll, map_pitch, map_yaw, heading_variance);
            publishGNSSIMU(rtk_imu);
        }
        
        // 状态报告
        publishStatusDiagnostics(compensated_msg);
    }
    
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        last_laser_odom_ = *msg;
        has_laser_odom_ = true;
    }
    
    void applyInstallationCompensation(chcnav::hcinspvatzcb& msg) {
        if (!enable_installation_compensation_) return;
        
        double yaw = msg.yaw;
        double heading = msg.heading;
        double heading2 = msg.heading2;
        
        // 符号补偿
        if (reverse_yaw_sign_) yaw = -yaw;
        if (reverse_heading_sign_) {
            heading = -heading;
            heading2 = -heading2;
        }
        
        // 偏移补偿
        yaw += yaw_offset_deg_;
        heading += heading_offset_deg_;
        heading2 += heading2_offset_deg_;
        
        // 角度范围规范化
        while (yaw > 180.0) yaw -= 360.0;
        while (yaw < -180.0) yaw += 360.0;
        
        while (heading >= 360.0) heading -= 360.0;
        while (heading < 0.0) heading += 360.0;
        
        while (heading2 >= 360.0) heading2 -= 360.0;
        while (heading2 < 0.0) heading2 += 360.0;
        
        // 更新消息
        msg.yaw = yaw;
        msg.heading = heading;
        msg.heading2 = heading2;
    }
    
    bool isValidRTKData(const chcnav::hcinspvatzcb& msg) {
        // 检查RTK状态
        if (msg.stat.size() >= 2 && msg.stat[1] < min_rtk_status_) {
            ROS_DEBUG_THROTTLE(2.0, "RTK status insufficient: %d < %d", msg.stat[1], min_rtk_status_);
            return false;
        }
        
        // 检查坐标有效性
        if (std::isnan(msg.latitude) || std::isnan(msg.longitude) || std::isnan(msg.altitude)) {
            return false;
        }
        
        // 检查姿态有效性
        if (std::isnan(msg.roll) || std::isnan(msg.pitch) || std::isnan(msg.yaw)) {
            return false;
        }
        
        // 检查位置精度
        if (msg.position_stdev.size() >= 2) {
            double pos_accuracy = std::max(msg.position_stdev[0], msg.position_stdev[1]);
            if (pos_accuracy > min_gps_accuracy_) {
                ROS_DEBUG_THROTTLE(2.0, "RTK position accuracy insufficient: %.3fm > %.3fm", 
                         pos_accuracy, min_gps_accuracy_);
                return false;
            }
        }
        
        return true;
    }
    
    void extractCovarianceFromRTK(const chcnav::hcinspvatzcb& msg,
                                 double& xy_variance, double& z_variance, double& heading_variance) {
        // 位置方差
        if (msg.position_stdev.size() >= 3) {
            xy_variance = std::max(msg.position_stdev[0] * msg.position_stdev[0],
                                  msg.position_stdev[1] * msg.position_stdev[1]);
            z_variance = msg.position_stdev[2] * msg.position_stdev[2];
        } else {
            xy_variance = 0.04;  // 20cm std
            z_variance = 0.25;   // 50cm std
        }
        
        // 航向角方差
        if (msg.euler_stdev.size() >= 3) {
            double heading_std_deg = msg.euler_stdev[2];
            heading_variance = (heading_std_deg * M_PI / 180.0) * (heading_std_deg * M_PI / 180.0);
        } else {
            heading_variance = 0.01; // ~6° std
        }
        
        // 确保合理范围
        xy_variance = std::max(std::min(xy_variance, 1.0), 0.0001);
        z_variance = std::max(std::min(z_variance, 4.0), 0.0025);
        heading_variance = std::max(std::min(heading_variance, 0.1), 0.0001);
        
        // 保存诊断信息
        last_position_accuracy_ = sqrt(xy_variance);
        last_heading_accuracy_ = sqrt(heading_variance) * 180.0 / M_PI;
    }
    
    void initializeOriginWithRTK(const chcnav::hcinspvatzcb& msg) {
        if (use_laser_for_origin_init_ && has_laser_odom_) {
            initializeOriginWithLaserReference(msg);
        } else {
            setOriginDirectly(msg);
        }
        
        if (origin_set_) {
            saveOriginToFile(msg);
        }
    }
    
    void initializeOriginWithLaserReference(const chcnav::hcinspvatzcb& msg) {
        try {
            double laser_x = last_laser_odom_.pose.pose.position.x;
            double laser_y = last_laser_odom_.pose.pose.position.y;
            
            int utm_zone;
            bool utm_north;
            double rtk_utm_x, rtk_utm_y;
            
            GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude,
                                          utm_zone, utm_north, rtk_utm_x, rtk_utm_y);
            
            origin_utm_x_ = rtk_utm_x - laser_x;
            origin_utm_y_ = rtk_utm_y - laser_y;
            utm_zone_ = utm_zone;
            utm_north_ = utm_north;
            origin_alt_ = msg.altitude;
            
            GeographicLib::UTMUPS::Reverse(utm_zone_, utm_north_, origin_utm_x_, origin_utm_y_,
                                          origin_lat_, origin_lon_);
            
            origin_set_ = true;
            
            ROS_INFO("RTK origin initialized with laser reference:");
            ROS_INFO("  RTK position: (%.6f, %.6f)", msg.latitude, msg.longitude);
            ROS_INFO("  Laser position: (%.2f, %.2f)", laser_x, laser_y);
            ROS_INFO("  Calculated origin: (%.6f, %.6f)", origin_lat_, origin_lon_);
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to initialize with laser reference: %s", e.what());
            setOriginDirectly(msg);
        }
    }
    
    void setOriginDirectly(const chcnav::hcinspvatzcb& msg) {
        origin_lat_ = msg.latitude;
        origin_lon_ = msg.longitude;
        origin_alt_ = msg.altitude;
        
        try {
            GeographicLib::UTMUPS::Forward(origin_lat_, origin_lon_,
                                          utm_zone_, utm_north_, 
                                          origin_utm_x_, origin_utm_y_);
            origin_set_ = true;
            
            ROS_INFO("RTK origin set directly: (%.6f, %.6f, %.1f)", 
                     origin_lat_, origin_lon_, origin_alt_);
                     
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to set RTK origin: %s", e.what());
        }
    }
    
    void initializeHeadingAlignment(const chcnav::hcinspvatzcb& msg) {
        if (has_laser_odom_) {
            tf2::Quaternion laser_q;
            tf2::fromMsg(last_laser_odom_.pose.pose.orientation, laser_q);
            double laser_roll, laser_pitch, laser_yaw;
            tf2::Matrix3x3(laser_q).getRPY(laser_roll, laser_pitch, laser_yaw);
            
            double rtk_yaw_rad = msg.heading2 * M_PI / 180.0;
            map_heading_offset_ = laser_yaw - rtk_yaw_rad;
            
            heading_offset_set_ = true;
            
            ROS_INFO("Heading alignment initialized:");
            ROS_INFO("  Laser yaw: %.2f°", laser_yaw * 180.0 / M_PI);
            ROS_INFO("  RTK heading2: %.2f°", msg.heading2);
            ROS_INFO("  Offset: %.2f°", map_heading_offset_ * 180.0 / M_PI);
        } else {
            map_heading_offset_ = 0.0;
            heading_offset_set_ = true;
        }
    }
    
    bool convertToMapFrameWithRotation(const chcnav::hcinspvatzcb& msg,
                                      double& x, double& y, double& z,
                                      double& roll, double& pitch, double& yaw) {
        try {
            // 1. GPS到UTM转换
            int zone;
            bool north;
            double utm_x, utm_y;
            
            GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude,
                                          zone, north, utm_x, utm_y);
            
            // 2. UTM到本地坐标（平移）
            double local_x = utm_x - origin_utm_x_;
            double local_y = utm_y - origin_utm_y_;
            z = msg.altitude - origin_alt_;
            
            // 3. 姿态转换
            roll = msg.roll * M_PI / 180.0;
            pitch = msg.pitch * M_PI / 180.0;
            yaw = msg.heading2 * M_PI / 180.0; //ENU坐标系
            
            //转变成FLU坐标系
            yaw-=M_PI/2;
            
            // 4. 应用航向对齐旋转
            if (use_heading_alignment_ && heading_offset_set_) {
                yaw += map_heading_offset_;
                
                // 同时旋转位置坐标
                double cos_offset = cos(map_heading_offset_-M_PI/2);
                double sin_offset = sin(map_heading_offset_-M_PI/2);
                
                x = local_x * cos_offset - local_y * sin_offset;
                y = local_x * sin_offset + local_y * cos_offset;
            } else {
                double cos_offset = cos(-M_PI/2);
                double sin_offset = sin(-M_PI/2);
                x = local_x * cos_offset - local_y * sin_offset;
                y = local_x * sin_offset + local_y * cos_offset;
                //x = local_x;
                //y = local_y;
            }
            
            // 5. 角度规范化
            while (yaw > M_PI) yaw -= 2.0 * M_PI;
            while (yaw < -M_PI) yaw += 2.0 * M_PI;
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(1.0, "RTK coordinate conversion failed: %s", e.what());
            return false;
        }
    }
    
    nav_msgs::Odometry createRTKOdometry(const chcnav::hcinspvatzcb& msg,
                                        double x, double y, double z,
                                        double roll, double pitch, double yaw,
                                        double xy_variance, double z_variance, double heading_variance) {
        nav_msgs::Odometry odom;
        
        odom.header.stamp = msg.header.stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_link_frame_;
        
        // 位置
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        
        //yaw+=90;
        // 姿态
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        odom.pose.pose.orientation = tf2::toMsg(q);
        
        // 位置协方差
        std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
        odom.pose.covariance[0] = xy_variance;     // x
        odom.pose.covariance[7] = xy_variance;     // y
        odom.pose.covariance[14] = z_variance;     // z
        odom.pose.covariance[21] = 0.01;           // roll
        odom.pose.covariance[28] = 0.01;           // pitch
        odom.pose.covariance[35] = heading_variance; // yaw
        
        // 速度（从ENU速度转换）
        if (msg.enu_velocity.x != 0 || msg.enu_velocity.y != 0) {
            odom.twist.twist.linear.x = msg.enu_velocity.x;
            odom.twist.twist.linear.y = msg.enu_velocity.y;
            odom.twist.twist.linear.z = msg.enu_velocity.z;
        }
        
        // 角速度
        if (msg.vehicle_angular_velocity.x != 0 || msg.vehicle_angular_velocity.y != 0 || msg.vehicle_angular_velocity.z != 0) {
            odom.twist.twist.angular.x = msg.vehicle_angular_velocity.x * M_PI / 180.0;
            odom.twist.twist.angular.y = msg.vehicle_angular_velocity.y * M_PI / 180.0;
            odom.twist.twist.angular.z = msg.vehicle_angular_velocity.z * M_PI / 180.0;
        }
        
        // 速度协方差
        std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
        if (msg.enu_velocity_stdev.size() >= 3) {
            odom.twist.covariance[0] = msg.enu_velocity_stdev[0] * msg.enu_velocity_stdev[0];
            odom.twist.covariance[7] = msg.enu_velocity_stdev[1] * msg.enu_velocity_stdev[1];
            odom.twist.covariance[14] = msg.enu_velocity_stdev[2] * msg.enu_velocity_stdev[2];
        } else {
            odom.twist.covariance[0] = xy_variance * 0.01;
            odom.twist.covariance[7] = xy_variance * 0.01;
            odom.twist.covariance[14] = z_variance * 0.01;
        }
        odom.twist.covariance[35] = heading_variance * 0.1;
        
        return odom;
    }
    
    geometry_msgs::PoseWithCovarianceStamped createRTKPose(const chcnav::hcinspvatzcb& msg,
                                                           double x, double y, double z,
                                                           double roll, double pitch, double yaw,
                                                           double xy_variance, double z_variance, double heading_variance) {
        geometry_msgs::PoseWithCovarianceStamped pose;
        
        pose.header.stamp = msg.header.stamp;
        pose.header.frame_id = map_frame_;
        
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.pose.pose.orientation = tf2::toMsg(q);
        
        std::fill(pose.pose.covariance.begin(), pose.pose.covariance.end(), 0.0);
        pose.pose.covariance[0] = xy_variance;
        pose.pose.covariance[7] = xy_variance;
        pose.pose.covariance[14] = z_variance;
        pose.pose.covariance[21] = 0.01;
        pose.pose.covariance[28] = 0.01;
        pose.pose.covariance[35] = heading_variance;
        
        return pose;
    }
    
    sensor_msgs::Imu createRTKIMU(const chcnav::hcinspvatzcb& msg,
                                  double roll, double pitch, double yaw,
                                  double heading_variance) {
        sensor_msgs::Imu imu;
        
        imu.header.stamp = msg.header.stamp;
        imu.header.frame_id = base_link_frame_;
        
        // 姿态
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        imu.orientation = tf2::toMsg(q);
        
        // 姿态协方差
        std::fill(imu.orientation_covariance.begin(), imu.orientation_covariance.end(), 0.0);
        imu.orientation_covariance[0] = 0.01;  // roll
        imu.orientation_covariance[4] = 0.01;  // pitch
        imu.orientation_covariance[8] = heading_variance; // yaw
        
        // 角速度
        if (msg.raw_angular_velocity.x != 0 || msg.raw_angular_velocity.y != 0 || msg.raw_angular_velocity.z != 0) {
            imu.angular_velocity.x = msg.raw_angular_velocity.x * M_PI / 180.0;
            imu.angular_velocity.y = msg.raw_angular_velocity.y * M_PI / 180.0;
            imu.angular_velocity.z = msg.raw_angular_velocity.z * M_PI / 180.0;
        }
        
        // 线性加速度
        if (msg.raw_acceleration.x != 0 || msg.raw_acceleration.y != 0 || msg.raw_acceleration.z != 0) {
            imu.linear_acceleration.x = msg.raw_acceleration.x;
            imu.linear_acceleration.y = msg.raw_acceleration.y;
            imu.linear_acceleration.z = msg.raw_acceleration.z;
        }
        
        // 协方差（固定值）
        std::fill(imu.angular_velocity_covariance.begin(), imu.angular_velocity_covariance.end(), 0.0);
        imu.angular_velocity_covariance[0] = 0.01;
        imu.angular_velocity_covariance[4] = 0.01;
        imu.angular_velocity_covariance[8] = 0.01;
        
        std::fill(imu.linear_acceleration_covariance.begin(), imu.linear_acceleration_covariance.end(), 0.0);
        imu.linear_acceleration_covariance[0] = 0.02;
        imu.linear_acceleration_covariance[4] = 0.02;
        imu.linear_acceleration_covariance[8] = 0.02;
        
        return imu;
    }
    
    void publishGNSSOdometry(const nav_msgs::Odometry& odom) {
        gnss_odom_pub_.publish(odom);
        last_gnss_odom_ = odom;
        has_last_gnss_ = true;
    }
    
    void publishGNSSPose(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        gnss_pose_pub_.publish(pose);
    }
    
    void publishGNSSIMU(const sensor_msgs::Imu& imu) {
        gnss_imu_pub_.publish(imu);
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }
    
    void publishStatusDiagnostics(const chcnav::hcinspvatzcb& msg) {
        if (rtk_message_count_ % 100 == 0) {
            std::stringstream ss;
            ss << "RTK Status: " << rtk_message_count_ << " messages processed. ";
            ss << "Position accuracy: " << std::fixed << std::setprecision(3) << last_position_accuracy_ << "m, ";
            ss << "Heading accuracy: " << std::fixed << std::setprecision(2) << last_heading_accuracy_ << "°, ";
            if (msg.stat.size() >= 2) {
                ss << "RTK status: " << static_cast<int>(msg.stat[1]);
            }
            publishStatus(ss.str());
        }
    }
    
    bool loadSavedOriginAndTransform() {
        std::string origin_file = maps_database_path_ + "/" + map_name_ + "_origin.yaml";
        
        if (!boost::filesystem::exists(origin_file)) {
            ROS_WARN("Origin file not found: %s", origin_file.c_str());
            return false;
        }
        
        try {
            YAML::Node config = YAML::LoadFile(origin_file);
            
            origin_lat_ = config["gps_origin"]["latitude"].as<double>();
            origin_lon_ = config["gps_origin"]["longitude"].as<double>();
            origin_alt_ = config["gps_origin"]["altitude"].as<double>();
            
            utm_zone_ = config["utm_info"]["zone"].as<int>();
            utm_north_ = config["utm_info"]["north"].as<bool>();
            origin_utm_x_ = config["utm_info"]["utm_x"].as<double>();
            origin_utm_y_ = config["utm_info"]["utm_y"].as<double>();
            
            if (config["transform_info"]) {
                map_heading_offset_ = config["transform_info"]["heading_offset"].as<double>();
                heading_offset_set_ = true;
            }
            
            origin_set_ = true;
            
            ROS_INFO("Loaded saved RTK origin and transform from: %s", origin_file.c_str());
            ROS_INFO("  GPS origin: (%.6f, %.6f, %.1f)", origin_lat_, origin_lon_, origin_alt_);
            ROS_INFO("  UTM origin: (%.2f, %.2f) Zone: %d%c", 
                     origin_utm_x_, origin_utm_y_, utm_zone_, utm_north_ ? 'N' : 'S');
            if (heading_offset_set_) {
                ROS_INFO("  Heading offset: %.2f°", map_heading_offset_ * 180.0 / M_PI);
            }
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load origin file: %s", e.what());
            return false;
        }
    }
    
    void saveOriginToFile(const chcnav::hcinspvatzcb& msg) {
        if (map_name_.empty()) return;
        
        try {
            YAML::Node config;
            
            // 地图信息
            config["map_info"]["map_name"] = map_name_;
            config["map_info"]["creation_time"] = getCurrentTimeString();
            config["map_info"]["coordinate_system"] = "WGS84";
            
            // GPS原点
            config["gps_origin"]["latitude"] = origin_lat_;
            config["gps_origin"]["longitude"] = origin_lon_;
            config["gps_origin"]["altitude"] = origin_alt_;
            
            // UTM信息
            config["utm_info"]["zone"] = utm_zone_;
            config["utm_info"]["north"] = utm_north_;
            config["utm_info"]["utm_x"] = origin_utm_x_;
            config["utm_info"]["utm_y"] = origin_utm_y_;
            
            // 变换信息
            if (heading_offset_set_) {
                config["transform_info"]["heading_offset"] = map_heading_offset_;
            }
            
            // RTK质量信息
            if (msg.stat.size() >= 2) {
                config["rtk_quality"]["status"] = static_cast<int>(msg.stat[1]);
            }
            config["rtk_quality"]["position_accuracy"] = last_position_accuracy_;
            config["rtk_quality"]["heading_accuracy"] = last_heading_accuracy_;
            
            std::string file_path = maps_database_path_ + "/" + map_name_ + "_origin.yaml";
            std::ofstream file(file_path);
            if (file.is_open()) {
                file << "# RTK GNSS Origin Configuration" << std::endl;
                file << "# Generated on: " << getCurrentTimeString() << std::endl;
                file << config << std::endl;
                file.close();
                
                ROS_INFO("Saved RTK origin to: %s", file_path.c_str());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to save origin file: %s", e.what());
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtk_gnss_processor");
    
    try {
        RTKGNSSProcessor processor;
        ros::spin();
    } catch (std::exception& e) {
        ROS_ERROR("RTK GNSS Processor failed: %s", e.what());
        return -1;
    }
    
    return 0;
}
