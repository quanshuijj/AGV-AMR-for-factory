/**
 * @file gnss_processor.cpp
 * @brief 修改版GNSS处理器 - 读取保存的原点坐标
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <GeographicLib/UTMUPS.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp> 
#include <cmath>

class SimpleGNSSProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 配置参数
    std::string map_frame_;
    std::string base_link_frame_;
    double gps_xy_stdev_;
    double gps_z_stdev_;
    
    // 地图相关参数
    std::string map_name_;
    std::string maps_database_path_;
    bool use_saved_origin_;
    
    // 原点状态
    bool origin_set_;
    double origin_lat_, origin_lon_, origin_alt_;
    double origin_utm_x_, origin_utm_y_;
    int utm_zone_;
    bool utm_north_;
    
    // 速度计算
    nav_msgs::Odometry last_odom_;
    bool has_last_odom_;
    
    ros::Timer param_timer_;

public:
    SimpleGNSSProcessor() : origin_set_(false), has_last_odom_(false) {
        // 加载参数
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("map_frame", map_frame_, "map");
        pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        pnh.param<double>("gps_xy_stdev", gps_xy_stdev_, 0.02);
        pnh.param<double>("gps_z_stdev", gps_z_stdev_, 0.05);
        
        // 地图和原点相关参数
        pnh.param<std::string>("map_name", map_name_, "");
        pnh.param<std::string>("maps_database_path", maps_database_path_, 
                              std::string(getenv("HOME")) + "/.ros/maps");
        pnh.param<bool>("use_saved_origin", use_saved_origin_, false);
        
        // 创建地图数据库目录
        boost::filesystem::create_directories(maps_database_path_);
        
        // 初始化发布订阅
        gps_sub_ = nh_.subscribe("/gnss/fix", 10, &SimpleGNSSProcessor::gpsCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gnss_odometry", 10);
        
        // 尝试加载保存的原点
        if (use_saved_origin_ && !map_name_.empty()) {
            loadSavedOrigin();
        }
        
        // 构造函数中增加：
        param_timer_ = nh_.createTimer(ros::Duration(1.0),  // 1秒刷新周期
              &SimpleGNSSProcessor::paramTimerCallback, this);
        
        ROS_INFO("Simple GNSS Processor started");
        ROS_INFO("  Map name: %s", map_name_.empty() ? "none" : map_name_.c_str());
        ROS_INFO("  Use saved origin: %s", use_saved_origin_ ? "yes" : "no");
        ROS_INFO("  Maps database: %s", maps_database_path_.c_str());
    }

private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // 检查GPS有效性
        if (!isValidGPS(*msg)) return;
        
        // 如果没有设置原点，设置首次GPS为原点
        if (!origin_set_) {
            setOrigin(*msg);
            return;
        }
        
        // 转换GPS到局部坐标
        double local_x, local_y, local_z;
        if (!convertToLocal(*msg, local_x, local_y, local_z)) return;
        ROS_INFO("  当前局部坐标: (%.6f, %.6f, %.6f)", local_x, local_y, local_z);
        // 创建并发布里程计
        nav_msgs::Odometry odom = createOdometry(*msg, local_x, local_y, local_z);
        publishOdometry(odom);
        //publishTransform(odom);
    }
    
    bool isValidGPS(const sensor_msgs::NavSatFix& msg) {
        return msg.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
               !std::isnan(msg.latitude) && !std::isnan(msg.longitude);
    }
    
    
    
// 实现定时回调
    void paramTimerCallback(const ros::TimerEvent&) {
        nh_.getParamCached("gps_xy_stdev", gps_xy_stdev_);
        nh_.getParamCached("gps_z_stdev", gps_z_stdev_);
        ROS_DEBUG_THROTTLE(10, "更新参数: xy_stdev=%.4f, z_stdev=%.4f", 
                          gps_xy_stdev_, gps_z_stdev_);
    }
    
    /**
     * @brief 加载保存的原点坐标
     */
    bool loadSavedOrigin() {
        std::string origin_file = maps_database_path_ + "/" + map_name_ + "_origin.yaml";
        
        if (!boost::filesystem::exists(origin_file)) {
            ROS_WARN("Origin file not found: %s", origin_file.c_str());
            ROS_WARN("Will use first GPS fix as origin");
            return false;
        }
        
        try {
            YAML::Node config = YAML::LoadFile(origin_file);
            
            // 读取GPS原点坐标
            origin_lat_ = config["gps_origin"]["latitude"].as<double>();
            origin_lon_ = config["gps_origin"]["longitude"].as<double>();
            origin_alt_ = config["gps_origin"]["altitude"].as<double>();
            
            // 读取UTM信息
            utm_zone_ = config["utm_info"]["zone"].as<int>();
            utm_north_ = config["utm_info"]["north"].as<bool>();
            origin_utm_x_ = config["utm_info"]["utm_x"].as<double>();
            origin_utm_y_ = config["utm_info"]["utm_y"].as<double>();
            
            origin_set_ = true;
            
            ROS_INFO("Loaded saved origin from: %s", origin_file.c_str());
            ROS_INFO("  GPS Origin: (%.6f, %.6f, %.1f)", origin_lat_, origin_lon_, origin_alt_);
            ROS_INFO("  UTM Origin: (%.2f, %.2f) Zone: %d%c", 
                     origin_utm_x_, origin_utm_y_, utm_zone_, utm_north_ ? 'N' : 'S');
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load origin file: %s", e.what());
            ROS_WARN("Will use first GPS fix as origin");
            return false;
        }
    }
    
    void setOrigin(const sensor_msgs::NavSatFix& msg) {
        origin_lat_ = msg.latitude;
        origin_lon_ = msg.longitude;
        origin_alt_ = msg.altitude;
        
        try {
            GeographicLib::UTMUPS::Forward(origin_lat_, origin_lon_,
                                          utm_zone_, utm_north_, 
                                          origin_utm_x_, origin_utm_y_);
            origin_set_ = true;
            
            ROS_INFO("Origin set from GPS: (%.6f, %.6f, %.1f) -> UTM(%.2f, %.2f)", 
                     origin_lat_, origin_lon_, origin_alt_, origin_utm_x_, origin_utm_y_);
                     
            // 如果是定位模式但没有找到保存的原点，提醒用户
            if (use_saved_origin_ && !map_name_.empty()) {
                ROS_WARN("Using current GPS as origin, but expected to load saved origin!");
                ROS_WARN("Make sure origin file exists: %s/%s_origin.yaml", 
                         maps_database_path_.c_str(), map_name_.c_str());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to set origin: %s", e.what());
        }
    }
    
    bool convertToLocal(const sensor_msgs::NavSatFix& msg, 
                       double& x, double& y, double& z) {
        try {
            int zone;
            bool north;
            double utm_x, utm_y;
            
            GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude,
                                          zone, north, utm_x, utm_y);
            
            // 检查UTM区域是否一致
            if (zone != utm_zone_ || north != utm_north_) {
                ROS_WARN_THROTTLE(5.0, "GPS crossed UTM zone boundary: %d%c -> %d%c", 
                         utm_zone_, utm_north_ ? 'N' : 'S', zone, north ? 'N' : 'S');
            }
            
            x = utm_x - origin_utm_x_;
            y = utm_y - origin_utm_y_;
            z = msg.altitude - origin_alt_;
            
            return true;
        } catch (const std::exception& e) {
            ROS_WARN("Coordinate conversion failed: %s", e.what());
            return false;
        }
    }
    
    nav_msgs::Odometry createOdometry(const sensor_msgs::NavSatFix& gps_msg,
                                     double x, double y, double z) {
        nav_msgs::Odometry odom;
        
        // 基本信息
        odom.header.stamp = gps_msg.header.stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_link_frame_;
        
        // 位置
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        
        // 朝向 (使用速度方向或保持上次)
        setOrientation(odom);
        
        // 协方差
        setCovariance(odom);
        
        // 速度
        setVelocity(odom);
        
        return odom;
    }
    
    void setOrientation(nav_msgs::Odometry& odom) {
        if (has_last_odom_) {
            // 计算速度方向
            double dx = odom.pose.pose.position.x - last_odom_.pose.pose.position.x;
            double dy = odom.pose.pose.position.y - last_odom_.pose.pose.position.y;
            double speed = std::sqrt(dx*dx + dy*dy);
            
            if (speed > 0.1) {  // 速度 > 0.1 m/s 时使用运动方向
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                odom.pose.pose.orientation = tf2::toMsg(q);
            } else {
                // 低速时保持上次朝向
                odom.pose.pose.orientation = last_odom_.pose.pose.orientation;
            }
        } else {
            // 首次设为单位四元数
            odom.pose.pose.orientation.w = 1.0;
        }
    }
    
    void setCovariance(nav_msgs::Odometry& odom) {
        // 位置协方差
        odom.pose.covariance[0] = gps_xy_stdev_ * gps_xy_stdev_;  // x
        odom.pose.covariance[7] = gps_xy_stdev_ * gps_xy_stdev_;  // y
        odom.pose.covariance[14] = gps_z_stdev_ * gps_z_stdev_;   // z
        odom.pose.covariance[35] = 1.0;  // yaw (GPS无法直接提供，设大不确定性)
        
        // 速度协方差
        double vel_std = gps_xy_stdev_ * 0.1;  // 速度精度约为位置精度的10%
        odom.twist.covariance[0] = vel_std * vel_std;   // vx
        odom.twist.covariance[7] = vel_std * vel_std;   // vy
        odom.twist.covariance[35] = 0.5;                // vyaw
    }
    
    void setVelocity(nav_msgs::Odometry& odom) {
        if (!has_last_odom_) {
            return;  // 首次无速度信息
        }
        
        double dt = (odom.header.stamp - last_odom_.header.stamp).toSec();
        if (dt <= 0 || dt > 1.0) return;  // 异常时间间隔
        
        // 计算线速度
        double dx = odom.pose.pose.position.x - last_odom_.pose.pose.position.x;
        double dy = odom.pose.pose.position.y - last_odom_.pose.pose.position.y;
        double dz = odom.pose.pose.position.z - last_odom_.pose.pose.position.z;
        
        odom.twist.twist.linear.x = dx / dt;
        odom.twist.twist.linear.y = dy / dt;
        odom.twist.twist.linear.z = dz / dt;
        
        // 角速度 (简化计算)
        tf2::Quaternion q_curr, q_last;
        tf2::fromMsg(odom.pose.pose.orientation, q_curr);
        tf2::fromMsg(last_odom_.pose.pose.orientation, q_last);
        
        tf2::Quaternion q_diff = q_curr * q_last.inverse();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_diff).getRPY(roll, pitch, yaw);
        odom.twist.twist.angular.z = yaw / dt;
    }
    
    void publishOdometry(const nav_msgs::Odometry& odom) {
        odom_pub_.publish(odom);
        
        // 保存用于下次速度计算
        last_odom_ = odom;
        has_last_odom_ = true;
        
        ROS_DEBUG("Published GNSS odometry: (%.2f, %.2f, %.2f)", 
                 odom.pose.pose.position.x, 
                 odom.pose.pose.position.y, 
                 odom.pose.pose.position.z);
    }
    
    void publishTransform(const nav_msgs::Odometry& odom) {
        geometry_msgs::TransformStamped transform;
        transform.header = odom.header;
        transform.child_frame_id = odom.child_frame_id;
        
        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.y;
        transform.transform.translation.z = odom.pose.pose.position.z;
        transform.transform.rotation = odom.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_gnss_processor");
    
    try {
        SimpleGNSSProcessor processor;
        ros::spin();
    } catch (std::exception& e) {
        ROS_ERROR("Simple GNSS Processor failed: %s", e.what());
        return -1;
    }
    
    return 0;
}
