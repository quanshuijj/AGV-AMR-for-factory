/**
 * @file enhanced_chcnav_rtk_driver.cpp
 * @brief 增强的CHC导航RTK驱动器 - 支持双天线和完整协方差
 * @author Your Name
 * @date 2025
 */

#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "std_msgs/String.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <map>
#include <string>
#include <iomanip>
#include <sstream>

using namespace std;

#define M_PI 3.14159265358979323846

class EnhancedCHCNavRTKDriver {
private:
    // ROS基础设施
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布器
    ros::Publisher imu_pub_;
    ros::Publisher navsat_pub_;
    ros::Publisher raw_imu_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher heading_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Publisher status_pub_;
    
    // 订阅器
    ros::Subscriber pvt_source_;
    
    // TF发布器
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    // 配置参数
    std::string imu_frame_id_;
    std::string gps_frame_id_;
    bool publish_tf_;
    bool enable_diagnostics_;
    int message_timeout_ms_;
    
    // RTK状态映射
    std::map<int, std::string> rtk_status_names_;
    std::map<int, std::string> system_status_names_;
    
    // 统计信息
    int message_count_;
    ros::Time last_message_time_;
    ros::Time node_start_time_;
    
    // 质量监控
    double current_position_accuracy_;
    double current_heading_accuracy_;
    int current_rtk_status_;
    int current_system_status_;
    
    // 定时器
    ros::Timer diagnostics_timer_;
    ros::Timer tf_timer_;

public:
    EnhancedCHCNavRTKDriver() : 
        private_nh_("~"),
        message_count_(0),
        current_position_accuracy_(999.0),
        current_heading_accuracy_(999.0),
        current_rtk_status_(0),
        current_system_status_(0) {
        
        initializeParameters();
        initializeStatusMaps();
        initializePublishers();
        initializeSubscribers();
        initializeStaticTransforms();
        initializeTimers();
        
        node_start_time_ = ros::Time::now();
        
        ROS_INFO("Enhanced CHC Navigation RTK Driver started");
        ROS_INFO("  IMU frame: %s", imu_frame_id_.c_str());
        ROS_INFO("  GPS frame: %s", gps_frame_id_.c_str());
        ROS_INFO("  Publish TF: %s", publish_tf_ ? "yes" : "no");
        ROS_INFO("  Diagnostics: %s", enable_diagnostics_ ? "enabled" : "disabled");
        ROS_INFO("  Message timeout: %d ms", message_timeout_ms_);
    }

private:
    void initializeParameters() {
        private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
        private_nh_.param<std::string>("gps_frame_id", gps_frame_id_, "gps");
        private_nh_.param<bool>("publish_tf", publish_tf_, true);
        private_nh_.param<bool>("enable_diagnostics", enable_diagnostics_, true);
        private_nh_.param<int>("message_timeout_ms", message_timeout_ms_, 1000);
    }
    
    void initializeStatusMaps() {
        // RTK状态映射（stat[1]）
        rtk_status_names_[0] = "NO_FIX";
        rtk_status_names_[1] = "SPP";          // 单点定位定向
        rtk_status_names_[2] = "DGPS";         // 伪距差分定位定向
        rtk_status_names_[3] = "PPS";          // 组合推算
        rtk_status_names_[4] = "RTK_FIXED";    // RTK稳定解定位定向
        rtk_status_names_[5] = "RTK_FLOAT";    // RTK浮点解定位定向
        rtk_status_names_[6] = "SPP_NO_HDG";   // 单点定位不定向
        rtk_status_names_[7] = "DGPS_NO_HDG";  // 伪距差分定位不定向
        rtk_status_names_[8] = "RTK_FIXED_NO_HDG"; // RTK稳定解定位不定向
        rtk_status_names_[9] = "RTK_FLOAT_NO_HDG"; // RTK浮点解定位不定向
        
        // 系统状态映射（stat[0]）
        system_status_names_[0] = "INITIALIZING";
        system_status_names_[1] = "GNSS_ONLY";
        system_status_names_[2] = "INS_GNSS";
        system_status_names_[3] = "INS_ONLY";
    }
    
    void initializePublishers() {
        imu_pub_ = private_nh_.advertise<sensor_msgs::Imu>("imu", 100);
        navsat_pub_ = private_nh_.advertise<sensor_msgs::NavSatFix>("fix", 100);
        raw_imu_pub_ = private_nh_.advertise<sensor_msgs::Imu>("raw_imu", 100);
        velocity_pub_ = private_nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 100);
        heading_pub_ = private_nh_.advertise<geometry_msgs::QuaternionStamped>("heading", 100);
        status_pub_ = private_nh_.advertise<std_msgs::String>("status", 1, true);
        
        if (enable_diagnostics_) {
            diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
        }
    }
    
    void initializeSubscribers() {
        pvt_source_ = private_nh_.subscribe("/chcnav/devpvt", 100, 
            &EnhancedCHCNavRTKDriver::pvtCallback, this);
    }
    
    void initializeStaticTransforms() {
        if (!publish_tf_) return;
        
        // 发布静态变换
        geometry_msgs::TransformStamped transform;
        
        // map -> chcnav
        transform.header.seq = 1;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "chcnav";
        
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation = tf2::toMsg(q);
        
        static_tf_broadcaster_.sendTransform(transform);
        
        // chcnav -> gps
        transform.header.frame_id = "chcnav";
        transform.child_frame_id = gps_frame_id_;
        static_tf_broadcaster_.sendTransform(transform);
        
        // chcnav -> imu_link
        //transform.child_frame_id = imu_frame_id_;
        //static_tf_broadcaster_.sendTransform(transform);
    }
    
    void initializeTimers() {
        if (enable_diagnostics_) {
            diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0), 
                &EnhancedCHCNavRTKDriver::diagnosticsCallback, this);
        }
        
        if (publish_tf_) {
            tf_timer_ = nh_.createTimer(ros::Duration(0.1), 
                &EnhancedCHCNavRTKDriver::tfCallback, this);
        }
    }
    
    void pvtCallback(const chcnav::hcinspvatzcb::ConstPtr &msg) {
        message_count_++;
        last_message_time_ = msg->header.stamp;
        
        // 更新状态信息
        updateStatusInfo(*msg);
        
        // 发布NavSatFix（高精度GPS）
        publishNavSatFix(*msg);
        
        // 发布IMU（融合的姿态和运动数据）
        publishIMU(*msg);
        
        // 发布原始IMU（设备原始数据）
        publishRawIMU(*msg);
        
        // 发布速度信息
        publishVelocity(*msg);
        
        // 发布航向角
        publishHeading(*msg);
        
        // 发布状态信息
        publishStatus(*msg);
        
        // 诊断信息（每10条消息输出一次）
        if (message_count_ % 10 == 0) {
            ROS_DEBUG("RTK: msg=%d, status=%s, pos_acc=%.3fm, hdg_acc=%.2f°", 
                     message_count_, 
                     getRTKStatusName(current_rtk_status_).c_str(),
                     current_position_accuracy_, 
                     current_heading_accuracy_);
        }
    }
    
    void updateStatusInfo(const chcnav::hcinspvatzcb &msg) {
        // 更新RTK状态
        if (msg.stat.size() >= 2) {
            current_rtk_status_ = msg.stat[1];
            current_system_status_ = msg.stat[0];
        }
        
        // 更新精度信息
        if (msg.position_stdev.size() >= 3) {
            current_position_accuracy_ = std::max(msg.position_stdev[0], msg.position_stdev[1]);
        }
        
        if (msg.euler_stdev.size() >= 3) {
            current_heading_accuracy_ = msg.euler_stdev[2];
        }
    }
    
    void publishNavSatFix(const chcnav::hcinspvatzcb &msg) {
        sensor_msgs::NavSatFix fix;
        
        // 头部信息
        fix.header = msg.header;
        fix.header.frame_id = gps_frame_id_;
        
        // 位置信息
        fix.latitude = msg.latitude;
        fix.longitude = msg.longitude;
        fix.altitude = msg.altitude;
        
        // 状态信息
        fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS | 
                            sensor_msgs::NavSatStatus::SERVICE_GLONASS |
                            sensor_msgs::NavSatStatus::SERVICE_GALILEO |
                            sensor_msgs::NavSatStatus::SERVICE_COMPASS;
        
        // 根据RTK状态设置fix状态
        if (msg.stat.size() >= 2) {
            switch (msg.stat[1]) {
                case 4:  // RTK固定解
                case 5:  // RTK浮点解
                case 8:  // RTK固定解不定向
                case 9:  // RTK浮点解不定向
                    fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                case 2:  // 伪距差分
                case 7:  // 伪距差分不定向
                    fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 1:  // 单点定位
                case 6:  // 单点定位不定向
                    fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                    break;
                default:
                    fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                    break;
            }
        } else {
            fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }
        
        // 协方差矩阵（关键改进：使用RTK设备提供的精确协方差）
        if (msg.position_stdev.size() >= 3) {
            // 使用设备提供的标准差
            double lat_var = msg.position_stdev[0] * msg.position_stdev[0];
            double lon_var = msg.position_stdev[1] * msg.position_stdev[1];
            double alt_var = msg.position_stdev[2] * msg.position_stdev[2];
            
            fix.position_covariance[0] = lat_var;   // 纬度方差
            fix.position_covariance[4] = lon_var;   // 经度方差
            fix.position_covariance[8] = alt_var;   // 高度方差
            
            // 设置协方差类型
            fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            
            ROS_DEBUG_THROTTLE(5.0, "GPS covariance: lat=%.4f, lon=%.4f, alt=%.4f", 
                              lat_var, lon_var, alt_var);
        } else {
            // 使用保守的默认值
            fix.position_covariance[0] = 0.04;   // 20cm std
            fix.position_covariance[4] = 0.04;   // 20cm std
            fix.position_covariance[8] = 0.25;   // 50cm std
            fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }
        
        navsat_pub_.publish(fix);
    }
    
    void publishIMU(const chcnav::hcinspvatzcb &msg) {
        sensor_msgs::Imu imu;
        
        // 头部信息
        imu.header = msg.header;
        imu.header.frame_id = imu_frame_id_;
        
        // 姿态（从RTK设备的融合姿态）
        // 注意：处理CHC设备的坐标系约定
        double roll_rad = msg.roll * M_PI / 180.0;
        double pitch_rad = -msg.pitch * M_PI / 180.0;  // 注意符号
        double yaw_rad = 0.0;
        
        // 选择最佳的航向角源
        if (msg.stat.size() >= 2 && (msg.stat[1] == 4 || msg.stat[1] == 5)) {
            // RTK固定解或浮点解，使用heading2（双天线航向角）
            yaw_rad = -msg.heading2 * M_PI / 180.0;  // 转换坐标系
        } else {
            // 其他情况使用yaw角
            yaw_rad = msg.yaw * M_PI / 180.0;
        }
        
        tf2::Quaternion q;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);
        imu.orientation = tf2::toMsg(q);
        
        // 姿态协方差
        if (msg.euler_stdev.size() >= 3) {
            double roll_var = (msg.euler_stdev[0] * M_PI / 180.0) * (msg.euler_stdev[0] * M_PI / 180.0);
            double pitch_var = (msg.euler_stdev[1] * M_PI / 180.0) * (msg.euler_stdev[1] * M_PI / 180.0);
            double yaw_var = (msg.euler_stdev[2] * M_PI / 180.0) * (msg.euler_stdev[2] * M_PI / 180.0);
            
            imu.orientation_covariance[0] = roll_var;
            imu.orientation_covariance[4] = pitch_var;
            imu.orientation_covariance[8] = yaw_var;
        } else {
            // 根据RTK状态设置不同的协方差
            if (current_rtk_status_ >= 4) {  // RTK解
                imu.orientation_covariance[0] = 0.01;   // 0.1° std
                imu.orientation_covariance[4] = 0.01;
                imu.orientation_covariance[8] = 0.001;  // 0.06° std for heading
            } else {
                imu.orientation_covariance[0] = 0.1;    // 0.3° std
                imu.orientation_covariance[4] = 0.1;
                imu.orientation_covariance[8] = 0.1;    // 0.3° std
            }
        }
        
        // 角速度（车辆坐标系）
        imu.angular_velocity.x = msg.vehicle_angular_velocity.x * M_PI / 180.0;
        imu.angular_velocity.y = msg.vehicle_angular_velocity.y * M_PI / 180.0;
        imu.angular_velocity.z = msg.vehicle_angular_velocity.z * M_PI / 180.0;
        
        // 线性加速度（车辆坐标系，含重力）
        imu.linear_acceleration.x = msg.vehicle_linear_acceleration.x;
        imu.linear_acceleration.y = msg.vehicle_linear_acceleration.y;
        imu.linear_acceleration.z = msg.vehicle_linear_acceleration.z;
        
        // 角速度和加速度协方差（固定值，根据设备规格调整）
        std::fill(imu.angular_velocity_covariance.begin(), imu.angular_velocity_covariance.end(), 0.0);
        imu.angular_velocity_covariance[0] = 0.01;
        imu.angular_velocity_covariance[4] = 0.01;
        imu.angular_velocity_covariance[8] = 0.01;
        
        std::fill(imu.linear_acceleration_covariance.begin(), imu.linear_acceleration_covariance.end(), 0.0);
        imu.linear_acceleration_covariance[0] = 0.02;
        imu.linear_acceleration_covariance[4] = 0.02;
        imu.linear_acceleration_covariance[8] = 0.02;
        
        imu_pub_.publish(imu);
    }
    
    void publishRawIMU(const chcnav::hcinspvatzcb &msg) {
        sensor_msgs::Imu raw_imu;
        
        // 头部信息
        raw_imu.header = msg.header;
        raw_imu.header.frame_id = imu_frame_id_;
        
        // 原始角速度（设备坐标系）
        raw_imu.angular_velocity.x = msg.raw_angular_velocity.x * M_PI / 180.0;
        raw_imu.angular_velocity.y = msg.raw_angular_velocity.y * M_PI / 180.0;
        raw_imu.angular_velocity.z = msg.raw_angular_velocity.z * M_PI / 180.0;
        
        // 原始加速度（设备坐标系，含重力）
        raw_imu.linear_acceleration.x = msg.raw_acceleration.x;
        raw_imu.linear_acceleration.y = msg.raw_acceleration.y;
        raw_imu.linear_acceleration.z = msg.raw_acceleration.z;
        
        // 姿态无效（原始IMU不提供姿态）
        raw_imu.orientation_covariance[0] = -1;
        
        // 协方差
        std::fill(raw_imu.angular_velocity_covariance.begin(), raw_imu.angular_velocity_covariance.end(), 0.0);
        raw_imu.angular_velocity_covariance[0] = 1e-5;  // x轴
        raw_imu.angular_velocity_covariance[4] = 1.2e-5; // y轴 (稍差)
        raw_imu.angular_velocity_covariance[8] = 8e-6;   // z轴
        
        std::fill(raw_imu.linear_acceleration_covariance.begin(), raw_imu.linear_acceleration_covariance.end(), 0.0);
        raw_imu.linear_acceleration_covariance[0] = 1.5e-4; // x轴 (最差)
        raw_imu.linear_acceleration_covariance[4] = 1.3e-4; // y轴
        raw_imu.linear_acceleration_covariance[8] = 1e-4;   // z轴 (最好)
        
        raw_imu_pub_.publish(raw_imu);
    }
    
    void publishVelocity(const chcnav::hcinspvatzcb &msg) {
        geometry_msgs::TwistWithCovarianceStamped velocity;
        
        // 头部信息
        velocity.header = msg.header;
        velocity.header.frame_id = imu_frame_id_;
        
        // ENU速度
        velocity.twist.twist.linear.x = msg.enu_velocity.x;  // 东向
        velocity.twist.twist.linear.y = msg.enu_velocity.y;  // 北向
        velocity.twist.twist.linear.z = msg.enu_velocity.z;  // 天向
        
        // 角速度
        velocity.twist.twist.angular.x = msg.vehicle_angular_velocity.x * M_PI / 180.0;
        velocity.twist.twist.angular.y = msg.vehicle_angular_velocity.y * M_PI / 180.0;
        velocity.twist.twist.angular.z = msg.vehicle_angular_velocity.z * M_PI / 180.0;
        
        // 速度协方差
        if (msg.enu_velocity_stdev.size() >= 3) {
            velocity.twist.covariance[0] = msg.enu_velocity_stdev[0] * msg.enu_velocity_stdev[0];  // vx
            velocity.twist.covariance[7] = msg.enu_velocity_stdev[1] * msg.enu_velocity_stdev[1];  // vy
            velocity.twist.covariance[14] = msg.enu_velocity_stdev[2] * msg.enu_velocity_stdev[2]; // vz
        } else {
            velocity.twist.covariance[0] = 0.01;   // 10cm/s std
            velocity.twist.covariance[7] = 0.01;
            velocity.twist.covariance[14] = 0.01;
        }
        
        // 角速度协方差
        velocity.twist.covariance[21] = 0.01;  // ωx
        velocity.twist.covariance[28] = 0.01;  // ωy
        velocity.twist.covariance[35] = 0.01;  // ωz
        
        velocity_pub_.publish(velocity);
    }
    
    void publishHeading(const chcnav::hcinspvatzcb &msg) {
        geometry_msgs::QuaternionStamped heading;
        
        // 头部信息
        heading.header = msg.header;
        heading.header.frame_id = imu_frame_id_;
        
        // 使用双天线航向角（heading2）
        double yaw_rad = -msg.heading2 * M_PI / 180.0;  // 坐标系转换
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        heading.quaternion = tf2::toMsg(q);
        
        heading_pub_.publish(heading);
    }
    
    void publishStatus(const chcnav::hcinspvatzcb &msg) {
        std::stringstream ss;
        ss << "RTK Status: " << getRTKStatusName(current_rtk_status_);
        ss << " | System: " << getSystemStatusName(current_system_status_);
        ss << " | Pos Acc: " << std::fixed << std::setprecision(3) << current_position_accuracy_ << "m";
        ss << " | Hdg Acc: " << std::fixed << std::setprecision(2) << current_heading_accuracy_ << "°";
        ss << " | Satellites: " << msg.ns << "/" << msg.ns2;
        ss << " | Age: " << std::fixed << std::setprecision(1) << msg.age << "s";
        
        std_msgs::String status_msg;
        status_msg.data = ss.str();
        status_pub_.publish(status_msg);
    }
    
    void diagnosticsCallback(const ros::TimerEvent&) {
        if (!enable_diagnostics_) return;
        
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.header.stamp = ros::Time::now();
        
        // RTK系统状态
        diagnostic_msgs::DiagnosticStatus rtk_status;
        rtk_status.name = "RTK Navigation System";
        rtk_status.hardware_id = "CHC_RTK";
        
        // 确定诊断级别
        if (current_rtk_status_ >= 4) {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            rtk_status.message = "RTK solution available";
        } else if (current_rtk_status_ >= 1) {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            rtk_status.message = "GNSS solution available, no RTK";
        } else {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            rtk_status.message = "No GNSS solution";
        }
        
        // 检查消息超时
        if ((ros::Time::now() - last_message_time_).toSec() > message_timeout_ms_ / 1000.0) {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::STALE;
            rtk_status.message = "RTK messages timeout";
        }
        
        // 添加详细信息
        addDiagnosticValue(rtk_status, "RTK Status", getRTKStatusName(current_rtk_status_));
        addDiagnosticValue(rtk_status, "System Status", getSystemStatusName(current_system_status_));
        addDiagnosticValue(rtk_status, "Position Accuracy", current_position_accuracy_, "m");
        addDiagnosticValue(rtk_status, "Heading Accuracy", current_heading_accuracy_, "deg");
        addDiagnosticValue(rtk_status, "Messages Received", message_count_);
        addDiagnosticValue(rtk_status, "Uptime", (ros::Time::now() - node_start_time_).toSec(), "s");
        
        diagnostics.status.push_back(rtk_status);
        diagnostics_pub_.publish(diagnostics);
    }
    
    void tfCallback(const ros::TimerEvent&) {
        // 重新发布静态变换（某些情况下需要周期性发布）
        initializeStaticTransforms();
    }
    
    std::string getRTKStatusName(int status) {
        auto it = rtk_status_names_.find(status);
        if (it != rtk_status_names_.end()) {
            return it->second;
        }
        return "UNKNOWN(" + std::to_string(status) + ")";
    }
    
    std::string getSystemStatusName(int status) {
        auto it = system_status_names_.find(status);
        if (it != system_status_names_.end()) {
            return it->second;
        }
        return "UNKNOWN(" + std::to_string(status) + ")";
    }
    
    void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status, 
                           const std::string& key, 
                           const std::string& value) {
        diagnostic_msgs::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    }
    
    void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status, 
                           const std::string& key, 
                           double value, 
                           const std::string& unit = "") {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << value;
        if (!unit.empty()) {
            ss << " " << unit;
        }
        addDiagnosticValue(status, key, ss.str());
    }
    
    void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status, 
                           const std::string& key, 
                           int value) {
        addDiagnosticValue(status, key, std::to_string(value));
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "enhanced_chcnav_rtk_driver");
    
    try {
        EnhancedCHCNavRTKDriver driver;
        
        ROS_INFO("Enhanced CHC Navigation RTK Driver ready");
        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("Enhanced CHC Navigation RTK Driver failed: %s", e.what());
        return -1;
    }
    
    return 0;
}
