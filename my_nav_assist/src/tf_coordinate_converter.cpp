#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class TFCoordinateConverter {
private:
    // ROS核心组件
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 消息订阅和发布
    ros::Subscriber global_odom_sub_;           // 订阅EKF全局定位 (map->base_link)
    ros::Subscriber local_odom_sub_;            // 订阅EKF局部定位 (odom->base_link)
    ros::Publisher local_odom_pub_;             // 发布局部里程计
    ros::Publisher pose_pub_;                   // 发布位姿 (可选)
    
    // TF处理组件
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 定时器
    ros::Timer publish_timer_;
    ros::Timer diagnostics_timer_;
    
    // 配置参数
    std::string map_frame_;                     // 全局坐标系名称
    std::string odom_frame_;                    // 局部坐标系名称  
    std::string base_link_frame_;               // 机器人坐标系名称
    double publish_frequency_;                  // 发布频率
    bool publish_tf_;                          // 是否发布TF变换
    bool publish_pose_;                        // 是否发布位姿消息
    bool publish_odom_to_base_link_;           // 是否发布odom->base_link变换
    double transform_tolerance_;               // 变换容忍度
    double max_time_diff_;                     // 最大时间差容忍度
    
    // 状态变量
    nav_msgs::Odometry latest_global_odom_;    // 最新的全局里程计 (map->base_link)
    nav_msgs::Odometry latest_local_odom_;     // 最新的局部里程计 (odom->base_link)
    nav_msgs::Odometry output_local_odom_;     // 输出的局部里程计
    bool global_odom_received_;                // 是否收到全局里程计
    bool local_odom_received_;                 // 是否收到局部里程计
    bool local_odom_initialized_;              // 局部里程计是否初始化
    
    // 时间管理
    ros::Time last_global_time_;               // 最后一次全局里程计时间
    ros::Time last_local_time_;                // 最后一次局部里程计时间
    
    // 变换缓存
    tf2::Transform map_to_odom_transform_;     // map->odom变换
    tf2::Transform odom_to_base_link_transform_; // odom->base_link变换
    bool map_to_odom_valid_;                   // map->odom变换是否有效
    bool odom_to_base_link_valid_;             // odom->base_link变换是否有效
    
    // 统计信息
    size_t global_odom_count_;                 // 全局里程计计数
    size_t local_odom_count_;                  // 局部里程计计数
    size_t transform_publish_count_;           // 变换发布计数
    
public:
    /**
     * @brief 构造函数
     */
    TFCoordinateConverter() : 
        private_nh_("~"), 
        tf_listener_(tf_buffer_),
        global_odom_received_(false),
        local_odom_received_(false),
        local_odom_initialized_(false),
        map_to_odom_valid_(false),
        odom_to_base_link_valid_(false),
        global_odom_count_(0),
        local_odom_count_(0),
        transform_publish_count_(0) {
        
        // 加载参数
        loadParameters();
        
        // 初始化发布者和订阅者
        initializePubSub();
        
        // 启动定时器
        initializeTimers();
        
        // 初始化变换
        map_to_odom_transform_.setIdentity();
        odom_to_base_link_transform_.setIdentity();
        
        ROS_INFO("Enhanced TF Coordinate Converter initialized successfully");
        printConfiguration();
    }
    
private:
    /**
     * @brief 加载ROS参数
     */
    void loadParameters() {
        // 坐标系参数
        private_nh_.param<std::string>("map_frame", map_frame_, "map");
        private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        private_nh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        
        // 发布参数
        private_nh_.param<double>("publish_frequency", publish_frequency_, 50.0);
        private_nh_.param<bool>("publish_tf", publish_tf_, true);
        private_nh_.param<bool>("publish_pose", publish_pose_, false);
        private_nh_.param<bool>("publish_odom_to_base_link", publish_odom_to_base_link_, true);
        
        // 时间参数
        private_nh_.param<double>("transform_tolerance", transform_tolerance_, 0.1);
        private_nh_.param<double>("max_time_diff", max_time_diff_, 0.1);
        
        // 参数验证
        validateParameters();
    }
    
    /**
     * @brief 验证参数有效性
     */
    void validateParameters() {
        if (publish_frequency_ <= 0 || publish_frequency_ > 1000) {
            ROS_WARN("Invalid publish_frequency: %.2f, using 50.0 Hz", publish_frequency_);
            publish_frequency_ = 50.0;
        }
        
        if (transform_tolerance_ < 0 || transform_tolerance_ > 1.0) {
            ROS_WARN("Invalid transform_tolerance: %.3f, using 0.1s", transform_tolerance_);
            transform_tolerance_ = 0.1;
        }
        
        if (max_time_diff_ < 0 || max_time_diff_ > 1.0) {
            ROS_WARN("Invalid max_time_diff: %.3f, using 0.1s", max_time_diff_);
            max_time_diff_ = 0.1;
        }
    }
    
    /**
     * @brief 初始化发布者和订阅者
     */
    void initializePubSub() {
        // 订阅EKF的全局定位结果 (map->base_link)
        global_odom_sub_ = nh_.subscribe("/global_odometry", 10, 
                                       &TFCoordinateConverter::globalOdomCallback, this);
        
        // 订阅EKF的局部定位结果 (odom->base_link)
        local_odom_sub_ = nh_.subscribe("/local_odometry", 10, 
                                      &TFCoordinateConverter::localOdomCallback, this);
        
        // 发布转换后的局部里程计
        local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("converted_local_odometry", 10);
        
        // 可选: 发布位姿消息
        if (publish_pose_) {
            pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
        }
    }
    
    /**
     * @brief 初始化定时器
     */
    void initializeTimers() {
        // 主发布定时器
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_frequency_), 
                                       &TFCoordinateConverter::publishTimerCallback, this);
        
        // 诊断定时器
        diagnostics_timer_ = nh_.createTimer(ros::Duration(5.0), 
                                           &TFCoordinateConverter::diagnosticsCallback, this);
    }
    
    /**
     * @brief 全局里程计回调函数 (map->base_link)
     * @param msg EKF输出的全局里程计消息
     */
    void globalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 验证消息格式
        if (!validateOdomMessage(msg, map_frame_, base_link_frame_, "global")) {
            return;
        }
        
        // 保存全局里程计数据
        latest_global_odom_ = *msg;
        global_odom_received_ = true;
        last_global_time_ = msg->header.stamp;
        global_odom_count_++;
        
        // 更新坐标变换
        updateCoordinateTransforms();
    }
    
    /**
     * @brief 局部里程计回调函数 (odom->base_link)
     * @param msg EKF输出的局部里程计消息
     */
    void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 验证消息格式
        if (!validateOdomMessage(msg, odom_frame_, base_link_frame_, "local")) {
            return;
        }
        
        // 保存局部里程计数据
        latest_local_odom_ = *msg;
        local_odom_received_ = true;
        last_local_time_ = msg->header.stamp;
        local_odom_count_++;
        
        // 更新odom->base_link变换
        updateOdomToBaseLinkTransform();
        
        // 更新坐标变换
        updateCoordinateTransforms();
    }
    
    /**
     * @brief 验证里程计消息
     */
    bool validateOdomMessage(const nav_msgs::Odometry::ConstPtr& msg, 
                           const std::string& expected_frame, 
                           const std::string& expected_child_frame,
                           const std::string& odom_type) {
        // 检查坐标系
        if (msg->header.frame_id != expected_frame) {
            ROS_WARN_THROTTLE(1.0, "%s odometry frame_id mismatch: got '%s', expected '%s'", 
                             odom_type.c_str(), msg->header.frame_id.c_str(), expected_frame.c_str());
            return false;
        }
        
        if (msg->child_frame_id != expected_child_frame) {
            ROS_WARN_THROTTLE(1.0, "%s odometry child_frame_id mismatch: got '%s', expected '%s'", 
                             odom_type.c_str(), msg->child_frame_id.c_str(), expected_child_frame.c_str());
            return false;
        }
        
        // 检查时间戳
        if (msg->header.stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0, "%s odometry has zero timestamp", odom_type.c_str());
            return false;
        }
        
        // 检查数值有效性
        if (!isFinite(msg->pose.pose)) {
            ROS_WARN_THROTTLE(1.0, "%s odometry contains invalid values", odom_type.c_str());
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 检查数值是否有限
     */
    bool isFinite(const geometry_msgs::Pose& pose) {
        return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && 
               std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
               std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
               std::isfinite(pose.orientation.w);
    }
    
    /**
     * @brief 更新odom->base_link变换
     */
    void updateOdomToBaseLinkTransform() {
        if (!local_odom_received_) return;
        
        try {
            // 从局部里程计获取odom->base_link变换
            tf2::fromMsg(latest_local_odom_.pose.pose, odom_to_base_link_transform_);
            odom_to_base_link_valid_ = true;
            
            ROS_DEBUG_THROTTLE(5.0, "Updated odom->base_link transform from local odometry");
            
        } catch (std::exception &e) {
            ROS_WARN_THROTTLE(1.0, "Failed to update odom->base_link transform: %s", e.what());
            odom_to_base_link_valid_ = false;
        }
    }
    
    /**
     * @brief 更新坐标变换
     */
    void updateCoordinateTransforms() {
        if (!global_odom_received_ || !local_odom_received_) return;
        
        // 检查时间同步
        if (!checkTimeSynchronization()) {
            ROS_WARN_THROTTLE(2.0, "Global and local odometry timestamps are not synchronized");
            return;
        }
        
        try {
            // 从全局里程计获取map->base_link变换
            tf2::Transform map_to_base_link_tf;
            tf2::fromMsg(latest_global_odom_.pose.pose, map_to_base_link_tf);
            
            // 从局部里程计获取odom->base_link变换
            tf2::Transform odom_to_base_link_tf;
            tf2::fromMsg(latest_local_odom_.pose.pose, odom_to_base_link_tf);
            
            // 计算map->odom变换
            // map->base_link = map->odom * odom->base_link
            // 因此: map->odom = map->base_link * (odom->base_link)^-1
            map_to_odom_transform_ = map_to_base_link_tf * odom_to_base_link_tf.inverse();
            map_to_odom_valid_ = true;
            
            ROS_DEBUG_THROTTLE(5.0, "Updated map->odom transform successfully");
            
        } catch (std::exception &e) {
            ROS_WARN_THROTTLE(1.0, "Failed to update coordinate transforms: %s", e.what());
            map_to_odom_valid_ = false;
        }
    }
    
    /**
     * @brief 检查时间同步
     */
    bool checkTimeSynchronization() {
        if (!global_odom_received_ || !local_odom_received_) return false;
        
        ros::Duration time_diff = last_global_time_ - last_local_time_;
        double abs_time_diff = std::abs(time_diff.toSec());
        
        if (abs_time_diff > max_time_diff_) {
            ROS_DEBUG_THROTTLE(1.0, "Time difference too large: %.3f seconds", abs_time_diff);
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 发布定时器回调
     */
    void publishTimerCallback(const ros::TimerEvent& event) {
        // 检查数据可用性
        if (!global_odom_received_) {
            ROS_WARN_THROTTLE(5.0, "No global odometry received yet");
            return;
        }
        
        if (!local_odom_received_) {
            ROS_WARN_THROTTLE(5.0, "No local odometry received yet");
            return;
        }
        
        // 检查数据新鲜度
        checkDataFreshness();
        
        // 发布完整的变换链
        if (publish_tf_) {
            publishTransformChain();
        }
        
        publishConvertedLocalOdometry();
        
        if (publish_pose_) {
            publishPoseMessage();
        }
    }
    
    /**
     * @brief 检查数据新鲜度
     */
    void checkDataFreshness() {
        ros::Time now = ros::Time::now();
        
        if (global_odom_received_) {
            ros::Duration global_age = now - last_global_time_;
            if (global_age.toSec() > 1.0) {
                ROS_WARN_THROTTLE(1.0, "Global odometry data is %.2f seconds old", 
                                 global_age.toSec());
            }
        }
        
        if (local_odom_received_) {
            ros::Duration local_age = now - last_local_time_;
            if (local_age.toSec() > 1.0) {
                ROS_WARN_THROTTLE(1.0, "Local odometry data is %.2f seconds old", 
                                 local_age.toSec());
            }
        }
    }
    
    /**
     * @brief 发布完整的变换链 map->odom->base_link
     */
    void publishTransformChain() {
        ros::Time now = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> transforms;
        
        // 1. 发布map->odom变换
        if (map_to_odom_valid_) {
            geometry_msgs::TransformStamped map_to_odom_msg;
            map_to_odom_msg.header.stamp = now;
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_msg.transform = tf2::toMsg(map_to_odom_transform_);
            transforms.push_back(map_to_odom_msg);
        }
        
        // 2. 发布odom->base_link变换 (如果启用)
        if (publish_odom_to_base_link_ && odom_to_base_link_valid_) {
            geometry_msgs::TransformStamped odom_to_base_link_msg;
            odom_to_base_link_msg.header.stamp = now;
            odom_to_base_link_msg.header.frame_id = odom_frame_;
            odom_to_base_link_msg.child_frame_id = base_link_frame_;
            odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link_transform_);
            transforms.push_back(odom_to_base_link_msg);
        }
        
        // 批量发布变换
        if (!transforms.empty()) {
            tf_broadcaster_.sendTransform(transforms);
            transform_publish_count_++;
            ROS_DEBUG_THROTTLE(5.0, "Published %zu transforms in chain", transforms.size());
        }
    }
    
    /**
     * @brief 发布转换后的局部里程计
     */
    void publishConvertedLocalOdometry() {
        if (!local_odom_received_) return;
        
        // 直接发布局部里程计（可能经过处理）
        output_local_odom_ = latest_local_odom_;
        output_local_odom_.header.stamp = ros::Time::now();
        
        local_odom_pub_.publish(output_local_odom_);
        
        ROS_DEBUG_THROTTLE(5.0, "Published converted local odometry");
    }
    
    /**
     * @brief 发布位姿消息 (可选)
     */
    void publishPoseMessage() {
        if (!global_odom_received_) return;
        
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = latest_global_odom_.header;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.pose = latest_global_odom_.pose.pose;
        pose_msg.pose.covariance = latest_global_odom_.pose.covariance;
        
        pose_pub_.publish(pose_msg);
    }
    
    /**
     * @brief 诊断回调函数
     */
    void diagnosticsCallback(const ros::TimerEvent& event) {
        printStatus();
    }
    
    /**
     * @brief 打印系统状态
     */
    void printStatus() {
        ROS_INFO("=== Enhanced TF Coordinate Converter Status ===");
        ROS_INFO("Global odom received: %s (count: %zu)", 
                 global_odom_received_ ? "Yes" : "No", global_odom_count_);
        ROS_INFO("Local odom received: %s (count: %zu)", 
                 local_odom_received_ ? "Yes" : "No", local_odom_count_);
        ROS_INFO("Map->odom transform valid: %s", 
                 map_to_odom_valid_ ? "Yes" : "No");
        ROS_INFO("Odom->base_link transform valid: %s", 
                 odom_to_base_link_valid_ ? "Yes" : "No");
        ROS_INFO("Transform chain publishes: %zu", transform_publish_count_);
        
        if (global_odom_received_) {
            ros::Duration global_age = ros::Time::now() - last_global_time_;
            ROS_INFO("Global data age: %.3f seconds", global_age.toSec());
        }
        
        if (local_odom_received_) {
            ros::Duration local_age = ros::Time::now() - last_local_time_;
            ROS_INFO("Local data age: %.3f seconds", local_age.toSec());
        }
        
        // 打印当前变换信息
        if (map_to_odom_valid_) {
            tf2::Vector3 translation = map_to_odom_transform_.getOrigin();
            tf2::Quaternion rotation = map_to_odom_transform_.getRotation();
            double roll, pitch, yaw;
            tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
            
            ROS_INFO("Current map->odom transform:");
            ROS_INFO("  Translation: [%.3f, %.3f, %.3f]", 
                     translation.x(), translation.y(), translation.z());
            ROS_INFO("  Rotation: [%.3f, %.3f, %.3f] rad", roll, pitch, yaw);
        }
        
        if (odom_to_base_link_valid_) {
            tf2::Vector3 translation = odom_to_base_link_transform_.getOrigin();
            tf2::Quaternion rotation = odom_to_base_link_transform_.getRotation();
            double roll, pitch, yaw;
            tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
            
            ROS_INFO("Current odom->base_link transform:");
            ROS_INFO("  Translation: [%.3f, %.3f, %.3f]", 
                     translation.x(), translation.y(), translation.z());
            ROS_INFO("  Rotation: [%.3f, %.3f, %.3f] rad", roll, pitch, yaw);
        }
        
        // 检查时间同步状态
        if (global_odom_received_ && local_odom_received_) {
            ros::Duration sync_diff = last_global_time_ - last_local_time_;
            ROS_INFO("Time synchronization: %.3f seconds difference", sync_diff.toSec());
        }
        
        ROS_INFO("===============================================");
    }
    
    /**
     * @brief 打印配置信息
     */
    void printConfiguration() {
        ROS_INFO("=== Enhanced TF Coordinate Converter Configuration ===");
        ROS_INFO("Coordinate frames:");
        ROS_INFO("  Map frame: %s", map_frame_.c_str());
        ROS_INFO("  Odom frame: %s", odom_frame_.c_str());
        ROS_INFO("  Base link frame: %s", base_link_frame_.c_str());
        ROS_INFO("Publishing settings:");
        ROS_INFO("  Frequency: %.1f Hz", publish_frequency_);
        ROS_INFO("  Publish TF: %s", publish_tf_ ? "Yes" : "No");
        ROS_INFO("  Publish odom->base_link: %s", publish_odom_to_base_link_ ? "Yes" : "No");
        ROS_INFO("  Publish pose: %s", publish_pose_ ? "Yes" : "No");
        ROS_INFO("  Transform tolerance: %.3f seconds", transform_tolerance_);
        ROS_INFO("  Max time difference: %.3f seconds", max_time_diff_);
        ROS_INFO("======================================================");
    }
    
    /**
     * @brief 获取系统统计信息
     */
    void getStatistics(size_t& global_count, size_t& local_count, 
                      size_t& transform_count, bool& is_healthy) {
        global_count = global_odom_count_;
        local_count = local_odom_count_;
        transform_count = transform_publish_count_;
        is_healthy = global_odom_received_ && local_odom_received_ && 
                    map_to_odom_valid_ && odom_to_base_link_valid_;
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "enhanced_tf_coordinate_converter");
    
    try {
        ROS_INFO("Starting Enhanced TF Coordinate Converter...");
        
        // 创建节点实例
        TFCoordinateConverter converter;
        
        ROS_INFO("Enhanced TF Coordinate Converter started successfully");
        ROS_INFO("Waiting for global and local odometry data...");
        
        // 运行ROS事件循环
        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("Enhanced TF Coordinate Converter failed: %s", e.what());
        return -1;
    } catch (...) {
        ROS_ERROR("Enhanced TF Coordinate Converter failed with unknown exception");
        return -1;
    }
    
    ROS_INFO("Enhanced TF Coordinate Converter shutdown gracefully");
    return 0;
}
