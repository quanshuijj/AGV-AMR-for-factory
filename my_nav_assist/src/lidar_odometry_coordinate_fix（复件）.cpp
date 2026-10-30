#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

/**
 * @brief 激光里程计坐标修正器
 * 
 * 此类可以集成到现有的激光里程计代码中，
 * 或者作为独立节点运行来修正坐标系输出
 */
class LidarOdometryCoordinateFix {
private:
    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 静态变换缓存
    tf2::Transform camera_to_base_link_;     // camera -> base_link 静态变换
    bool static_transform_ready_;
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_;
    
    // 配置参数
    std::string map_frame_;
    std::string camera_frame_;               // 激光里程计原始输出的坐标系
    std::string base_link_frame_;
    std::string lidar_frame_;               // 激光雷达坐标系 (通常是velodyne)
    
public:
    /**
     * @brief 构造函数
     */
    LidarOdometryCoordinateFix() : 
        tf_listener_(tf_buffer_), 
        static_transform_ready_(false) {
        
        // 加载参数
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("map_frame", map_frame_, "map");
        private_nh.param<std::string>("camera_frame", camera_frame_, "camera");
        private_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        private_nh.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
        
        // 初始化发布者
        corrected_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/laser_odometry", 10);
        
        // 等待并加载静态变换
        initializeStaticTransforms();
        
        ROS_INFO("LidarOdometryCoordinateFix initialized");
        ROS_INFO("  Map frame: %s", map_frame_.c_str());
        ROS_INFO("  Camera frame: %s", camera_frame_.c_str());
        ROS_INFO("  Base link frame: %s", base_link_frame_.c_str());
        ROS_INFO("  Lidar frame: %s", lidar_frame_.c_str());
    }
    
    /**
     * @brief 初始化静态变换
     * 
     * 获取从camera(或激光雷达)到base_link的静态变换关系
     */
    void initializeStaticTransforms() {
        ROS_INFO("Waiting for static transforms...");
        
        while (!static_transform_ready_ && ros::ok()) {
            try {
                // 尝试获取 camera -> base_link 变换
                geometry_msgs::TransformStamped transform_stamped;
                
                // 首先尝试camera -> base_link
                try {
                    transform_stamped = tf_buffer_.lookupTransform(
                        base_link_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
                    
                    tf2::fromMsg(transform_stamped.transform, camera_to_base_link_);
                    static_transform_ready_ = true;
                    
                    ROS_INFO("Successfully loaded %s -> %s transform:", 
                             camera_frame_.c_str(), base_link_frame_.c_str());
                    printTransform(camera_to_base_link_);
                    
                } catch (tf2::TransformException &ex) {
                    // 如果camera->base_link不存在，尝试lidar->base_link
                    ROS_DEBUG("Camera transform not found, trying lidar transform...");
                    
                    transform_stamped = tf_buffer_.lookupTransform(
                        base_link_frame_, lidar_frame_, ros::Time(0), ros::Duration(1.0));
                    
                    tf2::fromMsg(transform_stamped.transform, camera_to_base_link_);
                    static_transform_ready_ = true;
                    
                    ROS_INFO("Successfully loaded %s -> %s transform:", 
                             lidar_frame_.c_str(), base_link_frame_.c_str());
                    printTransform(camera_to_base_link_);
                    
                    // 更新camera_frame_以匹配实际使用的坐标系
                    camera_frame_ = lidar_frame_;
                }
                
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Failed to get static transform: %s", ex.what());
                ROS_WARN("Make sure static_transform_publisher is running:");
                ROS_WARN("  rosrun tf2_ros static_transform_publisher x y z qx qy qz qw %s %s", 
                         base_link_frame_.c_str(), camera_frame_.c_str());
                ros::Duration(1.0).sleep();
            }
        }
    }
    
    /**
     * @brief 打印变换信息
     */
    void printTransform(const tf2::Transform& transform) {
        tf2::Vector3 translation = transform.getOrigin();
        tf2::Quaternion rotation = transform.getRotation();
        double roll, pitch, yaw;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
        
        ROS_INFO("  Translation: [%.3f, %.3f, %.3f] m", 
                 translation.x(), translation.y(), translation.z());
        ROS_INFO("  Rotation: [%.3f, %.3f, %.3f] rad ([%.1f, %.1f, %.1f] deg)", 
                 roll, pitch, yaw, 
                 roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
    }
    
    /**
     * @brief 主要的坐标转换函数
     * 
     * 这个函数应该在你的激光里程计算法中调用，
     * 用来替换原有的TF发布和里程计发布代码
     * 
     * @param map_to_camera 原始的激光里程计结果 (map -> camera/lidar)
     * @param timestamp 时间戳
     * @param linear_velocity 线速度 (可选)
     * @param angular_velocity 角速度 (可选)
     */
    void publishCorrectedOdometry(const tf2::Transform& map_to_camera, 
                                  const ros::Time& timestamp,
                                  const geometry_msgs::Vector3& linear_velocity = geometry_msgs::Vector3(),
                                  const geometry_msgs::Vector3& angular_velocity = geometry_msgs::Vector3()) {
        
        if (!static_transform_ready_) {
            ROS_WARN_THROTTLE(1.0, "Static transform not ready, skipping publication");
            return;
        }
        
        // 计算修正后的变换: map -> base_link = map -> camera * camera -> base_link
        tf2::Transform map_to_base_link = map_to_camera * camera_to_base_link_;
        
        // 发布TF变换
        publishTransform(map_to_base_link, timestamp);
        
        // 发布里程计消息
        publishOdometry(map_to_base_link, timestamp, linear_velocity, angular_velocity);
    }
    
    /**
     * @brief 发布TF变换
     */
    void publishTransform(const tf2::Transform& map_to_base_link, const ros::Time& timestamp) {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = timestamp;
        transform_stamped.header.frame_id = map_frame_;
        transform_stamped.child_frame_id = base_link_frame_;
        
        tf2::toMsg(map_to_base_link, transform_stamped.transform);
        tf_broadcaster_.sendTransform(transform_stamped);
    }
    
    /**
     * @brief 发布里程计消息
     */
    void publishOdometry(const tf2::Transform& map_to_base_link, 
                        const ros::Time& timestamp,
                        const geometry_msgs::Vector3& linear_velocity,
                        const geometry_msgs::Vector3& angular_velocity) {
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_link_frame_;
        
        // 设置位姿
        tf2::toMsg(map_to_base_link, odom_msg.pose.pose);
        
        // 设置速度
        odom_msg.twist.twist.linear = linear_velocity;
        odom_msg.twist.twist.angular = angular_velocity;
        
        // 设置协方差矩阵 (根据你的算法精度调整)
        setOdometryCovariance(odom_msg);
        
        // 发布里程计
        corrected_odom_pub_.publish(odom_msg);
    }
    
    /**
     * @brief 设置里程计协方差矩阵
     */
    void setOdometryCovariance(nav_msgs::Odometry& odom_msg) {
        // 位置协方差 (根据激光里程计精度设置)
        odom_msg.pose.covariance[0] = 0.1;   // x位置方差
        odom_msg.pose.covariance[7] = 0.1;   // y位置方差  
        odom_msg.pose.covariance[14] = 0.2;  // z位置方差
        odom_msg.pose.covariance[21] = 0.05; // roll方差
        odom_msg.pose.covariance[28] = 0.05; // pitch方差
        odom_msg.pose.covariance[35] = 0.1;  // yaw方差
        
        // 速度协方差
        odom_msg.twist.covariance[0] = 0.05;  // vx方差
        odom_msg.twist.covariance[7] = 0.05;  // vy方差
        odom_msg.twist.covariance[14] = 0.1;  // vz方差
        odom_msg.twist.covariance[21] = 0.02; // vroll方差
        odom_msg.twist.covariance[28] = 0.02; // vpitch方差
        odom_msg.twist.covariance[35] = 0.05; // vyaw方差
    }
    
    /**
     * @brief 获取静态变换 (供外部调用)
     */
    bool getStaticTransform(tf2::Transform& camera_to_base_link) {
        if (static_transform_ready_) {
            camera_to_base_link = camera_to_base_link_;
            return true;
        }
        return false;
    }
    
    /**
     * @brief 检查是否就绪
     */
    bool isReady() const {
        return static_transform_ready_;
    }
};
