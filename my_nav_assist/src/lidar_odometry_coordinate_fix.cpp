#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

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
    // 新增：坐标变换有效性检查[1](@ref)
    bool isValidTransform(const tf2::Transform& transform) {
        const tf2::Vector3& origin = transform.getOrigin();
        return !(std::isnan(origin.x()) || std::isnan(origin.y()) || std::isnan(origin.z()));
    }
    
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
        
        while (ros::ok() && !static_transform_ready_) {
            try {
                // 优化：增加超时时间和重试机制[7](@ref)
                geometry_msgs::TransformStamped transform_stamped = 
                    tf_buffer_.lookupTransform(base_link_frame_, camera_frame_, 
                                              ros::Time(0), ros::Duration(2.0));
                tf2::fromMsg(transform_stamped.transform, camera_to_base_link_);
                static_transform_ready_ = true;
                printTransform(camera_to_base_link_);
                
            } catch (const tf2::TransformException& ex) {
                try {
                    ROS_WARN("Camera transform not found, trying lidar: %s", ex.what());
                    
                    geometry_msgs::TransformStamped transform_stamped = 
                        tf_buffer_.lookupTransform(base_link_frame_, lidar_frame_, 
                                                  ros::Time(0), ros::Duration(2.0));
                    
                    tf2::fromMsg(transform_stamped.transform, camera_to_base_link_);
                    static_transform_ready_ = true;
                    camera_frame_ = lidar_frame_;
                    printTransform(camera_to_base_link_);
                    
                } catch (const tf2::TransformException& ex2) {
                    ROS_ERROR("Static transform failure: %s", ex2.what());
                    ros::Duration(1.0).sleep();
                }
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
            ROS_WARN_THROTTLE(5.0, "Static transform not ready");
            return;
        }
        
        // 新增：变换有效性检查
        if (!isValidTransform(map_to_camera)) {
            ROS_ERROR("Invalid input transform detected!");
            return;
        }
        
        try {
            //tf2::Transform cam_to_velodyne;
            //tf2::Quaternion rotation;
            //rotation.setRPY(0, -M_PI/2, 0); // 绕Y轴旋转-90度
            //cam_to_velodyne.setRotation(rotation);
            //cam_to_velodyne.setOrigin(tf2::Vector3(0, 0, 0));
            //tf2::Transform map_to_velodyne = map_to_camera * cam_to_velodyne;
            //tf2::Transform map_to_base_link = map_to_velodyne * camera_to_base_link_;
            tf2::Transform map_to_base_link = map_to_camera * camera_to_base_link_;
            
            // 获取map_to_camera的旋转欧拉角
            double roll_cam, pitch_cam, yaw_cam;
            tf2::Matrix3x3(map_to_camera.getRotation()).getRPY(roll_cam, pitch_cam, yaw_cam);

            // 获取map_to_base_link的旋转欧拉角
            double roll_base, pitch_base, yaw_base;
            tf2::Matrix3x3(map_to_base_link.getRotation()).getRPY(roll_base, pitch_base, yaw_base);

            // 调试输出（包含位置和旋转信息）
            ROS_INFO_THROTTLE(1.0, "map_to_camera: Pos[%.3f, %.3f, %.3f] Rot[%.3f, %.3f, %.3f] rad", 
            map_to_camera.getOrigin().x(), 
            map_to_camera.getOrigin().y(), 
            map_to_camera.getOrigin().z(),
            roll_cam, pitch_cam, yaw_cam);

            ROS_INFO_THROTTLE(1.0, "map_to_base_link: Pos[%.3f, %.3f, %.3f] Rot[%.3f, %.3f, %.3f] rad", 
            map_to_base_link.getOrigin().x(), 
            map_to_base_link.getOrigin().y(), 
            map_to_base_link.getOrigin().z(),
            roll_base, pitch_base, yaw_base);
            
            if (!isValidTransform(map_to_base_link)) {
                ROS_ERROR("Computed transform invalid!");
                return;
            }
            
            //publishTransform(map_to_base_link, timestamp);
            publishOdometry(map_to_base_link, timestamp, linear_velocity, angular_velocity);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Transform computation error: %s", e.what());
        }
    }
    
    /**
     * @brief 发布TF变换
     */
    void publishTransform(const tf2::Transform& map_to_base_link, const ros::Time& timestamp) {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = timestamp;
        transform_stamped.header.frame_id = map_frame_;
        transform_stamped.child_frame_id = base_link_frame_;
        
        // 修复方案1：直接赋值
        transform_stamped.transform = tf2::toMsg(map_to_base_link);
        
        // 备用方案：手动转换（若方案1失败）
        /*
        transform_stamped.transform.translation.x = map_to_base_link.getOrigin().x();
        transform_stamped.transform.translation.y = map_to_base_link.getOrigin().y();
        transform_stamped.transform.translation.z = map_to_base_link.getOrigin().z();
        transform_stamped.transform.rotation = tf2::toMsg(map_to_base_link.getRotation());
        */
        
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
