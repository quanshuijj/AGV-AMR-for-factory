#include "lidar_odometry_coordinate_fix.cpp"

class StandaloneLidarOdometryFix {
private:
    ros::NodeHandle nh_;
    LidarOdometryCoordinateFix coordinate_fix_;
    
    // 订阅原始激光里程计输出
    ros::Subscriber original_odom_sub_;
    
    // 记录上一帧数据用于速度计算
    nav_msgs::Odometry last_odom_;
    bool last_odom_valid_;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    
public:
    StandaloneLidarOdometryFix() : last_odom_valid_(false) ,tfListener(tfBuffer){
        // 订阅原始的激光里程计输出 (map->camera格式)
        original_odom_sub_ = nh_.subscribe("/original_laser_odometry", 10, 
            &StandaloneLidarOdometryFix::originalOdomCallback, this);
        
        ROS_INFO("Standalone Lidar Odometry Fix started");
        ROS_INFO("Subscribing to: /original_laser_odometry");
        ROS_INFO("Publishing corrected odometry to: /laser_odometry");
    }
    
    // 2. 坐标转换函数
    geometry_msgs::PoseStamped transformCamToMap(
        const geometry_msgs::PoseStamped& cam_pose) 
    {
        geometry_msgs::PoseStamped map_pose;
        try {
            // 关键步骤：查询最新有效变换（避免时间跳跃）
            const ros::Time query_time = cam_pose.header.stamp;
            const ros::Duration timeout(0.1); 
        
            // 执行坐标变换
            map_pose = tfBuffer.transform(
                cam_pose, 
                "map",   // 目标坐标系
                timeout  // 超时设置
            );
        } 
        catch (tf2::TransformException &ex) {
            // 异常处理（分级策略）
            if (std::string(ex.what()).find("future") != std::string::npos) {
                 ROS_WARN_THROTTLE(1.0, "TF数据滞后：%s", ex.what());
             } else {
                ROS_ERROR("TF转换失败：%s", ex.what());
             }
              // 返回空姿态或使用预测机制
             map_pose.header.stamp = ros::Time(0); 
        }
        return map_pose;
    }
    
    /**
     * @brief 原始激光里程计回调函数
     */
    void originalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!coordinate_fix_.isReady()) {
            ROS_WARN_THROTTLE(1.0, "Coordinate fix not ready");
            return;
        }
        geometry_msgs::PoseStamped cam_pose;
        cam_pose.header = msg->header;       // 保持原始时间戳
        cam_pose.pose = msg->pose.pose;
        // 执行转换
        geometry_msgs::PoseStamped map_pose = transformCamToMap(cam_pose);
        
        // 从消息中提取变换
        tf2::Transform map_to_camera;
        tf2::fromMsg(map_pose.pose, map_to_camera);
        // 调试输出（包含位置和旋转信息）
            ROS_INFO_THROTTLE(1.0, "origin data: Pos[%.3f, %.3f, %.3f]", 
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y, 
            msg->pose.pose.position.z);
            
        nav_msgs::Odometry current_odom_=*msg;
        current_odom_.pose.pose=map_pose.pose;
        // 计算速度 (基于上一帧)
        geometry_msgs::Vector3 linear_vel, angular_vel;
        calculateVelocityFromOdom(current_odom_, linear_vel, angular_vel);
        
        // 发布修正后的里程计
        coordinate_fix_.publishCorrectedOdometry(
            map_to_camera,
            msg->header.stamp,
            linear_vel,
            angular_vel
        );
        // 保存当前帧用于下次速度计算
        last_odom_=current_odom_;
        //last_odom_ = *msg;
        //last_odom_.pose.pose=cam_pose.pose;
        last_odom_valid_ = true;    
        
    }
    
    /**
     * @brief 从里程计消息计算速度
     */
    void calculateVelocityFromOdom(const nav_msgs::Odometry& current_odom,
                                   geometry_msgs::Vector3& linear_vel,
                                   geometry_msgs::Vector3& angular_vel) {
        if (!last_odom_valid_) {
            // 第一帧，速度设为0
            linear_vel = geometry_msgs::Vector3();
            angular_vel = geometry_msgs::Vector3();
            return;
        }
        
        // 计算时间差
        double dt = (current_odom.header.stamp - last_odom_.header.stamp).toSec();
        if (dt <= 0 || dt > 1.0) {  // 避免异常时间间隔
            linear_vel = geometry_msgs::Vector3();
            angular_vel = geometry_msgs::Vector3();
            return;
        }
        
        // 计算位置差
        double dx = current_odom.pose.pose.position.x - last_odom_.pose.pose.position.x;
        double dy = current_odom.pose.pose.position.y - last_odom_.pose.pose.position.y;
        double dz = current_odom.pose.pose.position.z - last_odom_.pose.pose.position.z;
        
        // 计算线速度
        linear_vel.x = dx / dt;
        linear_vel.y = dy / dt;
        linear_vel.z = dz / dt;
        
        // 计算角速度
        tf2::Quaternion q_current, q_last;
        tf2::fromMsg(current_odom.pose.pose.orientation, q_current);
        tf2::fromMsg(last_odom_.pose.pose.orientation, q_last);
        
        // 计算角度差
        tf2::Quaternion q_diff = q_current * q_last.inverse();
        tf2::Matrix3x3 m(q_diff);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        angular_vel.x = roll / dt;
        angular_vel.y = pitch / dt;
        angular_vel.z = yaw / dt;
    }
};


/**
 * @brief 主函数 - 独立节点模式
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_odometry_coordinate_fix");
    ROS_INFO("test start");
    try {
        // 检查启动模式
        ros::NodeHandle private_nh("~");
        bool standalone_mode;
        private_nh.param<bool>("standalone_mode", standalone_mode, true);
        ROS_INFO("test start");
        
        if (standalone_mode) {
            ROS_INFO("Running in standalone mode");
            StandaloneLidarOdometryFix standalone_fix;
            ros::spin();
        } else {
            ROS_INFO("Running in library mode (for integration)");
            ROS_INFO("Create LidarOdometryCoordinateFix object in your code");
            // 这种模式下，LidarOdometryCoordinateFix 被作为库使用
        }
        
    } catch (std::exception& e) {
        ROS_ERROR("Lidar Odometry Coordinate Fix failed: %s", e.what());
        return -1;
    }
    
    return 0;
}

