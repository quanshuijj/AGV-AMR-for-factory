#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TFCoordinateConverter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber global_odom_sub_;          // 订阅全局里程计
    tf2_ros::TransformBroadcaster tf_broadcaster_; // TF广播器
    std::string base_link_frame_;              // 机器人本体坐标系
    
    ros::Publisher local_odom_pub_;  //修正后里程计输出
    
    
    public:
    TFCoordinateConverter() {
        // 参数初始化
        nh_.param<std::string>("base_link_frame", base_link_frame_, "base_footprint");
        
        // 订阅全局里程计（原数据不变）
        global_odom_sub_ = nh_.subscribe("/global_odometry", 10, 
                                       &TFCoordinateConverter::globalOdomCallback, this);
        ROS_INFO("TFCoordinateConverter initialized. Publishing odom->base_link transforms...");
        // 发布修正后的局部里程计
        local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_fix", 10);
    }

private:
    /**
     * @brief 全局里程计回调函数
     * @brief 动态发布 odom->base_link 变换
     */
    void globalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 1. 构造TF变换消息
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();   // 使用当前时间戳
        transform.header.frame_id = "odom";           // 父坐标系：odom
        transform.child_frame_id = "base_footprint";//base_link_frame_;  // 子坐标系：base_link
        
        // 2. 从里程计消息提取位姿（数据不变）
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        // 3. 发布动态TF
        tf_broadcaster_.sendTransform(transform);
        
        //
        nav_msgs::Odometry odomMsg;
        odomMsg=*msg;
        odomMsg.header.frame_id = "odom"; 
        local_odom_pub_.publish(odomMsg);
        ROS_DEBUG_THROTTLE(1.0, "Published odom->%s transform", base_link_frame_.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_coordinate_converter");
    TFCoordinateConverter converter;
    ros::spin();
    return 0;
}
