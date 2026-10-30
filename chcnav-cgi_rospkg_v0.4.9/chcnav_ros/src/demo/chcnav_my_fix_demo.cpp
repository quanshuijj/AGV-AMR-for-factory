#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <map>
#include <string>

using namespace std;

#define M_PI 3.14159265358979323846

/**
 * @brief hcinspvatzcb msg callback
 *
 * @param msg
 * */
static void pvt_callback(const chcnav::hcinspvatzcb::ConstPtr &msg);

static ros::Publisher gs_imu_pub;
static ros::Publisher gs_fix_pub;
static ros::Publisher fix_imu_pub;
int numCount=0;
// 添加全局参数句柄
static ros::NodeHandle* global_nh_ptr = nullptr;

/**
 * @brief broadcaste static tf.
 * */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "chcnav_fix_demo");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    global_nh_ptr = &nh;

    tf2_ros::StaticTransformBroadcaster stf_pub;
    tf2::Quaternion qtn;

    geometry_msgs::TransformStamped ts;

    ts.header.seq = 1;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "map";
    ts.child_frame_id = "chcnav";

    ts.transform.translation.x = 0;
    ts.transform.translation.y = 0;
    ts.transform.translation.z = 0;

    qtn.setRPY(0, 0, 0);

    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    stf_pub.sendTransform(ts);

    ts.header.frame_id = "chcnav";
    ts.child_frame_id = "c_rs232";

    stf_pub.sendTransform(ts);

    gs_imu_pub = private_nh.advertise<sensor_msgs::Imu>("imu", 1000);
    gs_fix_pub = private_nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);
    fix_imu_pub=private_nh.advertise<sensor_msgs::Imu>("myimu", 1000);

    ros::Subscriber pvt_source = private_nh.subscribe("/chcnav/devpvt", 1000, pvt_callback);

    // 初始化参数服务器值
    nh.setParam("gps_xy_stdev", 0.02);
    nh.setParam("gps_z_stdev", 0.05);
    
    ros::spin();

    return 0;
}


void set_covariances(sensor_msgs::Imu& imu_msg, uint8_t gnss_status) {
        // 角速度协方差（固定值，根据传感器噪声调整）
        imu_msg.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
        
        // 线性加速度协方差
        imu_msg.linear_acceleration_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
        
        // 姿态协方差：根据GNSS定位精度动态设置
        if (gnss_status >= 4) {  // RTK稳定解或浮点解
            imu_msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
        } else {  // 单点或差分模式
            imu_msg.orientation_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
        }
}

static void pvt_callback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    sensor_msgs::Imu imu;
    sensor_msgs::NavSatFix fix;
    sensor_msgs::Imu myimu;
    numCount++;
    
    // publish NavSatFix msg
    fix.header = msg->header;
    fix.altitude = msg->altitude;
    fix.longitude = msg->longitude;
    fix.latitude = msg->latitude;
    fix.status.service = fix.status.SERVICE_COMPASS;

    if (msg->stat[1] == 4 || msg->stat[1] == 8||msg->stat[1] == 5 || msg->stat[1] == 9)
        fix.status.status = fix.status.STATUS_FIX;
    else
        fix.status.status = fix.status.STATUS_NO_FIX;

    // 添加位置协方差设置[6](@ref)
    if(msg->position_stdev.size() >= 3) {
        // 计算水平位置标准差 (stdevxy = sqrt(σ_lat² + σ_lon²))
        double stdevxy = std::sqrt(
            msg->position_stdev[0] * msg->position_stdev[0] +
            msg->position_stdev[1] * msg->position_stdev[1]
        );
        
        // 获取垂直位置标准差 (stdevz = σ_alt)
        double stdevz = msg->position_stdev[2];
        
        // 设置到参数服务器[7,8](@ref)
        global_nh_ptr->setParam("gps_xy_stdev", stdevxy);
        global_nh_ptr->setParam("gps_z_stdev", stdevz);
        
        // 设置NavSatFix的协方差矩阵[6](@ref)
        fix.position_covariance[0] = msg->position_stdev[0] * msg->position_stdev[0];  // lat variance
        fix.position_covariance[4] = msg->position_stdev[1] * msg->position_stdev[1];  // lon variance
        fix.position_covariance[8] = stdevz * stdevz;  // alt variance
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        
        // 每5秒输出一次参数值（避免日志过多）
        static ros::Time last_log = ros::Time(0);
        if(ros::Time::now() - last_log > ros::Duration(5.0)) {
            ROS_INFO("更新GNSS标准差参数: stdevxy=%.3fm, stdevz=%.3fm", stdevxy, stdevz);
            last_log = ros::Time::now();
        }
    }
    
    if(numCount>=10)
    {
      gs_fix_pub.publish(fix);
      //ROS_INFO("  计数器清零: (%.6f)", numCount);
      numCount=0;
    }
    //gs_fix_pub.publish(fix);

    // publish imu msg
    imu.header = msg->header;
    imu.header.stamp = ros::Time::now();

    imu.angular_velocity.x = msg->vehicle_angular_velocity.x / 180 * M_PI;
    imu.angular_velocity.y = msg->vehicle_angular_velocity.y / 180 * M_PI;
    imu.angular_velocity.z = msg->vehicle_angular_velocity.z / 180 * M_PI;

    imu.linear_acceleration.x = msg->vehicle_linear_acceleration.x;
    imu.linear_acceleration.y = msg->vehicle_linear_acceleration.y;
    imu.linear_acceleration.z = msg->vehicle_linear_acceleration.z;

    // yaw: 车体坐标系下双天线航向角，取值范围 [-180, +180]，遵循右手定则，逆时针为正。
    // heading: 为车体坐标系下速度航向角，也称航迹角。取值范围 [0, 360] ，顺时针为正。
    // heading2: 为车体坐标系下双天线航向角，取值范围 [0, 360] ，顺时针为正。
    float yaw = 0.0;
    if (msg->yaw <= 180)
        yaw = -1 * msg->heading2;
    else
        yaw = 360 - msg->heading2;

    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll / 180 * M_PI, -msg->pitch / 180 * M_PI, yaw / 180 * M_PI);

    gs_imu_pub.publish(imu);
    
    // publish my imu msg
    myimu.header=msg->header;
    //myimu.header.stamp = ros::Time::now();
    myimu.header.stamp = ros::Time(msg->week * 7 * 86400 + msg->second); // 将GPS周+秒转为Unix时间戳
    myimu.header.frame_id = "imu_link"; 
    
    myimu.angular_velocity.x = msg->raw_angular_velocity.x / 180 * M_PI;
    myimu.angular_velocity.y = msg->raw_angular_velocity.y / 180 * M_PI;
    myimu.angular_velocity.z = msg->raw_angular_velocity.z / 180 * M_PI;
    
    myimu.linear_acceleration.x = msg->raw_acceleration.x;
    myimu.linear_acceleration.y = msg->raw_acceleration.y;
    myimu.linear_acceleration.z = msg->raw_acceleration.z;
    
    //myimu.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll / 180 * M_PI, msg->pitch / 180 * M_PI, yaw / 180 * M_PI);
    /*tf2::Quaternion q;
    q.setRPY(
        msg->roll * M_PI / 180.0,   // roll (deg→rad)
        msg->pitch * M_PI / 180.0,  // pitch (deg→rad)
        msg->yaw * M_PI / 180.0      // yaw (deg→rad)
    );
    myimu.orientation = tf2::toMsg(q);*/
    //set_covariances(myimu, msg->stat[1]);
    myimu.orientation_covariance[0] = -1;   
    fix_imu_pub.publish(myimu);
}
