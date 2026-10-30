#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf

def publish_fake_data():
    rospy.init_node('fake_sensors')
    
    # 初始化发布器
    odom_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=10)
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        # 发布静态轮速里程计 (沿X轴匀速运动)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = 1.0 * rospy.get_time()  # 随时间增加
        odom.pose.pose.orientation.w = 1.0  # 无旋转
        odom_pub.publish(odom)
        
        # 发布静态IMU (仅提供Yaw角速度)
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "base_link"
        imu.angular_velocity.z = 0.1  # 固定角速度
        imu.linear_acceleration.z = 9.8  # 重力补偿
        imu_pub.publish(imu)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_data()
    except rospy.ROSInterruptException:
        pass
