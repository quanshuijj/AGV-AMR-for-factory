#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def generate_scene():
    points = []
    
    # 生成地面和天花板（20x7平面）
    for x in np.arange(0, 20, 0.1):
        for y in np.arange(-3.5, 3.5, 0.1):
            points.append([x, y, 0.0])   # 地面（Z=0）
            points.append([x, y, 3.0])   # 天花板（Z=3）
    
    # 生成两侧墙壁（避开门口）
    wall_opening_positions = [5.0, 10.0, 15.0]  # 门洞中心位置
    for x in np.arange(0, 20, 0.1):
        for z in np.arange(0, 3, 0.1):
            # 左侧墙（Y=-3.5）
            if all(abs(x - pos) > 1.5 for pos in wall_opening_positions):  # 避开门洞
                points.append([x, -3.5, z])
            # 右侧墙（Y=3.5）
            if all(abs(x - pos) > 1.5 for pos in wall_opening_positions):
                points.append([x, 3.5, z])
    
    # 添加柱状障碍物（位置随机但确保可通过性）
    obstacle_positions = [
        [8.0, 0.0], [12.0, 1.2], [18.0, -1.0]  # 坐标[X,Y]
    ]
    for center_x, center_y in obstacle_positions:
        for angle in np.arange(0, 2*np.pi, 0.1):
            for z in np.arange(0, 2, 0.1):
                x = center_x + 0.3 * np.cos(angle)
                y = center_y + 0.3 * np.sin(angle)
                points.append([x, y, z])
    
    return np.array(points)

def publish_pointcloud():
    rospy.init_node('scene_publisher')
    pub = rospy.Publisher('/navigation_scene', PointCloud2, queue_size=1)
    scene_points = generate_scene()
    
    # 转换为ROS消息
    header = rospy.Header(frame_id="map", stamp=rospy.Time.now())
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    pc_msg = pc2.create_cloud(header, fields, scene_points)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(pc_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pointcloud()
    except rospy.ROSInterruptException:
        pass
