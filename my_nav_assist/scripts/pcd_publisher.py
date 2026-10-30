#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import os

class PCDPublisher:
    def __init__(self):
        rospy.init_node('pcd_map_publisher', anonymous=True)
        
        # 参数配置
        self.pcd_path = rospy.get_param('~pcd_path', 'maps/map.pcd')  # 默认路径
        self.frame_id = rospy.get_param('~frame_id', 'map')           # 坐标系
        self.pub_rate = rospy.get_param('~rate', 1.0)                 # 发布频率(Hz)
        
        # 发布器初始化
        self.pub = rospy.Publisher('/global_pointcloud_map', PointCloud2, queue_size=1)
        
        # 加载点云
        self.cloud_data = self.load_pcd()
        if self.cloud_data is None:
            rospy.logerr("Failed to load PCD file. Shutting down.")
            rospy.signal_shutdown("PCD load failure")
        
        rospy.loginfo(f"Loaded PCD with {len(self.cloud_data)} points")

    def load_pcd(self):
        """读取PCD文件并返回点云坐标数组"""
        try:
            if not os.path.exists(self.pcd_path):
                rospy.logerr(f"PCD file not found: {self.pcd_path}")
                return None
            
            pcd = o3d.io.read_point_cloud(self.pcd_path)
            return np.asarray(pcd.points)
        except Exception as e:
            rospy.logerr(f"PCD read error: {str(e)}")
            return None

    def publish_pointcloud(self):
        """将点云数据转为ROS消息并发布"""
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        
        # 构建PointCloud2消息[6,7](@ref)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg = PointCloud2(
            header=header,
            height=1,  # 无序点云
            width=len(self.cloud_data),
            fields=fields,
            is_bigendian=False,
            point_step=12,  # 每个点12字节 (3个float32)
            row_step=12 * len(self.cloud_data),
            data=self.cloud_data.astype(np.float32).tobytes(),
            is_dense=True  # 无无效点
        )
        self.pub.publish(cloud_msg)

    def run(self):
        rate = rospy.Rate(self.pub_rate)
        rospy.loginfo(f"Publishing point cloud on /global_pointcloud_map at {self.pub_rate}Hz")
        while not rospy.is_shutdown():
            self.publish_pointcloud()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PCDPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
