import rospy
import ros_numpy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2  # 明确导入 PointCloud2 类型

def pointcloud2_to_pcd(msg, output_path):
    # 将 PointCloud2 转为 NumPy 数组，提取 XYZ 坐标
    pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    points = np.zeros((pc_array.shape[0], 3), dtype=np.float32)
    points[:, 0] = pc_array['x']
    points[:, 1] = pc_array['y']
    points[:, 2] = pc_array['z']

    # 创建 PCL 点云对象并保存
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_array(points)
    pcl.save(pcl_cloud, output_path)
    rospy.loginfo(f"Saved PCD to {output_path}")

# 示例调用
def callback(msg):
    pointcloud2_to_pcd(msg, "/home/lujiaqi/navCarProject_ws/src/test_scence/PCD/cloud.pcd")

rospy.init_node('pcd_saver')
rospy.Subscriber("/pointcloud_topic", PointCloud2, callback)
rospy.spin()
