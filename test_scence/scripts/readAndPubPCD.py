import numpy as np
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy


def pcd_to_pointcloud2(pcd_file, frame_id="base_link", axis='x', degrees=90):
    cloud = o3d.io.read_point_cloud(pcd_file)
    if not cloud.has_points():
        raise ValueError("PCD文件无有效点云数据")

    # 处理旋转轴
    rotation_axis = {'x': [1.0, 0.0, 0.0], 'y': [0.0, 1.0, 0.0]}.get(axis.lower())
    if rotation_axis is None:
        raise ValueError("旋转轴必须是 'x' 或 'y'")

    # === 修正点1：确保数组内存连续且类型精确 ===
    axis_vector = np.array(rotation_axis, dtype=np.float64).reshape(3, 1).copy()
    axis_vector = np.ascontiguousarray(axis_vector)  # 强制C连续内存[1](@ref)

    theta = np.radians(degrees)
    try:
        # 尝试调用Open3D API
        R = cloud.get_rotation_matrix_from_axis_angle(axis_vector, theta)
    except TypeError:
        # 若失败则手动构造矩阵（绕X/Y轴90°）
        if axis.lower() == 'x':
            R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
        else:  # 'y'
            R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)

    cloud.rotate(R, center=(0, 0, 0))
    # 转换为ROS消息
    points = np.asarray(cloud.points)
    structured_data = np.zeros(len(points), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
    structured_data['x'], structured_data['y'], structured_data['z'] = points[:, 0], points[:, 1], points[:, 2]

    cloud_msg = ros_numpy.msgify(PointCloud2, structured_data)
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = frame_id
    cloud_msg.height = 1  # 非有序点云设为1
    cloud_msg.width = points.shape[0]  # 点数

    return cloud_msg



if __name__ == "__main__":
    rospy.init_node('pcd_publisher')
    pub = rospy.Publisher('/pointcloud_topic', PointCloud2, queue_size=10)

    pcd_path = rospy.get_param('~pcd_path', 'test.pcd')
    rotate_axis = "x"

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            cloud_msg = pcd_to_pointcloud2(pcd_path, axis=rotate_axis,degrees=0)
            pub.publish(cloud_msg)
            rospy.loginfo(f"已发布旋转后的点云 (绕{rotate_axis.upper()}轴旋转90°)")
        except Exception as e:
            rospy.logerr(f"处理错误: {str(e)}")
        rate.sleep()
