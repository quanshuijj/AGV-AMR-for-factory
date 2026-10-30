#!/usr/bin/env python
import rospy
from direct_lidar_odometry.srv import save_pcd, save_pcdRequest

def call_service():
    rospy.wait_for_service('/robot/dlo_map/save_pcd')  # 等待服务可用
    try:
        # 创建服务代理
        save_service = rospy.ServiceProxy('/robot/dlo_map/save_pcd', save_pcd)
        req = save_pcdRequest(leaf_size=LEAF_SIZE, save_path=SAVE_PATH)  # 正确初始化请求
        resp = save_service(req)  # 传入请求对象
        rospy.loginfo("Save PCD result: %s", resp.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('pcd_saver_node')
    # 从参数服务器获取参数（或直接硬编码）
    LEAF_SIZE = rospy.get_param('~leaf_size', 0.05)  # 默认值0.05
    SAVE_PATH = rospy.get_param('~save_path', '/tmp/map.pcd')
    call_service()
