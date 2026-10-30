#!/usr/bin/env python
import rospy
from hdl_graph_slam.srv import SaveMap, SaveMapRequest

def call_service():
    rospy.wait_for_service('/hdl_graph_slam/save_map')  # 等待服务可用
    try:
        # 创建服务代理
        save_service = rospy.ServiceProxy('/hdl_graph_slam/save_map', SaveMap)
        req = SaveMapRequest(resolution=LEAF_SIZE, destination=SAVE_PATH)  # 正确初始化请求
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
