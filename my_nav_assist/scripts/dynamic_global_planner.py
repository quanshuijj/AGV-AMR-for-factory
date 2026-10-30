#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态全局规划频率调节器
功能：平时低频全局规划，前方4米内检测到障碍物时快速全局重规划

使用方法：
  rosrun my_nav_assist dynamic_global_planner.py
  
或在launch文件中：
  <node pkg="my_nav_assist" type="dynamic_global_planner.py" name="dynamic_global_planner" output="screen">
    <param name="obstacle_distance" value="4.0"/>
    <param name="normal_freq" value="0.2"/>
    <param name="fast_freq" value="2.0"/>
  </node>
"""

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

class DynamicGlobalPlanner:
    def __init__(self):
        rospy.init_node('dynamic_global_planner', anonymous=True)
        
        # ========== 参数配置 ==========
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 4.0)  # 检测距离4米
        self.detection_angle = rospy.get_param('~detection_angle', 60.0)     # 前方±30度
        self.normal_freq = rospy.get_param('~normal_freq', 0.2)              # 正常：0.2Hz（每5秒）
        self.fast_freq = rospy.get_param('~fast_freq', 2.0)                  # 快速：2Hz（每0.5秒）
        self.move_base_name = rospy.get_param('~move_base_name', 'move_base_node')
        
        # 状态变量
        self.obstacle_in_front = False
        self.current_mode = 'normal'  # 'normal' or 'fast'
        self.last_switch_time = rospy.Time.now()
        self.switch_cooldown = rospy.Duration(1.0)  # 切换冷却时间
        
        # 动态配置客户端
        self.client = None
        self.connect_to_move_base()
        
        # 订阅局部代价地图
        self.costmap_sub = rospy.Subscriber(
            f'/{self.move_base_name}/local_costmap/costmap',
            OccupancyGrid,
            self.costmap_callback,
            queue_size=1
        )
        
        # 订阅点云（备选检测方式）
        self.pc_sub = rospy.Subscriber(
            '/velodyne_points',
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1
        )
        
        # 定时检查
        self.timer = rospy.Timer(rospy.Duration(0.2), self.check_and_adjust)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("[动态全局规划] 启动成功")
        rospy.loginfo(f"  检测距离: {self.obstacle_distance}m")
        rospy.loginfo(f"  正常频率: {self.normal_freq}Hz (每{1/self.normal_freq:.1f}秒)")
        rospy.loginfo(f"  快速频率: {self.fast_freq}Hz (每{1/self.fast_freq:.1f}秒)")
        rospy.loginfo("=" * 50)
        
    def connect_to_move_base(self):
        """连接move_base动态配置"""
        try:
            self.client = dynamic_reconfigure.client.Client(
                f'/{self.move_base_name}',
                timeout=5.0
            )
            rospy.loginfo(f"[动态全局规划] 已连接 /{self.move_base_name}")
        except Exception as e:
            rospy.logwarn(f"[动态全局规划] 连接失败: {e}")
            
    def costmap_callback(self, msg):
        """从局部代价地图检测前方障碍物"""
        try:
            resolution = msg.info.resolution
            width = msg.info.width
            height = msg.info.height
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            
            # 机器人在地图中的位置
            robot_x = int(-origin_x / resolution)
            robot_y = int(-origin_y / resolution)
            
            # 检测范围
            check_dist = int(self.obstacle_distance / resolution)
            angle_rad = math.radians(self.detection_angle / 2)
            
            data = np.array(msg.data).reshape((height, width))
            
            # 扫描前方区域
            for dist in range(5, min(check_dist, width - robot_x)):  # 从0.25m开始
                for angle in np.linspace(-angle_rad, angle_rad, 15):
                    cx = robot_x + int(dist * math.cos(angle))
                    cy = robot_y + int(dist * math.sin(angle))
                    
                    if 0 <= cx < width and 0 <= cy < height:
                        if data[cy, cx] >= 90:  # 高代价或致命障碍物
                            self.obstacle_in_front = True
                            return
                            
            self.obstacle_in_front = False
            
        except Exception as e:
            pass
            
    def pointcloud_callback(self, msg):
        """从点云检测前方障碍物（备选）"""
        try:
            angle_limit = math.radians(self.detection_angle / 2)
            
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point
                
                # 前方检测：x>0.5且<4m，高度0.3-2.5m
                if 0.5 < x < self.obstacle_distance and 0.3 < z < 2.5:
                    angle = abs(math.atan2(y, x))
                    if angle < angle_limit:
                        self.obstacle_in_front = True
                        return
                        
        except Exception as e:
            pass
            
    def check_and_adjust(self, event):
        """检查并调整全局规划频率"""
        now = rospy.Time.now()
        
        # 冷却时间检查
        if (now - self.last_switch_time) < self.switch_cooldown:
            return
            
        if self.client is None:
            self.connect_to_move_base()
            return
            
        try:
            if self.obstacle_in_front and self.current_mode != 'fast':
                # 检测到障碍物 → 切换到快速模式
                rospy.logwarn(f"[动态全局规划] ⚠️ 前方{self.obstacle_distance}m内有障碍物 → 快速重规划 {self.fast_freq}Hz")
                self.client.update_configuration({'planner_frequency': self.fast_freq})
                self.current_mode = 'fast'
                self.last_switch_time = now
                
            elif not self.obstacle_in_front and self.current_mode != 'normal':
                # 无障碍物 → 恢复正常模式
                rospy.loginfo(f"[动态全局规划] ✓ 前方无障碍 → 恢复低频规划 {self.normal_freq}Hz")
                self.client.update_configuration({'planner_frequency': self.normal_freq})
                self.current_mode = 'normal'
                self.last_switch_time = now
                
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[动态全局规划] 配置更新失败: {e}")
            
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DynamicGlobalPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
