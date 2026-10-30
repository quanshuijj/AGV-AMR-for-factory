#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航系统监控程序 v2.0
======================
监控从cmd_vel到CAN执行的完整数据链路

数据流监控：
  /cmd_vel (TEB输出)
      ↓
  ROSWorkManager.velCMDCallback
      ↓
  SetVelCMDSignal(vx, rz)
      ↓
  NavCarManagerNode.velDataHandle
      ↓
  CalIKControlModel.calAKMControlParams_new
      ↓
  sendVelControlMsg → CAN 0x8F910D8
      ↓
  底盘电机执行 → CAN反馈 0x4Fx10D0

功能：
1. 实时监控cmd_vel、odometry、TF、路径等关键数据
2. 监控CAN发送指令和电机反馈
3. 检测异常状态（协方差过大、速度超限、指令延迟等）
4. 计算路径跟踪误差和控制延迟
5. 程序退出时自动保存完整日志到CSV文件

作者: NavCar项目组
日期: 2026-01-24
"""

import rospy
import signal
import sys
import os
import csv
import math
import threading
import numpy as np
from datetime import datetime
from collections import deque
import struct
import subprocess
import select
import time

# ROS消息类型
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Int32, Float64MultiArray
from tf2_msgs.msg import TFMessage
import tf2_ros

# ==================== 配置参数 ====================
class Config:
    # 日志保存路径
    LOG_DIR = os.path.expanduser("~/navCarProject_ws/logs")
    
    # 监控频率
    DISPLAY_RATE = 2.0  # Hz，终端显示刷新率
    LOG_RATE = 10.0     # Hz，数据记录频率
    
    # 异常阈值
    MAX_LINEAR_VEL = 0.5        # m/s，最大线速度
    MAX_ANGULAR_VEL = 0.1       # rad/s，最大角速度（液压转向限制）
    MAX_COVARIANCE = 10.0       # 协方差异常阈值
    TF_TIMEOUT = 1.0            # TF超时阈值（秒）
    CMD_VEL_TIMEOUT = 2.0       # cmd_vel超时阈值（秒）
    
    # TEB配置的速度限制（用于对比）
    TEB_MAX_VEL_X = 0.1
    TEB_MAX_VEL_THETA = 0.07
    
    # 历史数据缓存大小
    HISTORY_SIZE = 1000
    
    # CAN配置
    CAN_INTERFACE = 'can0'
    CAN_VEL_CMD_ID = 0x8F910D8      # 速度控制指令ID
    CAN_STEERING_CMD_ID = 0x8F110D8  # 转向控制指令ID
    CAN_MOTOR_CMD_ID = 0x8F210D8     # 电机控制指令ID
    
    # 电机反馈CAN ID
    CAN_MOTOR_FB_IDS = [0x4F110D0, 0x4F210D0, 0x4F310D0, 0x4F410D0]
    
    # 编码器反馈CAN ID（转向角度反馈）
    CAN_ENCODER_IDS = [0x191, 0x192, 0x193, 0x194]
    
    # 控制参数转换系数（与calikcontrolmodel.cpp一致）
    VEL_RATIO_PARAMS = 0.002        # 速度转换系数
    STEERING_RATIO_PARAMS = 0.5     # 转向角度转换系数（度）
    MAX_STEERING_ANGLE_DEG = 35.0   # 最大转向角度

# ==================== CAN数据解析器 ====================
class CANDataParser:
    """CAN数据解析器 - 解析速度控制指令和电机反馈"""
    
    @staticmethod
    def parse_vel_cmd(data):
        """
        解析速度控制指令 (CAN ID: 0x8F910D8)
        数据格式: [控制模式, 速度值, 速度方向, 转向值, 转向方向]
        """
        if len(data) < 5:
            return None
        
        control_mode = data[0]
        vel_unsigned = data[1]
        vel_direction = data[2]  # 0=前进, 1=后退
        theta_unsigned = data[3]
        theta_direction = data[4]  # 0=左转, 1=右转
        
        # 转换为实际值
        actual_vel = vel_unsigned * Config.VEL_RATIO_PARAMS
        if vel_direction == 1:
            actual_vel = -actual_vel
        
        actual_theta = theta_unsigned * Config.STEERING_RATIO_PARAMS
        if theta_direction == 1:
            actual_theta = -actual_theta
        
        return {
            'control_mode': control_mode,
            'vel_raw': vel_unsigned,
            'vel_dir': vel_direction,
            'theta_raw': theta_unsigned,
            'theta_dir': theta_direction,
            'actual_vel': actual_vel,
            'actual_theta': actual_theta
        }
    
    @staticmethod
    def parse_motor_feedback(can_id, data):
        """
        解析电机反馈 (CAN ID: 0x4Fx10D0)
        """
        if len(data) < 8:
            return None
        
        # 解析电机组号
        motor_group = 0
        if can_id == 0x4F110D0:
            motor_group = 1
        elif can_id == 0x4F210D0:
            motor_group = 2
        elif can_id == 0x4F310D0:
            motor_group = 3
        elif can_id == 0x4F410D0:
            motor_group = 4
        
        # 解析使能状态
        enable_byte = data[0]
        enabled = (enable_byte == 0x0D or enable_byte == 0x00)
        
        # 解析电机编号
        motor_number = data[1]
        
        # 解析转速 (第5、6字节，大端序)
        speed_raw = (data[4] << 8) | data[5]
        
        # 转换为实际转速
        if speed_raw > 32767:
            rpm = (65535.0 - speed_raw) / 10000.0 * 3000.0 / 24.7
        else:
            rpm = -speed_raw / 10000.0 * 3000.0 / 24.7
        
        return {
            'motor_group': motor_group,
            'motor_number': motor_number,
            'enabled': enabled,
            'speed_raw': speed_raw,
            'rpm': rpm
        }
    
    @staticmethod
    def parse_encoder_feedback(can_id, data):
        """
        解析编码器反馈 (CAN ID: 0x191-0x194)
        编码器用于反馈实际转向角度
        数据格式: [低位, 高位]
        角度计算: (数值/8192*360-180) 度
        """
        if len(data) < 2:
            return None
        
        # 确定编码器ID
        encoder_id = 0
        if can_id == 0x191:
            encoder_id = 1  # 右前
        elif can_id == 0x192:
            encoder_id = 2  # 左前
        elif can_id == 0x193:
            encoder_id = 3  # 右后
        elif can_id == 0x194:
            encoder_id = 4  # 左后
        
        # 解析编码器原始值 (低位在前，高位在后)
        encoder_raw = (data[1] << 8) | data[0]
        
        # 计算角度: (数值/8192*360-180)
        angle = (encoder_raw / 8192.0 * 360.0) - 180.0
        
        return {
            'encoder_id': encoder_id,
            'encoder_raw': encoder_raw,
            'angle': angle  # 实际转向角度（度）
        }

# ==================== CAN监控器 ====================
class CANMonitor:
    """CAN总线监控器 - 使用candump命令监控CAN数据"""
    
    def __init__(self, interface='can0'):
        self.interface = interface
        self.running = False
        self.process = None
        self.thread = None
        
        # 数据回调
        self.vel_cmd_callback = None
        self.motor_fb_callback = None
        self.encoder_fb_callback = None
        
        # 最新数据
        self.latest_vel_cmd = None
        self.latest_motor_fb = {}
        self.latest_encoder_fb = {}  # 编码器反馈（转向角度）
        self.lock = threading.Lock()
        
        # 统计
        self.vel_cmd_count = 0
        self.motor_fb_count = 0
        self.encoder_fb_count = 0
    
    def start(self, vel_cmd_callback=None, motor_fb_callback=None, encoder_fb_callback=None):
        """启动CAN监控"""
        self.vel_cmd_callback = vel_cmd_callback
        self.motor_fb_callback = motor_fb_callback
        self.encoder_fb_callback = encoder_fb_callback
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        rospy.loginfo(f"CAN监控已启动 (接口: {self.interface})")
    
    def stop(self):
        """停止CAN监控"""
        self.running = False
        if self.process:
            self.process.terminate()
            self.process = None
        if self.thread:
            self.thread.join(timeout=2.0)
    
    def _monitor_loop(self):
        """监控循环"""
        try:
            # 启动candump进程
            cmd = ['candump', self.interface, '-t', 'A']  # -t A: 绝对时间戳
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            while self.running:
                # 非阻塞读取
                if self.process.poll() is not None:
                    rospy.logwarn("candump进程已退出")
                    break
                
                line = self.process.stdout.readline()
                if line:
                    self._parse_candump_line(line.strip())
                    
        except FileNotFoundError:
            rospy.logwarn("candump命令未找到，CAN监控功能不可用")
        except Exception as e:
            rospy.logerr(f"CAN监控异常: {e}")
    
    def _parse_candump_line(self, line):
        """解析candump输出行"""
        # 格式: (1234567890.123456)  can0  08F910D8   [5]  01 45 00 3D 00
        try:
            parts = line.split()
            if len(parts) < 4:
                return
            
            # 提取时间戳
            timestamp_str = parts[0].strip('()')
            timestamp = float(timestamp_str)
            
            # 提取CAN ID
            can_id_str = parts[2]
            can_id = int(can_id_str, 16)
            
            # 提取数据长度和数据
            data_len_str = parts[3].strip('[]')
            data_len = int(data_len_str)
            
            data_bytes = []
            for i in range(4, 4 + data_len):
                if i < len(parts):
                    data_bytes.append(int(parts[i], 16))
            
            # 处理速度控制指令
            if can_id == Config.CAN_VEL_CMD_ID:
                parsed = CANDataParser.parse_vel_cmd(data_bytes)
                if parsed:
                    parsed['timestamp'] = timestamp
                    parsed['raw_data'] = data_bytes
                    with self.lock:
                        self.latest_vel_cmd = parsed
                        self.vel_cmd_count += 1
                    if self.vel_cmd_callback:
                        self.vel_cmd_callback(parsed)
            
            # 处理电机反馈
            elif can_id in Config.CAN_MOTOR_FB_IDS:
                parsed = CANDataParser.parse_motor_feedback(can_id, data_bytes)
                if parsed:
                    parsed['timestamp'] = timestamp
                    parsed['can_id'] = can_id
                    with self.lock:
                        self.latest_motor_fb[can_id] = parsed
                        self.motor_fb_count += 1
                    if self.motor_fb_callback:
                        self.motor_fb_callback(parsed)
            
            # 处理编码器反馈（转向角度）
            elif can_id in Config.CAN_ENCODER_IDS:
                parsed = CANDataParser.parse_encoder_feedback(can_id, data_bytes)
                if parsed:
                    parsed['timestamp'] = timestamp
                    parsed['can_id'] = can_id
                    with self.lock:
                        self.latest_encoder_fb[can_id] = parsed
                        self.encoder_fb_count += 1
                    if self.encoder_fb_callback:
                        self.encoder_fb_callback(parsed)
                        
        except Exception as e:
            pass  # 忽略解析错误
    
    def get_latest_vel_cmd(self):
        """获取最新的速度控制指令"""
        with self.lock:
            return self.latest_vel_cmd.copy() if self.latest_vel_cmd else None
    
    def get_latest_motor_fb(self):
        """获取最新的电机反馈"""
        with self.lock:
            return self.latest_motor_fb.copy()
    
    def get_latest_encoder_fb(self):
        """获取最新的编码器反馈（转向角度）"""
        with self.lock:
            return self.latest_encoder_fb.copy()

# ==================== 数据记录器 ====================
class DataLogger:
    """数据日志记录器"""
    
    def __init__(self):
        self.start_time = datetime.now()
        self.session_id = self.start_time.strftime("%Y%m%d_%H%M%S")
        
        # 确保日志目录存在
        if not os.path.exists(Config.LOG_DIR):
            os.makedirs(Config.LOG_DIR)
        
        # 各类数据存储
        self.cmd_vel_data = []
        self.odom_local_data = []
        self.odom_global_data = []
        self.laser_odom_data = []
        self.tf_data = []
        self.path_error_data = []
        self.imu_data = []
        self.anomaly_data = []
        
        # CAN数据存储
        self.can_vel_cmd_data = []
        self.can_motor_fb_data = []
        self.can_encoder_fb_data = []  # 编码器反馈（转向角度）
        self.control_chain_data = []  # cmd_vel到CAN的完整链路数据
        self.angular_chain_data = []  # 角速度控制链路数据
        
        self.summary_data = {
            'total_cmd_vel': 0,
            'total_odom': 0,
            'total_can_vel_cmd': 0,
            'total_can_motor_fb': 0,
            'total_can_encoder_fb': 0,
            'total_anomalies': 0,
            'max_path_error': 0.0,
            'avg_path_error': 0.0,
            'max_covariance': 0.0,
            'cmd_vel_out_of_range': 0,
            'max_control_delay': 0.0,
            'avg_control_delay': 0.0,
            'max_steering_error': 0.0,  # 最大转向误差
            'avg_steering_error': 0.0,  # 平均转向误差
        }
        
        # 线程锁
        self.lock = threading.Lock()
    
    def log_cmd_vel(self, timestamp, linear_x, linear_y, angular_z):
        with self.lock:
            self.cmd_vel_data.append({
                'timestamp': timestamp,
                'linear_x': linear_x,
                'linear_y': linear_y,
                'angular_z': angular_z
            })
            self.summary_data['total_cmd_vel'] += 1
    
    def log_odom_local(self, timestamp, x, y, z, yaw, vx, vy, vz, cov_xx, cov_yy, cov_yaw):
        with self.lock:
            self.odom_local_data.append({
                'timestamp': timestamp,
                'x': x, 'y': y, 'z': z, 'yaw': yaw,
                'vx': vx, 'vy': vy, 'vz': vz,
                'cov_xx': cov_xx, 'cov_yy': cov_yy, 'cov_yaw': cov_yaw
            })
            self.summary_data['total_odom'] += 1
            self.summary_data['max_covariance'] = max(
                self.summary_data['max_covariance'], cov_xx, cov_yy
            )
    
    def log_odom_global(self, timestamp, x, y, z, yaw, vx, vy, vz):
        with self.lock:
            self.odom_global_data.append({
                'timestamp': timestamp,
                'x': x, 'y': y, 'z': z, 'yaw': yaw,
                'vx': vx, 'vy': vy, 'vz': vz
            })
    
    def log_laser_odom(self, timestamp, x, y, z, yaw, vx, vy, vz):
        with self.lock:
            self.laser_odom_data.append({
                'timestamp': timestamp,
                'x': x, 'y': y, 'z': z, 'yaw': yaw,
                'vx': vx, 'vy': vy, 'vz': vz
            })
    
    def log_tf(self, timestamp, parent, child, tx, ty, tz, rx, ry, rz, rw):
        with self.lock:
            self.tf_data.append({
                'timestamp': timestamp,
                'parent': parent, 'child': child,
                'tx': tx, 'ty': ty, 'tz': tz,
                'rx': rx, 'ry': ry, 'rz': rz, 'rw': rw
            })
    
    def log_path_error(self, timestamp, cross_track_error, heading_error, closest_point_idx):
        with self.lock:
            self.path_error_data.append({
                'timestamp': timestamp,
                'cross_track_error': cross_track_error,
                'heading_error': heading_error,
                'closest_point_idx': closest_point_idx
            })
            self.summary_data['max_path_error'] = max(
                self.summary_data['max_path_error'], abs(cross_track_error)
            )
    
    def log_imu(self, timestamp, ax, ay, az, gx, gy, gz, ox, oy, oz, ow):
        with self.lock:
            self.imu_data.append({
                'timestamp': timestamp,
                'ax': ax, 'ay': ay, 'az': az,
                'gx': gx, 'gy': gy, 'gz': gz,
                'ox': ox, 'oy': oy, 'oz': oz, 'ow': ow
            })
    
    def log_can_vel_cmd(self, data):
        """记录CAN速度控制指令"""
        with self.lock:
            self.can_vel_cmd_data.append({
                'timestamp': data['timestamp'],
                'control_mode': data['control_mode'],
                'vel_raw': data['vel_raw'],
                'vel_dir': data['vel_dir'],
                'theta_raw': data['theta_raw'],
                'theta_dir': data['theta_dir'],
                'actual_vel': data['actual_vel'],
                'actual_theta': data['actual_theta'],
                'raw_data': ' '.join(f'{b:02X}' for b in data['raw_data'])
            })
            self.summary_data['total_can_vel_cmd'] += 1
    
    def log_can_motor_fb(self, data):
        """记录CAN电机反馈"""
        with self.lock:
            self.can_motor_fb_data.append({
                'timestamp': data['timestamp'],
                'can_id': f"0x{data['can_id']:08X}",
                'motor_group': data['motor_group'],
                'motor_number': data['motor_number'],
                'enabled': data['enabled'],
                'speed_raw': data['speed_raw'],
                'rpm': data['rpm']
            })
            self.summary_data['total_can_motor_fb'] += 1
    
    def log_can_encoder_fb(self, data):
        """记录CAN编码器反馈（转向角度）"""
        with self.lock:
            self.can_encoder_fb_data.append({
                'timestamp': data['timestamp'],
                'can_id': f"0x{data['can_id']:03X}",
                'encoder_id': data['encoder_id'],
                'encoder_raw': data['encoder_raw'],
                'angle': data['angle']
            })
            self.summary_data['total_can_encoder_fb'] += 1
    
    def log_control_chain(self, timestamp, cmd_vel_vx, cmd_vel_vz, can_vel, can_theta, 
                         delay_ms, odom_vx, odom_vz):
        """记录完整控制链路数据"""
        with self.lock:
            self.control_chain_data.append({
                'timestamp': timestamp,
                'cmd_vel_vx': cmd_vel_vx,
                'cmd_vel_vz': cmd_vel_vz,
                'can_vel': can_vel,
                'can_theta': can_theta,
                'delay_ms': delay_ms,
                'odom_vx': odom_vx,
                'odom_vz': odom_vz
            })
            if delay_ms > self.summary_data['max_control_delay']:
                self.summary_data['max_control_delay'] = delay_ms
    
    def log_angular_chain(self, timestamp, cmd_vel_vz, target_theta, actual_theta_avg, 
                          steering_error, delay_ms):
        """记录角速度控制链路数据"""
        with self.lock:
            self.angular_chain_data.append({
                'timestamp': timestamp,
                'cmd_vel_vz': cmd_vel_vz,  # 目标角速度 (rad/s)
                'target_theta': target_theta,  # 目标转向角 (度)
                'actual_theta_avg': actual_theta_avg,  # 实际平均转向角 (度)
                'steering_error': steering_error,  # 转向误差 (度)
                'delay_ms': delay_ms
            })
            if abs(steering_error) > self.summary_data['max_steering_error']:
                self.summary_data['max_steering_error'] = abs(steering_error)
    
    def log_anomaly(self, timestamp, anomaly_type, description, value):
        with self.lock:
            self.anomaly_data.append({
                'timestamp': timestamp,
                'type': anomaly_type,
                'description': description,
                'value': value
            })
            self.summary_data['total_anomalies'] += 1
    
    def save_all(self):
        """保存所有数据到CSV文件"""
        rospy.loginfo("正在保存监控数据...")
        
        session_dir = os.path.join(Config.LOG_DIR, f"nav_monitor_{self.session_id}")
        if not os.path.exists(session_dir):
            os.makedirs(session_dir)
        
        # 保存cmd_vel数据
        self._save_csv('cmd_vel', self.cmd_vel_data,
                      ['timestamp', 'linear_x', 'linear_y', 'angular_z'])
        
        # 保存local_odom数据
        self._save_csv('odom_local', self.odom_local_data,
                      ['timestamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 
                       'cov_xx', 'cov_yy', 'cov_yaw'])
        
        # 保存global_odom数据
        self._save_csv('odom_global', self.odom_global_data,
                      ['timestamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz'])
        
        # 保存laser_odom数据
        self._save_csv('odom_laser', self.laser_odom_data,
                      ['timestamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz'])
        
        # 保存TF数据
        self._save_csv('tf', self.tf_data,
                      ['timestamp', 'parent', 'child', 'tx', 'ty', 'tz', 
                       'rx', 'ry', 'rz', 'rw'])
        
        # 保存路径误差数据
        self._save_csv('path_error', self.path_error_data,
                      ['timestamp', 'cross_track_error', 'heading_error', 'closest_point_idx'])
        
        # 保存IMU数据
        self._save_csv('imu', self.imu_data,
                      ['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 
                       'ox', 'oy', 'oz', 'ow'])
        
        # 保存CAN速度控制指令
        self._save_csv('can_vel_cmd', self.can_vel_cmd_data,
                      ['timestamp', 'control_mode', 'vel_raw', 'vel_dir', 
                       'theta_raw', 'theta_dir', 'actual_vel', 'actual_theta', 'raw_data'])
        
        # 保存CAN电机反馈
        self._save_csv('can_motor_fb', self.can_motor_fb_data,
                      ['timestamp', 'can_id', 'motor_group', 'motor_number',
                       'enabled', 'speed_raw', 'rpm'])
        
        # 保存控制链路数据
        self._save_csv('control_chain', self.control_chain_data,
                      ['timestamp', 'cmd_vel_vx', 'cmd_vel_vz', 'can_vel', 'can_theta',
                       'delay_ms', 'odom_vx', 'odom_vz'])
        
        # 保存编码器反馈数据
        self._save_csv('can_encoder_fb', self.can_encoder_fb_data,
                      ['timestamp', 'can_id', 'encoder_id', 'encoder_raw', 'angle'])
        
        # 保存角速度控制链路数据
        self._save_csv('angular_chain', self.angular_chain_data,
                      ['timestamp', 'cmd_vel_vz', 'target_theta', 'actual_theta_avg',
                       'steering_error', 'delay_ms'])
        
        # 保存异常数据
        self._save_csv('anomalies', self.anomaly_data,
                      ['timestamp', 'type', 'description', 'value'])
        
        # 保存汇总报告
        self._save_summary()
        
        rospy.loginfo(f"数据已保存到 {session_dir}/")
    
    def _save_csv(self, name, data, fieldnames):
        if not data:
            return
        
        session_dir = os.path.join(Config.LOG_DIR, f"nav_monitor_{self.session_id}")
        filepath = os.path.join(session_dir, f"{name}.csv")
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)
        rospy.loginfo(f"  - {name}.csv: {len(data)} 条记录")
    
    def _save_summary(self):
        session_dir = os.path.join(Config.LOG_DIR, f"nav_monitor_{self.session_id}")
        filepath = os.path.join(session_dir, "summary.txt")
        
        duration = (datetime.now() - self.start_time).total_seconds()
        
        # 计算平均路径误差
        if self.path_error_data:
            errors = [d['cross_track_error'] for d in self.path_error_data]
            self.summary_data['avg_path_error'] = sum(abs(e) for e in errors) / len(errors)
        
        # 计算平均控制延迟
        if self.control_chain_data:
            delays = [d['delay_ms'] for d in self.control_chain_data if d['delay_ms'] > 0]
            if delays:
                self.summary_data['avg_control_delay'] = sum(delays) / len(delays)
        
        with open(filepath, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("              导航监控系统 v2.0 - 会话汇总报告\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"会话ID: {self.session_id}\n")
            f.write(f"开始时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"持续时间: {duration:.1f} 秒\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("ROS话题统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  /cmd_vel 消息总数: {self.summary_data['total_cmd_vel']}\n")
            f.write(f"  /local_odometry 消息总数: {self.summary_data['total_odom']}\n")
            f.write(f"  cmd_vel 超限次数: {self.summary_data['cmd_vel_out_of_range']}\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("CAN总线统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  速度控制指令 (0x8F910D8): {self.summary_data['total_can_vel_cmd']}\n")
            f.write(f"  电机反馈消息: {self.summary_data['total_can_motor_fb']}\n")
            f.write(f"  编码器反馈 (0x191-0x194): {self.summary_data['total_can_encoder_fb']}\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("转向控制统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  最大转向误差: {self.summary_data['max_steering_error']:.2f}°\n")
            f.write(f"  平均转向误差: {self.summary_data['avg_steering_error']:.2f}°\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("控制链路统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  最大控制延迟: {self.summary_data['max_control_delay']:.1f} ms\n")
            f.write(f"  平均控制延迟: {self.summary_data['avg_control_delay']:.1f} ms\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("路径跟踪统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  最大横向误差: {self.summary_data['max_path_error']:.3f} m\n")
            f.write(f"  平均横向误差: {self.summary_data['avg_path_error']:.3f} m\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("定位质量统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  最大协方差: {self.summary_data['max_covariance']:.3f}\n\n")
            
            f.write("-" * 50 + "\n")
            f.write("异常事件统计:\n")
            f.write("-" * 50 + "\n")
            f.write(f"  异常事件总数: {self.summary_data['total_anomalies']}\n\n")
            
            if self.anomaly_data:
                f.write("最近50条异常事件:\n")
                for anomaly in self.anomaly_data[-50:]:
                    f.write(f"  [{anomaly['timestamp']:.2f}] {anomaly['type']}: "
                           f"{anomaly['description']}\n")
        
        rospy.loginfo(f"  - summary.txt: 汇总报告")

# ==================== 导航监控器 ====================
class NavMonitor:
    """导航系统监控器"""
    
    def __init__(self):
        rospy.init_node('nav_monitor', anonymous=True)
        
        self.logger = DataLogger()
        self.running = True
        
        # 当前状态
        self.current_pose = None
        self.current_vel = None
        self.current_cmd_vel = None
        self.current_global_path = None
        self.current_local_path = None
        self.nav_mode = 0
        
        # 时间戳记录
        self.last_cmd_vel_time = None
        self.last_cmd_vel_data = None
        self.last_odom_time = None
        self.last_tf_time = {}
        
        # 历史数据（用于实时显示）
        self.cmd_vel_history = deque(maxlen=Config.HISTORY_SIZE)
        self.odom_history = deque(maxlen=Config.HISTORY_SIZE)
        self.path_error_history = deque(maxlen=Config.HISTORY_SIZE)
        self.can_vel_cmd_history = deque(maxlen=100)
        self.can_motor_fb_history = deque(maxlen=100)
        self.can_encoder_fb_history = deque(maxlen=100)  # 编码器反馈历史
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # CAN监控器
        self.can_monitor = CANMonitor(Config.CAN_INTERFACE)
        
        # 订阅话题
        self._setup_subscribers()
        
        # 启动CAN监控
        self.can_monitor.start(
            vel_cmd_callback=self._can_vel_cmd_callback,
            motor_fb_callback=self._can_motor_fb_callback,
            encoder_fb_callback=self._can_encoder_fb_callback
        )
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # 启动显示线程
        self.display_thread = threading.Thread(target=self._display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()
        
        # 启动控制链路分析线程
        self.chain_analysis_thread = threading.Thread(target=self._chain_analysis_loop)
        self.chain_analysis_thread.daemon = True
        self.chain_analysis_thread.start()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("  导航监控系统 v2.0 已启动")
        rospy.loginfo(f"  日志将保存到: {Config.LOG_DIR}")
        rospy.loginfo("  监控内容:")
        rospy.loginfo("    - ROS话题: cmd_vel, odometry, tf, path...")
        rospy.loginfo("    - CAN总线: 速度指令(0x8F910D8), 电机反馈")
        rospy.loginfo("  按 Ctrl+C 停止并保存数据")
        rospy.loginfo("=" * 60)
    
    def _setup_subscribers(self):
        """设置话题订阅"""
        # 速度指令
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback, queue_size=10)
        
        # 里程计
        rospy.Subscriber('/local_odometry', Odometry, self._local_odom_callback, queue_size=10)
        rospy.Subscriber('/global_odometry', Odometry, self._global_odom_callback, queue_size=10)
        rospy.Subscriber('/laser_odometry', Odometry, self._laser_odom_callback, queue_size=10)
        
        # 路径
        rospy.Subscriber('/move_base_node/SBPLLatticePlanner/plan', Path, 
                        self._global_path_callback, queue_size=1)
        rospy.Subscriber('/move_base_node/TebLocalPlannerROS/local_plan', Path,
                        self._local_path_callback, queue_size=1)
        rospy.Subscriber('/move_base_node/TebLocalPlannerROS/global_plan', Path,
                        self._teb_global_path_callback, queue_size=1)
        
        # TF
        rospy.Subscriber('/tf', TFMessage, self._tf_callback, queue_size=100)
        
        # IMU
        rospy.Subscriber('/chcnav_fix_demo/myimu', Imu, self._imu_callback, queue_size=10)
        
        # 导航模式
        rospy.Subscriber('/nav_mode_switch', Int32, self._nav_mode_callback, queue_size=1)
        
        # 点云（仅监控频率）
        rospy.Subscriber('/velodyne_points', PointCloud2, self._pointcloud_callback, queue_size=1)
        rospy.Subscriber('/rslidar_points', PointCloud2, self._pointcloud_callback, queue_size=1)
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        rospy.loginfo("\n收到停止信号，正在保存数据...")
        self.running = False
        self.can_monitor.stop()
        self.logger.save_all()
        rospy.loginfo("监控程序已退出")
        sys.exit(0)
    
    # ==================== ROS回调函数 ====================
    
    def _cmd_vel_callback(self, msg):
        """cmd_vel回调"""
        timestamp = rospy.get_time()
        self.last_cmd_vel_time = timestamp
        self.last_cmd_vel_data = msg
        
        self.current_cmd_vel = msg
        
        # 记录数据
        self.logger.log_cmd_vel(timestamp, msg.linear.x, msg.linear.y, msg.angular.z)
        
        # 添加到历史
        self.cmd_vel_history.append({
            'time': timestamp,
            'vx': msg.linear.x,
            'vz': msg.angular.z
        })
        
        # 检测异常：速度超限
        if abs(msg.linear.x) > Config.TEB_MAX_VEL_X * 1.5:
            self.logger.log_anomaly(timestamp, 'CMD_VEL_LINEAR_EXCEEDED',
                                   f'线速度超限: {msg.linear.x:.3f} > {Config.TEB_MAX_VEL_X}',
                                   msg.linear.x)
            self.logger.summary_data['cmd_vel_out_of_range'] += 1
        
        if abs(msg.angular.z) > Config.TEB_MAX_VEL_THETA * 1.5:
            self.logger.log_anomaly(timestamp, 'CMD_VEL_ANGULAR_EXCEEDED',
                                   f'角速度超限: {msg.angular.z:.3f} > {Config.TEB_MAX_VEL_THETA}',
                                   msg.angular.z)
            self.logger.summary_data['cmd_vel_out_of_range'] += 1
    
    def _local_odom_callback(self, msg):
        """局部里程计回调"""
        timestamp = rospy.get_time()
        self.last_odom_time = timestamp
        
        # 提取位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 四元数转欧拉角
        q = msg.pose.pose.orientation
        yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        # 提取速度
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z
        
        # 提取协方差（取对角线主要元素）
        cov = msg.pose.covariance
        cov_xx = cov[0]   # x位置协方差
        cov_yy = cov[7]   # y位置协方差
        cov_yaw = cov[35] # yaw协方差
        
        self.current_pose = (x, y, yaw)
        self.current_vel = (vx, vy, vz)
        
        # 记录数据
        self.logger.log_odom_local(timestamp, x, y, z, yaw, vx, vy, vz, cov_xx, cov_yy, cov_yaw)
        
        # 添加到历史
        self.odom_history.append({
            'time': timestamp,
            'x': x, 'y': y, 'yaw': yaw,
            'vx': vx, 'vz': vz,
            'cov_xx': cov_xx, 'cov_yy': cov_yy
        })
        
        # 检测异常：协方差过大
        if cov_xx > Config.MAX_COVARIANCE or cov_yy > Config.MAX_COVARIANCE:
            self.logger.log_anomaly(timestamp, 'COVARIANCE_HIGH',
                                   f'位置协方差过大: cov_xx={cov_xx:.2f}, cov_yy={cov_yy:.2f}',
                                   max(cov_xx, cov_yy))
        
        # 计算路径跟踪误差
        if self.current_global_path:
            self._calculate_path_error(timestamp, x, y, yaw)
    
    def _global_odom_callback(self, msg):
        """全局里程计回调"""
        timestamp = rospy.get_time()
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z
        
        self.logger.log_odom_global(timestamp, x, y, z, yaw, vx, vy, vz)
    
    def _laser_odom_callback(self, msg):
        """激光里程计回调"""
        timestamp = rospy.get_time()
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z
        
        self.logger.log_laser_odom(timestamp, x, y, z, yaw, vx, vy, vz)
    
    def _global_path_callback(self, msg):
        """全局路径回调"""
        if msg.poses:
            self.current_global_path = [(p.pose.position.x, p.pose.position.y) 
                                        for p in msg.poses]
            rospy.loginfo_throttle(10, f"收到全局路径，{len(msg.poses)} 个点")
    
    def _local_path_callback(self, msg):
        """局部路径回调"""
        if msg.poses:
            self.current_local_path = [(p.pose.position.x, p.pose.position.y)
                                       for p in msg.poses]
    
    def _teb_global_path_callback(self, msg):
        """TEB全局路径回调"""
        pass
    
    def _tf_callback(self, msg):
        """TF回调"""
        timestamp = rospy.get_time()
        
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            
            # 只记录关键TF
            if (parent, child) in [('map', 'odom'), ('odom', 'base_footprint'), 
                                   ('base_footprint', 'lidar_link')]:
                t = transform.transform.translation
                r = transform.transform.rotation
                
                self.logger.log_tf(timestamp, parent, child,
                                  t.x, t.y, t.z, r.x, r.y, r.z, r.w)
                
                self.last_tf_time[(parent, child)] = timestamp
    
    def _imu_callback(self, msg):
        """IMU回调"""
        timestamp = rospy.get_time()
        
        self.logger.log_imu(
            timestamp,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
    
    def _nav_mode_callback(self, msg):
        """导航模式回调"""
        self.nav_mode = msg.data
        rospy.loginfo(f"导航模式切换: {self._get_nav_mode_string(msg.data)}")
    
    def _pointcloud_callback(self, msg):
        """点云回调（仅监控）"""
        pass
    
    # ==================== CAN回调函数 ====================
    
    def _can_vel_cmd_callback(self, data):
        """CAN速度控制指令回调"""
        self.logger.log_can_vel_cmd(data)
        self.can_vel_cmd_history.append(data)
    
    def _can_motor_fb_callback(self, data):
        """CAN电机反馈回调"""
        self.logger.log_can_motor_fb(data)
        self.can_motor_fb_history.append(data)
    
    def _can_encoder_fb_callback(self, data):
        """CAN编码器反馈回调（转向角度）"""
        self.logger.log_can_encoder_fb(data)
        self.can_encoder_fb_history.append(data)
    
    # ==================== 控制链路分析 ====================
    
    def _chain_analysis_loop(self):
        """控制链路分析循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                # 获取最新的cmd_vel和CAN指令
                if self.last_cmd_vel_data and self.can_vel_cmd_history:
                    latest_can = self.can_vel_cmd_history[-1] if self.can_vel_cmd_history else None
                    
                    if latest_can and self.last_cmd_vel_time:
                        # 计算延迟
                        delay_ms = (latest_can['timestamp'] - self.last_cmd_vel_time) * 1000
                        
                        # 获取里程计反馈的速度
                        odom_vx = 0.0
                        odom_vz = 0.0
                        if self.odom_history:
                            latest_odom = self.odom_history[-1]
                            odom_vx = latest_odom['vx']
                            odom_vz = latest_odom['vz']
                        
                        # 记录控制链路数据
                        self.logger.log_control_chain(
                            rospy.get_time(),
                            self.last_cmd_vel_data.linear.x,
                            self.last_cmd_vel_data.angular.z,
                            latest_can['actual_vel'],
                            latest_can['actual_theta'],
                            delay_ms,
                            odom_vx,
                            odom_vz
                        )
                        
                        # 角速度链路分析
                        encoder_fb = self.can_monitor.get_latest_encoder_fb()
                        if encoder_fb and len(encoder_fb) > 0:
                            # 计算平均实际转向角
                            actual_angles = [fb['angle'] for fb in encoder_fb.values()]
                            actual_theta_avg = sum(actual_angles) / len(actual_angles)
                            
                            # 目标转向角（从CAN指令）
                            target_theta = latest_can['actual_theta']
                            
                            # 转向误差
                            steering_error = actual_theta_avg - target_theta
                            
                            # 记录角速度链路数据
                            self.logger.log_angular_chain(
                                rospy.get_time(),
                                self.last_cmd_vel_data.angular.z,
                                target_theta,
                                actual_theta_avg,
                                steering_error,
                                delay_ms
                            )
            except Exception as e:
                pass
            
            rate.sleep()
    
    # ==================== 辅助函数 ====================
    
    def _quaternion_to_yaw(self, x, y, z, w):
        """四元数转yaw角"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _calculate_path_error(self, timestamp, x, y, yaw):
        """计算路径跟踪误差"""
        if not self.current_global_path or len(self.current_global_path) < 2:
            return
        
        # 找到最近点
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(self.current_global_path):
            dist = math.sqrt((x - px)**2 + (y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 计算横向误差（cross-track error）
        cross_track_error = min_dist
        
        # 计算航向误差
        if closest_idx < len(self.current_global_path) - 1:
            px1, py1 = self.current_global_path[closest_idx]
            px2, py2 = self.current_global_path[closest_idx + 1]
            path_heading = math.atan2(py2 - py1, px2 - px1)
            heading_error = self._normalize_angle(yaw - path_heading)
        else:
            heading_error = 0.0
        
        # 确定横向误差的符号（左正右负）
        if closest_idx < len(self.current_global_path) - 1:
            px1, py1 = self.current_global_path[closest_idx]
            px2, py2 = self.current_global_path[closest_idx + 1]
            cross = (px2 - px1) * (y - py1) - (py2 - py1) * (x - px1)
            if cross < 0:
                cross_track_error = -cross_track_error
        
        self.logger.log_path_error(timestamp, cross_track_error, heading_error, closest_idx)
        
        self.path_error_history.append({
            'time': timestamp,
            'cte': cross_track_error,
            'he': heading_error
        })
    
    def _normalize_angle(self, angle):
        """归一化角度到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _get_nav_mode_string(self, mode):
        """获取导航模式字符串"""
        modes = {
            0: "空闲模式",
            1: "TEB导航模式",
            2: "GlobalPath跟踪模式",
            3: "斜行接近模式",
            4: "姿态调整模式",
            5: "到达目标"
        }
        return modes.get(mode, f"未知模式({mode})")
    
    # ==================== 显示函数 ====================
    
    def _display_loop(self):
        """显示循环"""
        rate = rospy.Rate(Config.DISPLAY_RATE)
        
        while self.running and not rospy.is_shutdown():
            self._print_status()
            rate.sleep()
    
    def _print_status(self):
        """打印当前状态"""
        # 清屏
        print("\033[2J\033[H", end="")
        
        print("=" * 80)
        print("                    导航系统实时监控 v2.0")
        print("=" * 80)
        print(f"  时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  导航模式: {self._get_nav_mode_string(self.nav_mode)}")
        print("-" * 80)
        
        # ===== 控制链路显示 =====
        print("\n【控制链路: cmd_vel → CAN → 电机】")
        print("-" * 40)
        
        # cmd_vel
        print("  ┌─ /cmd_vel (TEB输出)")
        if self.current_cmd_vel:
            vx = self.current_cmd_vel.linear.x
            vz = self.current_cmd_vel.angular.z
            vx_status = "✓" if abs(vx) <= Config.TEB_MAX_VEL_X else "⚠超限"
            vz_status = "✓" if abs(vz) <= Config.TEB_MAX_VEL_THETA else "⚠超限"
            print(f"  │    线速度: {vx:+.4f} m/s  {vx_status}")
            print(f"  │    角速度: {vz:+.4f} rad/s {vz_status}")
        else:
            print("  │    等待数据...")
        
        # CAN速度指令
        print("  │")
        print("  ├─ CAN 0x8F910D8 (速度控制指令)")
        latest_can = self.can_monitor.get_latest_vel_cmd()
        if latest_can:
            raw_str = ' '.join(f'{b:02X}' for b in latest_can.get('raw_data', []))
            print(f"  │    原始数据: [{raw_str}]")
            print(f"  │    控制模式: {latest_can['control_mode']}")
            print(f"  │    速度: {latest_can['actual_vel']:+.4f} m/s (原始:{latest_can['vel_raw']}, 方向:{latest_can['vel_dir']})")
            print(f"  │    转角: {latest_can['actual_theta']:+.1f}° (原始:{latest_can['theta_raw']}, 方向:{latest_can['theta_dir']})")
        else:
            print("  │    等待CAN数据...")
        
        # 电机反馈
        print("  │")
        print("  ├─ 电机反馈 (0x4Fx10D0)")
        motor_fb = self.can_monitor.get_latest_motor_fb()
        if motor_fb:
            for can_id, fb in sorted(motor_fb.items()):
                status = "使能" if fb['enabled'] else "禁用"
                print(f"  │    电机组{fb['motor_group']}: {fb['rpm']:+.1f} RPM ({status})")
        else:
            print("  │    等待电机反馈...")
        
        # ===== 角速度控制链路 =====
        print("  │")
        print("  └─ 编码器反馈 (0x191-0x194) - 实际转向角度")
        encoder_fb = self.can_monitor.get_latest_encoder_fb()
        if encoder_fb:
            encoder_names = {1: "右前", 2: "左前", 3: "右后", 4: "左后"}
            angles = []
            for can_id, fb in sorted(encoder_fb.items()):
                name = encoder_names.get(fb['encoder_id'], f"#{fb['encoder_id']}")
                print(f"       {name}: {fb['angle']:+.2f}° (原始:{fb['encoder_raw']})")
                angles.append(fb['angle'])
            
            # 计算转向误差
            if angles and latest_can:
                avg_angle = sum(angles) / len(angles)
                target_angle = latest_can['actual_theta']
                error = avg_angle - target_angle
                error_status = "✓" if abs(error) < 5.0 else "⚠误差大"
                print(f"       ────────────────────────────")
                print(f"       平均实际转角: {avg_angle:+.2f}°")
                print(f"       目标转角:     {target_angle:+.1f}°")
                print(f"       转向误差:     {error:+.2f}° {error_status}")
        else:
            print("       等待编码器数据...")
        
        # ===== 角速度完整链路汇总 =====
        print("\n【角速度控制链路汇总: cmd_vel.angular.z → CAN转角 → 编码器反馈】")
        print("-" * 40)
        if self.current_cmd_vel and latest_can and encoder_fb:
            cmd_vz = self.current_cmd_vel.angular.z
            can_theta = latest_can['actual_theta']
            actual_angles = [fb['angle'] for fb in encoder_fb.values()]
            actual_avg = sum(actual_angles) / len(actual_angles)
            
            print(f"  cmd_vel.angular.z:  {cmd_vz:+.4f} rad/s")
            print(f"  → CAN目标转角:      {can_theta:+.1f}°")
            print(f"  → 实际平均转角:     {actual_avg:+.2f}°")
            print(f"  → 转向执行误差:     {(actual_avg - can_theta):+.2f}°")
        else:
            print("  等待完整数据...")
        
        # ===== 里程计显示 =====
        print("\n【位姿估计】")
        print("-" * 40)
        if self.current_pose:
            x, y, yaw = self.current_pose
            yaw_deg = math.degrees(yaw)
            print(f"  位置: ({x:+.3f}, {y:+.3f}) m")
            print(f"  航向: {yaw_deg:+.1f}°")
            
            if self.odom_history:
                latest = self.odom_history[-1]
                cov_status = "✓" if latest['cov_xx'] < Config.MAX_COVARIANCE else "⚠过大"
                print(f"  协方差: xx={latest['cov_xx']:.2f}, yy={latest['cov_yy']:.2f} {cov_status}")
                print(f"  实测速度: vx={latest['vx']:+.4f} m/s, vz={latest['vz']:+.4f} rad/s")
        else:
            print("  等待数据...")
        
        # ===== 路径跟踪显示 =====
        print("\n【路径跟踪】")
        print("-" * 40)
        if self.current_global_path:
            print(f"  全局路径点数: {len(self.current_global_path)}")
            if self.path_error_history:
                latest = self.path_error_history[-1]
                cte = latest['cte']
                he_deg = math.degrees(latest['he'])
                cte_status = "✓" if abs(cte) < 1.0 else "⚠偏离"
                print(f"  横向误差: {cte:+.3f} m {cte_status}")
                print(f"  航向误差: {he_deg:+.1f}°")
        else:
            print("  等待全局路径...")
        
        # ===== 统计信息 =====
        print("\n【会话统计】")
        print("-" * 40)
        print(f"  ROS消息: cmd_vel={self.logger.summary_data['total_cmd_vel']}, "
              f"odom={self.logger.summary_data['total_odom']}")
        print(f"  CAN消息: 速度指令={self.can_monitor.vel_cmd_count}, "
              f"电机反馈={self.can_monitor.motor_fb_count}, "
              f"编码器={self.can_monitor.encoder_fb_count}")
        print(f"  异常事件: {self.logger.summary_data['total_anomalies']}")
        print(f"  最大路径误差: {self.logger.summary_data['max_path_error']:.3f} m")
        print(f"  最大转向误差: {self.logger.summary_data['max_steering_error']:.2f}°")
        print(f"  最大协方差: {self.logger.summary_data['max_covariance']:.2f}")
        
        print("\n" + "=" * 80)
        print("  按 Ctrl+C 停止监控并保存数据")
        print("=" * 80)
    
    def run(self):
        """运行监控器"""
        rospy.spin()

# ==================== 主程序 ====================
if __name__ == '__main__':
    try:
        monitor = NavMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"监控程序异常: {e}")
        import traceback
        traceback.print_exc()
