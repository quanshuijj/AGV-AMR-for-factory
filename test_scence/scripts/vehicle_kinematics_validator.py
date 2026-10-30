#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车辆运动学模型验证程序
从虚拟CAN口vcan0读取数据并验证运动学计算的正确性
"""

import rospy
import math
import numpy as np
import can
import struct
import threading
import time
import json
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion
import tf.transformations as tf_trans


class CANMessageDecoder:
    """CAN消息解码器"""
    
    def __init__(self):
        # CAN ID定义
        self.STEERING_CONTROL_ID = 0x8F110D8
        self.MOTOR_CONTROL_ID = 0x8F210D8
        
        # 解码参数
        self.STEERING_RATIO_PARAMS = 0.2  # 转向角度比例 (deg/bit)
        self.MOTOR_RATIO_PARAMS = 12.0    # 电机转速比例 (rpm/bit)
        self.WHEEL_CIRCUMFERENCE = 1.41999  # 轮胎周长 (m)
        
    def decode_steering_message(self, data):
        """
        解码转向控制消息 (CAN ID: 0x8F110D8)
        数据格式: [右前轮, 左前轮, 右后轮, 左后轮, 模式控制]
        """
        if len(data) < 5:
            return None
            
        steering_data = {
            'right_front_raw': data[0],
            'left_front_raw': data[1], 
            'right_rear_raw': data[2],
            'left_rear_raw': data[3],
            'control_mode': data[4],
            'timestamp': time.time()
        }
        
        # 转换为实际角度 (度)
        steering_data['right_front_angle'] = (data[0] - 128) * self.STEERING_RATIO_PARAMS
        steering_data['left_front_angle'] = (data[1] - 128) * self.STEERING_RATIO_PARAMS
        steering_data['right_rear_angle'] = (data[2] - 128) * self.STEERING_RATIO_PARAMS
        steering_data['left_rear_angle'] = (data[3] - 128) * self.STEERING_RATIO_PARAMS
        
        return steering_data
    
    def decode_motor_message(self, data):
        """
        解码电机控制消息 (CAN ID: 0x8F210D8)
        数据格式: [左前1, 左前2, 右前1, 右前2, 左后1, 左后2, 右后1, 右后2]
        """
        if len(data) < 8:
            return None
            
        motor_data = {
            'vlf1_raw': data[0],
            'vlf2_raw': data[1],
            'vrf1_raw': data[2], 
            'vrf2_raw': data[3],
            'vlr1_raw': data[4],
            'vlr2_raw': data[5],
            'vrr1_raw': data[6],
            'vrr2_raw': data[7],
            'timestamp': time.time()
        }
        
        # 转换为实际转速 (rpm) 和线速度 (m/s)
        motor_data['vlf1_rpm'] = (data[0] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vlf2_rpm'] = (data[1] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vrf1_rpm'] = (data[2] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vrf2_rpm'] = (data[3] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vlr1_rpm'] = (data[4] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vlr2_rpm'] = (data[5] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vrr1_rpm'] = (data[6] - 128) * self.MOTOR_RATIO_PARAMS/25
        motor_data['vrr2_rpm'] = (data[7] - 128) * self.MOTOR_RATIO_PARAMS/25
        
        # 转换为线速度 (m/s)
        motor_data['vlf1_speed'] = motor_data['vlf1_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vlf2_speed'] = motor_data['vlf2_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vrf1_speed'] = motor_data['vrf1_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vrf2_speed'] = motor_data['vrf2_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vlr1_speed'] = motor_data['vlr1_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vlr2_speed'] = motor_data['vlr2_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vrr1_speed'] = motor_data['vrr1_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        motor_data['vrr2_speed'] = motor_data['vrr2_rpm'] * self.WHEEL_CIRCUMFERENCE / 60
        
        return motor_data


class VehicleKinematicsValidator:
    """车辆运动学验证器"""
    
    def __init__(self):
        rospy.init_node('vehicle_kinematics_validator', anonymous=True)
        
        # 车辆物理参数
        self.L = 2.5  # 轴距 (m)
        self.W = 1.25  # 轮距 (m)
        self.W2 = 0.375  # 每侧双轮距离的一半 (m)
        self.minRadius = 4.330127019 # 最小转弯半径 (m)
        self.wheelCircumference = 1.41928  # 轮胎周长 (m)
        self.REDUCTION_RATION = 25  # 电机减速比
        # CAN接口设置
        self.can_interface = 'vcan0'
        self.can_bus = None
        self.can_decoder = CANMessageDecoder()
        
        # 数据存储
        self.latest_steering_data = None
        self.latest_motor_data = None
        self.validation_results = []
        
        # 线程控制
        self.running = True
        self.can_thread = None
        
        # ROS发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_test', Twist, queue_size=1)
        self.validation_result_pub = rospy.Publisher('/validation_result', String, queue_size=1)
        self.decoded_steering_pub = rospy.Publisher('/decoded_steering', String, queue_size=1)
        self.decoded_motor_pub = rospy.Publisher('/decoded_motor', String, queue_size=1)
        
        # Gazebo服务 (可选)
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.gazebo_available = True
            rospy.loginfo("Gazebo服务可用")
        except:
            self.gazebo_available = False
            rospy.logwarn("Gazebo服务不可用，跳过Gazebo集成")
        
        # 初始化CAN接口
        self.init_can_interface()
        
        rospy.loginfo("车辆运动学验证器初始化完成")
        
    def init_can_interface(self):
        """初始化CAN接口"""
        try:
            self.can_bus = can.interface.Bus(channel=self.can_interface, 
                                           bustype='socketcan',
                                           receive_own_messages=False)
            rospy.loginfo(f"成功连接到CAN接口: {self.can_interface}")
            
            # 启动CAN消息接收线程
            self.can_thread = threading.Thread(target=self.can_receive_loop)
            self.can_thread.daemon = True
            self.can_thread.start()
            
        except Exception as e:
            rospy.logerr(f"无法连接到CAN接口 {self.can_interface}: {e}")
            rospy.logwarn("请确保已设置虚拟CAN接口:")
            rospy.logwarn("sudo modprobe vcan")
            rospy.logwarn("sudo ip link add dev vcan0 type vcan")
            rospy.logwarn("sudo ip link set up vcan0")
            self.can_bus = None
    
    def can_receive_loop(self):
        """CAN消息接收循环"""
        rospy.loginfo("开始监听CAN消息...")
        
        while self.running and not rospy.is_shutdown():
            try:
                if self.can_bus is None:
                    time.sleep(0.1)
                    continue
                    
                # 接收CAN消息 (超时1秒)
                message = self.can_bus.recv(timeout=1.0)
                
                if message is None:
                    continue
                    
                self.process_can_message(message)
                
            except can.CanError as e:
                rospy.logerr(f"CAN接收错误: {e}")
                time.sleep(0.1)
            except Exception as e:
                rospy.logerr(f"CAN接收线程错误: {e}")
                time.sleep(0.1)
    
    def process_can_message(self, message):
        """处理接收到的CAN消息"""
        can_id = message.arbitration_id
        data = list(message.data)
        
        rospy.logdebug(f"收到CAN消息: ID=0x{can_id:X}, 数据={data}")
        
        if can_id == self.can_decoder.STEERING_CONTROL_ID:
            # 转向控制消息
            steering_data = self.can_decoder.decode_steering_message(data)
            if steering_data:
                self.latest_steering_data = steering_data
                self.publish_decoded_steering(steering_data)
                rospy.loginfo(f"收到转向控制: 左前={steering_data['left_front_angle']:.1f}°, "
                            f"右前={steering_data['right_front_angle']:.1f}°, "
                            f"模式={steering_data['control_mode']}")
                
        elif can_id == self.can_decoder.MOTOR_CONTROL_ID:
            # 电机控制消息
            motor_data = self.can_decoder.decode_motor_message(data)
            if motor_data:
                self.latest_motor_data = motor_data
                self.publish_decoded_motor(motor_data)
                rospy.loginfo(f"收到电机控制: 左前轮={motor_data['vlf1_speed']:.2f}m/s, "
                            f"右前轮={motor_data['vrf1_speed']:.2f}m/s")
                
        # 如果同时收到转向和电机数据，进行运动学验证
        if self.latest_steering_data and self.latest_motor_data:
            # 检查时间戳，确保数据同步 (容忍100ms差异)
            time_diff = abs(self.latest_steering_data['timestamp'] - 
                          self.latest_motor_data['timestamp'])
            if time_diff < 0.1:
                self.validate_kinematics_from_can()
    
    def publish_decoded_steering(self, steering_data):
        """发布解码后的转向数据"""
        msg = String()
        msg.data = json.dumps(steering_data, indent=2)
        self.decoded_steering_pub.publish(msg)
    
    def publish_decoded_motor(self, motor_data):
        """发布解码后的电机数据"""
        msg = String()
        msg.data = json.dumps(motor_data, indent=2)
        self.decoded_motor_pub.publish(msg)
    
    def validate_kinematics_from_can(self):
        """从CAN数据验证运动学模型"""
        if not self.latest_steering_data or not self.latest_motor_data:
            return
            
        rospy.loginfo("开始运动学验证...")
        
        # 从转向角度反推期望的vx和rz
        control_mode = self.latest_steering_data['control_mode']
        
        # 获取转向角度 (转换为弧度)
        left_front_angle_rad = math.radians(self.latest_steering_data['left_front_angle'])
        right_front_angle_rad = math.radians(self.latest_steering_data['right_front_angle'])
        steering_speeds={
            'left_front_angle':self.latest_steering_data['left_front_angle'],
            'right_front_angle':self.latest_steering_data['right_front_angle']
        }
        rospy.loginfo(f"转向测试: 左前={steering_speeds['left_front_angle']:.1f}°, "
                      f"右前={steering_speeds['right_front_angle']:.1f}°, ")

        
        # 获取轮速
        motor_speeds = {
            'vlf1': self.latest_motor_data['vlf1_speed'],
            'vlf2': self.latest_motor_data['vlf2_speed'],
            'vrf1': self.latest_motor_data['vrf1_speed'],
            'vrf2': self.latest_motor_data['vrf2_speed'],
            'vlr1': self.latest_motor_data['vlr1_speed'],
            'vlr2': self.latest_motor_data['vlr2_speed'],
            'vrr1': self.latest_motor_data['vrr1_speed'],
            'vrr2': self.latest_motor_data['vrr2_speed']
        }
        rospy.loginfo(f"电机测试: 左前={motor_speeds['vlf1']:.1f}°, "
                      f"右前={motor_speeds['vrf1']:.1f}°, ")
        
        # 计算车辆整体运动参数
        estimated_vx, estimated_rz = self.estimate_vehicle_motion(motor_speeds, steering_speeds, control_mode)
        
        # 验证运动学一致性
        validation_result = self.check_kinematics_consistency(
            estimated_vx, estimated_rz, control_mode,
            self.latest_steering_data, self.latest_motor_data
        )
        
        # 发布验证结果
        self.publish_validation_result(validation_result)
        
        # 发布到cmd_vel用于Gazebo仿真
        if self.gazebo_available:
            cmd_vel = Twist()
            cmd_vel.linear.x = estimated_vx
            cmd_vel.angular.z = estimated_rz
            self.cmd_vel_pub.publish(cmd_vel)
        
        # 清除已处理的数据
        self.latest_steering_data = None
        self.latest_motor_data = None
    
    def estimate_vehicle_motion(self, motor_speeds, steering_speeds, control_mode):
        """从轮速估算车辆整体运动"""
        # 计算左右侧平均速度
        front_avg = (motor_speeds['vlf1'] + motor_speeds['vlf2'] +
                   motor_speeds['vlr1'] + motor_speeds['vlr2']) / 4
        rear_avg = (motor_speeds['vrf1'] + motor_speeds['vrf2'] +
                    motor_speeds['vrr1'] + motor_speeds['vrr2']) / 4
        left_front_avg=(motor_speeds['vlf1'] + motor_speeds['vlf2'] )/2
        right_front_avg = (motor_speeds['vrf1'] + motor_speeds['vrf2']) / 2
        rospy.loginfo(f"左前速度: 左前轮={left_front_avg:.2f}m/s, "
                      f"右前速度={right_front_avg:.2f}m/s")
        if(steering_speeds['left_front_angle']==0):
            estimated_vx=left_front_avg
            estimated_rz=0
            return estimated_vx, estimated_rz
        R=self.W/2+self.L/(2*math.tan(steering_speeds['left_front_angle']/180*math.pi))
        estimated_vx=R*left_front_avg*2*math.sin(steering_speeds['left_front_angle']/180*math.pi)/self.L

        #estimated_vx=R*left_front_avg/math.sqrt(R*R+self.L*self.L/4)
        estimated_rz=estimated_vx/R
        # 估算线速度和角速度
        return estimated_vx, estimated_rz
    
    def check_kinematics_consistency(self, vx, rz, control_mode, steering_data, motor_data):
        """检查运动学一致性"""
        result = {
            'timestamp': time.time(),
            'estimated_motion': {'vx': vx, 'rz': rz},
            'control_mode': control_mode,
            'steering_angles': {
                'left_front': steering_data['left_front_angle'],
                'right_front': steering_data['right_front_angle'],
                'left_rear': steering_data['left_rear_angle'],
                'right_rear': steering_data['right_rear_angle']
            },
            'motor_speeds': {
                'vlf1': motor_data['vlf1_speed'],
                'vlf2': motor_data['vlf2_speed'],
                'vrf1': motor_data['vrf1_speed'],
                'vrf2': motor_data['vrf2_speed'],
                'vlr1': motor_data['vlr1_speed'],
                'vlr2': motor_data['vlr2_speed'],
                'vrr1': motor_data['vrr1_speed'],
                'vrr2': motor_data['vrr2_speed']
            },
            'validation': {}
        }
        
        # 验证转向角度合理性
        if rz != 0:
            # 计算理论转弯半径
            actual_radius = abs(vx / rz) if rz != 0 else float('inf')
            result['validation']['turning_radius'] = actual_radius
            
            # 检查最小转弯半径
            if actual_radius < self.minRadius:
                result['validation']['radius_warning'] = f"转弯半径 {actual_radius:.2f}m 小于最小值 {self.minRadius}m"
            
            # 验证阿克曼转向几何
            if control_mode in [0, 1]:  # 四轮转向或前轮转向
                theoretical_angles = self.calculate_theoretical_steering_angles(vx, rz, control_mode)
                
                angle_errors = {
                    'left_front_error': abs(theoretical_angles['left_front'] - math.radians(steering_data['left_front_angle'])),
                    'right_front_error': abs(theoretical_angles['right_front'] - math.radians(steering_data['right_front_angle']))
                }
                
                result['validation']['steering_accuracy'] = angle_errors
                result['validation']['max_steering_error'] = max(angle_errors.values()) * 180 / math.pi
        
        # 验证轮速一致性
        speed_consistency = self.check_speed_consistency(motor_data, vx, rz, control_mode)
        result['validation']['speed_consistency'] = speed_consistency
        
        # 总体验证状态
        max_angle_error = result['validation'].get('max_steering_error', 0)
        max_speed_error = speed_consistency.get('max_error', 0)
        
        if max_angle_error < 2.0 and max_speed_error < 0.1:  # 2度角度误差，0.1m/s速度误差
            result['validation']['status'] = 'PASS'
        elif max_angle_error < 5.0 and max_speed_error < 0.2:
            result['validation']['status'] = 'WARNING'
        else:
            result['validation']['status'] = 'FAIL'
        
        return result
    
    def calculate_theoretical_steering_angles(self, vx, rz, control_mode):
        """计算理论转向角度"""
        if rz == 0:
            return {'left_front': 0, 'right_front': 0, 'left_rear': 0, 'right_rear': 0}
        
        if control_mode == 0:  # 四轮转向
            deltaa = math.atan(self.L / ((2 * vx / rz) - self.W / 2))
            deltab = math.atan(self.L / ((2 * vx / rz) + self.W / 2))
        elif control_mode == 1:  # 前轮转向
            deltaa = math.atan(self.L / ((vx / rz) - self.W / 2))
            deltab = math.atan(self.L / ((vx / rz) + self.W / 2))
        else:
            deltaa = deltab = 0
        
        return {
            'left_front': deltaa,
            'right_front': deltab,
            'left_rear': -deltaa if control_mode == 0 else 0,
            'right_rear': -deltab if control_mode == 0 else 0
        }
    
    def check_speed_consistency(self, motor_data, vx, rz, control_mode):
        """检查轮速一致性"""
        theoretical_speeds = self.calculate_theoretical_wheel_speeds(vx, rz, control_mode)
        
        errors = {}
        for wheel in ['vlf1', 'vlf2', 'vrf1', 'vrf2', 'vlr1', 'vlr2', 'vrr1', 'vrr2']:
            actual = motor_data[f'{wheel}_speed']
            theoretical = theoretical_speeds.get(wheel, actual)
            errors[f'{wheel}_error'] = abs(actual - theoretical)
        
        return {
            'individual_errors': errors,
            'max_error': max(errors.values()),
            'rms_error': math.sqrt(sum(e**2 for e in errors.values()) / len(errors))
        }
    
    def calculate_theoretical_wheel_speeds(self, vx, rz, control_mode):
        """计算理论轮速"""
        speeds = {}
        
        if rz == 0:
            # 直行，所有轮速相同
            for wheel in ['vlf1', 'vlf2', 'vrf1', 'vrf2', 'vlr1', 'vlr2', 'vrr1', 'vrr2']:
                speeds[wheel] = vx
        else:
            # 转弯，根据控制模式计算
            R = vx / rz
            
            if control_mode in [0, 3]:  # 四轮转向或遥控模式
                R0 = math.sqrt((R - self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                R1 = math.sqrt((R + self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                
                speeds['vlf1'] = abs((R0 - self.W2/2) * rz)
                speeds['vlf2'] = abs((R0 + self.W2/2) * rz)
                speeds['vrf1'] = abs((R1 - self.W2/2) * rz)
                speeds['vrf2'] = abs((R1 + self.W2/2) * rz)
                speeds['vlr1'] = speeds['vlf1']
                speeds['vlr2'] = speeds['vlf2']
                speeds['vrr1'] = speeds['vrf1']
                speeds['vrr2'] = speeds['vrf2']
                
            elif control_mode == 1:  # 前轮转向
                # 更复杂的计算，参考C++代码
                try:
                    R1 = math.sqrt((math.sqrt(R**2 - (self.L**2)/4) - self.W/2)**2 + self.L**2) * R / abs(R)
                    R2 = math.sqrt((math.sqrt(R**2 - (self.L**2)/4) + self.W/2)**2 + self.L**2) * R / abs(R)
                    R3 = math.sqrt(R**2 - (self.L**2)/4) * R / abs(R) - self.W/2
                    R4 = math.sqrt(R**2 - (self.L**2)/4) * R / abs(R) + self.W/2
                    
                    speeds['vlf1'] = abs((R1 - self.W2/2) * rz)
                    speeds['vlf2'] = abs((R1 + self.W2/2) * rz)
                    speeds['vrf1'] = abs((R2 - self.W2/2) * rz)
                    speeds['vrf2'] = abs((R2 + self.W2/2) * rz)
                    speeds['vlr1'] = abs((R3 - self.W2/2) * rz)
                    speeds['vlr2'] = abs((R3 + self.W2/2) * rz)
                    speeds['vrr1'] = abs((R4 - self.W2/2) * rz)
                    speeds['vrr2'] = abs((R4 + self.W2/2) * rz)
                except:
                    # 如果计算失败，使用简化模型
                    for wheel in ['vlf1', 'vlf2', 'vrf1', 'vrf2', 'vlr1', 'vlr2', 'vrr1', 'vrr2']:
                        speeds[wheel] = vx
        
        # 处理反向运动
        if vx < 0:
            for wheel in speeds:
                speeds[wheel] = -speeds[wheel]
        
        return speeds
    
    def publish_validation_result(self, result):
        """发布验证结果"""
        msg = String()
        msg.data = json.dumps(result, indent=2)
        self.validation_result_pub.publish(msg)
        
        # 打印关键信息
        status = result['validation']['status']
        print(f"\n{'='*60}")
        print(f"运动学验证结果: {status}")
        print(f"{'='*60}")
        print(f"估算运动: vx={result['estimated_motion']['vx']:.3f}m/s, rz={result['estimated_motion']['rz']:.3f}rad/s")
        print(f"控制模式: {result['control_mode']}")
        
        if 'turning_radius' in result['validation']:
            print(f"转弯半径: {result['validation']['turning_radius']:.2f}m")
        
        if 'max_steering_error' in result['validation']:
            print(f"最大转向角误差: {result['validation']['max_steering_error']:.2f}°")
        
        if 'speed_consistency' in result['validation']:
            print(f"最大轮速误差: {result['validation']['speed_consistency']['max_error']:.3f}m/s")
        
        print(f"{'='*60}\n")
    
    def run(self):
        """运行验证器"""
        rospy.loginfo("车辆运动学验证器开始运行...")
        rospy.loginfo("等待CAN消息... (请确保有程序在vcan0上发送数据)")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.can_bus:
                self.can_bus.shutdown()
            rospy.loginfo("验证器已停止")


def setup_vcan():
    """设置虚拟CAN接口的辅助函数"""
    import subprocess
    import os
    
    print("检查虚拟CAN接口设置...")
    
    try:
        # 检查vcan0是否存在
        result = subprocess.run(['ip', 'link', 'show', 'vcan0'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print("vcan0不存在，正在创建...")
            if os.geteuid() != 0:
                print("需要root权限设置虚拟CAN接口:")
                print("sudo modprobe vcan")
                print("sudo ip link add dev vcan0 type vcan") 
                print("sudo ip link set up vcan0")
                return False
            else:
                subprocess.run(['modprobe', 'vcan'])
                subprocess.run(['ip', 'link', 'add', 'dev', 'vcan0', 'type', 'vcan'])
                subprocess.run(['ip', 'link', 'set', 'up', 'vcan0'])
                print("vcan0创建成功")
        else:
            print("vcan0已存在")
            
        # 检查接口是否UP
        if 'UP' not in result.stdout:
            subprocess.run(['ip', 'link', 'set', 'up', 'vcan0'])
            print("vcan0已启动")
            
        return True
        
    except Exception as e:
        print(f"设置vcan0失败: {e}")
        return False


def main():
    try:
        print("车辆运动学验证器 - 从vcan0读取CAN数据")
        print("="*50)
        
        # 检查和设置虚拟CAN接口
        if not setup_vcan():
            print("\n请手动设置虚拟CAN接口后重试")
            return
        
        # 启动验证器
        validator = VehicleKinematicsValidator()
        
        print("\n验证器已启动，监听vcan0接口...")
        print("请启动你的C++程序在vcan0上发送CAN消息")
        print("按Ctrl+C停止验证器")
        
        validator.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序执行错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
