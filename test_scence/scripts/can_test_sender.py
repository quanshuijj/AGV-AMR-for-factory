#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN测试数据发送器
用于在vcan0上发送测试CAN消息，验证运动学模型
"""

import can
import time
import math
import argparse
import threading
from dataclasses import dataclass
from typing import List


@dataclass
class TestCase:
    """测试用例数据结构"""
    name: str
    vx: float          # 线速度 (m/s)
    rz: float          # 角速度 (rad/s)
    control_mode: int  # 控制模式
    duration: float    # 持续时间 (s)


class CANTestSender:
    """CAN测试数据发送器"""
    
    def __init__(self, interface='vcan0'):
        self.interface = interface
        self.bus = None
        self.running = False
        
        # 车辆参数 (与C++代码保持一致)
        self.L = 2.85  # 轴距 (m)
        self.W = 1.64  # 轮距 (m) 
        self.W2 = 0.42  # 每侧双轮距离的一半 (m)
        self.wheelCircumference = 1.885  # 轮胎周长 (m)
        
        # CAN协议参数
        self.STEERING_RATIO_PARAMS = 0.2  # 转向角度比例 (deg/bit)
        self.MOTOR_RATIO_PARAMS = 12.0    # 电机转速比例 (rpm/bit)
        
        # CAN ID定义
        self.STEERING_CONTROL_ID = 0x8F110D8
        self.MOTOR_CONTROL_ID = 0x8F210D8
        
        # 初始化CAN接口
        self.init_can()
    
    def init_can(self):
        """初始化CAN接口"""
        try:
            self.bus = can.interface.Bus(channel=self.interface, 
                                       bustype='socketcan')
            print(f"成功连接到CAN接口: {self.interface}")
        except Exception as e:
            print(f"无法连接到CAN接口 {self.interface}: {e}")
            print("请确保已设置虚拟CAN接口:")
            print("sudo modprobe vcan")
            print("sudo ip link add dev vcan0 type vcan")
            print("sudo ip link set up vcan0")
            self.bus = None
    
    def calculate_control_params(self, vx, rz, control_mode):
        """
        计算控制参数 - 复现C++逻辑
        """
        steering_angles = {'deltaa': 0.0, 'deltab': 0.0}
        motor_speeds = {
            'vlf1': 0.0, 'vlf2': 0.0, 'vrf1': 0.0, 'vrf2': 0.0,
            'vlr1': 0.0, 'vlr2': 0.0, 'vrr1': 0.0, 'vrr2': 0.0
        }
        
        if control_mode == 0:  # 四轮转向
            if rz != 0:
                steering_angles['deltaa'] = math.atan(self.L / ((2 * vx / rz) - self.W / 2))
                steering_angles['deltab'] = math.atan(self.L / ((2 * vx / rz) + self.W / 2))
                
                R = vx / rz
                R0 = math.sqrt((R - self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                R1 = math.sqrt((R + self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                
                motor_speeds['vlf1'] = abs((R0 - self.W2/2) * rz)
                motor_speeds['vlf2'] = abs((R0 + self.W2/2) * rz)
                motor_speeds['vrf1'] = abs((R1 - self.W2/2) * rz)
                motor_speeds['vrf2'] = abs((R1 + self.W2/2) * rz)
                motor_speeds['vlr1'] = motor_speeds['vlf1']
                motor_speeds['vlr2'] = motor_speeds['vlf2']
                motor_speeds['vrr1'] = motor_speeds['vrf1']
                motor_speeds['vrr2'] = motor_speeds['vrf2']
                
                if vx < 0:
                    for key in motor_speeds:
                        motor_speeds[key] = -motor_speeds[key]
            else:
                # 直行
                for key in motor_speeds:
                    motor_speeds[key] = vx
                    
        elif control_mode == 1:  # 前轮转向
            if rz != 0:
                steering_angles['deltaa'] = math.atan(self.L / ((vx / rz) - self.W / 2))
                steering_angles['deltab'] = math.atan(self.L / ((vx / rz) + self.W / 2))
                
                R = vx / rz
                try:
                    R1 = math.sqrt((math.sqrt(R**2 - (self.L**2)/4) - self.W/2)**2 + self.L**2) * R / abs(R)
                    R2 = math.sqrt((math.sqrt(R**2 - (self.L**2)/4) + self.W/2)**2 + self.L**2) * R / abs(R)
                    R3 = math.sqrt(R**2 - (self.L**2)/4) * R / abs(R) - self.W/2
                    R4 = math.sqrt(R**2 - (self.L**2)/4) * R / abs(R) + self.W/2
                    
                    motor_speeds['vlf1'] = abs((R1 - self.W2/2) * rz)
                    motor_speeds['vlf2'] = abs((R1 + self.W2/2) * rz)
                    motor_speeds['vrf1'] = abs((R2 - self.W2/2) * rz)
                    motor_speeds['vrf2'] = abs((R2 + self.W2/2) * rz)
                    motor_speeds['vlr1'] = abs((R3 - self.W2/2) * rz)
                    motor_speeds['vlr2'] = abs((R3 + self.W2/2) * rz)
                    motor_speeds['vrr1'] = abs((R4 - self.W2/2) * rz)
                    motor_speeds['vrr2'] = abs((R4 + self.W2/2) * rz)
                    
                    if vx < 0:
                        for key in motor_speeds:
                            motor_speeds[key] = -motor_speeds[key]
                except:
                    # 如果计算失败，使用简化模型
                    for key in motor_speeds:
                        motor_speeds[key] = vx
            else:
                # 直行
                for key in motor_speeds:
                    motor_speeds[key] = vx
                    
        elif control_mode == 3:  # 遥控模式 (与四轮转向相同)
            if rz != 0:
                R = vx / rz
                R0 = math.sqrt((R - self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                R1 = math.sqrt((R + self.W/2)**2 + (self.L**2)/4) * R / abs(R)
                
                motor_speeds['vlf1'] = abs((R0 - self.W2/2) * rz)
                motor_speeds['vlf2'] = abs((R0 + self.W2/2) * rz)
                motor_speeds['vrf1'] = abs((R1 - self.W2/2) * rz)
                motor_speeds['vrf2'] = abs((R1 + self.W2/2) * rz)
                motor_speeds['vlr1'] = motor_speeds['vlf1']
                motor_speeds['vlr2'] = motor_speeds['vlf2']
                motor_speeds['vrr1'] = motor_speeds['vrf1']
                motor_speeds['vrr2'] = motor_speeds['vrf2']
                
                if vx < 0:
                    for key in motor_speeds:
                        motor_speeds[key] = -motor_speeds[key]
            else:
                for key in motor_speeds:
                    motor_speeds[key] = vx
        
        return steering_angles, motor_speeds
    
    def convert_to_can_data(self, steering_angles, motor_speeds, control_mode):
        """
        转换为CAN数据格式 - 复现C++的convertToActualSteeringMsg和convertToActualMotorMsg
        """
        # 转向控制数据
        thetaa = steering_angles['deltaa'] * 180 / math.pi  # rad->deg
        thetab = steering_angles['deltab'] * 180 / math.pi  # rad->deg
        
        # 限制转向角度
        thetaa = max(-24, min(24, thetaa))
        thetab = max(-24, min(24, thetab))
        
        steering_data = [
            int(thetab / self.STEERING_RATIO_PARAMS + 128),   # 右前轮
            int(thetaa / self.STEERING_RATIO_PARAMS + 128),   # 左前轮
            int(-thetab / self.STEERING_RATIO_PARAMS + 128),  # 右后轮
            int(-thetaa / self.STEERING_RATIO_PARAMS + 128),  # 左后轮
            control_mode                                      # 模式控制
        ]
        
        # 确保数据在0-255范围内
        steering_data = [max(0, min(255, x)) for x in steering_data]
        
        # 电机控制数据
        motor_data = []
        for speed_key in ['vlf1', 'vlf2', 'vrf1', 'vrf2', 'vlr1', 'vlr2', 'vrr1', 'vrr2']:
            speed = motor_speeds[speed_key]
            rpm = (speed * 60) / self.wheelCircumference
            can_value = int(rpm / self.MOTOR_RATIO_PARAMS + 128)
            can_value = max(0, min(255, can_value))
            motor_data.append(can_value)
        
        return steering_data, motor_data
    
    def send_can_message(self, can_id, data):
        """发送CAN消息"""
        if self.bus is None:
            return False
            
        try:
            message = can.Message(arbitration_id=can_id, 
                                data=data, 
                                is_extended_id=True)
            self.bus.send(message)
            return True
        except Exception as e:
            print(f"发送CAN消息失败: {e}")
            return False
    
    def send_test_case(self, test_case: TestCase):
        """发送单个测试用例"""
        print(f"\n执行测试: {test_case.name}")
        print(f"参数: vx={test_case.vx:.2f}m/s, rz={test_case.rz:.3f}rad/s, mode={test_case.control_mode}")
        
        # 计算控制参数
        steering_angles, motor_speeds = self.calculate_control_params(
            test_case.vx, test_case.rz, test_case.control_mode
        )
        
        # 转换为CAN数据
        steering_data, motor_data = self.convert_to_can_data(
            steering_angles, motor_speeds, test_case.control_mode
        )
        
        print(f"转向CAN数据: {steering_data}")
        print(f"电机CAN数据: {motor_data}")
        
        # 发送CAN消息
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < test_case.duration:
            if not self.running:
                break
                
            # 发送转向控制消息
            if self.send_can_message(self.STEERING_CONTROL_ID, steering_data):
                message_count += 1
            
            # 发送电机控制消息
            if self.send_can_message(self.MOTOR_CONTROL_ID, motor_data):
                message_count += 1
            
            time.sleep(0.05)  # 50ms周期，与C++代码一致
        
        print(f"测试完成，发送了 {message_count} 条消息")
    
    def run_test_suite(self):
        """运行测试套件"""
        test_cases = [
            TestCase("直行测试", 1.0, 0.0, 0, 3.0),
            TestCase("四轮转向左转", 1.0, 0.1, 0, 3.0),
            TestCase("四轮转向右转", 1.0, -0.1, 0, 3.0),
            TestCase("前轮转向左转", 1.0, 0.1, 1, 3.0),
            TestCase("前轮转向右转", 1.0, -0.1, 1, 3.0),
            TestCase("低速转弯", 0.5, 0.2, 0, 3.0),
            TestCase("高速转弯", 2.0, 0.05, 0, 3.0),
            TestCase("倒车直行", -1.0, 0.0, 0, 3.0),
            TestCase("倒车转弯", -1.0, 0.1, 0, 3.0),
            TestCase("遥控模式测试", 1.0, 0.1, 3, 3.0),
        ]
        
        self.running = True
        
        try:
            for i, test_case in enumerate(test_cases):
                print(f"\n{'='*50}")
                print(f"测试 {i+1}/{len(test_cases)}")
                self.send_test_case(test_case)
                
                if i < len(test_cases) - 1:
                    print("等待2秒后进行下一个测试...")
                    time.sleep(2.0)
                    
        except KeyboardInterrupt:
            print("\n测试被用户中断")
        finally:
            self.running = False
        
        print(f"\n{'='*50}")
        print("所有测试完成!")
    
    def interactive_mode(self):
        """交互模式"""
        print("\n进入交互模式")
        print("输入 'quit' 或 'exit' 退出")
        
        self.running = True
        
        try:
            while self.running:
                print("\n请输入测试参数:")
                try:
                    vx = float(input("线速度 vx (m/s): "))
                    rz = float(input("角速度 rz (rad/s): "))
                    control_mode = int(input("控制模式 (0:四轮转向, 1:前轮转向, 3:遥控): "))
                    duration = float(input("持续时间 (秒): "))
                    
                    test_case = TestCase("交互测试", vx, rz, control_mode, duration)
                    self.send_test_case(test_case)
                    
                except ValueError:
                    print("输入格式错误，请重新输入")
                    continue
                except KeyboardInterrupt:
                    break
                    
                continue_test = input("\n继续测试? (y/n): ")
                if continue_test.lower() not in ['y', 'yes']:
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
        
        print("退出交互模式")
    
    def continuous_mode(self, vx, rz, control_mode):
        """连续发送模式"""
        print(f"\n连续发送模式: vx={vx}m/s, rz={rz}rad/s, mode={control_mode}")
        print("按Ctrl+C停止")
        
        # 计算控制参数
        steering_angles, motor_speeds = self.calculate_control_params(vx, rz, control_mode)
        steering_data, motor_data = self.convert_to_can_data(steering_angles, motor_speeds, control_mode)
        
        self.running = True
        message_count = 0
        
        try:
            while self.running:
                if self.send_can_message(self.STEERING_CONTROL_ID, steering_data):
                    message_count += 1
                if self.send_can_message(self.MOTOR_CONTROL_ID, motor_data):
                    message_count += 1
                
                if message_count % 100 == 0:
                    print(f"已发送 {message_count} 条消息")
                
                time.sleep(0.05)  # 50ms周期
                
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
        
        print(f"\n连续发送完成，总共发送 {message_count} 条消息")
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        if self.bus:
            self.bus.shutdown()


def main():
    parser = argparse.ArgumentParser(description='CAN测试数据发送器')
    parser.add_argument('--interface', '-i', default='vcan0', help='CAN接口名称')
    parser.add_argument('--mode', '-m', choices=['test', 'interactive', 'continuous'], 
                       default='test', help='运行模式')
    parser.add_argument('--vx', type=float, default=1.0, help='线速度 (m/s)')
    parser.add_argument('--rz', type=float, default=0.1, help='角速度 (rad/s)')
    parser.add_argument('--control', type=int, default=0, help='控制模式')
    
    args = parser.parse_args()
    
    print("CAN测试数据发送器")
    print("="*30)
    
    sender = CANTestSender(args.interface)
    
    if sender.bus is None:
        print("无法初始化CAN接口，程序退出")
        return
    
    try:
        if args.mode == 'test':
            print("运行测试套件模式")
            sender.run_test_suite()
        elif args.mode == 'interactive':
            print("运行交互模式")
            sender.interactive_mode()
        elif args.mode == 'continuous':
            print("运行连续发送模式")
            sender.continuous_mode(args.vx, args.rz, args.control)
            
    finally:
        sender.cleanup()
        print("程序结束")


if __name__ == '__main__':
    main()
