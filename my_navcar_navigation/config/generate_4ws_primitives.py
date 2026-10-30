#!/usr/bin/env python3
"""
生成4WS大型车辆运动基元文件 (large_vehicle_4ws.mprim)

车辆参数:
  - 轴距: 2.5m
  - 最小转弯半径: 3.2m
  - 最大转向角: 25° (机械限位)
  - 4WS逆向转向: 前后轮转向角相同,方向相反,瞬心在轴距中点

运动学:
  - R = L / (2 * tan(α))  (4WS逆向转向)
  - 当 α = arctan(L/(2R)) = arctan(2.5/(2*3.2)) ≈ 21.3° 时, R = 3.2m
"""

import math

# ========== 车辆参数 ==========
WHEELBASE = 2.5           # 轴距 (m)
RESOLUTION = 0.05         # 地图分辨率 (m/cell)
NUM_ANGLES = 32           # 角度离散数量
NUM_INTERMEDIATE_POSES = 10

# 4WS逆向转向参数
# R = L / (2 * tan(alpha)) => alpha = arctan(L / (2R))
MIN_TURN_RADIUS = 3.2     # 最小转弯半径 (m)
MAX_STEERING_ANGLE = 25.0 * math.pi / 180.0  # 最大机械转向角 (rad)

# 计算实际转向角 (达到最小转弯半径时)
STEERING_AT_MIN_RADIUS = math.atan(WHEELBASE / (2 * MIN_TURN_RADIUS))
print(f"达到最小转弯半径{ MIN_TURN_RADIUS}m所需的转向角: {math.degrees(STEERING_AT_MIN_RADIUS):.2f}°")

# 验证
R_check = WHEELBASE / (2 * math.tan(STEERING_AT_MIN_RADIUS))
print(f"验证: R = {WHEELBASE}/(2*tan({math.degrees(STEERING_AT_MIN_RADIUS):.2f}°)) = {R_check:.2f}m")

# ========== 基元定义 ==========
# 格式: (名称, 转向系数, 距离(m), 代价倍数)
# 转向系数范围 [0, 1]: 0=直行, 1=最大转向角
PRIMITIVES = [
    # 直行基元 - 代价最低
    ("FWD_LONG", 0.0, 2.5, 1),     # 长直行 2.5m
    ("FWD_MED", 0.0, 1.5, 1),      # 中直行 1.5m
    ("FWD_SHORT", 0.0, 0.75, 2),    # 短直行 0.75m
    
    # 缓弯 (0.4 = 8.5°)
    ("TURN_SLIGHT_L", 0.4, 1.5, 4),
    ("TURN_SLIGHT_R", -0.4, 1.5, 4),
    
    # 中等弯 (0.6 = 12.8°)
    ("TURN_MED_L", 0.6, 1.2, 6),
    ("TURN_MED_R", -0.6, 1.2, 6),
    
    # 较急弯 (0.8 = 17.0°)
    ("TURN_SHARP_L", 0.8, 1.0, 8),
    ("TURN_SHARP_R", -0.8, 1.0, 8),
    
    # 急弯 (1.0 = 21.3°, 对应最小转弯半径3.2m)
    ("TURN_TIGHT_L", 1.0, 0.8, 12),
    ("TURN_TIGHT_R", -1.0, 0.8, 12),
    
    # 倒车基元 - 代价高
    ("BACK_LONG", 0.0, -1.5, 20),
    ("BACK_MED", 0.0, -1.0, 25),
    ("BACK_SHORT", 0.0, -0.5, 30),
    
    # 倒车转弯
    ("BACK_TURN_L", 0.5, -0.8, 40),
    ("BACK_TURN_R", -0.5, -0.8, 40),
    
    # 微调基元
    ("MICRO_FWD", 0.0, 0.25, 3),
    ("MICRO_TURN_L", 0.3, 0.3, 5),
    ("MICRO_TURN_R", -0.3, 0.3, 5),
]

def compute_4ws_trajectory(start_angle, steering_coeff, distance):
    """
    计算4WS逆向转向的轨迹
    
    4WS逆向转向: 前后轮转向角相同(大小),方向相反
    瞬心在轴距中点
    
    参数:
      start_angle: 起始航向角 (rad)
      steering_coeff: 转向系数 [-1, 1]
        0 = 直行
        1 = 左转 (前后轮同时向左转, 前轮转α, 后轮转-α)
       -1 = 右转
      distance: 行驶距离 (m), 正数前进, 负数倒车
    """
    alpha = steering_coeff * STEERING_AT_MIN_RADIUS  # 实际转向角
    
    poses = []
    
    for i in range(NUM_INTERMEDIATE_POSES + 1):
        t = i / NUM_INTERMEDIATE_POSES
        d = t * distance
        
        if abs(alpha) < 1e-6 or abs(d) < 1e-6:
            # 直线
            dx = d * math.cos(start_angle)
            dy = d * math.sin(start_angle)
            dtheta = 0.0
        else:
            # 4WS弧线运动
            # 瞬心在轴距中点, 到前后轴距离 = L/2
            # R = (L/2) / tan(alpha)
            tan_alpha = math.tan(alpha)
            R = (WHEELBASE / 2.0) / tan_alpha
            
            # 角速度 (相对于瞬心)
            angular_change = d / R
            
            if alpha > 0:
                # 左转: 瞬心在车辆左侧
                # 圆心相对于车辆起点位置
                cx = -R * math.sin(start_angle)
                cy = R * math.cos(start_angle)
                
                # 当前弧长对应的角度
                theta = angular_change
                
                # 车辆位置 (相对于圆心)
                dx = cx + R * math.sin(start_angle + theta)
                dy = cy - R * math.cos(start_angle + theta)
            else:
                # 右转: 瞬心在车辆右侧
                cx = R * math.sin(start_angle)
                cy = -R * math.cos(start_angle)
                
                theta = -angular_change  # 右转为负角度
                
                dx = cx - R * math.sin(start_angle - theta)
                dy = cy + R * math.cos(start_angle - theta)
            
            dtheta = angular_change
        
        poses.append((dx, dy, dtheta))
    
    return poses

def generate_mprim(filename):
    """生成.mprim文件"""
    
    total = NUM_ANGLES * len(PRIMITIVES)
    
    with open(filename, 'w') as f:
        f.write(f"resolution_m: {RESOLUTION}\n")
        f.write(f"numberofangles: {NUM_ANGLES}\n")
        f.write(f"totalnumberofprimitives: {total}\n")
        
        prim_id = 0
        
        for angle_idx in range(NUM_ANGLES):
            start_angle = angle_idx * 2.0 * math.pi / NUM_ANGLES
            
            for name, coeff, dist, cost in PRIMITIVES:
                poses = compute_4ws_trajectory(start_angle, coeff, dist)
                
                # 终点
                dx_end, dy_end, dtheta_end = poses[-1]
                dx_cells = int(round(dx_end / RESOLUTION))
                dy_cells = int(round(dy_end / RESOLUTION))
                
                # 终点角度索引
                end_angle_raw = angle_idx + dtheta_end * NUM_ANGLES / (2 * math.pi)
                end_angle_idx = int(round(end_angle_raw)) % NUM_ANGLES
                
                # 写入
                f.write(f"primID: {prim_id}\n")
                f.write(f"startangle_c: {angle_idx}\n")
                f.write(f"endpose_c: {dx_cells} {dy_cells} {end_angle_idx}\n")
                f.write(f"additionalactioncostmult: {cost}\n")
                f.write(f"intermediateposes: {NUM_INTERMEDIATE_POSES}\n")
                
                for dx, dy, dtheta in poses:
                    f.write(f"{dx:.4f} {dy:.4f} {dtheta:.4f}\n")
                
                prim_id += 1
    
    return total

if __name__ == "__main__":
    output = "/home/zxh/navCarProject_ws/src/my_navcar_navigation/config/large_vehicle_4ws.mprim"
    
    print("=" * 60)
    print("生成4WS大型车辆运动基元")
    print("=" * 60)
    print(f"轴距: {WHEELBASE}m")
    print(f"最小转弯半径: {MIN_TURN_RADIUS}m")
    print(f"转向角(达到最小R时): {math.degrees(STEERING_AT_MIN_RADIUS):.2f}°")
    print(f"角度离散: {NUM_ANGLES}")
    print(f"基元类型: {len(PRIMITIVES)}")
    print("=" * 60)
    
    print("\n基元列表:")
    for name, coeff, dist, cost in PRIMITIVES:
        if abs(coeff) < 1e-6:
            turn_info = "直行"
        else:
            actual_angle = abs(coeff) * math.degrees(STEERING_AT_MIN_RADIUS)
            R = (WHEELBASE/2) / math.tan(abs(coeff) * STEERING_AT_MIN_RADIUS)
            turn_info = f"α={actual_angle:.1f}°, R={R:.2f}m"
        direction = "前进" if dist > 0 else "倒车"
        print(f"  {name}: {turn_info}, {direction}{abs(dist):.2f}m, 代价{cost}x")
    
    total = generate_mprim(output)
    print(f"\n生成文件: {output}")
    print(f"总基元数: {total}")
    print("完成!")
