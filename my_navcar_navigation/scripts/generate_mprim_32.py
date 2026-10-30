#!/usr/bin/env python3
"""
生成大型车辆（四轮阿克曼，逆向转向）运动原语文件
参数:
  - 分辨率: 0.05m/格
  - 最小转弯半径: 3.2m
  - 最大转向角: 23度
  - numberofangles: 32
  - 每个角度11个原语
"""

import math

# ===================== 车辆参数 =====================
RESOLUTION  = 0.05       # 米/格
R_MIN       = 3.2        # 最小转弯半径 (m)
MAX_STEER   = 23.0       # 最大转向角 (度) ← 你指定
NUM_ANGLES  = 32          # 角度离散数
NUM_POSES   = 10          # 中间采样点数

# 阿克曼几何: R = L / tan(δ)  →  L = R_min / tan(δ_max)
L = R_MIN / math.tan(math.radians(MAX_STEER))
print(f"[参数] 推算轴距 L = {L:.4f} m")

# 转向角定义
SLOW_DEG  =  8.0    # 缓转向
MED_DEG   = 15.0    # 中转向
FAST_DEG  = 23.0    # 急转向 (最大)

# ===================== 局部坐标系下的阿克曼圆弧公式 =====================
#
# 设 κ = 1/R 为曲率 (κ > 0: 左转 CCW; κ < 0: 右转 CW)
# 从起点出发行驶弧长 |κs|，航向变化 κs (弧度)
#
# 局部坐标 (车头朝 0° 时, x=前, y=左):
#   左转 (κ>0):  x = +R·sin(κs)           y = +R·(1-cos(κs))
#   右转 (κ<0):  x = -R·sin(|κs|)         y = -R·(1-cos(|κs|))
#
# 全局坐标 = 旋转矩阵 × 局部坐标:
#   x_w = x_l·cos(θ₀) - y_l·sin(θ₀)
#   y_w = x_l·sin(θ₀) + y_l·cos(θ₀)
# 航向角 θ_w = θ₀ + κs
#
# 倒车: 沿同一弧线反向行驶 (arc 符号取反，κ 不变)
#   x_rev(s) = -x_fwd(s)   y_rev(s) = -y_fwd(s)
#   θ_rev = θ₀ - κs

def turn_radius(steer_deg):
    if abs(steer_deg) < 0.01:
        return float('inf')
    return L / math.tan(math.radians(steer_deg))


def gen_poses_local(x0, y0, th0_rad, kappa, dist, n, reverse):
    """
    计算从 (x0,y0,th0) 出发的圆弧轨迹点。

    参数:
        kappa: 曲率 (rad/m), >0=左转, <0=右转, 0=直线
        dist:  弧长 (m), 恒正
        reverse: False=前进, True=倒车
    返回:
        [(x_w, y_w, th_w), ...] 单位: 米, 弧度
    """
    if abs(kappa) < 1e-8:
        # 直线
        sign = -1.0 if reverse else 1.0
        step = dist / n
        poses = []
        for i in range(n):
            d = sign * (i + 1) * step
            poses.append((x0 + d * math.cos(th0_rad),
                           y0 + d * math.sin(th0_rad),
                           th0_rad))
        return poses

    R = abs(1.0 / kappa)          # 转弯半径 (恒正)
    arc_total = -dist if reverse else dist  # 倒车: 弧长取负
    arc_step  = arc_total / n

    cos0, sin0 = math.cos(th0_rad), math.sin(th0_rad)
    poses = []
    for i in range(n):
        arc_s = (i + 1) * arc_step     # 当前弧长 (有符号)
        dth   = kappa * arc_s           # 航向变化
        th_w  = th0_rad + dth           # 全局航向

        # 局部坐标
        if kappa > 0:    # 左转 (CCW in local)
            x_l =  R * math.sin(dth)
            y_l =  R * (1.0 - math.cos(dth))
        else:             # 右转 (CW in local)
            x_l = -R * math.sin(-dth)
            y_l = -R * (1.0 - math.cos(-dth))

        # 全局坐标 (旋转)
        x_w = x_l * cos0 - y_l * sin0
        y_w = x_l * sin0 + y_l * cos0

        poses.append((x_w + x0, y_w + y0, th_w))

    return poses


def m2c(m):
    return round(m / RESOLUTION)


# =====================================================================
# 11 个原语定义:
#   ID  描述                    距离  代价  转向角(°)  倒车?
#   0   直行 0.5m               0.5   1      0         F
#   1   直行 1.0m               1.0   1      0         F
#   2   直行 2.0m               2.0   1      0         F
#   3   左缓转 8°                1.0  10    +8         F
#   4   右缓转 8°                1.0  10    -8         F
#   5   左中转 15°               1.0  15   +15         F
#   6   右中转 15°               1.0  15   -15         F
#   7   左急转 23°               1.0  25   +23         F
#   8   右急转 23°               1.0  25   -23         F
#   9   倒车缓左转 8°             1.0  10    +8         T
#  10   倒车缓右转 8°             1.0  10    -8         T
# =====================================================================
PRIM = [
    # (id, 描述,             距离, 代价,  转向角°, 倒车)
    (0,  "forward_0.5m",       0.5,  1,     0.0, False),
    (1,  "forward_1.0m",        1.0,  1,     0.0, False),
    (2,  "forward_2.0m",        2.0,  1,     0.0, False),
    (3,  "left_slow_8deg",      1.0, 10,    SLOW_DEG, False),
    (4,  "right_slow_8deg",     1.0, 10,   -SLOW_DEG, False),
    (5,  "left_med_15deg",      1.0, 15,    MED_DEG,  False),
    (6,  "right_med_15deg",     1.0, 15,   -MED_DEG,  False),
    (7,  "left_fast_23deg",     1.0, 25,   FAST_DEG, False),
    (8,  "right_fast_23deg",    1.0, 25,  -FAST_DEG, False),
    (9,  "backward_left_slow",  1.0, 10,    SLOW_DEG, True),
    (10, "backward_right_slow", 1.0, 10,   -SLOW_DEG, True),
]


def build_all():
    prims = []
    for ai in range(NUM_ANGLES):
        th0 = math.radians(ai * 360.0 / NUM_ANGLES)
        for pid, desc, dist, cost, steer_deg, rev in PRIM:
            R   = turn_radius(steer_deg)
            kappa = 1.0 / R if R != float('inf') else 0.0
            poses = gen_poses_local(0.0, 0.0, th0, kappa, dist, NUM_POSES, rev)
            ex, ey, eth = poses[-1]
            prims.append(dict(
                angle_idx=ai, prim_id=pid, desc=desc,
                ex=m2c(ex), ey=m2c(ey), eth=eth,
                cost=cost, n=NUM_POSES, poses=poses,
            ))
    return prims


def write_file(fpath, prims):
    with open(fpath, 'w') as f:
        f.write(f"resolution_m: {RESOLUTION}\n")
        f.write(f"numberofangles: {NUM_ANGLES}\n")
        f.write(f"totalnumberofprimitives: {len(prims)}\n")
        for ai in range(NUM_ANGLES):
            for p in sorted([x for x in prims if x['angle_idx'] == ai],
                             key=lambda x: x['prim_id']):
                f.write(f"primID: {p['prim_id']}\n")
                f.write(f"startangle_c: {ai}\n")
                f.write(f"endpose_c: {p['ex']} {p['ey']} {p['eth']:.4f}\n")
                f.write(f"additionalactioncostmult: {p['cost']}\n")
                f.write(f"intermediateposes: {p['n']}\n")
                for px, py, pth in p['poses']:
                    f.write(f"{px/RESOLUTION:.4f} {py/RESOLUTION:.4f} "
                            f"{pth:.4f}\n")
    print(f"写入: {fpath}")


def summary(prims):
    print(f"\n========== 统计 ==========")
    print(f"总原语: {len(prims)}, 角度: {NUM_ANGLES}, 每角度: {len(prims)//NUM_ANGLES}")

    tc, cc = {}, {}
    for p in prims:
        tc[p['desc']] = tc.get(p['desc'], 0) + 1
        cc[p['cost']] = cc.get(p['cost'], 0) + 1
    print("\n类型分布:")
    for d, n in sorted(tc.items()):
        print(f"  {d}: {n}")
    print("\n代价分布:")
    for c, n in sorted(cc.items()):
        print(f"  代价{c}: {n} 个")

    # 关键验证: startangle_c = 0
    print("\n========== 验证 startangle_c=0 ==========")
    a0 = sorted([x for x in prims if x['angle_idx'] == 0],
                 key=lambda x: x['prim_id'])
    for p in a0:
        exm = p['ex'] * RESOLUTION
        eym = p['ey'] * RESOLUTION
        dth = math.degrees(p['eth'])
        print(f"  ID={p['prim_id']:2d} {p['desc']:25s} "
              f"→ end=({exm:8.3f}m, {eym:8.3f}m)  "
              f"θ={dth:7.2f}°  cost={p['cost']}")

    # 验证 startangle_c=1 (π/4 = 45°)
    print("\n========== 验证 startangle_c=1 ==========")
    a1 = sorted([x for x in prims if x['angle_idx'] == 1],
                 key=lambda x: x['prim_id'])
    for p in a1:
        exm = p['ex'] * RESOLUTION
        eym = p['ey'] * RESOLUTION
        dth = math.degrees(p['eth'])
        print(f"  ID={p['prim_id']:2d} {p['desc']:25s} "
              f"→ end=({exm:8.3f}m, {eym:8.3f}m)  "
              f"θ={dth:7.2f}°  cost={p['cost']}")


if __name__ == '__main__':
    prims = build_all()
    summary(prims)
    out = ("/home/zxh/navCarProject_ws/src/my_navcar_navigation/config/"
           "large_vehicle_0_05m_r3p2_32angles.mprim")
    write_file(out, prims)
    print(f"\n文件已生成: {out}")
