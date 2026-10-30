#!/usr/bin/env python3
import numpy as np
import pcl
import cv2
import os
from sklearn.neighbors import KDTree

def pcd_to_gridmap(pcd_path, output_pgm="gridmap.pgm", resolution=0.05, z_min=0.2, z_max=1.0, density_threshold=3):
    """
    将PCD点云文件转换为二维栅格地图
    
    参数:
        pcd_path: 输入PCD文件路径
        output_pgm: 输出PGM文件名
        resolution: 栅格分辨率(米/像素)
        z_min: 有效点云最小高度(米)
        z_max: 有效点云最大高度(米)
        density_threshold: 障碍物密度阈值
    """
    # 1. 加载点云数据
    cloud = pcl.load(pcd_path)
    
    # 2. 预处理：高度滤波
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(z_min, z_max)
    cloud_filtered = passthrough.filter()
    
    # 获取点云坐标
    points = cloud_filtered.to_array()[:, :3]
    
    # 3. 投影到二维平面 (XY平面)
    xy_points = points[:, :2]
    
    # 4. 计算点云边界
    x_min, y_min = np.min(xy_points, axis=0)
    x_max, y_max = np.max(xy_points, axis=0)
    
    # 5. 创建栅格地图
    grid_width = int((x_max - x_min) / resolution) + 1
    grid_height = int((y_max - y_min) / resolution) + 1
    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)
    
    # 6. 构建KDTree加速查询
    kd_tree = KDTree(xy_points)
    
    # 7. 填充栅格地图
    for i in range(grid_height):
        for j in range(grid_width):
            # 计算当前栅格中心的世界坐标
            x = x_min + j * resolution + resolution/2
            y = y_min + i * resolution + resolution/2
            
            # 查询半径内的点云数量
            count = kd_tree.query_radius([[x, y]], r=resolution/2, count_only=True)[0]
            
            # 根据密度标记障碍物
            if count > density_threshold:
                grid[i, j] = 255  # 障碍物 (白色)
    
    # 8. 膨胀处理 (增加障碍物边界)
    kernel = np.ones((3, 3), np.uint8)
    grid = cv2.dilate(grid, kernel, iterations=1)
    
    # 9. 反转颜色 (PGM标准: 0=障碍物, 255=空闲)
    grid = 255 - grid
    
    # 10. 保存PGM文件
    cv2.imwrite(output_pgm, grid)
    
    # 11. 生成YAML元数据文件
    yaml_content = f"""image: {output_pgm}
resolution: {resolution}
origin: [{x_min:.3f}, {y_min:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(os.path.splitext(output_pgm)[0] + ".yaml", "w") as f:
        f.write(yaml_content)
    
    print(f"栅格地图已保存至: {output_pgm}")
    print(f"地图尺寸: {grid_width}x{grid_height} 像素")
    print(f"分辨率: {resolution} 米/像素")

if __name__ == "__main__":
    # 使用示例
    pcd_to_gridmap(
        pcd_path="/home/lujiaqi/navCarProject_ws/src/test_scence/PCD/1752246970445927.pcd",   # 替换为你的PCD文件路径
        output_pgm="output_map.pgm",
        resolution=0.05,              # 栅格分辨率 (0.05米/像素)
        z_min=0.2,                    # 地面以上0.2米
        z_max=1.0,                    # 高度不超过1.0米
        density_threshold=3           # 栅格内≥3个点视为障碍物
    )
