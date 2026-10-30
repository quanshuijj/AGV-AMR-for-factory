#!/usr/bin/env python3
import numpy as np
import pcl
import cv2
import os
from sklearn.neighbors import KDTree


def pcd_to_gridmap(pcd_path, output_pgm="gridmap.pgm", resolution=0.05,
                   height_min=0.2, height_max=1.0, density_threshold=3,
                   projection_plane='xy'):
    """
    将PCD点云文件转换为二维栅格地图（支持多平面投影）

    参数:
        pcd_path: 输入PCD文件路径
        output_pgm: 输出PGM文件名
        resolution: 栅格分辨率(米/像素)
        height_min: 有效点云最小高度(米)
        height_max: 有效点云最大高度(米)
        density_threshold: 障碍物密度阈值
        projection_plane: 投影平面 ('xy', 'xz' 或 'yz')
    """
    # 1. 加载点云数据
    cloud = pcl.load(pcd_path)

    # 2. 根据投影平面选择预处理方式
    if projection_plane == 'xy':
        # XY平面投影 - 在Z轴滤波
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(height_min, height_max)
        cloud_filtered = passthrough.filter()
        points = cloud_filtered.to_array()[:, :3]
        xy_points = points[:, [0, 1]]  # 取X,Y坐标
        axis_names = ('x', 'y')

    elif projection_plane == 'xz':
        # XZ平面投影 - 在Y轴滤波
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("y")
        passthrough.set_filter_limits(height_min, height_max)
        cloud_filtered = passthrough.filter()
        points = cloud_filtered.to_array()[:, :3]
        xy_points = points[:, [0, 2]]  # 取X,Z坐标
        axis_names = ('x', 'z')

    elif projection_plane == 'yz':
        # YZ平面投影 - 在X轴滤波
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("x")
        passthrough.set_filter_limits(height_min, height_max)
        cloud_filtered = passthrough.filter()
        points = cloud_filtered.to_array()[:, :3]
        xy_points = points[:, [1, 2]]  # 取Y,Z坐标
        axis_names = ('y', 'z')

    # 3. 计算点云边界
    min_vals = np.min(xy_points, axis=0)
    max_vals = np.max(xy_points, axis=0)

    # 4. 创建栅格地图
    grid_width = int((max_vals[0] - min_vals[0]) / resolution) + 1
    grid_height = int((max_vals[1] - min_vals[1]) / resolution) + 1
    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # 5. 构建KDTree加速查询
    kd_tree = KDTree(xy_points)

    # 6. 填充栅格地图
    for i in range(grid_height):
        for j in range(grid_width):
            # 计算当前栅格中心的世界坐标
            coord1 = min_vals[0] + j * resolution + resolution / 2
            coord2 = min_vals[1] + i * resolution + resolution / 2
            query_point = [[coord1, coord2]]

            # 查询半径内的点云数量
            count = kd_tree.query_radius(query_point, r=resolution / 2, count_only=True)[0]

            # 根据密度标记障碍物
            if count > density_threshold:
                grid[i, j] = 255  # 障碍物 (白色)

    # 7. 膨胀处理 (增加障碍物边界)
    kernel = np.ones((3, 3), np.uint8)
    grid = cv2.dilate(grid, kernel, iterations=1)

    # 8. 反转颜色 (PGM标准: 0=障碍物, 255=空闲)
    grid = 255 - grid

    # 9. 保存PGM文件
    cv2.imwrite(output_pgm, grid)

    # 10. 生成YAML元数据文件
    yaml_content = f"""image: {output_pgm}
resolution: {resolution}
origin: [{min_vals[0]:.3f}, {min_vals[1]:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
projection_plane: {projection_plane}
axis_names: {axis_names}
"""
    with open(os.path.splitext(output_pgm)[0] + ".yaml", "w") as f:
        f.write(yaml_content)

    print(f"栅格地图已保存至: {output_pgm}")
    print(f"投影平面: {projection_plane}")
    print(f"地图尺寸: {grid_width}x{grid_height} 像素")
    print(f"分辨率: {resolution} 米/像素")


if __name__ == "__main__":
    # 为三个平面分别生成地图
    base_path = "/home/lujiaqi/navCarProject_ws/src/test_scence/PCD/1752246970445927.pcd"

    # XY平面投影 (默认)
    pcd_to_gridmap(
        pcd_path=base_path,
        output_pgm="output_map_xy.pgm",
        resolution=0.05,
        height_min=0.2,
        height_max=1.0,
        density_threshold=3,
        projection_plane='xy'
    )

    # XZ平面投影
    pcd_to_gridmap(
        pcd_path=base_path,
        output_pgm="output_map_xz.pgm",
        resolution=0.02,
        height_min=-1.0,  # Y轴范围
        height_max=2.0,
        density_threshold=3,
        projection_plane='xz'
    )

    # YZ平面投影
    pcd_to_gridmap(
        pcd_path=base_path,
        output_pgm="output_map_yz.pgm",
        resolution=0.05,
        height_min=-1.0,  # X轴范围
        height_max=1.0,
        density_threshold=3,
        projection_plane='yz'
    )