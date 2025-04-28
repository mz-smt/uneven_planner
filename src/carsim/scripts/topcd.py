import numpy as np
import open3d as o3d

# 参数设置
length = 40   # x 方向长度
width = 40    # y 方向宽度
thickness = 0.1
z_center = 0.0 + thickness / 2
resolution = 0.02  # 网格分辨率
theta = 0.349066  # 20° 的弧度值，绕 Y 轴旋转

# 1. 生成原始平面点（未旋转）
x = np.arange(-length / 2, length / 2, resolution)
y = np.arange(-width / 2, width / 2, resolution)
xx, yy = np.meshgrid(x, y)
zz = np.full_like(xx, z_center)
points = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3)

# 2. 绕 Y 轴旋转（右手坐标系）
Ry = np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
])
rotated_points = points @ Ry.T  # 注意是点 * R.T

# 3. 保存为 PCD 文件
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(rotated_points)
o3d.io.write_point_cloud("slope20deg_fixed.pcd", pcd)
