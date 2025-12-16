import open3d as o3d
import imageio
import numpy as np

pcd = o3d.io.read_point_cloud("input.pcd")

frames = []

for i in range(200):  # 200 帧动画
    # 旋转模拟点云变化
    R = pcd.get_rotation_matrix_from_xyz((0, 0, i * 0.01))
    pcd_rot = pcd.rotate(R, center=(0, 0, 0))
    
    # 投影到 XY
    pts = np.asarray(pcd_rot.points).copy()
    pts[:, 2] = 0

    # 用 Open3D 可视化
    proj = o3d.geometry.PointCloud()
    proj.points = o3d.utility.Vector3dVector(pts)

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(proj)

    vis.poll_events()
    vis.update_renderer()
    img = vis.capture_screen_float_buffer()
    img = (np.asarray(img) * 255).astype(np.uint8)
    frames.append(img)

    vis.destroy_window()

# 保存为 GIF
imageio.mimsave("projection.gif", frames, fps=30)
