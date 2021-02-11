import open3d as o3d
import numpy as np

print("Load a ply point cloud, print it, and render it")
#pcd = o3d.io.read_point_cloud("3d_pointcloud/-64_192.pcd")
pcd = o3d.io.read_point_cloud("ground_ref/-384_1408.pcd")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])