import open3d as o3d
import numpy as np
import random
import copy 
import pyransac3d as pyrsc


# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("prueba/udem-ccu0.70.ply").voxel_down_sample(voxel_size=0.1)
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)
#print(points)

plano1 = pyrsc.Plane()

best_eq, best_inliers = plano1.fit(pts=points,thresh=0.09,minPoints=3000,maxIteration=100)
#print(best_eq)
#print(best_inliers)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0]) #SOLO PLANO SEGMENTADO
obb = plane.get_oriented_bounding_box()
obb2 = plane.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(best_inliers, invert=True) #NUBE DE PUNTOS CON PLANO RECORTADO

#o3d.visualization.draw_geometries([not_plane, plane, obb, obb2])
o3d.visualization.draw_geometries([not_plane,plane])