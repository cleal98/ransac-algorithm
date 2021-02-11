# This code reads a .bin file, stores the data in a .csv file, and 
# processes (filtering, segmenting, & clustering) LiDAR point
# cloud data using "open3d" library in Python.
 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import figure
import open3d as o3d
import csv
import pandas as pd
import math
import pyransac3d as pyrsc

def main():

    #Read point cloud
    pcd = o3d.io.read_point_cloud("prueba/udem-ccu0.70.ply")
    point_cloud_in_numpy = np.asarray(pcd.points)
    df_pc = pd.DataFrame(data=point_cloud_in_numpy, columns=["x", "y", "z"])
    #print(df_pc)

    #pcd_out = inrange_segment_plane(df_pc=df_pc)
    #pc = o3d.geometry.PointCloud()
    #pc.points = o3d.utility.Vector3dVector(pcd_out)

    # Downsampling
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)

    """
    # Segmentation
    plane_model, inliers = downpcd.segment_plane(distance_threshold=1, ransac_n=3000, num_iterations=100)
    print(len(inliers))
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = downpcd.select_by_index(inliers)
    outlier_cloud = downpcd.select_by_index(inliers, invert=True)
    print(inlier_cloud)
    """
    plane_model, inliers = ransac_algorithm(downpcd)
    print(len(inliers))
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = downpcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
    outlier_cloud = downpcd.select_by_index(inliers, invert=True)
    print(inlier_cloud)
    print(outlier_cloud)
    o3d.visualization.draw_geometries([outlier_cloud])


    # Clustering using DBSCAN
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=1.5, min_points=50, print_progress=True))
        # esp: Distance to neighbours in a cluster
        # min_points: Minimun number of points required to form a cluster
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} cluster`s")
    colors = plt.get_cmap("tab20")(labels/(max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Visualization
    o3d.visualization.draw_geometries([outlier_cloud])


def ransac_algorithm(pcd_load):

    points = np.asarray(pcd_load.points)
    #print(points)
    plano1 = pyrsc.Plane()
    best_eq, best_inliers = plano1.fit(pts=points,thresh=0.09,minPoints=3000,maxIteration=100)

    #plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0]) #SOLO PLANO SEGMENTADO
    #obb = plane.get_oriented_bounding_box()
    #obb2 = plane.get_axis_aligned_bounding_box()
    #obb.color = [0, 0, 1]
    #obb2.color = [0, 1, 0]
    #not_plane = pcd_load.select_by_index(best_inliers, invert=True) #NUBE DE PUNTOS CON PLANO RECORTADO

    #o3d.visualization.draw_geometries([not_plane, plane, obb, obb2])
    #o3d.visualization.draw_geometries([not_plane,plane])

    return best_eq,best_inliers

main()