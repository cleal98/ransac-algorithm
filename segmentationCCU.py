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

def main():

    #Read point cloud
    pcd = o3d.io.read_point_cloud("udem-ccu0.10.ply").voxel_down_sample(voxel_size=0.1)
    point_cloud_in_numpy = np.asarray(pcd.points)
    df_pc = pd.DataFrame(data=point_cloud_in_numpy, columns=["x", "y", "z"])
    #print(df_pc)

    range_points = inrange_segment_plane(df_pc=df_pc)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(range_points)

    # Downsampling
    downpcd = pc.voxel_down_sample(voxel_size=0.1)

    # Segmentation
    plane_model, inliers = downpcd.segment_plane(distance_threshold=1, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = downpcd.select_by_index(inliers)
    outlier_cloud = downpcd.select_by_index(inliers, invert=True)

    # Clustering using DBSCAN
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(inlier_cloud.cluster_dbscan(eps=2, min_points=10, print_progress=True))
        # esp: Distance to neighbours in a cluster
        # min_points: Minimun number of points required to form a cluster
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} cluster`s")
    colors = plt.get_cmap("tab20")(labels/(max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Visualization
    o3d.visualization.draw_geometries([inlier_cloud])


def inrange_segment_plane(df_pc):

    inrange_points = pd.DataFrame(columns={"X", "Y", "Z"})
    cont_pcd = 0
    cont_r = 0
    while (len(df_pc) > cont_pcd):
        rx = math.pow(df_pc.iloc[cont_pcd,0],2)
        ry = math.pow(df_pc.iloc[cont_pcd,1],2)
        r = math.sqrt(rx+ry)

        x = df_pc.iloc[cont_pcd,0]
        y = df_pc.iloc[cont_pcd,1]
        z = df_pc.iloc[cont_pcd,2]

        point0 = pd.DataFrame({"X": [x], "Y": [y], "Z": [z] })
        cont_r +=1
        inrange_points = inrange_points.append(point0, ignore_index=True)
        cont_pcd +=1

    range_points = inrange_points.to_numpy()

    return range_points

main()