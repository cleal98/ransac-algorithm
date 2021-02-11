# import standard libraries
import pandas as pd
import math
import numpy as np
import open3d as o3d

def main():

    #Read point cloud
    pcd = o3d.io.read_point_cloud("udem-ccu0.10.ply").voxel_down_sample(voxel_size=0.1)
    point_cloud_in_numpy = np.asarray(pcd.points)
    df_pc = pd.DataFrame(data=point_cloud_in_numpy, columns=["x", "y", "z"])

    pcd_nplane = inrange_segment_plane(df_pc=df_pc)

    return None

def inrange_segment_plane (df_pc):

    inrange_points = pd.DataFrame(columns={"X", "Y", "Z"})
    cont_pcd = 0
    cont_r = 0
    while (len(df_pc) > cont_pcd):
        rx = math.pow(df_pc.iloc[cont_pcd,0],2)
        ry = math.pow(df_pc.iloc[cont_pcd,1],2)
        r = math.sqrt(rx+ry)

        if ((r>=0) and (r <=80)):
            x = df_pc.iloc[cont_pcd,0]
            y = df_pc.iloc[cont_pcd,1]
            z = df_pc.iloc[cont_pcd,2]

            point0 = pd.DataFrame({"X": [x], "Y": [y], "Z": [z] })
            cont_r +=1
            inrange_points = inrange_points.append(point0, ignore_index=True)
        cont_pcd +=1

    range_points = inrange_points.to_numpy()

    #convert numpy array to Open3D.o3d.geometry.PointCloud and visualize
    pcd_range = o3d.geometry.PointCloud()
    pcd_range.points = o3d.utility.Vector3dVector(range_points)

    #Segment plane with RANSAC algorithm
    segm_plane = pcd_range.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)

    inliers_result = pd.DataFrame(segm_plane[1])

    # Convert list to numpy array
    inliers_result_arr = np.asarray(inliers_result)
    inliers_result_arr.astype(np.uint32)

    #Remove plane
    pcd_out = pcd_range.select_by_index(inliers_result_arr, invert=True)

    pcd_plane = pcd_range.select_by_index(inliers_result_arr, invert=False)

    # Visualize the point cloud within open3d
    o3d.visualization.draw_geometries([pcd_out])
    o3d.visualization.draw_geometries([pcd_plane])

    # labels = np.array(pcd_out.cluster_dbscan(eps = 0.5,min_points = 5, print_progress = True))
    # print("pcd_out")
    # print(pcd_out)
    # print("len labels ")
    # print(len(labels))

    # o3d.visualization.draw_geometries([labels])

    return pcd_out

main()