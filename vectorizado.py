# import standard libraries
import pandas as pd

import math
import numpy as np
import time
import pymap3d
import glob
import pyransac3d
import open3d as o3d

def main():

    #lidar_path = 'prueba/udem-ccu0.*.ply'
    #lidar_path = 'ground_ref/*.pcd'
    lidar_path = '3d_pointcloud/*.pcd'

    file = glob.glob(lidar_path)
    #file = sorted(file, key = lambda x: float(x[17:-4]))
    cont = 0

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name = 'Lidar')

    while (len(file)-1 > cont):

        #Read point cloud
        pcd = o3d.io.read_point_cloud(file[cont]).voxel_down_sample(voxel_size=0.05)
        pcd_nplane = inrange_segment_plane(pcd = pcd)

        vis.add_geometry(pcd_nplane)
        ctr = vis.get_view_control()
        ctr.set_front([0,0,1])
        ctr.set_zoom(0.4)
        vis.poll_events()
        vis.update_renderer()
        vis.run()
        vis.clear_geometries()
        cont += 1

    return None

def inrange_segment_plane (pcd):

    # Remove points out of range of 80 meters
    arr = np.asarray(pcd.points)
    r = np.sqrt(np.square(arr[:,0])+np.square(arr[:,1]))
    inrange = np.where(r < 5)
    pcd_inrange = pcd.select_by_index(inrange[0], invert = False)
    arr_inrange = np.asarray(pcd_inrange.points)

    # Segment plane with RANSAC algorithm
    #segmplane = dfpc0.segment_plane(distance_threshold=0.5, ransac_n=3, num_iterations=120)
    plane1 = pyransac3d.Plane()
    best_eq, best_inliers = plane1.fit(arr_inrange, thresh = 0.5, minPoints = 40000, maxIteration = 100)

    #inliers_result = pd.DataFrame(segmplane[1])

    # Convert list to numpy array
    #inliers_result_arr = np.asarray(inliers_result)
    #inliers_result_arr.astype(np.uint32)
    
    # Remove plane
    pcd_out = pcd_inrange.select_by_index(best_inliers, invert=True)

    #pcd_plane = dfpc0.select_by_index(inliers_result_arr, invert=False)
    
    # Visualize planes
    #o3d.visualization.draw_geometries([pcd_out])
    #o3d.visualization.draw_geometries([pcd_plane])

    return pcd_out

main()