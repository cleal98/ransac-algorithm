# This code reads .bin file (specially obtained from LIDAR),
# stores that to a.csv file, and plots the output in a point-cloud format.
import matplotlib.pyplot as plt
import numpy as np
import csv
import open3d as o3d

# Reading data #
inputPath = 'kitti-0.bin'
outputPath = 'pointsVisualization.csv'

num = np.fromfile(inputPath, dtype='float32', count=-1, sep='', offset=0)
new = np.asarray(num).reshape(-1, 4)

# Storing the encrypted (.bin) file in (.csv) file
for i in range(0, len(new)): # len(new)
    #print(new[i])
    with open(outputPath, 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(new[i])



X = num[0::4]
Y = num[1::4]
Z = num[2::4]
W = num[3::4]

# Creating point cloud
xyz = np.zeros((np.size(X), 3))
xyz[:, 0] = X
xyz[:, 1] = Y
xyz[:, 2] = Z
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(xyz)

o3d.visualization.draw_geometries([pc])