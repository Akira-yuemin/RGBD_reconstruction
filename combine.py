import numpy as np
import open3d as o3d
from tqdm import tqdm
import time

# Load the point cloud
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/REALTRANS30.ply")
array = np.asarray(pcd.points)
color = np.asarray(pcd.colors)

# Load the point cloud2
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/real_time30R.ply")
array2 = np.asarray(pcd.points)
color2 = np.asarray(pcd.colors)


combinep = np.vstack((array,array2))
combinec = np.vstack((color,color2))

# save point cloud
pcdfin = o3d.geometry.PointCloud()
pcdfin.points = o3d.utility.Vector3dVector(combinep)
pcdfin.colors = o3d.utility.Vector3dVector(combinec)

o3d.io.write_point_cloud('F:\cuhk\calibrate0304\material/test.ply',pcdfin)
print('finish transformation')
