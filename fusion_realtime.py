import numpy as np
import open3d as o3d
from tqdm import tqdm
import time

# Define a function to split the point cloud with a line
def split_point_cloud(point_cloud, dir, slope, y_intercept):
    # Calculate the z values for each point_cloud point when x is at its minimum
    z_values = point_cloud[:, 2] - (slope * point_cloud[:, 0] + y_intercept)
    
    # Use the sign of z_values to determine which side of the line each point lies on
    cloud1_indices = (z_values < 0)
    cloud2_indices = (z_values >= 0)
    
    # Split the point cloud based on the indices
    cloud1 = point_cloud[cloud1_indices]
    cloud2 = point_cloud[cloud2_indices]
    if dir <= 0:    # 切左Kinect的点 用右侧的点来判断切哪边 斜率是正在上面 斜率是负在下面
        return cloud1
    else:
        return cloud2
# Define a function to split the point cloud with a line
def split_point_cloud2(point_cloud, slope, y_intercept):
    # Calculate the z values for each point_cloud point when x is at its minimum
    z_values = point_cloud[:, 2] - (slope * point_cloud[:, 0] + y_intercept)
    
    # Use the sign of z_values to determine which side of the line each point lies on
    cloud1_indices = (z_values < 0)
    cloud2_indices = (z_values >= 0)
    
    # Split the point cloud based on the indices
    cloud1 = point_cloud[cloud1_indices]
    cloud2 = point_cloud[cloud2_indices]
    # 第二下切永远在下面
    return cloud2

#Define a function to split the point cloud with a line
def split_point_cloud3(point_cloud, dir, slope, y_intercept):
    # Calculate the z values for each point_cloud point when x is at its minimum
    z_values = point_cloud[:, 2] - (slope * point_cloud[:, 0] + y_intercept)
    
    # Use the sign of z_values to determine which side of the line each point lies on
    cloud1_indices = (z_values < 0)
    cloud2_indices = (z_values >= 0)
    
    # Split the point cloud based on the indices
    cloud1 = point_cloud[cloud1_indices]
    cloud2 = point_cloud[cloud2_indices]
    if dir <= 0:    # 切左Kinect的点 用右侧的点来判断切哪边 斜率是正在上面 斜率是负在下面
        return cloud2
    else:
        return cloud1
    
# Define a function to split the point cloud with a line
def split_point_cloud4(point_cloud, slope, y_intercept):
    # Calculate the z values for each point_cloud point when x is at its minimum
    z_values = point_cloud[:, 2] - (slope * point_cloud[:, 0] + y_intercept)
    
    # Use the sign of z_values to determine which side of the line each point lies on
    cloud1_indices = (z_values < 0)
    cloud2_indices = (z_values >= 0)
    
    # Split the point cloud based on the indices
    cloud1 = point_cloud[cloud1_indices]
    cloud2 = point_cloud[cloud2_indices]
    # 第二下切永远在下面
    return cloud1

def find_closest_point(point_cloud, target_x, target_y):
    distances = np.sqrt(np.sum((point_cloud[:, :2] - np.array([target_x, target_y]))**2, axis=1))
    closest_index = np.argmin(distances)
    closest_point = point_cloud[closest_index]
    return closest_point




# Load the point cloud
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/REALTRANS30.ply")  #left
array = np.asarray(pcd.points)
color = np.asarray(pcd.colors)



# Load the point cloud2
pcd2 = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/real_time30Rf.ply")  #right
array2 = np.asarray(pcd2.points)
color2 = np.asarray(pcd2.colors)
# threshold and slice 
stk = np.hstack((array2,color2))
#print(stk.shape)

maskb = (stk[:, 5] * 255 > 80) & (stk[:, 5] * 255 <= 255)
stk = stk[maskb]
maskr = (stk[:, 3] * 255 > 120) & (stk[:, 3] * 255 <= 255)   #250
stk = stk[maskr]
maskg = (stk[:, 4] * 255 > 120) & (stk[:, 4] * 255 <= 255)   #250
stk = stk[maskg]

array2 = stk[:,:3]
#print(stk)
#print('array:',array.shape)
color2 = stk[:,3:]


# Filter array2 based on min_x and min_y

combinep = np.vstack((array,array2))
combinec = np.vstack((color,color2))
pcdd = np.hstack((combinep,combinec))


#fiter part
# Find the index of the minimum z-coordinate
min_index = np.argmax(array[:, 2])
min_index2 = np.argmin(array2[:, 2])
# Extract x and y values from the minimum z point
min_x, min_y, min_z = array[min_index]
min_x2, min_y2, min_z2 = array2[min_index2]
print('left:',min_x, min_y, min_z)
print('right:',min_x2, min_y2, min_z2)


target_x = min_x
target_y = 100

mid_x, mid_y, mid_z = find_closest_point(array2, target_x, target_y)
print('mid:',mid_x, mid_y, mid_z)


# Calculate the slope of the line passing through (min_x, min_z) and (mid_x, mid_z)
# First line splitting
slope1 = (mid_z - min_z2) / (mid_x - min_x2)
perpendicular_slope1 = -1 / slope1
y_intercept1 = min_z2 - slope1 * min_x2

# direction
dir = (min_z2 - min_z) / (min_x2 - min_x)

cloud = split_point_cloud(pcdd, dir, slope1, y_intercept1)

# Second line splitting
slope2 = perpendicular_slope1
y_intercept2 = min_z2 - slope2 * min_x2

cloud= split_point_cloud2(cloud, slope2, y_intercept2)

# Third line splitting
slope3 = slope1
y_intercept3 = min_z - slope1 * (min_x-5)

cloud= split_point_cloud3(cloud, dir, slope3, y_intercept3)

# Fourth line splitting
slope4 = perpendicular_slope1
y_intercept4 = min_z - slope4 * min_x

cloud_final= split_point_cloud4(cloud, slope4, y_intercept4)


# save point cloud
pcd_final = o3d.geometry.PointCloud()
pcd_final.points = o3d.utility.Vector3dVector(cloud_final[:,:3])
pcd_final.colors = o3d.utility.Vector3dVector(cloud_final[:,3:])

o3d.io.write_point_cloud('F:\cuhk\calibrate0304\material/real_time_combine30fff.ply',pcd_final)
print('finish transformation')


#convex hull part
downsamplied_cloud = pcd_final.voxel_down_sample(voxel_size = 0.1)

downsamplied_cloud.estimate_normals(
    search_param = o3d.geometry.KDTreeSearchParamHybrid(
    radius=0.1, max_nn = 30
    )
)

hull, _ = downsamplied_cloud.compute_convex_hull()
print(hull)

hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)

hull_ls.paint_uniform_color((1, 0, 0))
o3d.visualization.draw_geometries([downsamplied_cloud, hull_ls])
print('done saving')


o3d.io.write_triangle_mesh("F:\cuhk\calibrate0304\material\convex hull/real_time.obj", hull)
print('done mesh')