import numpy as np
import cv2
import open3d as o3d
import pykinect_azure as pykinect
from tqdm import tqdm
import time




class Open3dVisualizer():
    def __init__(self, window_width=1600, window_height=1200):
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.2)
        self.o3d_started = False
        self.points_accumulated = []
        self.colors_accumulated = []


        # Create a window with specified width and height
        #self.vis = o3d.visualization.Visualizer()
        #self.vis.create_window(width=window_width, height=window_height)

    def __call__(self, points_3d, rgb_image=None):
        self.update(points_3d, rgb_image)

    def update(self, points_3d, rgb_image=None):
        points_3d = np.asarray(points_3d)
        

        # #Filter points within 40cm (0.4m) away from the sensor
        # distance_threshold = 0.1
        # distances = np.linalg.norm(points_3d, axis=1)
        # points_within_threshold = points_3d[distances <= distance_threshold]

        # # Update point cloud
        # self.point_cloud.points = o3d.utility.Vector3dVector(points_within_threshold)
        
    
        if rgb_image is not None:
            colors = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2RGB).reshape(-1, 3) / 255
            self.point_cloud.colors = o3d.utility.Vector3dVector(colors)

        self.point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        # if not self.o3d_started:
        #     self.vis.add_geometry(self.point_cloud)
        #     self.o3d_started = True
        # else:
        #     self.vis.update_geometry(self.point_cloud)

        # self.vis.poll_events()
        # self.vis.update_renderer()

        #speed up

        # bounding box function

        tem_color_box = np.empty([0,3])
        tem_point_box = np.empty([0,3])

        # for i,nums in tqdm(enumerate(points_3d), total=len(points_3d)):
        #     if nums[2] < 400:
        #         tem_point_box = np.vstack((tem_point_box,nums))
        #         tem_color_box = np.vstack((tem_color_box,colors[i,:]))
        
        # Find indices where the z-coordinate is less than 400
        indices = (points_3d[:, 2] > 300) & (points_3d[:, 2] < 400) &  (points_3d[:, 0] < 200)#right
        

        # Filter points and colors based on the condition
        filtered_points = points_3d[indices]
        filtered_colors = colors[indices]

        # If you still need `tem_point_box` and `tem_color_box` as numpy arrays
        tem_point_box = np.vstack((tem_point_box, filtered_points))
        tem_color_box = np.vstack((tem_color_box, filtered_colors))

        points_3d = tem_point_box
        colors = tem_color_box

        self.points_accumulated.append(points_3d)
        self.colors_accumulated.append(colors)
        

    def save_point_cloud(self, filename):
        try:
            all_points = np.concatenate(self.points_accumulated, axis=0)
            all_colors = np.concatenate(self.colors_accumulated, axis=0)

            self.point_cloud.points = o3d.utility.Vector3dVector(all_points)
            self.point_cloud.colors = o3d.utility.Vector3dVector(all_colors)
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.2)
            o3d.io.write_point_cloud(filename, self.point_cloud)
            print(f"Point cloud saved to {filename}")
        except Exception as e:
            print(f"Error saving point cloud: {e}")



# Initialize the library, if the library is not found, add the library path as argument
pykinect.initialize_libraries()





class Open3dVisualizer2():
    def __init__(self, window_width=1600, window_height=1200):
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.2)
        self.o3d_started = False
        self.points_accumulated = []
        self.colors_accumulated = []


        # Create a window with specified width and height
        #self.vis = o3d.visualization.Visualizer()
        #self.vis.create_window(width=window_width, height=window_height)

    def __call__(self, points_3d, rgb_image=None):
        self.update(points_3d, rgb_image)

    def update(self, points_3d, rgb_image=None):
        points_3d = np.asarray(points_3d)
        

        # #Filter points within 40cm (0.4m) away from the sensor
        # distance_threshold = 0.1
        # distances = np.linalg.norm(points_3d, axis=1)
        # points_within_threshold = points_3d[distances <= distance_threshold]

        # # Update point cloud
        # self.point_cloud.points = o3d.utility.Vector3dVector(points_within_threshold)
        
    
        if rgb_image is not None:
            colors = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2RGB).reshape(-1, 3) / 255
            self.point_cloud.colors = o3d.utility.Vector3dVector(colors)

        self.point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        # if not self.o3d_started:
        #     self.vis.add_geometry(self.point_cloud)
        #     self.o3d_started = True
        # else:
        #     self.vis.update_geometry(self.point_cloud)

        # self.vis.poll_events()
        # self.vis.update_renderer()

        #speed up

        # bounding box function

        tem_color_box = np.empty([0,3])
        tem_point_box = np.empty([0,3])

        # for i,nums in tqdm(enumerate(points_3d), total=len(points_3d)):
        #     if nums[2] < 400:
        #         tem_point_box = np.vstack((tem_point_box,nums))
        #         tem_color_box = np.vstack((tem_color_box,colors[i,:]))
        
        # Find indices where the z-coordinate is less than 400
        indices = (points_3d[:, 2] > 490) & (points_3d[:, 2] < 600) & (points_3d[:, 0] < 200) & (points_3d[:, 0] > -200)#right
        

        # Filter points and colors based on the condition
        filtered_points = points_3d[indices]
        filtered_colors = colors[indices]

        # If you still need `tem_point_box` and `tem_color_box` as numpy arrays
        tem_point_box = np.vstack((tem_point_box, filtered_points))
        tem_color_box = np.vstack((tem_color_box, filtered_colors))

        points_3d = tem_point_box
        colors = tem_color_box

        self.points_accumulated.append(points_3d)
        self.colors_accumulated.append(colors)
        

    def save_point_cloud(self, filename):
        try:
            all_points = np.concatenate(self.points_accumulated, axis=0)
            all_colors = np.concatenate(self.colors_accumulated, axis=0)

            self.point_cloud.points = o3d.utility.Vector3dVector(all_points)
            self.point_cloud.colors = o3d.utility.Vector3dVector(all_colors)
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.2)
            o3d.io.write_point_cloud(filename, self.point_cloud)
            print(f"Point cloud saved to {filename}")
        except Exception as e:
            print(f"Error saving point cloud: {e}")




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



#### PART 1
# Modify camera configuration
device_config = pykinect.default_configuration
device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED

# Set exposure value (0-15, higher values increase exposure)
device_config.exposure_time = 0  # Adjust as needed

# Start device
device = pykinect.start_device(device_index=1,config=device_config) #R0 L1

# Initialize the Open3d visualizer
visualizer = Open3dVisualizer()

# Time interval for each point cloud save
duration_per_save = 0  # seconds
num_saves = 1
start_time = cv2.getTickCount()
save_count = 0

while save_count < num_saves:
    # Get capture
    capture = device.update()

    # Get the 3D point cloud
    ret_point, points = capture.get_transformed_pointcloud()

    # Get the color image in the depth camera axis
    ret_color, color_image = capture.get_color_image()

    if not ret_color or not ret_point:
        continue

    # Update visualizer
    visualizer.update(points, color_image)

    # Check if it's time to save
    elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
    if elapsed_time >= duration_per_save:
        visualizer.save_point_cloud(filename=f'F:\cuhk\calibrate0304\material/real_time30L_TESTTTTTTTTTTT.ply')
        print(f'Point cloud saved {save_count + 1} times!')
        start_time = cv2.getTickCount()  # Reset the timer
        save_count += 1


    print("Combined first halves point cloud saved.")


######PART 2
# Initialize the library, if the library is not found, add the library path as argument
pykinect.initialize_libraries()

# Modify camera configuration
device_config = pykinect.default_configuration
device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED

# Set exposure value (0-15, higher values increase exposure)
device_config.exposure_time = 0  # Adjust as needed

# Start device
device = pykinect.start_device(device_index=0,config=device_config) #R0 L1

# Initialize the Open3d visualizer
visualizer2 = Open3dVisualizer2()

# Time interval for each point cloud save
duration_per_save = 0  # seconds
num_saves = 1
start_time = cv2.getTickCount()
save_count = 0

while save_count < num_saves:
    # Get capture
    capture = device.update()

    # Get the 3D point cloud
    ret_point, points = capture.get_transformed_pointcloud()

    # Get the color image in the depth camera axis
    ret_color, color_image = capture.get_color_image()

    if not ret_color or not ret_point:
        continue

    # Update visualizer
    visualizer2.update(points, color_image)

    # Check if it's time to save
    elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
    if elapsed_time >= duration_per_save:
        visualizer2.save_point_cloud(filename=f'F:\cuhk\calibrate0304\material/real_time30R_TESTTTTTTTTTTT.ply')
        print(f'Point cloud saved {save_count + 1} times!')
        start_time = cv2.getTickCount()  # Reset the timer
        save_count += 1


print("Combined first halves point cloud saved.")



## PART 3

# Define rotation matrix
# rotation_matrix = np.array([[0.3339,0.9028,0.2710],     
#                             [-0.1952,0.3475,-0.9171],
#                             [-0.9222,0.2534,0.2922]])
rotation_matrix = np.array([[0.9976,-0.0371,-0.0586],  # 0 to R
                            [0.0548,-0.0960,0.9939],
                            [0.0425,-0.9947,-0.0938]])

world_according_translation = np.array([-193,809,33])#LEFT 1(44,-152,-235)  RIGHT 2(109,485,-159)   original data ([65,637,76])
rotation_matrix_L = np.array([[-0.9991,-0.0086,0.0426],[-0.0405,0.5404,-0.8404],[-0.0158,-0.8413,-0.5403]])

l1_CORRDIN = np.dot(world_according_translation.T,rotation_matrix_L)  # translation of the L toR

coff_matrix = np.array([[ -0.9990,-0.0405,-0.0158],    # rotation matrix L to 0
                        [ -0.0086,0.5405,-0.8414],      
                        [ 0.0426,-0.8404,-0.5403]]) 

rotation_matrix = np.dot(coff_matrix,rotation_matrix)


# Define translation vector
#translation_vector = np.array([-117.9147,-294.7930,97.0288])   #A*B =(A.T * B.T).T
#translation_vector = np.array([300,-600,500])   #A*B =(A.T * B.T).T 135.391009623603,33.9384359371136,758.092394241776 ([135,-650,500]) 


# Load the point cloud
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/real_time30L_TESTTTTTTTTTTT.ply")
pcd = pcd.voxel_down_sample(voxel_size = 0.1)
# Convert the point cloud to a NumPy array
array = np.asarray(pcd.points)
color = np.asarray(pcd.colors)

indices = (array[:, 2] > 300) & (array[:, 2] < 400) &  (array[:, 0] < 200) &  (array[:, 0] > -150)


# Filter points and colors based on the condition
filtered_points = array[indices]
filtered_colors = color[indices]

# threshold and slice 
stk = np.hstack((filtered_points,filtered_colors))
#print(stk.shape)

maskb = (stk[:, 5] * 255 > 100) & (stk[:, 5] * 255 <= 255)
stk = stk[maskb]
maskr = (stk[:, 3] * 255 > 120) & (stk[:, 3] * 255 <= 255)   #250
stk = stk[maskr]
maskg = (stk[:, 4] * 255 > 120) & (stk[:, 4] * 255 <= 255)   #250
stk = stk[maskg]

array = stk[:,:3]
#print(stk)
#print('array:',array.shape)
rgb = stk[:,3:]
#print('rgb:',rgb.shape)

array_t = array.T
print(array_t.shape)
row, cloumn = array_t.shape
point_cloud_a = np.zeros((cloumn,3))


for i in tqdm(range(cloumn),desc = 'processing points'):
    time.sleep(0.001)
    At = array_t[:,i] + l1_CORRDIN
    rotate_clo = np.dot(At,rotation_matrix.T)
    #print(rotate_clo)
    tran_clo = rotate_clo
    #print('tran_clo:',tran_clo)
    point_cloud_a[i,:3] =  tran_clo

#print('pointcloud:',point_cloud_a.shape)

# save point cloud
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(point_cloud_a)
pcd2.colors = o3d.utility.Vector3dVector(rgb)

o3d.io.write_point_cloud('F:\cuhk\calibrate0304\material/REALTRANS30_TESTTTTT.ply',pcd2)
print('finish transformation')






#### PART 4 
# Load the point cloud
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/REALTRANS30_TESTTTTT.ply")  #left
array = np.asarray(pcd.points)
color = np.asarray(pcd.colors)



# Load the point cloud2
pcd2 = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/real_time30R_TESTTTTTTTTTTT.ply")  #right
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

o3d.io.write_point_cloud('F:\cuhk\calibrate0304\material/real_time_combine30_TESTTTTTT.ply',pcd_final)
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


o3d.io.write_triangle_mesh("F:\cuhk\calibrate0304\material\convex hull/real_time_TESTTTTTT.obj", hull)
print('done mesh')

