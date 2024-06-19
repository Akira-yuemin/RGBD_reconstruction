import numpy as np
import cv2
import open3d as o3d
import pykinect_azure as pykinect
from tqdm import tqdm

class Open3dVisualizer():
    def __init__(self, window_width=1600, window_height=1200):
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.05)
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
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size = 0.05)
            o3d.io.write_point_cloud(filename, self.point_cloud)
            print(f"Point cloud saved to {filename}")
        except Exception as e:
            print(f"Error saving point cloud: {e}")



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
        visualizer.save_point_cloud(filename=f'F:\cuhk\calibrate0304\material/red_brick.ply')
        print(f'Point cloud saved {save_count + 1} times!')
        start_time = cv2.getTickCount()  # Reset the timer
        save_count += 1


print("Combined first halves point cloud saved.")

