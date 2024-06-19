import numpy as np
import open3d as o3d
from tqdm import tqdm
import time

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
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/real_time30L.ply")
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

maskb = (stk[:, 5] * 255 > 80) & (stk[:, 5] * 255 <= 255)
stk = stk[maskb]
maskr = (stk[:, 3] * 255 > 105) & (stk[:, 3] * 255 <= 255)   #250
stk = stk[maskr]
maskg = (stk[:, 4] * 255 > 105) & (stk[:, 4] * 255 <= 255)   #250
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

o3d.io.write_point_cloud('F:\cuhk\calibrate0304\material/REALTRANS30.ply',pcd2)
print('finish transformation')

