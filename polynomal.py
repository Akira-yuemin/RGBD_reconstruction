import cv2
import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/night2/17brick_combine.ply")  #left
array = np.asarray(pcd.points)
color = np.asarray(pcd.colors)

pc1_xz = np.hstack((array[:, 0], array[:, 2]))  # Extracting columns 0 (x) and 2 (z)

# Reshape pc1_xz to be a 2D array with two columns (x, z)
pc1_xz = pc1_xz.reshape(-1, 2)

# Calculate the center of the points
center_x = int(np.mean(pc1_xz[:, 0]))
center_z = int(np.mean(pc1_xz[:, 1]))

# Calculate canvas size
canvas_size = int(max(np.max(pc1_xz[:, 0]), np.max(pc1_xz[:, 1])) * 2)

# Create a canvas larger than the maximum extent of the points
canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)

# Draw the points on the canvas with offset
for point in pc1_xz:
    x, z = point
    cv2.circle(canvas, (int(x) + canvas_size // 2 - center_x, int(z) + canvas_size // 2 - center_z), 1, (255, 255, 255), -1)  # Draw white points

# Find contours in the canvas
contours, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Create a blank image to draw contours
contour_image = np.zeros_like(canvas)

# Draw contours on the blank image
cv2.drawContours(contour_image, contours, -1, (255, 255, 255), thickness=1)

# Display the points and the contour image
cv2.imshow('Points', canvas)
cv2.imshow('Contours', contour_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
