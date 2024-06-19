import cv2
import numpy as np
import open3d as o3d
from scipy.interpolate import interp1d

# Read point cloud data
pcd = o3d.io.read_point_cloud("F:\cuhk\calibrate0304\material/REALTRANS30_TESTTTTT.ply")
array = np.asarray(pcd.points)

# Find minimum and maximum x, z coordinates
xmin = np.min(array[:, 0])
xmax = np.max(array[:, 0])
zmin = np.min(array[:, 2])
zmax = np.max(array[:, 2])

# Define the four corner points
corner_points = [(xmin, zmin), (xmin, zmax), (xmax, zmin), (xmax, zmax)]

# Define the midpoints
midpoints = [((xmin + xmax) / 2, zmin), (xmin, (zmin + zmax) / 2),
             ((xmin + xmax) / 2, zmax), (xmax, (zmin + zmax) / 2)]

# Initialize a list to store the coefficients of each polynomial
polynomial_coefficients = []

# Fit a polynomial line through the corner points and their midpoints
for i in range(len(corner_points)):
    x = [corner_points[i][0], midpoints[i][0]]
    z = [corner_points[i][1], midpoints[i][1]]
    poly_fit = np.polyfit(x, z, deg=2)  # Quadratic fit
    polynomial_coefficients.append(poly_fit)

    # Generate x values for interpolation
    x_interpolation = np.linspace(min(x), max(x), num=100)

    # Perform polynomial interpolation
    poly_line = np.poly1d(poly_fit)
    z_interpolation = poly_line(x_interpolation)

    # Print the polynomial equation for this segment
    equation = f"Polynomial Equation for segment {i + 1}:\n{poly_fit[0]}x^2 + {poly_fit[1]}x + {poly_fit[2]}"
    print(equation)

    # Visualize the results as before...

# Find closest points to the midpoints
closest_points = []
for midpoint in midpoints:
    distances = np.linalg.norm(array[:, [0, 2]] - np.array(midpoint), axis=1)  # Consider only first and third dimensions
    closest_index = np.argmin(distances)
    closest_point = array[closest_index]
    closest_points.append(closest_point)

# Sort closest points based on x coordinate
closest_points.sort(key=lambda p: p[0])

# Visualize the results
canvas_size = 500
canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)

# Draw the interpolated curve on the canvas
for i in range(0, len(closest_points), 2):
    x = [closest_points[i][0], closest_points[i + 1][0]]
    z = [closest_points[i][1], closest_points[i + 1][1]]
    poly_fit = np.polyfit(x, z, deg=2)
    poly_line = np.poly1d(poly_fit)
    x_interpolation = np.linspace(min(x), max(x), num=100)
    z_interpolation = poly_line(x_interpolation)
    scaled_x_interpolation = ((x_interpolation - xmin) / (xmax - xmin)) * canvas_size
    scaled_z_interpolation = ((z_interpolation - zmin) / (zmax - zmin)) * canvas_size
    for j in range(len(scaled_x_interpolation) - 1):
        cv2.line(canvas, (int(scaled_x_interpolation[j]), int(scaled_z_interpolation[j])),
                 (int(scaled_x_interpolation[j + 1]), int(scaled_z_interpolation[j + 1])), (255, 255, 255), 1)

# Draw points on the canvas
for point in array:
    x_scaled = ((point[0] - xmin) / (xmax - xmin)) * canvas_size
    z_scaled = ((point[2] - zmin) / (zmax - zmin)) * canvas_size
    cv2.circle(canvas, (int(x_scaled), int(z_scaled)), 2, (255, 255, 255), -1)  # Draw white points with size 2

# Resize canvas to make the window smaller
canvas_resized = cv2.resize(canvas, (250, 250), interpolation=cv2.INTER_AREA)

# Display the points and the polynomial curve
cv2.imshow('Points and Polynomial Curve', canvas_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
