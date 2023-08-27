import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from scipy import ndimage

# Map dimensions in meters
map_width = 1.68
map_height = 1.18

# Matrix dimensions corresponding to 1mm per pixel
matrix_width = 1680
matrix_height = 1180

c_err = 60

# Scaling factors to transform real-world coordinates to matrix indices
scale_x = matrix_width / map_width  # Uniform scale for x and y
scale_y = matrix_height / map_height  # Uniform scale for x and y

# Define constants from the XML
cell_x = 0.15
cell_y = 0.08
wall_thickness = 0.02

def d_err(d):
    d = np.float64(d)
    c2 = c_err * c_err
    return 1 - c2 / (c2 + d * d)

def compute_dist_map(matrix):
    """
    Compute distance transform map of the matrix.
    Zero pixels in the matrix are considered as obstacles.
    """
    return ndimage.distance_transform_edt(matrix)

def calc_grad_maps_m_estimator(matrix):
    image = np.float64(matrix)
    grad_y, grad_x = np.gradient(d_err(matrix))
    return grad_x, grad_y

def calc_grad_maps(matrix):
    image = np.float64(matrix)
    grad_y, grad_x = np.gradient(matrix)
    return grad_x, grad_y

def real_to_matrix(x, y):
    # Translate the real-world coordinates so the origin aligns with the center of the matrix
    x_translated = x + map_width / 2
    y_translated = y + map_height / 2

    # Scale coordinates
    j = int(np.round(x_translated * scale_y))
    i = int(np.round(y_translated * scale_x))

    # Flip the y-coordinate due to matrix representation
    i = matrix_height - i

    return i, j

def draw_obstacle(matrix, obstacle):
    size_x = obstacle['size'][0] * scale_x
    size_y = obstacle['size'][1] * scale_y
    pos_i, pos_j = real_to_matrix(obstacle['pos'][0], obstacle['pos'][1])
    rotation = obstacle.get('rotation', 0)

    # If rotated, swap the sizes
    if rotation == 90 or rotation == -90:
        size_x, size_y = size_y, size_x

    # Calculate bounds
    start_i = pos_i - int(np.round(size_y / 2))
    end_i = start_i + int(np.round(size_y))
    start_j = pos_j - int(np.round(size_x / 2))
    end_j = start_j + int(np.round(size_x))

    # Draw the obstacle
    matrix[start_i:end_i, start_j:end_j] = 0

    return matrix

# Define obstacles
incoming_warehouse = {
    'pos': (-0.47, 0.580),
    'size': (4 * cell_x + wall_thickness, wall_thickness),
    'rotation': 0
}
outgoing_warehouse = {
    'pos': (0.47, -0.58),
    'size': (4 * cell_x + wall_thickness, wall_thickness),
    'rotation': 180
}
machine_A = {
    'pos': (-0.347, -0.08),
    'size': (2 * cell_x + wall_thickness, wall_thickness),
    'rotation': -90
}
machine_B = {
    'pos': (0.347, 0.07),
    'size': (2 * cell_x + wall_thickness, wall_thickness),
    'rotation': -90
}

# Initialize matrix with free space
matrix = np.ones((matrix_height, matrix_width))

# Draw the obstacles
for obs in [incoming_warehouse, outgoing_warehouse, machine_A, machine_B]:
    draw_obstacle(matrix, obs)

# for obs in [outgoing_warehouse]:
#     draw_obstacle_outgoing(matrix, obs)

# Visualization
plt.imshow(matrix, cmap='gray', interpolation='none')
plt.colorbar()
plt.title('2D Matrix Map Representation')
plt.show()

# Convert matrix to a format suitable for visualization [0-255]
matrix_png = (matrix * 255).astype(np.uint8)

# Create an image from the array
image = Image.fromarray(matrix_png)

# Resize the image
resized_image = image.resize((800, 700), Image.LANCZOS)

# Save the resized image as PNG
resized_image.save('matrix.png')



# Add this part to the end of your code:

# Compute distance map
dist_map = compute_dist_map(matrix)

# Compute the gradients on the distance map
grad_x, grad_y = calc_grad_maps(dist_map)

# Apply the M-estimator
m_grad_x, m_grad_y = calc_grad_maps_m_estimator(dist_map)

# Visualization of Distance Map
plt.figure(figsize=(8, 6))
plt.imshow(dist_map, cmap='hot', interpolation='none')
plt.colorbar()
plt.title('Distance Map')
plt.show()

print((np.max(dist_map), np.min(dist_map)))

# Visualization of Gradient-X
plt.figure(figsize=(8, 6))
plt.imshow(grad_x, cmap='bwr', interpolation='none')
plt.colorbar()
plt.title('Gradient-X')
plt.show()

# Visualization of M-estimator Gradient-X
plt.figure(figsize=(8, 6))
plt.imshow(m_grad_x, cmap='bwr', interpolation='none')
plt.colorbar()
plt.title('M-estimator Gradient-X')
plt.show()

# Visualization of Gradient-Y
plt.figure(figsize=(8, 6))
plt.imshow(grad_y, cmap='bwr', interpolation='none')
plt.colorbar()
plt.title('Gradient-Y')
plt.show()

# Visualization of M-estimator Gradient-Y
plt.figure(figsize=(8, 6))
plt.imshow(m_grad_y, cmap='bwr', interpolation='none')
plt.colorbar()
plt.title('M-estimator Gradient-Y')
plt.show()

np.savetxt('distance_map.csv', dist_map, delimiter=',')
np.savetxt('m_estimator_gradient_x.csv', m_grad_x, delimiter=',')
np.savetxt('m_estimator_gradient_y.csv', m_grad_y, delimiter=',')
