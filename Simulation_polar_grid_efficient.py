"""
Author: AHMED Mahmoud
Date: 15/12/2023
Institution: UHA ENSISA M2 EEA

Professor: Rodolfo Orjuela

Project: Detection of navigable areas with LIDAR for an autonomous vehicle

Description: This code simulates Lidar data on a polar grid, allowing for the visualization of an occupancy matrix.
    The program generates random Lidar readings, places them on a polar grid,
    and highlights cells based on a specified density threshold.
    The resulting visualization provides insights into the occupancy of different grid cells.
    Using a different algorithm more efficient for real time application explained in the readme file.
"""

import numpy as np
import matplotlib.pyplot as plt

# Given lidar readings in the format (angle, distance)
lidar_readings = [(0.5236, 4), (0.5236, 3), (0.5236, 2.5), (1.6581, 6), (1.6581, 7), (3.3161, 2)]

# Extract angles and distances from lidar readings
x_values = [x for x, _ in lidar_readings]
y_values = [y for _, y in lidar_readings]

# Define parameters for polar grid
max_radius = 10
num_cells = 3
num_slices = 4
coef = 10
occupancy_threshold = 0.5

# Calculate the distance between circles in the polar grid
circle_distance = max_radius / num_cells

# Initialize an empty polar grid
polar_grid = np.zeros((num_cells, num_slices))

# Create angles for the slices in the polar grid
angle_step = 360 / num_slices
angles = np.deg2rad(np.arange(0, 360, angle_step))

# Initialize a matrix to store counts in each cell of the polar grid
matrix_size = (num_cells, num_slices)
matrix = np.zeros(matrix_size)

# Calculate the area for each cell in the polar grid
area_list = []
for cell_idx in range(1, num_cells + 1):
    # Calculate the area for the current cell
    area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
    area_list.append(area)
print(area_list)

# Populate the matrix with lidar readings
for x, y in lidar_readings:
    # Convert theta to degrees and ensure it's positive
    theta_deg = np.degrees(x)
    dis = int(y // circle_distance)
    ang = int(theta_deg // angle_step)
    matrix[dis, ang] += 1

print(matrix.astype(int))

# Convert the matrix to a NumPy array
matrix = np.array(matrix)

# Initialize the occupancy matrix
occupancy_matrix = np.zeros_like(matrix, dtype=float)

# Divide each row by the corresponding area and compare to the threshold
occupancy_matrix = matrix * coef / np.array(area_list)[:, None] > occupancy_threshold

print(occupancy_matrix.astype(int))

plt.figure(facecolor='white')
theta = np.linspace(0, 2 * np.pi, 2000)
ax = plt.subplot(111, polar=True, facecolor='white')

# Draw smaller circles(the parts per slice)
for i in range(1, num_cells+1):
    r = i * circle_distance
    ax.plot(theta, np.ones_like(theta) * r, color='gray')

# Draw lines from the center to the edge(the slices)
for angle in angles:
    ax.plot([angle, angle + np.pi], [0, polar_grid.shape[0] * circle_distance], color='gray')

# Scatter plot of lidar readings on the polar grid
ax.scatter(x_values, y_values, color='blue', marker='.', s=4)

ax.spines['polar'].set_visible(False)
ax.grid(False)
ax.set_yticklabels([])
plt.show()
