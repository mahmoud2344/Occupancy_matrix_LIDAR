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
"""

import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import random

lidar_readings = []     # List to store Lidar readings (angle, radius)


def get_grid_size():
    # Function to get grid size and visualize the polar grid

    # Get parameters from user input
    max_radius = 300
    num_cells = 6
    num_slices = 12
    b = 200
    den_value = float(entry_den.get())
    num_points = int(entry_points.get())

    # Calculate the distance between circles
    circle_distance = max_radius / num_cells

    # Create the polar grid
    polar_grid, angles = create_polar_grid(num_cells, num_slices)

    # Calculate cell areas and store information
    cell_areas = cell_area(num_cells, circle_distance, num_slices)

    # Scatter random points within the grid
    scatter_random_points(num_points, max_radius)
    random_radii, random_angles = scatter_random_points(num_points, max_radius)

    points_position(cell_areas, den_value, b)

    # Find cells with points and highlight them
    highlight_idx = points_position(cell_areas, den_value, b)

    # Create an occupancy matrix to represent filled cells
    create_occupancy_matrix(num_cells, num_slices, highlight_idx)
    occupancy_matrix = create_occupancy_matrix(num_cells, num_slices, highlight_idx)

    # Visualize the polar grid
    visualize_polar_grid(polar_grid, angles, circle_distance, max_radius, cell_areas, random_radii, random_angles, highlight_idx, occupancy_matrix)
    cell_area(num_cells, circle_distance, num_slices)


def create_occupancy_matrix(num_cells, num_slices, highlight_idx):
    # Create an occupancy matrix based on highlighted cell indices
    occupancy_matrix = [[0 for _ in range(num_slices)] for _ in range(num_cells)]

    for cell_index in highlight_idx:
        slice_index = (cell_index - 1) % num_slices
        cell_row = (cell_index - 1) // num_slices
        occupancy_matrix[cell_row][slice_index] = 1

    return occupancy_matrix


def create_polar_grid(num_cells, num_slices):
    # Create the polar grid
    polar_grid = np.zeros((num_cells, num_slices))

    # Create the angles
    angle_step = 360 / num_slices
    angles = np.deg2rad(np.arange(0, 360, angle_step))

    return polar_grid, angles


def cell_area(num_cells, circle_distance, num_slices):
    # Calculate and store cell areas
    area_list = []
    for slice_idx in range(num_slices):
        for cell_idx in range(1, num_cells + 1):
            # Calculate the area for the current cell
            area = ((cell_idx * circle_distance)**2*np.pi - ((cell_idx - 1) * circle_distance)**2*np.pi) / num_slices
            area_list.append(area)

    # Create a list of (area, cell_index) tuples
    cell_areas = []
    for cell_index, area in enumerate(area_list, start=1):
        slice_index = (cell_index - 1) // num_cells + 1
        cell_radius = (cell_index - 1) * circle_distance % (num_cells * circle_distance) + circle_distance
        location = (cell_radius, slice_index, cell_index)

        start_angle = 360 / num_slices * (slice_index - 1)
        end_angle = 360 / num_slices * slice_index
        start_radius = location[0] - circle_distance
        end_radius = location[0]
        cell_areas.append((area, cell_index, location, start_angle, end_angle, start_radius, end_radius))

    return cell_areas


def scatter_random_points(num_points, max_radius):
    # Scatter random points within the grid
    random_angles = [random.uniform(0, 2 * np.pi) for _ in range(num_points)]
    random_radii = [random.uniform(0, max_radius) for _ in range(num_points)]
    lidar_readings.clear()  # Clear the list to start fresh
    lidar_readings.extend(list(zip(random_angles, random_radii)))
    return random_radii, random_angles


def points_position(cell_areas, den_value, b):
    # Find cells with points based on density value
    highlight_idx = []  # Use a set to store unique cell indices with points

    for x, y in lidar_readings:
        # Convert theta to degrees and ensure it's positive
        theta_deg = np.degrees(x)

        for area, cell_index, location, start_angle, end_angle, start_radius, end_radius in cell_areas:
            if start_angle <= theta_deg <= end_angle and start_radius <= y <= end_radius:
                if cell_index not in highlight_idx:  # Check if the cell hasn't been highlighted already
                    # Calculate the density of points within the cell
                    points_count = sum(1 for x, y in lidar_readings if
                                       start_angle <= np.degrees(x) <= end_angle and start_radius <= y <= end_radius)
                    density = points_count * b / area

                    if density >= den_value:
                        highlight_idx.append(cell_index)
    return highlight_idx


def visualize_polar_grid(polar_grid, angles, circle_distance, max_radius, cell_areas, random_radii, random_angles, highlight_idx, occupancy_matrix):
    plt.figure(facecolor='white')
    theta = np.linspace(0, 2 * np.pi, 2000)
    ax = plt.subplot(111, polar=True, facecolor='white')

    # Draw smaller circles(the parts per slice)
    for i in range(1, 10000000, 1):
        r = i * circle_distance
        ax.plot(theta, np.ones_like(theta) * r, color='gray')
        if r >= max_radius:
            break

    # Draw lines from the center to the edge(the slices)
    for angle in angles:
        ax.plot([angle, angle + np.pi], [0, polar_grid.shape[0] * circle_distance], color='gray')

    # Scatter random points
    ax.scatter(random_angles, random_radii, color='blue', marker='.', s=4)

    # Highlight cells
    for index in highlight_idx:
        area, cell_index, location, start_angle, end_angle, start_radius, end_radius = cell_areas[index - 1]
        phy = np.linspace(np.radians(start_angle), np.radians(end_angle), 100)
        inner_radius = start_radius
        outer_radius = end_radius
        ax.fill_between(phy, inner_radius, outer_radius, color='red', alpha=0.3)

    for area, cell_index, location, start_angle, end_angle, start_radius, end_radius in cell_areas:
        print(
            f"Cell{cell_index}: Area ={area}, Location ={location}, Start Angle ={start_angle}, End Angle ={end_angle}, Start_r ={start_radius}, End_r ={end_radius}")

    print(f"belongs to Cell {highlight_idx}")
    print("Occupancy Matrix:")
    for row in occupancy_matrix:
        print(row)

    ax.spines['polar'].set_visible(False)
    ax.grid(False)
    ax.set_yticklabels([])
    plt.show()


# Create the grid size window
window = tk.Tk()
window.title("Grid Size")
window.geometry("200x100")

# Create labels and entry fields for input
label_density = tk.Label(window, text="density value:")
label_density.pack()
entry_den = tk.Entry(window)
entry_den.pack()

label_points = tk.Label(window, text="Number of Points:")
label_points.pack()
entry_points = tk.Entry(window)
entry_points.pack()

# Create a button to submit the grid size
btn_submit = tk.Button(window, text="Submit", command=get_grid_size)
btn_submit.pack()

# Start the window event loop
window.mainloop()
