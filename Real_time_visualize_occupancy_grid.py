"""
Author: AHMED Mahmoud
Date: 15/12/2023
Institution: UHA ENSISA M2 EEA

Professor: Rodolfo Orjuela

Project: Detection of navigable areas with LIDAR for an autonomous vehicle

Description: This code Uses the same efficient algorithm from our simulation,
    allowing for the visualization of an occupancy matrix in real time.
    The program Uses RPLIDAR data, and highlights cells based on a specified density threshold.
    The resulting visualization provides insights into the occupancy of different grid cells.
"""

import pygame
import math
from rplidar import RPLidar
import numpy as np

pygame.init()

PORT_NAME = 'COM4'

# User inputs
occupancy_threshold = 0.2
coef = 100
num_cells = 8
num_slices = 12
real_max_radius = 8000
screen_max_radius = 400

# User preferences
WIDTH, HEIGHT = 1920, 1080
FULLSCREEN = pygame.FULLSCREEN
BACKGROUND_COLOR = (10, 10, 10)
POINT_COLOR = (0, 255, 0)
POINT_RADIUS = 3
GRID_COLOR = (200, 200, 200)
red = (255, 0, 0, 0)


screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('LIDAR Visualization')

clock = pygame.time.Clock()

angle_step = 360 / num_slices
angles = np.deg2rad(np.arange(0, 360, angle_step))

circle_distance = screen_max_radius / num_cells

real_circle_distance = real_max_radius / num_cells

matrix_size = (num_cells, num_slices)


def polar_to_cartesian(r, theta):
    x = int(WIDTH / 2 + r * math.cos(math.radians(theta)))
    y = int(HEIGHT / 2 - r * math.sin(math.radians(theta)))
    return int(x), int(y)


area_list = []
for slice_idx in range(num_slices):
    for cell_idx in range(1, num_cells + 1):
        # Calculate the area for the current cell
        area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
        area_list.append(area)

sorted_areas = sorted(area_list)

# Create a list of (area, cell_index) tuples
cell_areas = []
cell_radius_idx = []
for cell, area in enumerate(sorted_areas, start=1):
    slice_index = (cell - 1) // num_cells + 1
    cell_radius = (cell - 1) * circle_distance % (num_cells * circle_distance) + circle_distance
    cell_radius_idx.append((cell_radius, slice_index))

# Sort the cells based on radius
sorted_cells = sorted(cell_radius_idx, key=lambda x: x[0])

# Create cell_areas based on the sorted order
for cell_index, (cell_radius, slice_index) in enumerate(sorted_cells, start=1):
    start_angle = 360 / num_slices * (slice_index - 1)
    end_angle = 360 / num_slices * slice_index
    start_radius = cell_radius - circle_distance
    end_radius = cell_radius
    cell_areas.append((sorted_areas[cell_index - 1], cell_index, cell_radius, start_angle, end_angle, start_radius, end_radius))

list1 = []
for cell_idx in range(1, num_cells + 1):
    # Calculate the area for the current cell
    area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
    list1.append(area)

print(list1)


def draw_polar_grid():
    for i in range(1, num_cells + 1):
        radius = i * circle_distance
        pygame.draw.circle(screen, GRID_COLOR, (WIDTH / 2, HEIGHT / 2), int(radius), 1)

    for angle in angles:
        x1 = WIDTH / 2 + screen_max_radius * math.cos(angle)
        y1 = HEIGHT / 2 + screen_max_radius * math.sin(angle)
        pygame.draw.line(screen, GRID_COLOR, (WIDTH / 2, HEIGHT / 2), (x1, y1), 1)

    dot_x = int(WIDTH / 2 + screen_max_radius * math.cos(math.radians(0)))
    dot_y = int(HEIGHT / 2 - screen_max_radius * math.sin(math.radians(0)))
    pygame.draw.circle(screen, (255, 0, 0, 0), (dot_x, dot_y), 5)
    dot_x = int(WIDTH / 2 + screen_max_radius * math.cos(math.radians(180)))
    dot_y = int(HEIGHT / 2 - screen_max_radius * math.sin(math.radians(180)))
    pygame.draw.circle(screen, POINT_COLOR, (dot_x, dot_y), 5)


def run():
    lidar = RPLidar(PORT_NAME)
    try:
        running = True
        for scan in lidar.iter_scans():
            last_points = []  # Store points for rendering
            matrix = np.zeros(matrix_size)

            screen.fill(BACKGROUND_COLOR)
            draw_polar_grid()

            for (_, angle, distance) in scan:
                dis = int(distance // real_circle_distance)
                ang = int((360-angle) // angle_step) % num_slices
                if 0 <= dis < num_cells:
                    matrix[dis, ang] += 1
                x = int(WIDTH / 2 + (distance / real_max_radius) * screen_max_radius * math.cos(math.radians(360 - angle)))
                y = int(HEIGHT / 2 - (distance / real_max_radius) * screen_max_radius * math.sin(math.radians(360 - angle)))
                last_points.append((x, y))
            print(f"occupancy:")
            print(f"{matrix.astype(int)}")

            matrix = np.array(matrix)

            occupancy_matrix = np.zeros_like(matrix, dtype=float)
            occupancy_matrix = matrix * coef / np.array(list1)[:num_cells, None] > occupancy_threshold

            print(occupancy_matrix.astype(int))

            for x, y in last_points:
                screen.set_at((x, y), POINT_COLOR)

            for dis in range(num_cells):
                for ang in range(num_slices):
                    if occupancy_matrix[dis, ang]:
                        cell_index = dis * num_slices + ang + 1
                        area, cell_index, center, start_angle, end_angle, start_radius, end_radius = cell_areas[cell_index - 1]

                        phy = np.linspace(start_angle, end_angle, 100)
                        inner_radius = start_radius
                        outer_radius = end_radius

                        polygon_points = [polar_to_cartesian(inner_radius, angle) for angle in phy]
                        polygon_points += [polar_to_cartesian(outer_radius, angle) for angle in reversed(phy)]

                        pygame.draw.polygon(screen, red, polygon_points)

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if not running:
                break

    except KeyboardInterrupt:
        pass

    lidar.stop()
    lidar.disconnect()
    pygame.quit()


if __name__ == '__main__':
    run()
