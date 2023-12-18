"""
Author: AHMED Mahmoud
Date: 15/12/2023
Institution: UHA ENSISA M2 EEA

Professor: Rodolfo Orjuela

Project: Detection of navigable areas with LIDAR for an autonomous vehicle

Description: This code Uses the same efficient algorithm from our simulation,
    allowing to obtain the occupancy matrix in real time without visualization in which is better for real time application.
    The program Uses RPLIDAR data, create the occupancy matrix using specified density threshold.
"""

from rplidar import RPLidar
import numpy as np

PORT_NAME = 'COM4'

# User inputs
occupancy_threshold = 0.2
coef = 100
num_cells = 8
num_slices = 12
real_max_radius = 8000
screen_max_radius = 400

angle_step = 360 / num_slices
angles = np.deg2rad(np.arange(0, 360, angle_step))

circle_distance = screen_max_radius / num_cells

real_circle_distance = real_max_radius / num_cells

matrix_size = (num_cells, num_slices)

list1 = []
for cell_idx in range(1, num_cells + 1):
    # Calculate the area for the current cell
    area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
    list1.append(area)

print(list1)


def run():
    lidar = RPLidar(PORT_NAME)
    try:
        running = True
        for scan in lidar.iter_scans():
            matrix = np.zeros(matrix_size)

            for (_, angle, distance) in scan:
                dis = int(distance // real_circle_distance)
                ang = int((360-angle) // angle_step) % num_slices
                if 0 <= dis < num_cells:
                    matrix[dis, ang] += 1
            print(f"occupancy:{matrix.astype(int)}")

            matrix = np.array(matrix)

            occupancy_matrix = np.zeros_like(matrix, dtype=float)
            occupancy_matrix = matrix * coef / np.array(list1)[:num_cells, None] > occupancy_threshold

            print(occupancy_matrix.astype(int))

            if not running:
                break

    except KeyboardInterrupt:
        pass

    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    run()
