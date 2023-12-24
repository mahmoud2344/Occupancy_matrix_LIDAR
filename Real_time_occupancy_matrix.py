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
# Import necessary libraries
from rplidar import RPLidar
import numpy as np
import keyboard

# Set the port name for the RPLIDAR device
PORT_NAME = 'COM4'

# User inputs
occupancy_threshold = 0.2  # Threshold for occupancy matrix
coef = 1000  # Coefficient for occupancy matrix calculation
num_cells = 8  # Number of cells in the matrix
num_slices = 12  # Number of slices in the matrix
real_max_radius = 8000  # Maximum radius in real-world units
screen_max_radius = 350  # Maximum radius for screen visualization
motor_speed = 500   # Lidar rotation speed

# Calculate angle step and angles for each slice
angle_step = 360 / num_slices
angles = np.deg2rad(np.arange(0, 360, angle_step))

# Calculate distances for screen visualization and real-world distances
circle_distance = screen_max_radius / num_cells
real_circle_distance = real_max_radius / num_cells

# Define the size of the occupancy matrix
matrix_size = (num_cells, num_slices)

# Calculate the area for each cell and store it in a list
list1 = []
for cell_idx in range(1, num_cells + 1):
    area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
    list1.append(area)

# Display the list of areas
print(list1)


# Define the main function for running the LIDAR scan
def run():
    # Initialize the RPLIDAR device
    lidar = RPLidar(PORT_NAME)
    try:
        lidar.motor_speed = motor_speed
        running = True
        # Continuously read LIDAR scans
        for scan in lidar.iter_scans(scan_type='express'):
            matrix = np.zeros(matrix_size)

            # Process each data point in the scan
            for (_, angle, distance) in scan:
                dis = int(distance // real_circle_distance)
                ang = int((360 - angle) // angle_step) % num_slices
                if 0 <= dis <= num_cells:
                    matrix[dis, ang] += 1

            # Display the number of points matrix
            print(f"number of points:")
            print(f"{matrix.astype(int)}")

            # Convert the occupancy matrix to a binary matrix based on the threshold
            matrix = np.array(matrix)
            occupancy_matrix = np.zeros_like(matrix, dtype=float)
            occupancy_matrix = matrix * coef / np.array(list1)[:num_cells, None] > occupancy_threshold

            # Display the binary occupancy matrix
            print(f"occupancy_matrix:")
            print(occupancy_matrix.astype(int))

            # Check for 'esc' key press to stop the program
            if keyboard.is_pressed('esc'):
                print("Stopping the program...")
                running = False
                break

            # Check for KeyboardInterrupt to stop the program
            if not running:
                break

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the lidar motor and disconnect the RPLIDAR device
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()


# Run the main function if the script is executed directly
if __name__ == '__main__':
    run()
