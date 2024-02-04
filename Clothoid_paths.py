"""
Author: AHMED Mahmoud
Date: 31/01/2024
Institution: UHA ENSISA M2 EEA

Professor: Rodolfo Orjuela

Project: Detection of navigable areas with LIDAR for an autonomous vehicle

Description: This code Uses the same efficient algorithm from our simulation,
    allowing for the visualization of an occupancy matrix in real time.
    The program Uses RPLIDAR data, and highlights cells based on a specified density threshold.
    The resulting visualization provides insights into the occupancy of different grid cells.
    And possible paths for our Vehicle using tentacles (clothoides) .
"""
# Import necessary libraries
import pygame
import math
from rplidar import RPLidar
import numpy as np
import keyboard
from pyclothoids import Clothoid
from math import pi

# Initialize Pygame
pygame.init()

# Set the port name for the RPLIDAR device
PORT_NAME = 'COM4'

# User inputs
occupancy_threshold = 0.2   # Threshold for occupancy matrix
coefficient = 500  # Coefficient for occupancy matrix calculation
num_cells = 16   # Number of cells in the matrix
num_slices = 36  # Number of slices in the matrix
real_max_radius = 8000  # Maximum radius in real-world units
screen_max_radius = 400  # Maximum radius for screen visualization
motor_speed = 700   # Lidar rotation speed
min_distance = 4000  # Minimum distance to find a free path
num_paths = 8  # Parameters for the clothoid paths(will be multiplied by 2)

# User preferences
WIDTH, HEIGHT = 1800, 960
FULLSCREEN = pygame.FULLSCREEN
BACKGROUND_COLOR = (10, 10, 10)
POINT_COLOR = (0, 255, 0)
GRID_COLOR = (200, 200, 200)
red = (255, 0, 0, 0)
mam = 0

# Set up the Pygame window
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('LIDAR Visualization')
clock = pygame.time.Clock()

# Calculate angle step and angles for each slice
angle_step = 360 / num_slices
angles = np.deg2rad(np.arange(0, 360, angle_step))

# Calculate distances for screen visualization and real-world distances
circle_distance = screen_max_radius / num_cells
real_circle_distance = real_max_radius / num_cells

# Define the size of the occupancy matrix
matrix_size = (num_cells, num_slices)

# Calculate number of cells that should be free based on the user input
dis_threshold = int(min_distance / real_circle_distance)

print(dis_threshold)


# Calculate areas for each cell and store in a list
area_list = []
for slice_idx in range(num_slices):
    for cell_idx in range(1, num_cells + 1):
        # Calculate the area for the current cell
        area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
        area_list.append(area)

# Sort areas for highlighting cells
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

# Create a list for areas of each cell
list1 = []
for cell_idx in range(1, num_cells + 1):
    # Calculate the area for the current cell
    area = ((cell_idx * circle_distance) ** 2 * np.pi - ((cell_idx - 1) * circle_distance) ** 2 * np.pi) / num_slices
    list1.append(area)

print(list1)


# Define a function to convert polar coordinates to Cartesian coordinates
def polar_to_cartesian(r, theta):
    """
    Convert polar coordinates to Cartesian coordinates.

    :param r: The radial distance from the origin point.
    :param theta: The angle measured in degrees.
    :return: The Cartesian coordinates (x, y).

    """
    theta = math.radians(theta)
    x = int(WIDTH / 2 + r * math.cos(theta))
    y = int(HEIGHT / 2 - r * math.sin(theta))
    return int(x), int(y)


# Function to draw the polar grid
def draw_polar_grid():
    """
    Draws a polar grid on the screen with circles representing distances and lines representing angles.

    :return: None
    """
    font = pygame.font.Font(None, 36)
    for i in range(1, num_cells + 1):
        radius = i * circle_distance
        pygame.draw.circle(screen, GRID_COLOR, (WIDTH / 2, HEIGHT / 2), int(radius), 1)

        # Add real distance labels
        label = font.render(f"{i * real_circle_distance/1000:.1f}m", True, GRID_COLOR)
        label_rect = label.get_rect(center=(WIDTH / 2, HEIGHT / 2 - radius))
        screen.blit(label, label_rect)

    for angle in angles:
        x1 = WIDTH / 2 + screen_max_radius * math.cos(angle)
        y1 = HEIGHT / 2 + screen_max_radius * math.sin(angle)
        pygame.draw.line(screen, GRID_COLOR, (WIDTH / 2, HEIGHT / 2), (x1, y1), 1)

        # Add angle labels
        angle_degrees = math.degrees(angle)
        label = font.render(f"{angle_degrees:.0f}°", True, GRID_COLOR)
        label_rect = label.get_rect(center=(WIDTH / 2 + screen_max_radius * 1.2 * math.cos(angle),
                                            HEIGHT / 2 - screen_max_radius * 1.2 * math.sin(angle)))
        screen.blit(label, label_rect)

    # Draw a red point to indicate the face of the LIDAR(opposite of the cord)
    dot_x = int(WIDTH / 2 + screen_max_radius * math.cos(math.radians(0)))
    dot_y = int(HEIGHT / 2 - screen_max_radius * math.sin(math.radians(0)))
    pygame.draw.circle(screen, red, (dot_x, dot_y), 5)


def Highlight_cells(occupancy_matrix, color):
    """
    :param occupancy_matrix: The matrix representing the occupancy of each cell.
    :param color: The color to highlight the cells with.
    :return: None

    This method takes an occupancy matrix and a color as parameters and highlights the cells in a given screen using the pygame library.
    The occupancy matrix represents the occupancy of each cell, and the color parameter determines the color of the highlighting.
    The method iterates over each cell in the occupancy matrix and checks if the cell is occupied. If so, it calculates the necessary parameters
    for drawing a polygon to represent the cell on the screen. The polygon is then drawn using the pygame.draw.polygon() method.
    """
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

                pygame.draw.polygon(screen, color, polygon_points)


draw_polar_grid()

# Initialize a list of clothoid matrices
clothoid_matrices = [np.zeros((num_cells, num_slices), dtype=int) for _ in range(2 * num_paths)]

for i in range(num_paths):
    # Calculate curvature for the current path
    curvaturex = -5 + i * -20
    curvaturey = 50 + i * 10

    # Create a Clothoid object with varying curvature
    clothoid = Clothoid.G1Hermite(0, 0, pi, curvaturex, curvaturey, 0)

    # Sample clothoid points
    num_points = 100
    x, y = clothoid.SampleXY(num_points)

    # Shift the clothoid by 90 degrees
    shifted_points = [(point[1], -point[0]) for point in zip(x, y)]

    # Convert shifted clothoid points to screen coordinates
    screen_points_clothoid = [(int(WIDTH / 2 + point[0]), int(HEIGHT / 2 - point[1])) for point in shifted_points]

    # Draw the original clothoid curve on the screen
    pygame.draw.lines(screen, (255, 0, 0), False, screen_points_clothoid, 2)

    # Identify cells that the original clothoid path goes through
    for point in screen_points_clothoid:
        x, y = point
        # Convert Cartesian coordinates to polar coordinates
        radius = ((x - WIDTH / 2) ** 2 + (y - HEIGHT / 2) ** 2) ** 0.5
        angle = -math.degrees(math.atan2(y - HEIGHT / 2, x - WIDTH / 2))

        # Calculate cell indices
        dis = int(radius // circle_distance)
        ang = int(angle // angle_step) % num_slices
        # Update the clothoid matrix
        if 0 <= dis < num_cells and 0 <= ang < num_slices:
            clothoid_matrices[i][dis, ang] = 1

    # Create a mirrored version of the clothoid path
    mirrored_points = [(-point[0], point[1]) for point in shifted_points]

    # Convert mirrored clothoid points to screen coordinates
    screen_points_mirrored = [(int(WIDTH / 2 + point[0]), int(HEIGHT / 2 - point[1])) for point in mirrored_points]

    # Draw the mirrored clothoid curve on the screen
    pygame.draw.lines(screen, (0, 0, 255), False, screen_points_mirrored, 2)

    # Identify cells that the mirrored clothoid path goes through
    for point in screen_points_mirrored:
        x, y = point
        # Convert Cartesian coordinates to polar coordinates
        radius = ((x - WIDTH / 2) ** 2 + (y - HEIGHT / 2) ** 2) ** 0.5
        angle = -math.degrees(math.atan2(y - HEIGHT / 2, x - WIDTH / 2))

        # Calculate cell indices
        dis = int(radius // circle_distance)
        ang = int(angle // angle_step) % num_slices
        # Update the clothoid matrix
        if 0 <= dis < num_cells and 0 <= ang < num_slices:
            clothoid_matrices[i + num_paths][dis, ang] = 1

# Update the Pygame window
pygame.display.update()

# Print the clothoid matrices
for i, clothoid_matrix in enumerate(clothoid_matrices):
    print(f"Clothoid Matrix {i+1}:")
    print(clothoid_matrix)

# Wait for ESC key press before entering the main loop
while True:
    if keyboard.is_pressed('esc'):
        print("Stopping the program...")
        break


def run():
    """
    Runs the Lidar scanning process and displays the results on the screen.

    :return: None
    """
    # Initialize the RPLIDAR device
    lidar = RPLidar(PORT_NAME)
    try:
        lidar.motor_speed = motor_speed
        running = True
        # Continuously read LIDAR scans
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=10000):
            last_points = []
            matrix = np.zeros(matrix_size)

            screen.fill(BACKGROUND_COLOR)

            # Process each data point in the scan
            for (_, angle, distance) in scan:
                dis = int(distance // real_circle_distance)
                ang = int((360-angle) // angle_step) % num_slices
                if 0 <= dis <= num_cells:
                    matrix[dis, ang] += 1
                x = int(WIDTH / 2 + (distance / real_max_radius) * screen_max_radius * math.cos(math.radians(360 - angle)))
                y = int(HEIGHT / 2 - (distance / real_max_radius) * screen_max_radius * math.sin(math.radians(360 - angle)))
                last_points.append((x, y))

            # Convert the occupancy matrix to a binary matrix based on the threshold
            occupancy_matrix = np.zeros_like(matrix, dtype=float)
            occupancy_matrix = matrix * coefficient / np.array(list1)[:num_cells, None] > occupancy_threshold

            # Iterate through clothoid matrices
            for path_idx, clothoid_matrix in enumerate(clothoid_matrices, start=1):
                print(f"\nChecking Clothoid Matrix {path_idx}:")

                # Check for overlap with the occupancy matrix
                overlap_matrix = np.logical_and(clothoid_matrix, occupancy_matrix)

                if np.any(overlap_matrix):
                    print(f"Clothoid Matrix {path_idx} has overlap with occupied cells.")
                    # Handle the case where the clothoid path is not a free path (e.g., update a variable, break out of a loop, etc.)
                else:
                    print(f"Clothoid Matrix {path_idx} is a free path.")

            Highlight_cells(occupancy_matrix, red)
            # Draw lidar data on the screen
            for x, y in last_points:
                screen.set_at((x, y), POINT_COLOR)

            draw_polar_grid()
            pygame.display.update()

            # Check for 'esc' key press to stop the program
            if keyboard.is_pressed('esc'):
                print("Stopping the program...")
                break
            # Add a condition to handle more user actions
            elif keyboard.is_pressed('p'):
                print("Pausing the scanning process...")
                last_points = []
                lidar.stop_motor()
                lidar.stop()
                lidar.disconnect()
                while True:
                    if keyboard.is_pressed('c'):
                        print("Resuming the scanning process...")
                        lidar.connect()
                        lidar.start_motor()
                        lidar.start()
                        break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if not running:
                break

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the lidar motor and disconnect the RPLIDAR device
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()


if __name__ == '__main__':
    run()
