"""
Author: AHMED Mahmoud
Date: 14/01/2024
Institution: UHA ENSISA M2 EEA

Professor: Rodolfo Orjuela

Project: Detection of navigable areas with LIDAR for an autonomous vehicle

Description: This code Uses the same efficient algorithm from our simulation,
    allowing for the visualization of an occupancy matrix in real time.
    The program Uses RPLIDAR data, and highlights cells based on a specified density threshold.
    The resulting visualization provides insights into the occupancy of different grid cells.
    And possible paths for our Vehicle.
"""
# Import necessary libraries
import pygame
import math
from rplidar import RPLidar
import numpy as np
import keyboard

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


def check_consecutive_zeros_and_draw(matrix):
    """
    Check for consecutive zeros in a matrix and draw a free path matrix.

    :param matrix: The input matrix
    :return: The free path matrix with consecutive zeros marked as 1

    Example usage:
        import numpy as np

        matrix = np.array([[1, 0, 0, 0],
                           [1, 0, 0, 1],
                           [1, 1, 0, 1]])
        free_path_matrix = check_consecutive_zeros_and_draw(matrix)
        print(free_path_matrix)

    Output:
        [[0 1 1 0]
         [0 1 1 0]
         [0 1 1 0]]
    """
    num_rows, num_cols = matrix.shape
    free_path_matrix = np.zeros((num_rows, num_cols), dtype=int)

    for col in range(num_cols):
        if matrix[0, col] == 0 and np.all(matrix[1:dis_threshold, col] == 0):
            start_angle = (col + 0.5) * angle_step  # Middle angle of the cells
            start_radius = 0
            end_radius = screen_max_radius

            # Convert polar coordinates to Cartesian coordinates
            start_x, start_y = polar_to_cartesian(start_radius, start_angle)
            end_x, end_y = polar_to_cartesian(end_radius, start_angle)

            # Draw a line from the center through the middle of the cells to the border of the last cell with 0
            pygame.draw.line(screen, (0, 255, 0), (start_x, start_y), (end_x, end_y), 2)

            # Update the free path matrix
            free_path_matrix[:, col] = 1

    return free_path_matrix


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

            # Check consecutive zeros and draw lines
            free_path_matrix = check_consecutive_zeros_and_draw(occupancy_matrix)

            # Print the free path matrix
            print("Free Path Matrix:")
            print(free_path_matrix)

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
