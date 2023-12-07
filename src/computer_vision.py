import cv2
import cv2.aruco as aruco
import numpy as np

def sort_map_points(pts):
    """Sort the points of the map in the following order: top-left, top-right, bottom-left, bottom-right

    Parameters:
        pts: a list of points representing the four corners of the map

    Returns:
        map_sorted_points: a list of points representing the four corners of the map in the following order: top-left, top-right, bottom-left, bottom-right
    """
    sorted_points = sorted(pts, key=lambda x: x[0])
    left = sorted(sorted_points[:2], key=lambda x: x[1])
    right = sorted(sorted_points[2:], key=lambda x: x[1])
    map_sorted_points = np.float32([left[0], right[0], left[1], right[1]])

    return map_sorted_points

def preprocess_image(frame):
    """Preprocess the image by applying a bilateral filter, converting it to grayscale, applying Canny edge detection and a Gaussian blur

    Parameters:
        frame: the image to preprocess

    Returns:
        binary_img: the preprocessed image
    """
    bilateral_img = cv2.bilateralFilter(frame, 9, 50, 50)
    bw_img = cv2.cvtColor(bilateral_img, cv2.COLOR_BGR2GRAY)
    binary_img = cv2.Canny(bw_img, 50, 100)
    binary_img = cv2.GaussianBlur(binary_img, (3, 3), 0)
    return binary_img

def capture_map_data(frame, binary_img, map_width, map_height):
    """Capture the map data from the image by finding the largest contour, approximating it to a quadrilateral and applying a perspective transform to it

    Parameters:
        frame: the image to capture the map data from
        binary_img: the preprocessed image
        map_width: the width of the map
        map_height: the height of the map

    Returns:
        capture_map: a boolean indicating whether the map was captured or not
        coord_to_transform: a list of points representing the four corners of the map
        map_img: the image of the map
        pts2: a list of points representing the four corners of the map in the following order: top-left, top-right, bottom-left, bottom-right
    """
    capture_map = False
    try:
        contours, _ = cv2.findContours(binary_img.copy(), cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return capture_map, None, None, None

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        largest_contour = sorted_contours[0].squeeze()

        epsilon_map = 0.02 * cv2.arcLength(largest_contour, True)
        approx_map = cv2.approxPolyDP(largest_contour, epsilon_map, True)

        if approx_map.shape[0] != 4:
            print("The map is not found!")
            return capture_map, None, None, None

        coord_to_transform = sort_map_points(approx_map.squeeze())
        pts2 = np.float32([[0, 0], [map_width, 0], [0, map_height], [map_width, map_height]])
        M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
        map_img = cv2.warpPerspective(frame, M, (map_width, map_height))
        capture_map = True

        return capture_map, coord_to_transform, map_img, pts2

    except Exception as e:
        print("Error: ", e)
        return capture_map, None, None, None


def capture_obstacle_data(map_img):
    """Capture the obstacle data from the image by applying a Gaussian blur to it, converting it to HSV, applying a mask to it to get the green color, finding the contours and approximating them to polygons

    Args:
        map_img: the image to capture the obstacle data from

    Returns:
        obstacle_masks: a list of masks representing the obstacles
    """
    map_img_copy = map_img.copy()
    map_img_copy = cv2.GaussianBlur(map_img_copy, (15, 15), 0)
    hsv_map_img = cv2.cvtColor(map_img_copy, cv2.COLOR_BGR2HSV)

    lower_green = np.array([41, 60, 0])
    upper_green = np.array([80, 255, 255]) 

    green_mask = cv2.inRange(hsv_map_img, lower_green, upper_green)
    # cv2.imshow('Obstacle detection', green_mask)

    obstacle_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if not obstacle_contours:
        return []

    obstacle_masks = []

    for contour in obstacle_contours:
        epsilon_obstacle = 0.02 * cv2.arcLength(contour, True)
        approx_obstacle = cv2.approxPolyDP(contour, epsilon_obstacle, True)

        if approx_obstacle.shape[0] > 5:
            continue  # Skip contours with more than 5 sides

        mask = np.zeros_like(green_mask)
        cv2.drawContours(mask, [approx_obstacle], -1, 255, -1)
        obstacle_masks.append(mask)

    return obstacle_masks

def draw_node(map_img, position, color, radius=9):
    """Draw a circle representing a node on the map

    Args:
        map_img: the image of the map
        position: the position to draw the circle at
        color: the color of the circle
        radius: radius of the circle (default: 9)
    """
    if position is not None:
        cv2.circle(map_img, position, radius, color, -1)

def draw_line(img, p, angle,length, color,thick=3):
    """Draw a line on the map

    Parameters:
        img: the image of the map
        p: the starting point of the line
        angle: the angle of the line
        length: the length of the line
        color: the color of the line
        thick (int, optional): thickness of the line (default: 3)
    """
    cv2.line(img,(int(p[0]),int(p[1])),(int(p[0]+length*np.cos(angle)),int(p[1]+length*np.sin(angle))),color,thick)

def create_grid(map_img, obstacle_masks, cell_size, metric_padding):
    """Create a grid from the map image and the obstacle masks, adding padding to the obstacles

    Parameters:
        map_img: the image of the map
        obstacle_masks: a list of masks representing the obstacles
        cell_size: the size of each cell in the grid

    Returns:
        grid: the grid created from the map image and the obstacle masks. It is a 2D array of 0s and 1s, where 0 represents a free cell and 1 represents an occupied cell
    """
    map_height, map_width = map_img.shape[:2]
    grid_rows = int(np.ceil(map_height / cell_size))
    grid_cols = int(np.ceil(map_width / cell_size))
    padding = int(metric_padding / cell_size)
    grid = np.zeros((grid_rows, grid_cols), dtype=int)
    final_obstacle_map = np.zeros_like(map_img)

    for obstacle_mask in obstacle_masks:
        final_obstacle_map |= obstacle_mask

    for row in range(grid_rows):
        for col in range(grid_cols):
            y_start, y_end = row * cell_size, (row + 1) * cell_size
            x_start, x_end = col * cell_size, (col + 1) * cell_size
            if row - grid_rows + 1  ==  0 or col - grid_cols + 1  == 0 or row == 0 or col == 0: 
                grid[row][col] = 1
            try:
                obstacle_mask_new = final_obstacle_map[y_start:y_end, x_start:x_end]

                if np.any((obstacle_mask_new > 0)):
                    grid[row][col] = 1
                    
                    row_p = min(row + padding, grid_rows - 1)
                    col_p = min(col + padding, grid_cols - 1)
                    grid[row_p][col_p] = 1

                    row_p = max(row - padding, 0)
                    col_p = min(col + padding, grid_cols - 1)
                    grid[row_p][col_p] = 1

                    row_p = min(row + padding, grid_rows - 1)
                    col_p = max(col - padding, 0)
                    grid[row_p][col_p] = 1

                    row_p = max(row - padding, 0)
                    col_p = max(col - padding, 0)
                    grid[row_p][col_p] = 1

                    row_p = min(row + padding, grid_rows - 1)
                    grid[row_p][col] = 1

                    row_p = max(row - padding, 0)
                    grid[row_p][col] = 1

                    col_p = max(col - padding, 0)
                    grid[row][col_p] = 1

                    col_p = min(col + padding, grid_cols - 1)
                    grid[row][col_p] = 1

            except IndexError:
                pass

    return grid

def draw_grid_on_map(map_img, grid, cell_size):
    """Draw the grid on the map image

    Parameters:
        map_img: the image of the map
        grid: the grid created from the map image and the obstacle masks. It is a 2D array of 0s and 1s, where 0 represents a free cell and 1 represents an occupied cell
        cell_size: the size of each cell in the grid

    Returns:
        grid_map: the map image with the grid drawn on it
    """
    grid_map = map_img.copy()
    color = (0, 0, 255)
    color_grid = (0,255,0) 
    
    for x in range(0, grid_map.shape[1], cell_size):
        cv2.line(grid_map, (x, 0), (x, grid_map.shape[0]), color, 1)

    for y in range(0, grid_map.shape[0], cell_size):
        cv2.line(grid_map, (0, y), (grid_map.shape[1], y), color, 1)

    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row, col] == 1: 
                x_start, x_end = col * cell_size, (col + 1) * cell_size
                y_start, y_end = row * cell_size, (row + 1) * cell_size
                cv2.rectangle(grid_map, (x_start, y_start), (x_end, y_end), color_grid)

    return grid_map

def draw_grid_path(map_img, grid, path, cell_size):
    """Draw the path on the grid which is drawn on the map image

    Parameters:
        map_img: the image of the map
        grid: the grid created from the map image and the obstacle masks.
        path: the path to draw on the map image
        cell_size: the size of each cell in the grid

    Returns:
        grid_path: the map image with the grid and the path drawn on it
    """
    grid_path = map_img.copy()
    color_grid = (255,255,0) 
    
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if (col,row) in path: 
                x_start, x_end = col * cell_size, (col + 1) * cell_size
                y_start, y_end = row * cell_size, (row + 1) * cell_size
                cv2.rectangle(grid_path, (x_start, y_start), (x_end, y_end), color_grid)

    return grid_path

def get_goal_position(map_img):
    """Get the position of the goal, first by converting the image to HSV, then by applying a mask to it to get the red color, then by finding the largest contour and approximating it to a circle

    Parameters:
        map_img: the image of the map

    Returns:
        goal_position: the position of the goal represented as a tuple of (x, y) coordinates
    """
    hsv_img = cv2.cvtColor(map_img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 20], dtype="uint8")  # Lower bound for red in HSV
    upper_red = np.array([10, 255, 255], dtype="uint8")  # Upper bound for red in HSV

    mask1 = cv2.inRange(hsv_img, lower_red, upper_red)

    lower_red = np.array([160, 100, 20], dtype="uint8")  # Lower bound for red in HSV
    upper_red = np.array([180, 255, 255], dtype="uint8")  # Upper bound for red in HSV

    mask2 = cv2.inRange(hsv_img, lower_red, upper_red)

    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.medianBlur(mask, 7)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    max_contour = max(contours, key=cv2.contourArea)

    if len(max_contour) > 0:
        (x, y), _ = cv2.minEnclosingCircle(max_contour)
        return (int(x), int(y))

    return None

def get_thymio_info(map_img):
    """Get the aruco marker information (center position and angle) from the image of the map which represents the Thymio

    Parameters:
        map_img: the image of the map in which the Thymio is represented by an aruco marker

    Returns:
        position: center position of the Thymio represented as a tuple of (x, y) coordinates
        angle_radians: angle of the Thymio in radians
    """
    dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    markerCorners, markerIds, rejectedCandidates = aruco.detectMarkers(map_img, dictionary)
    position = None
    angle_radians = -1

    if markerIds is not None:
        for i, corner in enumerate(markerCorners):
            try:
                aruco.drawDetectedMarkers(map_img, markerCorners)
                corner = corner.squeeze()
                position = corner.mean(axis=0)

                dx = corner[1][0] - corner[0][0]  # x_tr - x_tl
                dy = corner[1][1] - corner[0][1]  # y_tr - y_tl
                angle_radians = np.arctan2(dy, dx)

            except Exception as e:
                print(f"Error: {e}")
                return None, -1

    if position is None:
        return None, angle_radians
    else:
        position = tuple([int(pos) for pos in position])
        return position, ((angle_radians - np.pi / 2) % (2 * np.pi))

def draw_thymio_position(map_img, thymio_position):
    """Draw a circle representing the center of the Thymio on the map

    Parameters:
        map_img: the image of the map
        thymio_position: the position of the Thymio represented as a tuple of (x, y) coordinates
    """
    if thymio_position is not None:
        draw_node(map_img, (int(thymio_position[0]), int(thymio_position[1])), (255, 0, 0), 9)
