import cv2
import cv2.aruco as aruco
import numpy as np

def sort_map_points(pts):
    sorted_points = sorted(pts, key=lambda x: x[0])
    left = sorted(sorted_points[:2], key=lambda x: x[1])
    right = sorted(sorted_points[2:], key=lambda x: x[1])

    return np.float32([left[0], right[0], left[1], right[1]])

def preprocess_image(frame):
    bilateral_img = cv2.bilateralFilter(frame, 9, 50, 50)
    bw_img = cv2.cvtColor(bilateral_img, cv2.COLOR_BGR2GRAY)
    #blur = cv2.GaussianBlur(bw_img, (3, 3), 0)
    blur = cv2.medianBlur(bw_img, 15)
    binary_img = cv2.Canny(bw_img, 50, 100)
    #_, binary_img = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary_img = cv2.GaussianBlur(binary_img, (3, 3), 0)

    # cv2.imshow("binary img", binary_img)

    return binary_img

def capture_map_data(frame, binary_img, map_width, map_height):
    try:
        contours, _ = cv2.findContours(binary_img.copy(), cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, None

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        largest_contour = sorted_contours[0].squeeze()

        epsilon_map = 0.02 * cv2.arcLength(largest_contour, True)
        approx_map = cv2.approxPolyDP(largest_contour, epsilon_map, True)

        if approx_map.shape[0] != 4:
            print("The map is not found!")
            return False, None, None, None

        coord_to_transform = sort_map_points(approx_map.squeeze())
        pts2 = np.float32([[0, 0], [map_width, 0], [0, map_height], [map_width, map_height]])
        M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
        map_img = cv2.warpPerspective(frame, M, (map_width, map_height))

        return True, coord_to_transform, map_img, pts2

    except Exception as e:
        print("Error: ", e)
        return False, None, None, None

def capture_obstacle_data(map_img, padding):
    gray_map_img = preprocess_image(map_img)
    obstacle_contours, hierarchy = cv2.findContours(gray_map_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if not obstacle_contours:
        return []

    obstacle_masks = []

    for i, contour in enumerate(obstacle_contours):
        if hierarchy[0][i][2] != -1:
            continue

        if cv2.contourArea(contour, True) < 0:
            continue

        epsilon_obstacle = 0.02 * cv2.arcLength(contour, True)
        approx_obstacle = cv2.approxPolyDP(contour, epsilon_obstacle, True)

        if approx_obstacle.shape[0] > 5:
            continue

        mask = np.zeros_like(gray_map_img)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        obstacle_masks.append(mask)

    return obstacle_masks

def draw_node(map_img, position, color, radius=9):
    if position is not None:
        cv2.circle(map_img, position, radius, color, -1)

def create_grid(map_img, obstacle_masks, cell_size):
    map_height, map_width = map_img.shape[:2]
    grid_rows = int(np.ceil(map_height / cell_size))
    grid_cols = int(np.ceil(map_width / cell_size))

    grid = np.zeros((grid_rows, grid_cols), dtype=int)
    final_obstacle_map = np.zeros_like(map_img)

    for obstacle_mask in obstacle_masks:
        final_obstacle_map |= obstacle_mask

    for row in range(grid_rows):
        for col in range(grid_cols):
            y_start, y_end = row * cell_size, (row + 1) * cell_size
            x_start, x_end = col * cell_size, (col + 1) * cell_size
            try:
                obstacle_mask_new = final_obstacle_map[y_start:y_end, x_start:x_end]

                if np.any((obstacle_mask_new > 0)):
                    grid[row][col] = 1
                    padding = 2
                    grid[row + padding][col + padding] = 1
                    grid[row + padding][col - padding] = 1
                    grid[row - padding][col + padding] = 1
                    grid[row - padding][col - padding] = 1

            except IndexError as e:
                print(f"IndexError: {e}")
    return grid

def draw_grid_on_map(map_img, grid, cell_size):
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
    grid_path = map_img.copy()
    color = (0, 0, 255)
    color_grid = (255,255,0) 
    
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if (col,row) in path: 
                x_start, x_end = col * cell_size, (col + 1) * cell_size
                y_start, y_end = row * cell_size, (row + 1) * cell_size
                cv2.rectangle(grid_path, (x_start, y_start), (x_end, y_end), color_grid)

    return grid_path

def get_goal_position(map_img):
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
                #print(f"Marker ID: {markerIds[i]} Position (x, y): {thymio_position}")

                dx = corner[1][0] - corner[0][0]  # x_tr - x_tl
                dy = corner[1][1] - corner[0][1]  # y_tr - y_tl
                angle_radians = np.arctan2(dy, dx)
                #print(f"Angle: {angle_radians}")

            except Exception as e:
                print(f"Error: {e}")
                return None, -1

    if position is None:
        return None, angle_radians
    else:
        position = tuple([int(pos) for pos in position])
        return position, angle_radians

def draw_thymio_position(map_img, thymio_position):
    if thymio_position is not None:
        draw_node(map_img, (int(thymio_position[0]), int(thymio_position[1])), (255, 0, 0), 9)
