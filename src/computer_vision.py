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

    return binary_img

def capture_map_data(frame, binary_img, map_width, map_height):
    try:
        contours, _ = cv2.findContours(binary_img.copy(), cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        largest_contour = sorted_contours[0].squeeze()

        epsilon_map = 0.02 * cv2.arcLength(largest_contour, True)
        approx_map = cv2.approxPolyDP(largest_contour, epsilon_map, True)

        if approx_map.shape[0] != 4:
            print("The map is not found!")
            return False, None, None

        coord_to_transform = sort_map_points(approx_map.squeeze())
        pts2 = np.float32([[0, 0], [map_width, 0], [0, map_height], [map_width, map_height]])
        M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
        map_img = cv2.warpPerspective(frame, M, (map_width, map_height))

        return True, coord_to_transform, map_img, pts2

    except Exception as e:
        print("Error: ", e)
        return False, None, None

def capture_obstacle_data(map_img, padding):
    gray_map_img = preprocess_image(map_img)
    obstacle_contours, hierarchy = cv2.findContours(gray_map_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if not obstacle_contours:
        return []

    obstacle_masks = []
    nodes = []
    unreachable_nodes = {}

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

        centroid = np.mean(approx_obstacle, axis=0)

        for point in approx_obstacle:
            new_point = calculate_new_point(point, centroid, padding)
            if new_point is not None:
                tuple_new_point = tuple(new_point)
                nodes.append(tuple_new_point)
                unreachable_nodes[tuple_new_point] = []

        for node in unreachable_nodes.keys():
            for neighbor in nodes:
                if node == neighbor:
                    continue

                if neighbor in unreachable_nodes[node]:
                    continue

                if is_reachable(node, neighbor, obstacle_masks, map_img):
                    unreachable_nodes[node].append(neighbor)

    return unreachable_nodes, obstacle_masks

def is_reachable(node, neighbor, obstacle_masks, map_img):
    line_img = np.zeros_like(obstacle_masks[0])
    node_int = tuple(int(val) for val in node)
    neighbor_int = tuple(int(val) for val in neighbor)
    cv2.line(line_img, node_int, neighbor_int, 255, 3)

    for mask in obstacle_masks:
        if np.any(cv2.bitwise_and(line_img, mask)):
            return False
    return True

def calculate_new_point(point, centroid, padding):
    original_point = point[0]
    vector = original_point - centroid[0]
    norm = np.linalg.norm(vector)

    if norm == 0 or np.any(np.isnan(vector / norm)):
        return None

    return original_point + padding * (vector / norm)

def draw_reachable_nodes(map_img, nodes):
    for point in nodes:
        point_int = tuple(int(val) for val in point)
        cv2.circle(map_img, point_int, 5, (0, 0, 255), -1)

def draw_node(map_img, position, color, radius=9):
    if position is not None:
        cv2.circle(map_img, position, radius, color, -1)

def draw_path(map_img, path):
    for i in range(len(path) - 1):
        start = tuple(int(val) for val in path[i])
        end = tuple(int(val) for val in path[i + 1])
        cv2.line(map_img, start, end, (0, 255, 0), 2)

def draw_unreachable_nodes(map_img, unreachable_nodes):
    color = (0, 0, 255)
    for node, neighbors in unreachable_nodes.items():
        node_int = tuple(int(val) for val in node)
        cv2.circle(map_img, node_int, 5, color, -1)

        for neighbor in neighbors:
            neighbor_int = tuple(int(val) for val in neighbor)
            cv2.line(map_img, node_int, neighbor_int, color, 2)

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

                    grid[row + 1][col + 1] = 1
                    grid[row + 1][col - 1] = 1
                    grid[row - 1][col + 1] = 1

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
                cv2.rectangle(grid_map, (x_start, y_start), (x_end, y_end), color_grid, -1)

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
                cv2.rectangle(grid_path, (x_start, y_start), (x_end, y_end), color_grid, -1)

    return grid_path

def simplify_path(path):
    simplified_path = [path[0]] 

    for i in range(1, len(path) - 1):
        current_point = np.array(path[i])
        next_point = np.array(path[i + 1])
        direction_vector = next_point - np.array(simplified_path[-1])
        
        if np.cross(direction_vector, current_point - np.array(simplified_path[-1])) == 0:
            continue  

        simplified_path.append(path[i]) 

    simplified_path.append(path[-1])

    return simplified_path

def transform_grid_to_metric(path, map_height, map_width, grid):
    metric_path = []
    grid_x = len(grid[0])
    grid_y = len(grid)
    for x,y in path:
        metric_path.append(((x * map_width) / grid_x, -(y * map_height) / grid_y))

    return metric_path

def get_goal_position(map_img):
    hsv_img = cv2.cvtColor(map_img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([160, 100, 20], dtype="uint8")
    upper_red = np.array([180, 255, 255], dtype="uint8")

    mask = cv2.inRange(hsv_img, lower_red, upper_red)
    mask = cv2.medianBlur(mask, 7)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    (x, y), _ = cv2.minEnclosingCircle(contours[0])

    return (int(x), int(y))

def get_thymio_info(map_img):
    dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    markerCorners, markerIds, rejectedCandidates = aruco.detectMarkers(map_img, dictionary)
    position = None
    angle_degrees = -1

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
                angle_degrees = np.degrees(angle_radians) % 360
                # print(f"Angle: {angle_degrees}")

            except Exception as e:
                print(f"Error: {e}")
                return None, -1

    position = tuple([int(pos) for pos in position])
    return position, angle_degrees

def draw_thymio_position(map_img, thymio_position):
    print(f'thymio position: {thymio_position}')
    if thymio_position is not None:
        draw_node(map_img, (int(thymio_position[0]), int(thymio_position[1])), (255, 0, 0), 9)
