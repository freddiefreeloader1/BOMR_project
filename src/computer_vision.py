import cv2 
import numpy as np
from ultralytics import YOLO

from Astar_coord import *
from Astar import *

def sort_map_points(pts):
    sorted_points = sorted(pts, key=lambda x: x[0])
    left = sorted(sorted_points[:2], key=lambda x: x[1])
    right = sorted(sorted_points[2:], key=lambda x: x[1])

    return np.float32([left[0], right[0], left[1], right[1]])

def preprocess_image(frame):
    bilateral_img = cv2.bilateralFilter(frame, 9, 50, 50)
    bw_img = cv2.cvtColor(bilateral_img, cv2.COLOR_BGR2GRAY)
    #blur = cv2.GaussianBlur(bw_img, (5, 5), 0)
    blur = cv2.medianBlur(bw_img, 13)
    #binary_img = cv2.Canny(blur, 100, 100)
    _, binary_img = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #binary_img = cv2.GaussianBlur(binary_img, (3, 3), 0)
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

        mask = np.zeros_like(gray_map_img)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        obstacle_masks.append(mask)

        cv2.drawContours(map_img, obstacle_contours, i, (255, 255, 0), 3)
        epsilon_obstacle = 0.02 * cv2.arcLength(contour, True)
        approx_obstacle = cv2.approxPolyDP(contour, epsilon_obstacle, True)
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

def draw_nodes(map_img, nodes):
    for point in nodes:
        point_int = tuple(int(val) for val in point)
        cv2.circle(map_img, point_int, 5, (0, 0, 255), -1)

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

def detect_goal(map_img, template_size=(200, 200)):
    template = cv2.imread('../assets/images/goal.jpeg')
    template = cv2.resize(template, template_size)

    map_img_gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    res = cv2.matchTemplate(map_img_gray, template_gray, cv2.TM_CCOEFF_NORMED)
    threshold = 0.7
    loc = np.where(res >= threshold)

    w, h = template_gray.shape[::-1]
    for pt in zip(*loc[::-1]):
        cv2.circle(map_img, (pt[0] + w // 2, pt[1] + h // 2), 3, (255, 0, 0), -1)

    return (pt[0] + w // 2, pt[1] + h // 2)

def detect_thymio(map_img, model):
    results = model(map_img, stream=True)

    for r in results:
        boxes = r.boxes

        for box in boxes:
            _, _, w, h = box.xywh[0]
            w, h = int(w), int(h)

            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            center = (x1 + (w // 2), y1 + (h // 2))

            cv2.circle(map_img, center, 7, (0, 255, 255), -1)
            cv2.rectangle(map_img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            confidence = math.ceil((box.conf[0] * 100)) / 100
            print("Confidence --->",confidence)

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 255, 0)
            thickness = 2

            cv2.putText(map_img, 'Thymio', org, font, fontScale, color, thickness)


def draw_goal(map_img, end):
    cv2.circle(map_img, end, 7, (255, 0, 0), -1)

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

def main():
    cap = cv2.VideoCapture(0)
    
    # Map and obstacle detection
    capture_data, capture_map, capture_obstacle = False, False, False
    max_width, max_height = 891, 1260
    padding = 50
    coord_to_transform = []
    pts2 = []

    # Global navigation
    unreachable_nodes = {}
    path = []

    # Thymio detection
    thymio_detected = False
    thymio_coordinates = (0, 0)
    model = YOLO("./src/runs/detect/train/weights/best.pt")

    while True:
        # ret, frame = cap.read()
        frame = cv2.imread("./assets/images/map.jpg")
        frame = cv2.resize(frame, (900,600))
        '''if not ret:
            print("Unable to capture video")
            break'''

        binary_img = preprocess_image(frame)

        try:
            if capture_data:
                capture_map, coord_to_transform, map_img, pts2 = capture_map_data(frame, binary_img, max_width, max_height)
                capture_data = False
                capture_obstacle = not capture_map

            if capture_map and not capture_obstacle:
                unreachable_nodes, obstacle_masks = capture_obstacle_data(map_img, padding)
                # print('--- Unreachable Nodes ---\n', unreachable_nodes)
                
                ## =========== Change star, end point here to try Astar =========== ##
                # A* Algorithm
                start = list(unreachable_nodes.keys())[9]
                # end = heapq.nlargest(1,list(unreachable_nodes.keys()),key= lambda x: euclidean_distance(start,x))[0]
                end = list(unreachable_nodes.keys())[24]
                path = astar(unreachable_nodes, start, end)

                capture_obstacle = True
                capture_map = False
                bw_map = cv2.cvtColor(map_img.copy(), cv2.COLOR_BGR2GRAY)
                cell_size = 20  
                grid = create_grid(bw_map, obstacle_masks, cell_size)
                print("Grid:\n", grid)
                # be careful, the origin is at the top right corner, and the positive x is downward, and positive y is left 
                start_grid = (40, 30) 
                end_grid = (0, 61)
                path_grid = astar_grid(grid, start_grid, end_grid, moves_8n)
                
                

            if thymio_detected:
                detect_thymio(map_img, model)

            if capture_obstacle:
                M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
                map_img = cv2.warpPerspective(frame, M, (max_width, max_height))

                draw_nodes(map_img, list(unreachable_nodes.keys()))
                # draw_path(map_img, path)
                # draw_unreachable_nodes(map_img, unreachable_nodes)
                # draw_goal(map_img, end)
                # draw_path(map_img, path_grid)

                map_img = draw_grid_on_map(map_img, grid, cell_size)
                map_img = draw_grid_path(map_img, grid, path_grid, cell_size)
                map_img = cv2.rotate(map_img, cv2.ROTATE_90_CLOCKWISE)
                map_img = cv2.resize(map_img, (900,600))
                cv2.imshow('Map', map_img)

            cv2.imshow('Original image', frame)
            key = cv2.waitKey(24)

            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('p'):
                print("Capturing map...")
                capture_data, capture_map, capture_obstacle = True, False, False
                thymio_detected = False
            elif key == ord('o'):
                print('Detecting Thymio...')
                thymio_detected = True

        except Exception as e:
            print("Error: ", e)
            continue

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()
