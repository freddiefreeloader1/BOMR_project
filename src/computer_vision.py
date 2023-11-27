import cv2 
import numpy as np

from Astar_coord import *

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

    neighbors = []
    for i, contour in enumerate(obstacle_contours):
        if hierarchy[0][i][2] != -1:
            continue

        if cv2.contourArea(contour, True) < 0:
            continue

        cv2.drawContours(map_img, obstacle_contours, i, (255, 255, 0), 3)
        epsilon_obstacle = 0.02 * cv2.arcLength(contour, True)
        approx_obstacle = cv2.approxPolyDP(contour, epsilon_obstacle, True)
        centroid = np.mean(approx_obstacle, axis=0)

        for point in approx_obstacle:
            new_point = calculate_new_point(point, centroid, padding)
            if new_point is not None:
                neighbors.append(tuple(new_point))

    return neighbors

def calculate_new_point(point, centroid, padding):
    original_point = point[0]
    vector = original_point - centroid[0]
    norm = np.linalg.norm(vector)

    if norm == 0 or np.any(np.isnan(vector / norm)):
        return None

    return original_point + padding * (vector / norm)

def draw_neighbours(map_img, neighbors):
    for point in neighbors:
        point_int = tuple(int(val) for val in point)
        cv2.circle(map_img, point_int, 5, (0, 0, 255), -1)

def draw_path(map_img, path):
    for i in range(len(path) - 1):
        start = tuple(int(val) for val in path[i])
        end = tuple(int(val) for val in path[i + 1])
        cv2.line(map_img, start, end, (0, 255, 0), 2)

def detect_goal(map_img):
    template = cv2.imread('../assets/images/goal.jpeg')
    template = cv2.resize(template, (200, 200))

    map_img_gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    res = cv2.matchTemplate(map_img_gray, template_gray, cv2.TM_CCOEFF_NORMED)
    threshold = 0.7
    loc = np.where(res >= threshold)

    w, h = template_gray.shape[::-1]
    for pt in zip(*loc[::-1]):
        cv2.circle(map_img, (pt[0] + w // 2, pt[1] + h // 2), 3, (255, 0, 0), -1)

    return (pt[0] + w // 2, pt[1] + h // 2)


def draw_goal(map_img, end):
    cv2.circle(map_img, end, 7, (255, 0, 0), -1)

def main():
    cap = cv2.VideoCapture(0)
    capture_data, capture_map, capture_obstacle = False, False, False
    max_width, max_height = 891, 1260
    padding = 50
    coord_to_transform = []
    pts2 = []
    neighbors = []
    path = []

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Unable to capture video")
            break

        binary_img = preprocess_image(frame)

        try:
            if capture_data:
                capture_map, coord_to_transform, map_img, pts2 = capture_map_data(frame, binary_img, max_width, max_height)
                capture_data = False
                capture_obstacle = not capture_map

            if capture_map and not capture_obstacle:
                neighbors = capture_obstacle_data(map_img, padding)
                
                if neighbors:
                    start = neighbors[1]
                    end = detect_goal(map_img)

                    neighbors.append(end)
                    path = astar(neighbors, start, end)

                    print('--- Path Info ---\n', f'Path length: {len(path)}\n', neighbors, start, end, f'Path: {path}')

                    capture_obstacle = True
                    capture_map = False

            if capture_obstacle:
                M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
                map_img = cv2.warpPerspective(frame, M, (max_width, max_height))

                draw_neighbours(map_img, neighbors)
                draw_path(map_img, path)
                draw_goal(map_img, end)

                map_img = cv2.rotate(map_img, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('Map', map_img)

            cv2.imshow('Original image', frame)
            key = cv2.waitKey(24)

            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('p'):
                print("Capturing map...")
                capture_data, capture_map, capture_obstacle = True, False, False
        except Exception as e:
            print("Error: ", e)
            continue

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()
