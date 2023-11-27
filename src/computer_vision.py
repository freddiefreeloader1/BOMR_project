import cv2 
import numpy as np 

def sort_map_points(pts):
    sorted_points = sorted(pts, key=lambda x: x[0])

    left = sorted(sorted_points[:2], key=lambda x: x[1])
    right = sorted(sorted_points[2:], key=lambda x: x[1])

    return np.float32([left[0], right[0], left[1], right[1]])

def get_map_region_of_interest(frame, coord_to_transform):
    img_height = frame.shape[0]
    roi_height = np.linalg.norm(coord_to_transform[0] - coord_to_transform[1])
    roi_width = np.linalg.norm(coord_to_transform[0] - coord_to_transform[3])

    aspect_ratio = roi_width / roi_height

    map_width = int(img_height * aspect_ratio)
    map_height = img_height

    return map_width, map_height

def preprocess_image(frame):
    bilateral_img = cv2.bilateralFilter(frame, 9, 50, 50)
    bw_img = cv2.cvtColor(bilateral_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(bw_img, (5, 5), 0)
    binary_img = cv2.Canny(blur, 100, 100)
    binary_img = cv2.GaussianBlur(binary_img, (7, 7), 0)
    return binary_img

def main():
    cap = cv2.VideoCapture(0)
    capture_map = False
    capture_data = False
    map_width, map_height = 0, 0
    padding = 20
    neighbours = []
    coord_to_transform = []

    while True:
        ret, frame = cap.read()
        map_img = frame.copy()

        if not ret:
            print("Unable to capture video")
            break

        binary_img = preprocess_image(frame)

        if capture_data:
            try:
                contours, hierarchy = cv2.findContours(binary_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) > 0:
                    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

                    largest_contour = sorted_contours[0].squeeze()

                    rows = hierarchy[0].shape[0]
                    neighbours = []

                    for i in range(rows):
                        if (hierarchy[0][i][2] == -1):
                            currentContour = contours[i]

                            contour_area = cv2.contourArea(currentContour)

                            if contour_area < 10:
                                continue

                            cv2.drawContours(map_img, contours, i, (255, 255, 0), 3)

                            epsilon_obstacle = 0.02 * cv2.arcLength(currentContour, True)
                            approx_obstacle = cv2.approxPolyDP(currentContour, epsilon_obstacle, True)

                            centroid = np.mean(approx_obstacle, axis=0)

                            for point in approx_obstacle:
                                original_point = point[0]
                                vector = original_point - centroid[0]

                                norm = np.linalg.norm(vector)

                                if norm == 0:
                                    continue

                                direction = vector / norm
                                new_point = original_point + padding * direction

                                if np.any(np.isnan(new_point)):
                                    continue
                                
                                neighbours.append(new_point.tolist())

                    epsilon_map = 0.02 * cv2.arcLength(largest_contour, True)
                    approx_map = cv2.approxPolyDP(largest_contour, epsilon_map, True)

                    if approx_map.shape[0] == 4:
                        coord_to_transform = sort_map_points(approx_map.squeeze())
                        map_width, map_height = get_map_region_of_interest(map_img, coord_to_transform)

                        print(len(neighbours))

                        capture_data = False
                        capture_map = True
                    else:
                        print("The map is not found!")
            except Exception as e:
                print("Error: ", e)

        ''' Show images'''
        cv2.imshow('Original image', frame)
        # cv2.imshow('Binary image', binary_img)

        if capture_map:
            for point in neighbours:
                point_int = tuple(int(val) for val in point)
                cv2.circle(frame, point_int, 5, (0, 0, 255), -1)

            pts2 = np.float32([[0, 0], [map_width, 0], [0, map_height], [map_width, map_height]])

            M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
            frame = cv2.warpPerspective(frame, M, (map_width, map_height))

            cv2.imshow('Map', frame)

        ''' Handle keyboard input'''
        key = cv2.waitKey(24)

        if key == ord('q'):
            print("Quitting...")
            break
        elif key == ord('p'):
            print("Capturing map...")
            capture_data = True

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()
