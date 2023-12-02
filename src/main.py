from computer_vision import *
from Astar_coord import *
from Astar import *

def main():
    cap = cv2.VideoCapture(0)
    
    # Map and obstacle detection variables
    capture_data, setup_finished = False, False
    max_width, max_height = 891, 1260
    padding = 50
    coord_to_transform = []
    pts2 = []

    # Global navigation variables
    plan_path = False
    unreachable_nodes = {}
    start = None
    end = None
    cell_size = 20
    start_grid = (0, 0) 
    end_grid = (40, 60)
    grid = np.array([])
    path_grid = np.array([])

    # Thymio variables
    detect_thymio = False
    thymio_position = (0, 0) # <- Kalman Filter and local navigation
    thymio_angle_degrees = 0 # <- Kalman Filter and local navigation

    while True:
        ret, frame = cap.read()
        map_img = frame.copy()

        if not ret:
            print("Unable to capture video")
            break

        binary_img = preprocess_image(frame)

        try:
            if capture_data:
                capture_map, coord_to_transform, map_img, pts2 = capture_map_data(frame, binary_img, max_width, max_height)
                
                if capture_map:
                    unreachable_nodes, obstacle_masks = capture_obstacle_data(map_img, padding)
                    end = get_goal_position(map_img)

                    print("Map and obstacles captured!")

                    capture_data = False
                    setup_finished = True

            elif setup_finished:
                M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
                map_img = cv2.warpPerspective(frame, M, (max_width, max_height))

                if detect_thymio:
                    thymio_position, thymio_angle_degrees = get_thymio_info(map_img)
                    # print(f'Position: {thymio_position}, Angle: {thymio_angle_degrees}')
                    draw_thymio_position(map_img, thymio_position)
                
                if plan_path:
                    start = thymio_position

                    ''' Path planning '''
                    bw_map = cv2.cvtColor(map_img.copy(), cv2.COLOR_BGR2GRAY)
                    grid = create_grid(bw_map, obstacle_masks, cell_size)
                    print("Grid:\n", grid)
                    path_grid = astar_grid(grid, start_grid, end_grid, moves_8n)
                    simplified_path = simplify_path(path_grid)
                    metric_path = transform_grid_to_metric(simplified_path, 600, 400, grid)
                    
                    plan_path = False

                draw_node(map_img, start, (0, 73, 255)) # <- Start node
                draw_reachable_nodes(map_img, list(unreachable_nodes.keys()))
                draw_node(map_img, end, (0, 255, 0)) # <- End node

                map_img = draw_grid_on_map(map_img, grid, cell_size)
                map_img = draw_grid_path(map_img, grid, path_grid, cell_size)
                #map_img = cv2.resize(map_img, (600, 400))

                cv2.imshow('Map', map_img)

            cv2.imshow('Original image', frame)

            ''' Keyboard options '''
            key = cv2.waitKey(24)

            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('p'): # Press p to prepare the map and obstacles
                print("Capturing map...")
                capture_data = True
            elif key == ord('d'): # Press d to detect the Thymio and start the path planning
                detect_thymio = True
                plan_path = True
                print('Detecting Thymio...')
            elif key == ord('s'):
                print('Detecting start...')
                start = detect_start(map_img)

        except Exception as e:
            print("Error: ", e)
            continue

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()
