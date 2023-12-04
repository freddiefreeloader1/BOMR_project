from computer_vision import *
from Astar_coord import *
from Astar import * 
from enum import Enum
from common import Quit

class CameraState(Enum):
    WAITING = 0,
    CAPTURING_DATA = 1,
    SETTING_UP = 2

cap = None
camera_state = CameraState.WAITING

# Map and obstacle detection variables
max_width, max_height = 1170, 735
padding = 50
coord_to_transform = []
pts2 = []

# Global navigation variables
plan_path = False

# Grid settıng
cell_size = 20
start_grid = () 
end_grid = ()
grid = np.array([])
path_grid = np.array([])

start = None # Path for local navigation
end = None # Path for local navigation
metric_path = [] # Path for local navigation

# Thymio variables
detect_thymio = False
thymio_position = (0, 0) # <- Kalman Filter and local navigation
thymio_angle = 0 # <- Kalman Filter and local navigation

def camera_handle_keys():
    global camera_state, detect_thymio, plan_path
    ''' Keyboard options '''    
    key = cv2.waitKey(24)

    if key == ord('q'):
        print("Quitting...")
        raise Quit
    elif key == ord('p'): # Press p to prepare the map and obstacles
        print("Capturing map...")
        camera_state = CameraState.CAPTURING_DATA
    elif key == ord('d'): # Press d to detect the Thymio and start the path planning
        detect_thymio = True
        plan_path = True
        print('Detecting Thymio...')

def CameraLoop():
    global camera_state, max_height, max_width, padding, coord_to_transform, plan_path, cell_size, start_grid, end_grid, grid, path_grid, start, end, metric_path, detect_thymio, thymio_angle, thymio_position
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    map_img = frame.copy()

    if not ret:
        print("Unable to capture video")
        raise Quit

    binary_img = preprocess_image(frame)

    try:
        if camera_state == CameraState.CAPTURING_DATA:
            capture_map, coord_to_transform, map_img, pts2 = capture_map_data(frame, binary_img, max_width, max_height)
            
            if capture_map:
                obstacle_masks = capture_obstacle_data(map_img, padding)
                print("Map and obstacles captured!")

                camera_state == CameraState.SETTING_UP

        elif camera_state == CameraState.SETTING_UP:
            M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
            map_img = cv2.warpPerspective(frame, M, (max_width, max_height))
            
            end = get_goal_position(map_img)
            # print("End", end)

            if detect_thymio:
                thymio_position, thymio_angle = get_thymio_info(map_img)
                # print(f'Position: {thymio_position}, Angle: {thymio_angle}')
                draw_thymio_position(map_img, thymio_position)
            
            if plan_path:
                if thymio_position is None:
                    return #continue
                
                start = thymio_position

                ''' Path planning '''
                map_img = cv2.resize(map_img, (max_width, max_height))
                grid, path_grid, simplified_path, metric_path = make_path(map_img, obstacle_masks, cell_size, start, end, grid, 
                max_width, max_height)
                print(metric_path)
                plan_path = False
                

            # Local navigatıon code 

            draw_node(map_img, start, (0, 73, 255)) # <- Start node
            draw_node(map_img, end, (255, 255, 0)) # <- End node

            map_img = draw_grid_on_map(map_img, grid, cell_size)
            map_img = draw_grid_path(map_img, grid, path_grid, cell_size)
            map_img = cv2.resize(map_img, (600, 400))

            cv2.imshow('Map', map_img)

        cv2.imshow('Original image', frame)
        camera_handle_keys()

    except Exception as e:
        print("Error: ", e)

def CameraClose():
    global cap
    cap.release()
    cv2.destroyAllWindows()

def CameraInit():
    global cap
    cap = cv2.VideoCapture(0)

if __name__ == "__main__":
    CameraInit()
    # replace loop with return
    try:
        while True:
            CameraLoop()
    except Quit:
        pass
    CameraClose()