from vision.computer_vision import *
from planning.Astar_coord import *
from planning.Astar import * 
from enum import Enum
from util.common import Quit
from util.constants import CAMERA_ID

# The camera has 5 states:
# CAPTURING DATA: starts when the user presses 'p'.
# SETTING_UP: starts if the obstacles were captured successfully
# DETECTING_THYMIO: starts when the user presses 'd'
# PLANNING_PATH: runs once a thymio aruco was detected!
# DONE: once the path is planned, jumps to the final state.

class CameraState(Enum):
    WAITING = 0
    CAPTURING_DATA = 1
    SETTING_UP = 2
    DETECTING_THYMIO = 3
    PLANNING_PATH = 4
    DONE = 5

# Depends on your hardware! (built in laptop cameras are usually 0)

cap = None
camera_state = CameraState.WAITING

# Map and obstacle detection variables
max_width, max_height = 1170, 735
#max_width,max_height = 600,840
coord_to_transform = []
pts2 = []

# Grid settıng
cell_size = 20
start_grid = () 
end_grid = ()
grid = np.array([])
path_grid = np.array([])
metric_padding = 70

start = None # Path for local navigation
end = None # Path for local navigation

# Thymio variables

def camera_handle_keys():
    global camera_state
    ''' Keyboard options '''    
    key = cv2.waitKey(24)

    if key == ord('q'):
        print("Quitting...")
        raise Quit
    elif key == ord('p'): # Press p to prepare the map and obstacles
        print("Capturing map...")
        camera_state = CameraState.CAPTURING_DATA
    elif key == ord('d'): # Press d to detect the Thymio and start the path planning
        camera_state = CameraState.DETECTING_THYMIO
        print('Detecting Thymio...')

def CameraLoop(shared):
    global camera_state, max_height, max_width, coord_to_transform, cell_size, start_grid, end_grid, grid, path_grid, start, end, pts2
    global grid,map_img, obstacle_masks
    ret, frame = cap.read()
    if not ret:
        print("Unable to capture video")
        raise Quit
    
    # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    map_img = frame.copy()
    map_copy = map_img.copy()
    binary_img = preprocess_image(frame)
    try:
        # Handle the 5 states of the camera.
        if camera_state == CameraState.CAPTURING_DATA:
            capture_map, coord_to_transform, map_img, pts2 = capture_map_data(frame, binary_img, max_width, max_height)
            
            if capture_map:
                map_img = cv2.resize(map_img, (max_width, max_height))
                obstacle_masks = capture_obstacle_data(map_img)
                print("Map and obstacles captured!")

                camera_state = CameraState.SETTING_UP

        elif camera_state.value >= CameraState.SETTING_UP.value: #setting up, detecting thymio, and planning path
            M = cv2.getPerspectiveTransform(coord_to_transform, pts2)
            map_img = cv2.warpPerspective(frame, M, (max_width, max_height))
            
            end = get_goal_position(map_img)
            # print("End", end)

            if camera_state.value >= CameraState.DETECTING_THYMIO.value:
                map_img = cv2.resize(map_img, (max_width, max_height)) 
                shared.thymio_position, shared.thymio_angle = get_thymio_info(map_img)
                # print(f'Position: {thymio_position}, Angle: {thymio_angle}')
                if shared.thymio_position is not None:
                    draw_thymio_position(map_img, shared.thymio_position)

                if not(shared.thymio_position is None) and camera_state == CameraState.DETECTING_THYMIO:
                    camera_state = CameraState.PLANNING_PATH
            
            if camera_state == CameraState.PLANNING_PATH:

                start = shared.thymio_position

                ''' Path planning '''
                map_img = cv2.resize(map_img, (max_width, max_height))
                grid, path_grid, simplified_path, shared.metric_path , map_copy = make_path(map_img, obstacle_masks, cell_size, start, end, grid, 
                metric_padding ,max_width, max_height)

                print(shared.metric_path)
                camera_state = CameraState.DONE
                

            # Local navigatıon code 

            draw_node(map_img, start, (0, 73, 255)) # <- Start node
            draw_node(map_img, end, (255, 255, 0)) # <- End node

            try:
                robotpoint = (1000*shared.robot.odometry.x, -1000*shared.robot.odometry.y)
                draw_node(map_img, robotpoint, (0, 0, 0))
                
                #draw_line(map_img,robotpoint,shared.thymio_angle,100,(0,0,0))
                
                #draw_line(map_img,robotpoint,-shared.heading-shared.robot.odometry.angle,100,(255,255,255))
            except Exception as e:
                pass
            
            shared.end = end

            map_img = draw_grid_on_map(map_img, grid, cell_size)
            if path_grid is not None:
                map_img = draw_grid_path(map_img, grid, path_grid, cell_size)
                cv2.imshow('A* Exploration', map_copy)
            map_img = cv2.resize(map_img, (max_width-100,max_height-100))

            cv2.imshow('Map', map_img)
            
            

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, 'Thymio: ' + str(shared.thymio_position), (10, 30), font, 0.5, (0, 0, 0), 2)
        cv2.putText(frame, 'Goal: ' + str(end), (10, 60), font, 0.5, (0, 0, 0), 2) 
        cv2.putText(frame, 'Angle degrees: ' + str(shared.thymio_angle), (10, 90), font, 0.5, (0, 0, 0), 2)

        cv2.imshow('Original image', frame)
        camera_handle_keys()

    except Exception as e:
        raise e

def CameraClose():
    global cap
    cap.release()
    cv2.destroyAllWindows()

def CameraInit():
    global cap
    cap = cv2.VideoCapture(CAMERA_ID)

if __name__ == "__main__":
    CameraInit()
    # replace loop with return
    try:
        while True:
            CameraLoop()
    except Quit:
        pass
    CameraClose()