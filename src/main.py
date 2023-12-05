from computer_vision import *
from robot_main import robot, init_robot_position, get_time
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit, SharedData

# This is a simple function that hard sets the robots data.
# TODO: initialize the kalman after this step, perhaps create a 'RobotStart(pos)'

def convert_camera_to_robot(position = None, angle = None, path = None):
    if angle is not None: angle = -angle
    if position is not None: 
        position = (position[0]/1000.0,-position[1]/1000.)
    if path is not None:
        path = np.array([(p[0]/1000.0,-p[1]/1000.) for p in path])
    
    return position, angle, path


def main():
    shared = SharedData()
    CameraInit()
    RobotInit()
    # replace loop with return
    try:
        while True:
            # Run the camera loop
            CameraLoop(shared)
            # If the camera has data for the robot, update it.
            if((shared.robot is None and len(shared.metric_path) > 0)):
                print("> Updating robot position")
                init_robot_position(shared,convert_camera_to_robot(shared.thymio_position,shared.thymio_angle,shared.metric_path)) #CAMERA ANGLE IS CLOCKWISE!!!
            
            if(shared.thymio_position is not None and shared.robot is not None):
                new_pos, new_angle, _ = convert_camera_to_robot(shared.thymio_position, shared.thymio_angle)
                robot.kalman.update_position(new_pos, get_time())
                robot.kalman.update_heading(new_angle, get_time())
                shared.thymio_position = None

            # If the robot was given a path, start running.
            if(len(shared.metric_path) > 0 and not(shared.robot is None)):
                RobotLoop(shared)

    except Quit:
        pass

    # A quit has been called (unlock that thymio!!!!)
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()