from computer_vision import *
from camera_main import thymio_angle, thymio_position, metric_path
from robot_main import robot, init_robot_position, get_time
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit

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

    CameraInit()
    RobotInit()
    # replace loop with return
    try:
        while True:
            
            global robot, metric_path, thymio_position, thymio_angle
            # Run the camera loop
            CameraLoop()
            print(metric_path,thymio_position,thymio_angle)
            # If the camera has data for the robot, update it.
            if((robot is None and len(metric_path) > 0)):
                print("> Updating robot position")
                init_robot_position(convert_camera_to_robot(thymio_position,thymio_angle,metric_path)) #CAMERA ANGLE IS CLOCKWISE!!!
            
            if(thymio_position is not None and robot is not None):
                new_pos, new_angle, _ = convert_camera_to_robot(thymio_position, thymio_angle)
                robot.kalman.update_position(new_pos, get_time())
                robot.kalman.update_heading(new_angle, get_time())
                thymio_position = None

            # If the robot was given a path, start running.
            if(len(metric_path) > 0 and not(robot is None)):
                RobotLoop()

    except Quit:
        pass

    # A quit has been called (unlock that thymio!!!!)
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()