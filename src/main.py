from computer_vision import *
from camera_main import thymio_angle, thymio_position, metric_path
from robot_main import robot, init_robot_position
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit

# This is a simple function that hard sets the robots data.
# TODO: initialize the kalman after this step, perhaps create a 'RobotStart(pos)'

def main():
    global robot, metric_path, thymio_position, thymio_angle

    CameraInit()
    RobotInit()
    # replace loop with return
    try:
        while True:
            # Run the camera loop
            CameraLoop()

            # If the camera has data for the robot, update it.
            if(robot.path_follower.path != metric_path):
                init_robot_position(metric_path,thymio_position,-thymio_angle) #CAMERA ANGLE IS CLOCKWISE!!!

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