from computer_vision import *
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop

def main():
    global robot, metric_path
    CameraInit()
    RobotInit()
    # replace loop with return
    try:
        while True:
            CameraLoop()
            if(robot.path_follower.path != metric_path):
                robot.path_follower.path = metric_path
            if(len(metric_path) > 0):
                RobotLoop()
    except SystemExit:
        pass
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()