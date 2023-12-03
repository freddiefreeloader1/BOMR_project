from computer_vision import *
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop

def main():
    CameraInit()
    RobotInit()
    # replace loop with return
    try:
        while True:
            CameraLoop()
            RobotLoop()
    except SystemExit:
        pass
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()