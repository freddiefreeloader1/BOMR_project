from computer_vision import *
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop

class Quit(SystemExit):
    pass

def main():
    CameraInit()
    # replace loop with return
    try:
        while True:
            CameraLoop()
    except Quit:
        pass
    CameraClose()

if __name__ == "__main__":
    main()