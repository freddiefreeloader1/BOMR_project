from main import Quit
from tdmclient import ClientAsync
from path_following import get_angle_to, Odometry,PathFollow,PID
from kalman import Kalman
import time
from robot import Robot

import math

robot = Robot(0,0,0,[(0, 0), (1, 0),(1,1),(3,1)])
state = 0
state_timer = 0
is_on = True

def RobotInit():
    pass

def RobotLoop():
    pass

def RobotClose():
    pass
