
from robot_drive.path_following import get_angle_to, Odometry,PathFollow
from kalman.kalman import Kalman
import time
from enum import Enum
from util.common import get_shared, set_shared
from util.constants import *

import math
start_time = time.time()

def get_time():
    return (time.time() - start_time)

class RobotState(Enum):
    FOLLOWING_PATH = 0,
    AVOIDING_WALLS = 1,
    AVOIDING_WALLS_COOLDOWN = 2,
    STOPPED = 3
class Robot:
    odometry = Odometry()
    path_follower = None
    kalman = None
    state = RobotState.FOLLOWING_PATH
    state_timer = 0


    def __init__(self, x = 0, y = 0, angle = 0, path = [(0,0),(2,0)]):
        self.odometry = Odometry(x,y,angle)
        self.path_follower = PathFollow(path)
        self.kalman = Kalman([x,y],angle,[0,0],[0,0],0,get_time())



    def update_odometry(self):
        self.odometry.x, self.odometry.y = self.kalman.get_position()
        self.odometry.angle = self.kalman.get_rotation()
        

### ---- HELPER FUNCTIONS FOR THYMIO ---- ###
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }
def change_acceleration(acc):
    return acc*ACCELERATION_SENSOR_TO_MPSS

def change_velocity(vel):
    vel = MOTOR_SENSOR_TO_MPS*vel
    return vel

### ---- ROBOT CODE ---- ###

#Steer the robot to a point
def steer(node, robot ,point):
    angle = get_angle_to(robot.odometry,point)
    
    # SPEED CONSTANTS
    forward_speed = ROBOT_FOLLOW_FORWARD_SPEED
    steer_gain = ROBOT_FOLLOW_STEER_AMOUNT

    steer = steer_gain * angle
    
    node.send_set_variables(motors(int(-steer + forward_speed ), int( steer + forward_speed)))
def steer_danger(node,robot):
    prox = node.v.prox.horizontal
    # STEER CONSTANTS
    speed = ROBOT_AVOID_FORWARD_SPEED
    obst_gain = ROBOT_AVOID_SENSOR_GAIN
    obst_rescind = ROBOT_AVOID_SENSOR_RESCIND
    obst_stop = ROBOT_AVOID_SENSOR_STOP

    touching_wall = any([p > PROX_TOUCHING_THRESHOLD for p in prox])

    lprox, mprox, rprox = get_proximity_sides(prox)
    lprox, mprox, rprox = lprox//100, mprox//100, rprox//100
    
    back = mprox * obst_stop + touching_wall * (ROBOT_AVOID_TOUCHING_WALL)

    lspeed = int(speed + obst_gain * lprox - back - obst_rescind * rprox)
    rspeed = int(speed + obst_gain * rprox - back - obst_rescind * lprox)
    node.send_set_variables(motors(lspeed,rspeed))

def get_proximity_sides(prox):
    left = prox[0]*PROXIMITIY_SMOOTHING + prox[1]*(1-PROXIMITIY_SMOOTHING)
    mid = prox[2]*PROXIMITIY_SMOOTHING + prox[1]*(1-PROXIMITIY_SMOOTHING)/2 + prox[3]*(1-PROXIMITIY_SMOOTHING)/2
    right = prox[4]*PROXIMITIY_SMOOTHING + prox[3]*(1-PROXIMITIY_SMOOTHING)
    return left, mid, right

# Async sensor reading update
def on_variables_changed(node, variables):
    shared = get_shared()
    try:
        #Proximity has been updated
        prox = variables["prox.horizontal"]

        lprox, mprox, rprox = get_proximity_sides(prox)
        # PROXIMITY CONSTANTS
        obstL = PROX_DANGER_MIN
        obstH = PROX_DANGER_MAX
        
       # print(prox[0],prox[4],state,state_timer)
        # handle states
        if(lprox > obstH or rprox > obstH or mprox > obstH):
            shared.robot.state = RobotState.AVOIDING_WALLS
        
        elif(lprox < obstL and rprox < obstL and mprox < obstL):
            if(shared.robot.state == RobotState.AVOIDING_WALLS):
                shared.robot.state_timer = STATE_COOLDOWN
                shared.robot.state = RobotState.AVOIDING_WALLS_COOLDOWN

        

    except KeyError:
        pass  # prox.horizontal not updated

    try:
        try:
            left_speed = variables["motor.left.speed"][0]
        except:
            left_speed = 0
        try:
            right_speed = variables["motor.right.speed"][0]
        except KeyError:
            right_speed = 0
            if(left_speed == 0):
                raise KeyError  #if neither was updated, skip

        #Update Odometry:
        dtheta = (right_speed - left_speed)*MOTOR_SENSOR_TO_SPINS
        
        shared.robot.kalman.update_spin(data=dtheta,time=get_time())
        

        avr_speed = (right_speed + left_speed)/2
        avr_speed = change_velocity(avr_speed)
        speed = [avr_speed,0]
        shared.robot.kalman.update_velocity(data = speed,time=get_time())
        
    
    except KeyError:
        pass  # motors not updated

    try:
        acc_sensor = variables["acc"]
        acc = [0,0]
        acc[0] = - change_acceleration(acc_sensor[1])
        acc[1] =   change_acceleration(acc_sensor[0])
        shared.robot.kalman.update_acceleration(data=acc[0:2],time=get_time())
    except KeyError:
        pass # acceleration not updated
    try:
        b = variables["button.center"]
        if(b[0]):
            shared.robot.state = RobotState.FOLLOWING_PATH if (shared.robot.state == RobotState.STOPPED) else RobotState.STOPPED
    except KeyError:
        pass 
    set_shared(shared)

