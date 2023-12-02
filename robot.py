from tdmclient import ClientAsync
from path_following import get_angle_to, Odometry,PathFollow,PID
from kalman import Kalman
import time

import math
start_time = time.time()
is_on = True
#TODO get time
def get_time():
    return (time.time() - start_time)

class Robot:
    odometry = Odometry()
    path_follower = None
    angle_PID = PID(1,0,0)
    kalman = None


    def __init__(self, x = 0, y = 0, angle = 0, path = [(0,0),(8,1)]):
        self.odometry = Odometry(x,y,angle)
        self.path_follower = PathFollow(path)
        self.kalman = Kalman([x,y],angle,[0,0],[0,0],0,get_time())



    def update_odometry(self):
        self.odometry.x, self.odometry.y = self.kalman.get_position()
        self.odometry.angle = self.kalman.get_rotation()

robot = Robot(0,0,0,[(0, 0), (1, 0),(1,1),(3,1)])
state = 0
state_timer = 0

### ---- HELPER FUNCTIONS FOR THYMIO ---- ###
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

### ---- ROBOT CODE ---- ###

#Steer the robot to a point
def steer(node, robot ,point):
    angle = get_angle_to(robot.odometry,point)
    
    print("TARGET: {:.2f}, ROBOT: {:.2f}, {:.2f} angle - {:.2f}".format(robot.path_follower.current_edge,robot.odometry.x,robot.odometry.y,math.degrees(robot.odometry.angle)))
    # SPEED CONSTANTS
    forward_speed = 250
    steer_gain = 150
    steer_max = 70

    steer = steer_gain * angle

    node.send_set_variables(motors(int(-steer + forward_speed ), int( steer + forward_speed)))

def steer_danger(node,robot):
    prox = node.v.prox.horizontal
    # STEER CONSTANTS
    speed = 200
    obst_gain = 12
    obst_rescind = 4
    obst_stop = 15

    back = (prox[2]//100) * obst_stop
    node.send_set_variables(motors(speed + obst_gain * (prox[0] // 100) - back - obst_rescind * (prox[4]//100),speed + obst_gain * (prox[4] // 100) - back - obst_rescind * (prox[0]//100)))

# Async sensor reading update
def on_variables_changed(node, variables):
    
    global robot
    try:
        global state, state_timer
        #Proximity has been updated
        prox = variables["prox.horizontal"]
        # PROXIMITY CONSTANTS
        obstL = 10
        obstH = 20
        STATE_COOLDOWN = 10
       # print(prox[0],prox[4],state,state_timer)
        # handle states
        if(prox[0] > obstH or prox[4] > obstH):
            if(state == 1):
                state_timer = STATE_COOLDOWN
            state = 1
        
        elif(prox[0] < obstL and prox[4] < obstL):
            if(state == 0):
                state_timer = STATE_COOLDOWN
            if(STATE_COOLDOWN <= 0):
                state = 0

        

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
        #ODOMETRY CONSTANTS
        ROBOT_DIAMETER = 0.25
        ENCODER_TO_MPS = 0.01
        MOTOR_READ_FREQ = 10
        constant_spin = 2*math.pi/(4.6*400)
        #Update Odometry:
        dtheta = (right_speed - left_speed)*constant_spin
        #dtheta = 2*math.pi/(4.6*(right_speed - left_speed))
        #2*pi/4.6 = (400)*x 
        robot.kalman.update_spin(data=dtheta,time=get_time())
        
    except KeyError:
        pass  # motors not updated

    try:
        acc = variables["acc"]
        
        robot.kalman.update_acceleration(data=acc[0:2],time=get_time())
    except KeyError:
        pass # acceleration not updated
    try:
        global is_on
        b = variables["button.center"]
        if(b[0]):
            is_on = not is_on
    except KeyError:
        pass 

with ClientAsync() as client:
    async def prog():
        global robot, state, state_timer
        with await client.lock() as node:
            #Set up listener functions
            await node.watch(variables=True)
            node.add_variables_changed_listener(on_variables_changed)
            while True:
                # Update odometry:
                robot.update_odometry()

                # path follow loop:

                point, _ = robot.path_follower.getLookaheadEdge(robot.odometry)
                if(is_on):
                    if(state == 0):
                        steer(node, robot, point)
                    elif(state == 1):
                        steer_danger(node,robot)
                    state_timer -= 1
                    if(state_timer < 0):
                        state = 0
                else:
                    node.send_set_variables(motors(0,0))
                
                if(robot.path_follower.current_edge >= len(robot.path_follower.path)-1):
                    await node.set_variables(motors(0,0))
                    break
                await client.sleep(0.05)
    
    client.run_async_program(prog)