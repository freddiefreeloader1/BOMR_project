from tdmclient import ClientAsync
from path_following import Robot, get_angle_to
import math
robot = Robot(0,0,0,[(0, 0), (1, 1),(2,1),(3,3)])

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
    
    print("TARGET: {:.2f}, {} ROBOT: {:.2f}, {:,2f} angle - {:.2f}".format(angle,point,robot.odometry.x,robot.odometry.y,robot.odometry.angle))
    # SPEED CONSTANTS
    forward_speed = 100
    steer_gain = 70
    steer_max = 70

    steer = steer_gain * angle
    node.send_set_variables(motors(int(-steer + forward_speed ), int( steer + forward_speed)))

# Async sensor reading update
def on_variables_changed(node, variables):
    try:
        #TODO: proximity values
        pass
    except KeyError:
        pass  # prox.horizontal not updated

    try:
        global robot
        try:
            left_speed = variables["motor.left.speed"][0]
        except:
            left_speed = 0
        try:
            right_speed = variables["motor.right.speed"][0]
        except KeyError:
            right_speed = 0
    
        #ODOMETRY CONSTANTS
        robot_diameter_m = 0.1
        speed_to_m = 0.01
        motor_read_freq = 100

        #Update Odometry:
        robot.odometry.x += (math.cos(robot.odometry.angle)*speed_to_m*(left_speed + right_speed)/2)/motor_read_freq
        robot.odometry.y += (math.sin(robot.odometry.angle)*speed_to_m*(left_speed + right_speed)/2)/motor_read_freq #speed in cm to robot pos in meters
        dtheta = (right_speed - left_speed) * speed_to_m / robot_diameter_m
        robot.odometry.angle = (robot.odometry.angle + dtheta / motor_read_freq) % (2*math.pi)
    except KeyError:
        pass  # motors not updated

with ClientAsync() as client:
    async def prog():
        global robot
        with await client.lock() as node:
            #Set up listener functions
            await node.watch(variables=True)
            node.add_variables_changed_listener(on_variables_changed)
            while True:
                # path follow loop:

                point = await robot.path_follower.getLookaheadEdge(robot.odometry)

                steer(node, robot, point)

                await client.sleep(0.05)
    
    client.run_async_program(prog)