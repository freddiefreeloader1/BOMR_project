from tdmclient import ClientAsync
from path_following import Robot
import math
robot = Robot(0,0,0,[(0, 0), (1, 1), (2, 1)])
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

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
    
        print(robot.odometry.x, robot.odometry.y, robot.odometry.angle)
        robot_diameter_m = 0.1
        speed_to_m = 0.01
        motor_read_freq = 100

        robot.odometry.x += (math.cos(robot.odometry.angle)*speed_to_m*(left_speed + right_speed)/2)/motor_read_freq
        robot.odometry.y += (math.sin(robot.odometry.angle)*speed_to_m*(left_speed + right_speed)/2)/motor_read_freq #speed in cm to robot pos in meters
        dtheta = (right_speed - left_speed) * speed_to_m / robot_diameter_m
        robot.odometry.angle += dtheta / motor_read_freq # TODO %2PI
    except KeyError:
        pass  # motors not updated

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(on_variables_changed)
            while True:
                await client.sleep(0.05)
    
    client.run_async_program(prog)