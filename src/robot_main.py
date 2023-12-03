from main import Quit
from tdmclient import ClientAsync
from path_following import get_angle_to, Odometry,PathFollow,PID
from kalman import Kalman
import time
from robot import *

import math

# Thymio classes


robot = Robot(0,0,0,[(0, 0), (1, 0),(1,1),(3,1)])

with ClientAsync() as client:
    async def prog():
        global robot
        with await client.lock() as node:
            #Set up listener functions
            await node.watch(variables=True)
            node.add_variables_changed_listener(on_variables_changed)
            while True:
                # Update odometry:
                robot.update_odometry()

                # path follow loop:

                point, _ = robot.path_follower.getLookaheadEdge(robot.odometry)
                if(robot.state == RobotState.FOLLOWING_PATH):
                    steer(node, robot, point)
                elif(robot.state == RobotState.AVOIDING_WALLS):
                    steer_danger(node,robot)
                elif(robot.state == RobotState.STOPPED):
                    node.send_set_variables(motors(0,0))
                robot.state_timer -= 1
                if(robot.state_timer < 0 and robot.state != RobotState.STOPPED):
                    robot.state == RobotState.FOLLOWING_PATH
                
                if(robot.path_follower.current_edge >= len(robot.path_follower.path)-1):
                    await node.set_variables(motors(0,0))
                    break
                await client.sleep(0.05)
    
    client.run_async_program(prog)