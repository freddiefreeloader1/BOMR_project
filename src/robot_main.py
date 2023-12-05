from tdmclient import ClientAsync
from robot import *
from common import Quit

import math

# Thymio classes

client = ClientAsync()
node = None

async def _robot_init():    
    global client,node
    node = await client.wait_for_node()
    await node.lock()

    #Set up listener functions
    await node.watch(variables=True)
    node.add_variables_changed_listener(on_variables_changed)


def RobotInit():
    client.run_async_program(_robot_init)

def RobotLoop():
    global robot,client,node
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
        node.set_variables(motors(0,0))
        raise Quit

def RobotClose():
    global node
    node.unlock()

async def RobotAll():
    global client,node
    with await client.lock() as node2:
        node = node2
        
        await node.watch(variables=True)
        node.add_variables_changed_listener(on_variables_changed)

        while True:
            RobotLoop()
            await client.sleep(0.05)
        

if __name__ == "__main__":
    RobotInit()
    try:
        while True:
            RobotLoop()
    except Quit:
        pass
    RobotClose()

    print("out")
    RobotClose()
    exit(0)