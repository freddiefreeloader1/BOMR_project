from tdmclient import ClientAsync
from robot import *

import math

# Thymio classes


robot = Robot(0,0,0,[(0, 0), (1, 0),(1,1),(3,1)])
client = ClientAsync()
node = None

def RobotInit():
    async def prog():    
        global client,node
        node = await client.wait_for_node()
        await node.lock()

        #Set up listener functions
        await node.watch(variables=True)
        await node.add_variables_changed_listener(on_variables_changed)

    client.run_async_program(prog)

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
        exit(1)

def RobotClose():
    global node
    node.unlock()

if __name__ == "__main__":
    RobotInit()
    # replace loop with return
    try:
        while True:
            RobotLoop()
            time.sleep(0.01)
    except Exception:
        pass
    RobotClose()