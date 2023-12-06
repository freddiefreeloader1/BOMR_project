from tdmclient import ClientAsync, aw
from robot import *
from common import Quit, SharedData, shared, convert_camera_to_robot

import math

# Thymio classes

client = ClientAsync()
node = None

def init_robot_position(shared,pos,angle,path):
    shared.robot = Robot(pos[0],pos[1],angle,path)

def RobotInit():
    global client,node
    node = aw(client.wait_for_node())
    aw(node.lock())

    #Set up listener functions
    aw(node.watch(variables=True))
    node.add_variables_changed_listener(on_variables_changed)
    node.send_set_variables(motors(0,0))


def RobotLoop(shared):
    global client,node
# Update odometry:
    shared.robot.update_odometry()

    # path follow loop:
    point, _ = shared.robot.path_follower.getLookaheadEdge(shared.robot.odometry)

    shared.path_shared.append(point)
    
    if(shared.robot.state == RobotState.FOLLOWING_PATH):
        steer(node, shared.robot, point)
    elif(shared.robot.state == RobotState.AVOIDING_WALLS):
        steer_danger(node,shared.robot)
    elif(shared.robot.state == RobotState.STOPPED):
        node.send_set_variables(motors(0,0))
    shared.robot.state_timer -= 1
    if(shared.robot.state_timer < 0 and shared.robot.state != RobotState.STOPPED):
        shared.robot.state == RobotState.FOLLOWING_PATH
        steer(node, shared.robot, point)
    
    if(shared.robot.path_follower.current_edge >= len(shared.robot.path_follower.path)-1):
        node.send_set_variables(motors(0,0))
        raise Quit
    
    aw(client.sleep(0.02))

def RobotClose():
    global node
    node.unlock()

async def RobotAll(shared):
    global client,node
    with await client.lock() as node2:
        node = node2
        
        await node.watch(variables=True)
        node.add_variables_changed_listener(on_variables_changed)

        while True:
            RobotLoop(shared)
        

if __name__ == "__main__":
    RobotInit()
    s = SharedData()
    s.robot = Robot(0,0)
    set_shared(s)
    try:
        while True:
            RobotLoop(shared)
    except Quit:
        pass
    RobotClose()

    print("out")
    RobotClose()
    exit(0)