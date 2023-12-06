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
    
    steer(node, shared.robot, point)
    
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