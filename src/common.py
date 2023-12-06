import numpy as np
class Quit(SystemExit):
    pass

class SharedData():
    robot = None
    thymio_angle = 0
    thymio_position = (0,0)
    metric_path = []
    path_shared = []
    end = (0,0)
    heading = 0

shared = SharedData()

def get_shared():
    global shared
    return shared
def set_shared(s):
    global shared
    shared.robot = s.robot
    shared.thymio_position = s.thymio_position
    shared.thymio_angle = s.thymio_angle
    shared.metric_path = s.metric_path
    shared.path_shared = s.path_shared
    shared.end = s.end
    shared.heading = s.heading

def convert_camera_to_robot(position = None, angle = None, path = None):
    if angle is not None: angle = -angle 
    if position is not None: 
        position = (position[0]/1000.0, -position[1]/1000.)
    if path is not None:
        path = np.array([(p[0]/1000.0, -p[1]/1000.) for p in path])
    
    return position, angle, path
