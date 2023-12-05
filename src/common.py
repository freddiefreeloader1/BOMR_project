class Quit(SystemExit):
    pass

class SharedData():
    robot = None
    thymio_angle = 0
    thymio_position = (0,0)
    metric_path = []

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
