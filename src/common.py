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
