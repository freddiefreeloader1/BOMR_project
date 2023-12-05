class Quit(SystemExit):
    pass

class SharedData():
    robot = None
    thymio_position = 0
    thynio_location = (0,0)
    metric_path = []
