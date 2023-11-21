# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

# womp womp i use numpy :/
import numpy as np

# These are classes which store data.
class Odometry:
    x = None
    y = None
    angle = None
    speed_l = None
    speed_r = None
    def __init__(self, x=None,y=None,angle=None,speed_l=None,speed_r=None):
        self.x = x
        self.y = y
        self.angle = angle
        self.speed_l = speed_l
        self.speed_r = speed_r

# This class has a function to retrieve data, 
# get() ~ which returns None if nothing has changed since the last reading
class Stream:
    data = Odometry()
    is_data_new = False
    def __init__(self):
        pass

    def get_last(self):
        return self.data
    def get(self):
        # UPDATE THE DATA
        if not self.is_data_new:
            return None
        else:
            return self.data
        
# Stores function follow()
# this  is its own class so i can store 
class PathFollow:
    path = None
    path_lookahead = 0.2
    def __init__(self, path, path_lookahead = 0.2):
        self.path = path
        self.path_lookahead = path_lookahead
    
    def getClosestEdge(self, odometry):
        best_distance = 999999
        best_index = -1
        for i, j, index in zip(self.path, self.path[1:], range(len(self.path)-1)):
            p1 = np.asarray(i)
            p2 = np.asarray(j)
            point = np.asarray((odometry.x, odometry.y))

            vecpath = p2 - p1
            pr = np.asarray((odometry.x - p1[0], odometry.y - p1[1])) #relative point to vector

            distance = None
            projection = None
            # Percentage on the current line
            t = (pr[0] * vecpath[0] + pr[1] * vecpath[1])/(vecpath[1]*vecpath[1] + vecpath[0]*vecpath[0])
            if(t < 0):
                projection = p1
                distance = np.linalg.norm(point - p1)
            elif (t>1):
                distance = np.linalg.norm(point - p2)
            else:
                distance = np.abs(np.cross(p2-p1, p1-point)) / np.linalg.norm(p2-p1)
            
            if distance <= best_distance:
                best_distance = distance
                best_index = index
                best_projection = ((t.),t)
            
        return *best
            
            


class Robot:
    odometry = Odometry()
    path_follower = None
    def __init__(self, x = 0, y = 0, angle = 0, path = [(0,0),(1,1)]):
        self.odometry = Odometry(x,y,angle)
        self.path_follower = PathFollow(path)
    

robot = Robot(1,0,0,[(0,0),(1,1),(2,1)])
print(robot.path_follower.getClosestEdge(robot.odometry))
    