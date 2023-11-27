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
    
    # Return:
    # best edge index (bounded)
    # best distance
    # projection from point to that edge
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
            # Create a bounded projection
            if(t < 0):
                projection = p1
            elif (t>1):
                projection = p2
            else:
                projection = (vecpath[0]*t + p1[0], vecpath[1]*t + p1[1])
            
            distance = np.linalg.norm(point-projection)
            
            if distance <= best_distance:
                best_distance = distance
                best_index = index
                best_projection = (projection, t)
            
        return best_index, best_distance, best_projection
    

    def getLookaheadEdge(self, odometry):
        lookahead_left = self.path_lookahead
        current_path_index, distance, projection = self.getClosestEdge(odometry)
        remaining_path_edges = [(p1,p2) for p1, p2 in zip(self.path[current_path_index:], self.path[current_path_index+1:])]
        print(remaining_path_edges)
        point = np.array(projection[0])
        while(lookahead_left > 0 and len(remaining_path_edges)>0):
            curr_edge_remainder = np.linalg.norm(np.array(point)-remaining_path_edges[0][1])
            if(curr_edge_remainder <= lookahead_left):
                lookahead_left -= curr_edge_remainder
                point = remaining_path_edges[0][1]
                remaining_path_edges = remaining_path_edges[1:]
            else:
                path = np.array(remaining_path_edges[0][1]) - remaining_path_edges[0][0]
                point += (lookahead_left*(path))/np.linalg.norm(path)
                lookahead_left = 0
        
        return point, current_path_index

class PID:
    p = 0
    i = 0
    d = 0
    last_current = 0
    i_sum = 0
    def __init__(self, p, i , d):
        self.p = p
        self.i = i
        self.d = d
    def get(self, current,setpoint):
        feedback = (setpoint - current) * self.pg - (current - self.last_current)*self.d + (self.i_sum * self.i)

        i_sum += (current-self.last_current)
        self.last_current = current

        return feedback


class Robot:
    odometry = Odometry()
    path_follower = None
    angle_PID = PID(1,0,0)

    def __init__(self, x = 0, y = 0, angle = 0, path = [(0,0),(1,1)]):
        self.odometry = Odometry(x,y,angle)
        self.path_follower = PathFollow(path)
