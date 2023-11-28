import numpy as np
from filterpy.kalman import KalmanFilter

# This function takes a body's reading and angle
def change_frame(body_data,body_orientation):
    R = np.array([[np.cos(body_orientation), -np.sin(body_orientation)],
                  [np.sin(body_orientation), np.cos(body_orientation)]])
    accel_local = R.dot(body_data)
    return accel_local


class Kalman:
    t_position_x = 0
    kf_pos = KalmanFilter(dim_x=6, dim_z=2)
    kf_rot = KalmanFilter(dim_x=2,dim_z=1)
    def __init__(self, position = [0,0], heading = 0, acceleration = [0,0], velocity = [0,0], spin = 0):
        
        self.kf_pos.x = np.array([position[0], position[1],  velocity[0], velocity[1], acceleration[0],acceleration[1]])
        self.kf_rot.x = np.array([heading, spin])

        self.kf_pos.P = 100*np.eye(6)
        self.kf_rot.P = 100*np.eye(2)

    def get_rotation():
        return None
    
    def get_position():
        return None
    
    def _compute_kf_pos(self,time):
        pass
    def _compute_kf_rot(self,time):
        pass
    # Update the accelerometor readings from the robot
    def update_acceleration(self,data, time):
        
        self._compute_kf_pos()

    # Update the change in heading from the last update
    # (defacto, this is calculated using the wheel speeds)
    def update_spin(self,data, time):
        pass
    
    # Update the absolute reading of the robots angle in the world
    def update_heading(self,data, time):
        pass

    # Update the absolute position of the robot in the world
    def update_position(data, time):
        pass

kalman = Kalman()

kalman.getAngle()
kalman.getPos()

