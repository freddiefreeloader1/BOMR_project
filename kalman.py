import numpy as np
from filterpy.kalman import KalmanFilter

# This function takes a body's reading and angle
def change_frame(body_data,body_orientation):
    R = np.array([[np.cos(body_orientation), -np.sin(body_orientation)],
                  [np.sin(body_orientation), np.cos(body_orientation)]])
    accel_local = R.dot(body_data)
    return accel_local


class Kalman:
    time_pos = 0
    time_rot = 0
    kf_pos = KalmanFilter(dim_x=6, dim_z=2)
    kf_rot = KalmanFilter(dim_x=2,dim_z=1)

    ACCEL_NOISE = 1
    def __init__(self, position = [0,0], heading = 0, acceleration = [0,0], velocity = [0,0], spin = 0, time = 0):
        self.time_pos = self.time_rot = time
        self.kf_pos.x = np.array([position[0], position[1],  velocity[0], velocity[1], acceleration[0],acceleration[1]])
        self.kf_rot.x = np.array([heading, spin])

        self.kf_pos.P = 100*np.eye(6)
        self.kf_rot.P = 100*np.eye(2)

    def get_rotation():
        return None
    
    def get_position(self):
        return self.kf_pos.x.T[0:1]
    
    def _compute_kf_pos(self,data, dt):
        noise = self.NOISE_ACCEL # most dominant solution
        
        self.kf_pos.F = np.array([[1,0,dt,0,0.5*dt**2,0],
                     [0,1,0,dt,0,0.5*dt**2],
                     [0,0,1,0,dt,0],
                     [0,0,0,1,0,dt],
                     [0,0,0,0,1,0],
                     [0,0,0,0,0,1]])
        
        self.kf_pos.Q = np.array([
        [1/4 * noise**2 * dt**4, 0, 0, 0, 0, 0],
        [0, 1/4 * noise**2 * dt**4, 0, 0, 0, 0],
        [0, 0, noise**2 * dt**2, 0, 0, 0],
        [0, 0, 0, noise**2 * dt**2, 0, 0],
        [0, 0, 0, 0, noise**2, 0],
        [0, 0, 0, 0, 0, noise**2]])

        self.kf_pos.predict()
        self.kf_pos.update(data)

    def _compute_kf_rot(self,time):
        pass
    # Update the accelerometor readings from the robot
    def update_acceleration(self,data, rotation, time):

        # calculate time since last measurement and update it
        dt = time- self.time_pos
        self.time_pos = time

        # Transform data to local
        data = change_frame(data, rotation)

        # Update kf parameters for multiplication
        self.kf_pos.H = np.array([[0,0,0,0,1,0],
                                  [0,0,0,0,0,1]])
        self.kf_pos.R = (self.NOISE_ACCEL**2)*np.eye(2)

        self._compute_kf_pos(data,dt)
        


    # Update the change in heading from the last update
    # (defacto, this is calculated using the wheel speeds)
    def update_spin(self,data, time):
        dt = time- self.time_rot
        self.time_rot = time

        pass
    
    # Update the absolute reading of the robots angle in the world
    def update_heading(self,data, time):
        dt = time- self.time_rot
        self.time_rot = time
        pass

    # Update the absolute position of the robot in the world
    def update_position(self, data, time):
        
        # calculate time since last measurement and update it
        dt = time- self.time_pos
        self.time_pos = time

        # Update kf parameters for multiplication
        self.kf_pos.H = np.array([[1,0,0,0,0,0],
                                  [0,1,0,0,0,0]])
        self.kf_pos.R = (self.NOISE_POS**2)*np.eye(2)

        self._compute_kf_pos(data,dt)

kalman = Kalman()

kalman.getAngle()
kalman.getPos()

