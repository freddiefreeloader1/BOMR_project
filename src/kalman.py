import numpy as np
from filterpy.kalman import KalmanFilter

#Change from body frame to local frame
def change_frame(body_data,body_orientation):
    '''
    Change from body frame to local frame.

    Parameters:
    - body_data: Data expressed in the body frame.
    - body_orientation: Orientation of the robot in the local frame (from kf_rot).

    Returns:
    Data expressed in the local frame.
    
    '''
    R = np.array([[np.cos(body_orientation), -np.sin(body_orientation)],
                  [np.sin(body_orientation), np.cos(body_orientation)]])
    accel_local = R.dot(body_data)
    return accel_local

#take into account the offset of the kalman 
def convert_angle(data,current_angle):
    if (current_angle<2*np.pi and current_angle > -2*np.pi): shift = 0
    else: shift = (current_angle)//(2*np.pi)
    return data+shift*(2*np.pi)


class Kalman:
    time_pos = 0
    time_rot = 0
    kf_pos = KalmanFilter(dim_x=6, dim_z=2)
    kf_rot = KalmanFilter(dim_x=2,dim_z=1)

    #save measurements
    accel_measurement = []
    vel_measurement = []
    spin_measurement = []

    #noise components 
    ACCEL_NOISE = 9.81/23
    VEL_NOISE = 0.001
    ROT_NOISE = 0.005
    POS_NOISE = 0.005
    SPIN_NOISE = 0.1

    def __init__(self, position = [0,0], heading = 0, acceleration = [0,0], velocity = [0,0], spin = 0, time = 0):
        self.time_pos = self.time_rot = time

        #starting position
        self.kf_pos.x = np.array([position[0], position[1],  velocity[0], velocity[1], acceleration[0],acceleration[1]])
        self.kf_rot.x = np.array([heading, spin])

        #trust in starting position
        self.kf_pos.P = 100*np.eye(6)
        self.kf_rot.P = 100*np.eye(2)

    #getter for kalman
    def get_rotation(self):
        data = self.kf_rot.x[0]
        data = data %(2*np.pi)
        return data
    
    def get_velocity(self):
        return self.kf_pos.x[2:4]
    
    def get_acceleration(self):
        return self.kf_pos.x[4:6]
    
    def get_position(self):
        return self.kf_pos.x[0:2]
    def get_spin(self):
        return self.kf_rot.x[1]
    
    def get_accel_meas(self):
        return self.accel_measurement
    
    def get_vel_meas(self):
        return self.vel_measurement
    
    def get_spin_meas(self):
        return self.spin_measurement
        
     
    #Update the F matrix, Q matrix and update the measurement for the position kalman
    def _compute_kf_pos(self,data, dt):
        #get accel_noise for process noise matrix
        noise = self.ACCEL_NOISE # most dominant solution
        
        #compute F according to dt
        self.kf_pos.F = np.array([[1,0,dt,0,0.5*dt**2,0],
                     [0,1,0,dt,0,0.5*dt**2],
                     [0,0,1,0,dt,0],
                     [0,0,0,1,0,dt],
                     [0,0,0,0,1,0],
                     [0,0,0,0,0,1]])
        
        #compute Q according to dt 
        self.kf_pos.Q = np.array([
        [1/4 * noise**2 * dt**4, 0, 0, 0, 0, 0],
        [0, 1/4 * noise**2 * dt**4, 0, 0, 0, 0],
        [0, 0, noise**2 * dt**2, 0, 0, 0],
        [0, 0, 0, noise**2 * dt**2, 0, 0],
        [0, 0, 0, 0, noise**2, 0],
        [0, 0, 0, 0, 0, noise**2]])

        #predict and update kalman
        self.kf_pos.predict()
        self.kf_pos.update(data)

    #Update the F matrix, Q matrix and update the measurement for the rotation kalman
    def _compute_kf_rot(self,data,dt):

        #get accel_noise for process noise matrix
        noise = self.SPIN_NOISE # most dominant solution
        
        #compute F according to dt
        self.kf_rot.F = np.array([[1,dt],
                                  [0,1]])
        
        #compute Q according to dt
        self.kf_rot.Q = np.array([
                [ noise*2 * dt**2, 0],
                [0, noise**2]])

        #predict and update kalman
        self.kf_rot.predict()
        self.kf_rot.update(data)

    # Update the accelerometor readings from the robot
    def update_acceleration(self,data,time):
       
        # calculate time since last measurement and update it
        dt = time- self.time_pos
        self.time_pos = time
        rotation = self.kf_rot.x.T[0]

        # Transform body frame to local frame
        data = change_frame(data, rotation)
        self.accel_measurement.append(data)

        # Update kf parameters to accept accel measurement
        self.kf_pos.H = np.array([[0,0,0,0,1,0],
                                  [0,0,0,0,0,1]])
        self.kf_pos.R = (self.ACCEL_NOISE**2)*np.eye(2)

        #update kalman
        self._compute_kf_pos(data,dt)

    def update_velocity(self,data,time):
       
        # calculate time since last measurement and update it
        dt = time- self.time_pos
        self.time_pos = time
        rotation = self.kf_rot.x.T[0]

        # Transform body frame to local frame
        data = change_frame(data, rotation)
        #save measurement
        self.vel_measurement.append(data)
        # Update kf parameters to accept velocity measurement
        self.kf_pos.H = np.array([[0,0,1,0,0,0],
                                  [0,0,0,1,0,0]])
        self.kf_pos.R = (self.VEL_NOISE**2)*np.eye(2)

        #update kalman
        self._compute_kf_pos(data,dt)

    # Update the change in heading from the last update
    def update_spin(self,data, time):

        # calculate time since last measurement and update it
        dt = time- self.time_rot
        self.time_rot = time

        # save measurement
        self.spin_measurement.append(data)

        # Update kf parameters to accept spin measurement
        self.kf_rot.H = np.array([[0,1]])
        self.kf_rot.R[0,0] = self.SPIN_NOISE**2

        # Update kf
        self._compute_kf_rot(data,dt)
    
    # Update the absolute reading of the robots angle in the world
    def update_heading(self,data, time):

        #commented because only needed if big turn and can cause issues
        #data = convert_angle(data,self.kf_rot.x[0])
        #current_angle = self.get_rotation()

        # calculate time since last measurement and update it
        dt = time- self.time_rot
        self.time_rot = time

        # Update kf parameters to accept spin measurement
        self.kf_rot.H = np.array([[1,0]])
        self.kf_rot.R[0,0] = self.ROT_NOISE**2

        # Updaye kf
        self._compute_kf_rot(data,dt)

    # Update the absolute position of the robot in the world
    def update_position(self, data, time):
        
        # calculate time since last measurement and update it
        dt = time- self.time_pos
        self.time_pos = time

        # Update kf parameters to accept measurement position
        self.kf_pos.H = np.array([[1,0,0,0,0,0],
                                  [0,1,0,0,0,0]])
        self.kf_pos.R = (self.POS_NOISE**2)*np.eye(2)

        # Update kf
        self._compute_kf_pos(data,dt)


