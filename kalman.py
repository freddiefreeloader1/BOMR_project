import numpy as np
from filterpy.kalman import KalmanFilter

# This function takes a body's reading and angle
def change_frame(body_data,body_orientation):
    R = np.array([[np.cos(body_orientation), -np.sin(body_orientation)],
                  [np.sin(body_orientation), np.cos(body_orientation)]])
    accel_local = R.dot(body_data)
    return accel_local


class Kalman:

    def __init__(self):
        pass

    def get():
        return None
    
    # Update the accelerometor readings from the robot
    def update_acceleration(data, time):
        pass

    # Update the change in heading from the last update
    # (defacto, this is calculated using the wheel speeds)
    def update_spin(data, time):
        pass
    
    # Update the absolute reading of the robots angle in the world
    def update_heading(data, time):
        pass

    # Update the absolute position of the robot in the world
    def update_position(data, time):
        pass
   