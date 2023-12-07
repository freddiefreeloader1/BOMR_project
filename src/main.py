from computer_vision import *
from robot_main import init_robot_position, get_time
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit, SharedData,shared,convert_camera_to_robot,plot_data

import matplotlib.pyplot as plt
import numpy as np

# This is a simple function that hard sets the robots data.
# TODO: initialize the kalman after this step, perhaps create a 'RobotStart(pos)'


def main():
    global shared
    RobotInit()
    CameraInit()

    kalman_history = []
    absolute_history = []
    kalman_history_orientation = []
    kalman_history_acce = []
    kalman_history_vel = []
    kalman_history_spin = []
    absolute_orientation = []
    # replace loop with return
    try:
        while True:
            # Run the camera loop
            CameraLoop(shared)

            # If the camera has data for the robot, update it.
            if((shared.robot is None and len(shared.metric_path) > 0)):
                a,b,c = convert_camera_to_robot(shared.thymio_position,shared.thymio_angle,shared.metric_path)
                init_robot_position(shared,a,b,c) 

            #If we have new position data update the kalman
            if(shared.thymio_position is not None and shared.robot is not None):
                new_pos, new_angle, _ = convert_camera_to_robot(shared.thymio_position, shared.thymio_angle)
                absolute_history.append(new_pos)
                absolute_orientation.append(new_angle)
                
                shared.robot.kalman.update_position(new_pos, get_time())
                shared.robot.kalman.update_heading(new_angle, get_time())
                shared.thymio_position = None

            # If the robot was given a path, start running.
            if(len(shared.metric_path) > 0 and not(shared.robot is None)):
                RobotLoop(shared)
            #update the kalman history measurement
            if shared.robot is not None:
                kalman_history_vel.append(shared.robot.kalman.get_velocity())
                kalman_history_acce.append(shared.robot.kalman.get_acceleration())
                kalman_history.append(shared.robot.kalman.get_position())
                kalman_history_orientation.append(shared.robot.kalman.get_rotation())
                kalman_history_spin.append(shared.robot.kalman.get_spin())
    except Quit:
        pass

    #get sensor measurement history from kalman
    acc_meas = np.array(shared.robot.kalman.get_accel_meas())
    vel_meas = np.array(shared.robot.kalman.get_vel_meas())
    spin_meas = np.array(shared.robot.kalman.get_spin_meas())
    metric_path = c
    path_shared = shared.path_shared
    plot_data (vel_meas,acc_meas,spin_meas,kalman_history_vel,kalman_history_acce,
               kalman_history_spin, absolute_history, kalman_history, metric_path,absolute_orientation,kalman_history_orientation, path_shared)
    
    
    # A quit has been called (unlock that thymio!!!!)
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()