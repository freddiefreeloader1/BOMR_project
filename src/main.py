from computer_vision import *
from robot_main import init_robot_position, get_time
from Astar_coord import *
from Astar import * 
from camera_main import CameraClose, CameraInit, CameraLoop
from robot_main import RobotClose, RobotInit, RobotLoop
from common import Quit, SharedData,shared,convert_camera_to_robot

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
                
                kalman_history.append(shared.robot.kalman.get_position())
                kalman_history_orientation.append(shared.robot.kalman.get_rotation())
                shared.thymio_position = None

            # If the robot was given a path, start running.
            if(len(shared.metric_path) > 0 and not(shared.robot is None)):
                RobotLoop(shared)
            if shared.robot is not None:
                kalman_history_vel.append(shared.robot.kalman.get_velocity())
                kalman_history_acce.append(shared.robot.kalman.get_acceleration())
                kalman_history_spin.append(shared.robot.kalman.get_spin())
    except Quit:
        pass

    acc_meas = np.array(shared.robot.kalman.get_accel_meas())
    vel_meas = np.array(shared.robot.kalman.get_vel_meas())
    spin_meas = np.array(shared.robot.kalman.get_spin_meas())
    meas_time = np.linspace(0,1,len(spin_meas))
    kalman_history_vel = np.array(kalman_history_vel)
    kalman_history_acce = np.array(kalman_history_acce)
    kalman_history_spin = np.array(kalman_history_spin)
    history_time = np.linspace(0,1,len(kalman_history_acce))
    # Plotting
    absolute_history = np.array(absolute_history)
    kalman_history = np.array(kalman_history)
    metric_path = np.array(c)
    absolute_orientation = np.array(absolute_orientation)
    kalman_history_orientation = np.array(kalman_history_orientation)
    path_display = np.array(shared.path_shared)

    plt.figure(figsize=(15, 10))
    plt.subplot(3,2,1)
    plt.plot(meas_time,vel_meas[:,0],label = 'Measured X velocity (wheel)')
    plt.plot(history_time,kalman_history_vel[:,0],label = 'Kalman X velocity')
    plt.title('Velocity comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(3,2,2)
    meas_time_acc = np.linspace(0,1,len(acc_meas))
    plt.plot(meas_time_acc,acc_meas[:,0],label = 'Measured X acceleration (accelerometer)')
    plt.plot(history_time,kalman_history_acce[:,0],label = 'Kalman X acceleration')
    plt.title('Acceleration comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    plt.subplot(3,2,3)
    plt.plot(meas_time_acc,acc_meas[:,1],label = 'Measured Y acceleration (accelerometer)')
    plt.plot(history_time,kalman_history_acce[:,1],label = 'Kalman Y acceleration')
    plt.title('Acceleration comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    plt.subplot(3,2,4)
    plt.plot(meas_time,vel_meas[:,1],label = 'Measured Y velocity (wheel)')
    plt.plot(history_time,kalman_history_vel[:,1],label = 'Kalman Y velocity')
    plt.title('Velocity comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(3,2,5)
    plt.plot(meas_time,spin_meas,label = 'Measured spin (wheel)')
    plt.plot(history_time,kalman_history_spin,label = 'Kalman spin')
    plt.title('Spin Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Spin rad/s')
    plt.legend()

    plt.subplot(3,2,6)
    meas_time_camera = np.linspace(0,1,len(absolute_orientation))
    plt.plot(meas_time_camera,absolute_orientation,label = 'Measured heading (camera)')
    x_time = np.linspace(0,1,len(kalman_history_orientation))
    plt.plot(x_time,kalman_history_orientation - (2*np.pi),label = 'Kalman heading')
    plt.title('Heading Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('Heading [rad]')
    plt.legend()

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(15, 10))

    plt.scatter(absolute_history[:, 0], absolute_history[:, 1], label='Measured position (camera)', s=1)
    plt.scatter(kalman_history[:, 0], kalman_history[:, 1], label='Kalman position', s=1)

    plt.title('Position Comparison')
    plt.xlabel('Arbitrary time')
    plt.ylabel('position [m]')
    plt.legend()
    
    plt.tight_layout()
    plt.show()



    plt.figure(figsize=(15, 10))

    plt.subplot(2, 2, 1)
    plt.plot(absolute_history[:, 0], absolute_history[:, 1], label='Absolute Position')
    plt.scatter(metric_path[:, 0], metric_path[:, 1], color='red', label='Metric Path Points', marker='x')
    plt.scatter(path_display[:,0],path_display[:,1],label = 'Path from shared',marker = 'o')
    plt.title('Absolute Position History')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(kalman_history[:, 0], kalman_history[:, 1], label='Kalman Position')
    plt.title('Kalman Position History')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(kalman_history_orientation, label='Kalman Orientation')
    plt.title('Orientation History')
    plt.xlabel('Time Steps')
    plt.ylabel('Orientation')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(absolute_orientation, label='Absolute Orientation')
    plt.title('Orientation History')
    plt.xlabel('Time Steps')
    plt.ylabel('Orientation')
    plt.legend()
    plt.tight_layout()
    plt.show()
    # A quit has been called (unlock that thymio!!!!)
    CameraClose()
    RobotClose()

if __name__ == "__main__":
    main()